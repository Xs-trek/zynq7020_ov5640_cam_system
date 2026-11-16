// SPDX-License-Identifier: GPL-2.0-only
/*
 * Zynq-7020 face accelerator platform/misc driver (R9 and R10 bitstreams).
 *
 * One job runs at a time.  RUN copies the input through the driver and blocks; SUBMIT starts a
 * job on an imported dma-buf and WAIT collects it.  The job's input must stay untouched until the
 * job ends, which is why every path that abandons a job soft-resets the accelerator first.  See
 * cam_face_accel_uapi.h for the interface.
 */

#include <asm/unaligned.h>
#include <crypto/algapi.h>
#include <crypto/hash.h>
#include <linux/clk.h>
#include <linux/completion.h>
#include <linux/crypto.h>
#include <linux/delay.h>
#include <linux/dma-buf.h>
#include <linux/dma-mapping.h>
#include <linux/firmware.h>
#include <linux/fs.h>
#include <linux/interrupt.h>
#include <linux/io.h>
#include <linux/iopoll.h>
#include <linux/miscdevice.h>
#include <linux/module.h>
#include <linux/mutex.h>
#include <linux/of.h>
#include <linux/platform_device.h>
#include <linux/scatterlist.h>
#include <linux/slab.h>
#include <linux/uaccess.h>

#include "cam_face_accel_uapi.h"
#include "face_accel_regs.h"

#define CAM_FA_DRIVER_NAME "cam-face-accel"
#define CAM_FA_FIRMWARE "cam-system/face-accel/unified128_dw32_weights_v1.bin"
#define CAM_FA_DEFAULT_TIMEOUT_MS 250u
#define CAM_FA_MAX_TIMEOUT_MS 2000u
#define CAM_FA_WEIGHT_MAGIC 0x42574146u /* little-endian "FAWB" */
#define CAM_FA_QUEUE_DEPTH 1u
/* No-progress watchdog of the PL: 100 ms at its 100 MHz clock. */
#define CAM_FA_PL_TIMEOUT_CYCLES 10000000u
#define CAM_FA_OUT_ANCHORS (FA_OUT_SCORES_LEN / 2u)

static const u8 cam_fa_weight_sha256[32] = {
	0x93, 0xae, 0x93, 0x38, 0x97, 0xbb, 0x51, 0x0a,
	0x2a, 0x47, 0xa3, 0x4a, 0x7e, 0xe2, 0x72, 0x49,
	0xa6, 0x69, 0x04, 0xcc, 0x09, 0x96, 0x4a, 0x19,
	0x38, 0x18, 0x78, 0x91, 0x23, 0x9c, 0xbf, 0xab,
};

struct cam_fa_buffer {
	void *cpu;
	dma_addr_t dma;
	size_t size;
};

struct cam_fa_import {
	struct dma_buf *dbuf;
	struct dma_buf_attachment *attach;
	struct sg_table *sgt;
	dma_addr_t dma;
};

struct cam_fa_dev;

/* One open file; imports belong to it. */
struct cam_fa_file {
	struct cam_fa_dev *fa;
	struct cam_fa_import imports[CAM_FACE_ACCEL_MAX_IMPORTS];
};

struct cam_fa_dev {
	struct device *dev;
	void __iomem *regs;
	struct clk *clk;
	int irq;
	struct miscdevice misc;
	struct mutex job_lock;	/* job state and the registers that start and end a job */
	struct completion done;
	struct cam_fa_buffer input;
	struct cam_fa_buffer weight;
	struct cam_fa_buffer scratch;
	struct cam_fa_buffer output;	/* cached: synced around each job */
	struct cam_fa_buffer pre;	/* R10 only */
	struct cam_fa_file *job_owner;	/* the file with a submitted job */
	u32 job_frame_id;
	unsigned long job_deadline;
	u32 hw_revision;
	u32 hw_features;
	bool weight_ready;
};

static inline u32 cam_fa_read(struct cam_fa_dev *fa, u32 offset)
{
	return ioread32(fa->regs + offset);
}

static inline void cam_fa_write(struct cam_fa_dev *fa, u32 offset, u32 value)
{
	iowrite32(value, fa->regs + offset);
}

static int cam_fa_wait_dma_ready(struct cam_fa_dev *fa)
{
	u32 debug_dma;
	int ret;

	ret = readl_poll_timeout(fa->regs + FA_DEBUG_DMA, debug_dma,
				 (debug_dma & FA_DEBUG_DMA_ARESETN) &&
				 !(debug_dma & FA_DEBUG_DMA_RESET_REQ),
				 10, 100000);
	if (ret)
		dev_err(fa->dev, "DataMover reset did not release: debug_dma=%#x\n",
			debug_dma);
	return ret;
}

static int cam_fa_sha256(const u8 *data, size_t len, u8 digest[32])
{
	struct crypto_shash *tfm;
	struct shash_desc *desc;
	int ret;

	tfm = crypto_alloc_shash("sha256", 0, 0);
	if (IS_ERR(tfm))
		return PTR_ERR(tfm);
	desc = kmalloc(sizeof(*desc) + crypto_shash_descsize(tfm), GFP_KERNEL);
	if (!desc) {
		crypto_free_shash(tfm);
		return -ENOMEM;
	}
	desc->tfm = tfm;
	ret = crypto_shash_digest(desc, data, len, digest);
	kfree(desc);
	crypto_free_shash(tfm);
	return ret;
}

static int cam_fa_load_weight(struct cam_fa_dev *fa)
{
	const struct firmware *fw;
	u8 digest[32];
	int ret;

	ret = request_firmware(&fw, CAM_FA_FIRMWARE, fa->dev);
	if (ret) {
		dev_err(fa->dev, "cannot load %s: %d\n", CAM_FA_FIRMWARE, ret);
		return ret;
	}
	if (fw->size != FACE_ACCEL_WEIGHT_SIZE ||
	    get_unaligned_le32(fw->data) != CAM_FA_WEIGHT_MAGIC) {
		dev_err(fa->dev, "invalid weight header/size (%zu)\n", fw->size);
		ret = -EINVAL;
		goto out;
	}
	ret = cam_fa_sha256(fw->data, fw->size, digest);
	if (ret)
		goto out;
	if (crypto_memneq(digest, cam_fa_weight_sha256, sizeof(digest))) {
		dev_err(fa->dev, "weight SHA-256 mismatch\n");
		ret = -EBADMSG;
		goto out;
	}
	memcpy(fa->weight.cpu, fw->data, fw->size);
	dma_wmb();
	fa->weight_ready = true;
	dev_info(fa->dev, "verified face accelerator weights (%zu bytes)\n",
		 fw->size);
out:
	release_firmware(fw);
	return ret;
}

/* The accelerator reaches DDR from FACE_ACCEL_BUF_MIN_ADDR and checks these at START. */
static bool cam_fa_dma_usable(dma_addr_t dma, size_t size)
{
	return dma >= FACE_ACCEL_BUF_MIN_ADDR &&
	       IS_ALIGNED(dma, FACE_ACCEL_BUF_ALIGN) &&
	       (u64)dma + size <= BIT_ULL(32);
}

static int cam_fa_buffer_alloc(struct cam_fa_dev *fa,
			       struct cam_fa_buffer *buf, size_t size)
{
	buf->size = size;
	buf->cpu = dmam_alloc_coherent(fa->dev, size, &buf->dma, GFP_KERNEL);
	if (!buf->cpu)
		return -ENOMEM;
	if (!cam_fa_dma_usable(buf->dma, size)) {
		dev_err(fa->dev, "unusable DMA address %pad size=%zu\n",
			&buf->dma, size);
		return -ERANGE;
	}
	return 0;
}

static void cam_fa_output_unmap(void *data)
{
	struct cam_fa_dev *fa = data;

	dma_unmap_single(fa->dev, fa->output.dma, fa->output.size,
			 DMA_FROM_DEVICE);
}

/* Cached and mapped for streaming: the output is read through copy_to_user every frame. */
static int cam_fa_output_alloc(struct cam_fa_dev *fa)
{
	struct cam_fa_buffer *buf = &fa->output;
	unsigned long pages;

	buf->size = FA_OUT_CAP_MIN;
	pages = devm_get_free_pages(fa->dev, GFP_KERNEL | __GFP_ZERO,
				    get_order(buf->size));
	if (!pages)
		return -ENOMEM;
	buf->cpu = (void *)pages;
	buf->dma = dma_map_single(fa->dev, buf->cpu, buf->size, DMA_FROM_DEVICE);
	if (dma_mapping_error(fa->dev, buf->dma))
		return -ENOMEM;
	if (!cam_fa_dma_usable(buf->dma, buf->size)) {
		dev_err(fa->dev, "unusable output address %pad\n", &buf->dma);
		cam_fa_output_unmap(fa);
		return -ERANGE;
	}
	return devm_add_action_or_reset(fa->dev, cam_fa_output_unmap, fa);
}

static void cam_fa_collect_result(struct cam_fa_dev *fa,
				  struct cam_face_accel_result *res)
{
	res->status = cam_fa_read(fa, FA_STATUS);
	res->error_code = (res->status & FA_STATUS_ERRCODE_MASK) >>
			  FA_STATUS_ERRCODE_SHIFT;
	res->frame_id_done = cam_fa_read(fa, FA_FRAME_ID_DONE);
	res->output_size = cam_fa_read(fa, FA_OUT_LEN);
	res->perf_cycles = cam_fa_read(fa, FA_PERF_CYCLES);
	res->perf_dep_wait = cam_fa_read(fa, FA_PERF_DEP_WAIT);
	res->perf_cmd_wait = cam_fa_read(fa, FA_PERF_CMD_WAIT);
	res->perf_dma_active = cam_fa_read(fa, FA_PERF_DMA_ACTIVE);
	res->perf_pre_active = cam_fa_read(fa, FA_PERF_PRE_ACTIVE);
	res->perf_op_active = cam_fa_read(fa, FA_PERF_OP_ACTIVE);
	res->perf_merge_active = cam_fa_read(fa, FA_PERF_MERGE_ACTIVE);
	res->debug_progress = cam_fa_read(fa, FA_DEBUG_PROGRESS);
	res->debug_dma = cam_fa_read(fa, FA_DEBUG_DMA);
	res->debug_transport = cam_fa_read(fa, FA_DEBUG_TRANSPORT);
	res->debug_dma_read = cam_fa_read(fa, FA_DEBUG_DMA_READ);
	res->debug_dma_write = cam_fa_read(fa, FA_DEBUG_DMA_WRITE);
}

static int cam_fa_reset_locked(struct cam_fa_dev *fa)
{
	u32 status;
	int ret;

	cam_fa_write(fa, FA_IRQ_ENABLE, 0);
	cam_fa_write(fa, FA_IRQ_STATUS, FA_IRQ_DONE);
	cam_fa_write(fa, FA_CTRL, FA_CTRL_SOFT_RESET);
	ret = readl_poll_timeout(fa->regs + FA_STATUS, status,
				 status & FA_STATUS_IDLE, 10, 100000);
	if (ret) {
		dev_err(fa->dev, "soft reset did not reach IDLE: status=%#x\n",
			status);
		return ret;
	}
	ret = cam_fa_wait_dma_ready(fa);
	if (ret)
		return ret;
	if (fa->hw_features & FA_FEATURE_DRAIN_RESET) {
		u32 info = cam_fa_read(fa, FA_RESET_INFO);

		if (info & FA_RESET_INFO_UNCLEAN) {
			dev_err(fa->dev,
				"soft reset hit the drain limit after %u cycles: DataMover reset with transfers in flight\n",
				(info & FA_RESET_INFO_DRAIN_CYCLES_MASK) >>
				FA_RESET_INFO_DRAIN_CYCLES_SHIFT);
			return -EIO;
		}
	}
	return 0;
}

static int cam_fa_validate_output(struct cam_fa_dev *fa,
				  const struct cam_face_accel_result *res)
{
	const fa_out_header_t *header = fa->output.cpu;
	const fa_tensor_desc_t *desc = (const void *)(header + 1);

	if ((res->status & FA_STATUS_ERROR) || !(res->status & FA_STATUS_DONE) ||
	    res->error_code != FA_ERR_NONE)
		return -EIO;
	if (res->frame_id_done != fa->job_frame_id ||
	    res->output_size != FA_OUT_LEN_V1)
		return -EBADMSG;
	if (header->magic != FA_OUT_MAGIC || header->frame_id != fa->job_frame_id ||
	    header->tensor_count != FA_OUT_TENSOR_COUNT ||
	    header->total_len != FA_OUT_LEN_V1)
		return -EBADMSG;
	if (desc[0].offset != FA_OUT_SCORES_OFFSET ||
	    desc[0].dtype != FA_DTYPE_INT8 ||
	    desc[0].dims[0] != 1 || desc[0].dims[1] != CAM_FA_OUT_ANCHORS ||
	    desc[0].dims[2] != 2 || desc[0].dims[3] != 0 ||
	    desc[0].zero_point != FA_OUT_ZERO_POINT ||
	    desc[1].offset != FA_OUT_BOXES_OFFSET ||
	    desc[1].dtype != FA_DTYPE_INT8 ||
	    desc[1].dims[0] != 1 || desc[1].dims[1] != CAM_FA_OUT_ANCHORS ||
	    desc[1].dims[2] != 4 || desc[1].dims[3] != 0 ||
	    desc[1].zero_point != FA_OUT_ZERO_POINT)
		return -EBADMSG;
	return 0;
}

static int cam_fa_job_timeout(u32 requested, u32 *timeout_ms)
{
	*timeout_ms = requested ? requested : CAM_FA_DEFAULT_TIMEOUT_MS;
	return *timeout_ms > CAM_FA_MAX_TIMEOUT_MS ? -EINVAL : 0;
}

static int cam_fa_start_locked(struct cam_fa_dev *fa, dma_addr_t input,
			       u32 frame_id, u32 timeout_ms)
{
	int ret;

	if (!(cam_fa_read(fa, FA_STATUS) & FA_STATUS_IDLE)) {
		ret = cam_fa_reset_locked(fa);
		if (ret)
			return ret;
	}
	ret = cam_fa_wait_dma_ready(fa);
	if (ret)
		return ret;
	dma_sync_single_for_device(fa->dev, fa->output.dma, fa->output.size,
				   DMA_FROM_DEVICE);
	reinit_completion(&fa->done);
	cam_fa_write(fa, FA_IRQ_ENABLE, 0);
	cam_fa_write(fa, FA_IRQ_STATUS, FA_IRQ_DONE);
	cam_fa_write(fa, FA_IN_BUF_BASE, lower_32_bits(input));
	cam_fa_write(fa, FA_IN_STRIDE, FACE_ACCEL_IN_STRIDE_DEF);
	cam_fa_write(fa, FA_OUT_BUF_BASE, lower_32_bits(fa->output.dma));
	cam_fa_write(fa, FA_OUT_BUF_CAP, FA_OUT_CAP_MIN);
	cam_fa_write(fa, FA_WT_BUF_BASE, lower_32_bits(fa->weight.dma));
	cam_fa_write(fa, FA_WT_BUF_SIZE, FACE_ACCEL_WEIGHT_SIZE);
	cam_fa_write(fa, FA_SCRATCH_BUF_BASE, lower_32_bits(fa->scratch.dma));
	cam_fa_write(fa, FA_SCRATCH_BUF_SIZE, FACE_ACCEL_SCRATCH_SIZE);
	if (fa->pre.cpu) {
		cam_fa_write(fa, FA_PRE_BUF_BASE, lower_32_bits(fa->pre.dma));
		cam_fa_write(fa, FA_PRE_BUF_SIZE, FACE_ACCEL_PRE_SIZE);
	}
	cam_fa_write(fa, FA_FRAME_ID, frame_id);
	cam_fa_write(fa, FA_MODEL_SEL, 0);
	cam_fa_write(fa, FA_TIMEOUT_CYCLES, CAM_FA_PL_TIMEOUT_CYCLES);
	cam_fa_write(fa, FA_IRQ_ENABLE, FA_IRQ_ENABLE_DONE);
	fa->job_frame_id = frame_id;
	fa->job_deadline = jiffies + msecs_to_jiffies(timeout_ms);
	dma_wmb();
	cam_fa_write(fa, FA_CTRL, FA_CTRL_START);
	return 0;
}

/* Ends the running job.  Unless it returns 0, the accelerator has been soft-reset. */
static int cam_fa_finish_locked(struct cam_fa_dev *fa,
				struct cam_face_accel_result *res)
{
	long left = (long)(fa->job_deadline - jiffies);
	long waited;
	int ret;

	waited = wait_for_completion_killable_timeout(&fa->done,
						      left > 0 ? left : 1);
	cam_fa_write(fa, FA_IRQ_ENABLE, 0);
	cam_fa_collect_result(fa, res);
	if (waited < 0) {
		ret = (int)waited;
		goto reset;
	}
	if (!waited) {
		ret = -ETIMEDOUT;
		goto reset;
	}
	dma_sync_single_for_cpu(fa->dev, fa->output.dma, fa->output.size,
				DMA_FROM_DEVICE);
	ret = cam_fa_validate_output(fa, res);
	if (ret)
		goto reset;
	return 0;

reset:
	dev_err_ratelimited(fa->dev,
			    "job failed ret=%d frame=%u done=%u status=%#x err=%#x out=%u debug=%#x dma=%#x transport=%#x read=%#x write=%#x\n",
			    ret, fa->job_frame_id, res->frame_id_done, res->status,
			    res->error_code, res->output_size, res->debug_progress,
			    res->debug_dma, res->debug_transport,
			    res->debug_dma_read, res->debug_dma_write);
	cam_fa_reset_locked(fa);
	return ret;
}

static int cam_fa_run_locked(struct cam_fa_dev *fa,
			     struct cam_face_accel_run *run)
{
	u32 timeout_ms;
	int ret;

	if (!fa->weight_ready)
		return -ENODATA;
	if (fa->job_owner)
		return -EBUSY;
	if (!run->input_ptr || !run->output_ptr ||
	    run->input_size != FACE_ACCEL_FRAME_SIZE ||
	    run->output_capacity < FA_OUT_LEN_V1)
		return -EINVAL;
	ret = cam_fa_job_timeout(run->timeout_ms, &timeout_ms);
	if (ret)
		return ret;
	if (copy_from_user(fa->input.cpu, u64_to_user_ptr(run->input_ptr),
			   FACE_ACCEL_FRAME_SIZE))
		return -EFAULT;
	ret = cam_fa_start_locked(fa, fa->input.dma, run->frame_id, timeout_ms);
	if (ret)
		return ret;
	ret = cam_fa_finish_locked(fa, &run->result);
	if (ret)
		return ret;
	if (copy_to_user(u64_to_user_ptr(run->output_ptr), fa->output.cpu,
			 run->result.output_size))
		return -EFAULT;
	return 0;
}

static int cam_fa_submit_locked(struct cam_fa_file *cfile,
				const struct cam_face_accel_submit *sub)
{
	struct cam_fa_dev *fa = cfile->fa;
	const struct cam_fa_import *imp;
	u32 timeout_ms;
	int ret;

	if (!fa->weight_ready)
		return -ENODATA;
	if (sub->reserved || !sub->input_handle ||
	    sub->input_handle > ARRAY_SIZE(cfile->imports))
		return -EINVAL;
	imp = &cfile->imports[sub->input_handle - 1];
	if (!imp->dbuf)
		return -EINVAL;
	if (fa->job_owner)
		return -EBUSY;
	ret = cam_fa_job_timeout(sub->timeout_ms, &timeout_ms);
	if (ret)
		return ret;
	ret = cam_fa_start_locked(fa, imp->dma, sub->frame_id, timeout_ms);
	if (ret)
		return ret;
	fa->job_owner = cfile;
	return 0;
}

static int cam_fa_wait_locked(struct cam_fa_file *cfile,
			      struct cam_face_accel_wait *wait)
{
	struct cam_fa_dev *fa = cfile->fa;
	int ret;

	if (wait->reserved || !wait->output_ptr ||
	    wait->output_capacity < FA_OUT_LEN_V1)
		return -EINVAL;
	if (fa->job_owner != cfile)
		return -ENODATA;
	ret = cam_fa_finish_locked(fa, &wait->result);
	fa->job_owner = NULL;
	if (ret)
		return ret;
	if (copy_to_user(u64_to_user_ptr(wait->output_ptr), fa->output.cpu,
			 wait->result.output_size))
		return -EFAULT;
	return 0;
}

static void cam_fa_import_release(struct cam_fa_import *imp)
{
	dma_buf_unmap_attachment(imp->attach, imp->sgt, DMA_TO_DEVICE);
	dma_buf_detach(imp->dbuf, imp->attach);
	dma_buf_put(imp->dbuf);
	memset(imp, 0, sizeof(*imp));
}

static int cam_fa_import_locked(struct cam_fa_file *cfile,
				struct cam_face_accel_import *req)
{
	struct cam_fa_dev *fa = cfile->fa;
	struct cam_fa_import imp = { };
	unsigned int slot;
	int ret;

	if (!fa->pre.cpu)
		return -EOPNOTSUPP;
	if (req->reserved[0] || req->reserved[1])
		return -EINVAL;
	for (slot = 0; slot < ARRAY_SIZE(cfile->imports); slot++)
		if (!cfile->imports[slot].dbuf)
			break;
	if (slot == ARRAY_SIZE(cfile->imports))
		return -ENOSPC;

	imp.dbuf = dma_buf_get(req->fd);
	if (IS_ERR(imp.dbuf))
		return PTR_ERR(imp.dbuf);
	imp.attach = dma_buf_attach(imp.dbuf, fa->dev);
	if (IS_ERR(imp.attach)) {
		ret = PTR_ERR(imp.attach);
		goto put;
	}
	imp.sgt = dma_buf_map_attachment(imp.attach, DMA_TO_DEVICE);
	if (IS_ERR(imp.sgt)) {
		ret = PTR_ERR(imp.sgt);
		goto detach;
	}
	imp.dma = sg_dma_address(imp.sgt->sgl);
	if (imp.dbuf->size < FACE_ACCEL_FRAME_SIZE || imp.sgt->nents != 1 ||
	    sg_dma_len(imp.sgt->sgl) < FACE_ACCEL_FRAME_SIZE ||
	    !cam_fa_dma_usable(imp.dma, FACE_ACCEL_FRAME_SIZE)) {
		dev_err(fa->dev, "unusable import: size=%zu segments=%u address %pad\n",
			imp.dbuf->size, imp.sgt->nents, &imp.dma);
		ret = -EINVAL;
		goto unmap;
	}
	cfile->imports[slot] = imp;
	req->handle = slot + 1;
	return 0;

unmap:
	dma_buf_unmap_attachment(imp.attach, imp.sgt, DMA_TO_DEVICE);
detach:
	dma_buf_detach(imp.dbuf, imp.attach);
put:
	dma_buf_put(imp.dbuf);
	return ret;
}

/*
 * The output is left to the accelerator from sync_for_device to sync_for_cpu, so the handler
 * touches registers only.  irq = IRQ_ENABLE & IRQ_STATUS, so IRQ_STATUS alone identifies the
 * source; STATUS.DONE is sticky and says nothing about this interrupt.
 */
static irqreturn_t cam_fa_irq(int irq, void *data)
{
	struct cam_fa_dev *fa = data;

	if (!(cam_fa_read(fa, FA_IRQ_STATUS) & FA_IRQ_DONE))
		return IRQ_NONE;
	cam_fa_write(fa, FA_IRQ_STATUS, FA_IRQ_DONE);
	complete(&fa->done);
	return IRQ_HANDLED;
}

static int cam_fa_open(struct inode *inode, struct file *file)
{
	struct miscdevice *misc = file->private_data;
	struct cam_fa_file *cfile;

	cfile = kzalloc(sizeof(*cfile), GFP_KERNEL);
	if (!cfile)
		return -ENOMEM;
	cfile->fa = container_of(misc, struct cam_fa_dev, misc);
	file->private_data = cfile;
	return 0;
}

static int cam_fa_release(struct inode *inode, struct file *file)
{
	struct cam_fa_file *cfile = file->private_data;
	struct cam_fa_dev *fa = cfile->fa;
	unsigned int i;

	/* The accelerator stops reading an import before it is unmapped. */
	mutex_lock(&fa->job_lock);
	if (fa->job_owner == cfile) {
		cam_fa_reset_locked(fa);
		fa->job_owner = NULL;
	}
	for (i = 0; i < ARRAY_SIZE(cfile->imports); i++)
		if (cfile->imports[i].dbuf)
			cam_fa_import_release(&cfile->imports[i]);
	mutex_unlock(&fa->job_lock);
	kfree(cfile);
	return 0;
}

static long cam_fa_ioctl(struct file *file, unsigned int cmd, unsigned long arg)
{
	struct cam_fa_file *cfile = file->private_data;
	struct cam_fa_dev *fa = cfile->fa;
	void __user *argp = (void __user *)arg;
	int ret = 0;

	if (_IOC_TYPE(cmd) != CAM_FACE_ACCEL_IOC_MAGIC)
		return -ENOTTY;
	switch (cmd) {
	case CAM_FACE_ACCEL_IOC_GET_INFO: {
		struct cam_face_accel_info info = {
			.uapi_version = CAM_FACE_ACCEL_INFO_VERSION,
			.ip_id = cam_fa_read(fa, FA_IP_ID),
			.input_width = FACE_ACCEL_IN_W,
			.input_height = FACE_ACCEL_IN_H,
			.input_stride = FACE_ACCEL_IN_STRIDE_DEF,
			.input_size = FACE_ACCEL_FRAME_SIZE,
			.output_size = FA_OUT_LEN_V1,
			.output_capacity = FA_OUT_CAP_MIN,
			.weight_size = FACE_ACCEL_WEIGHT_SIZE,
			.scratch_size = FACE_ACCEL_SCRATCH_SIZE,
			.flags = CAM_FACE_ACCEL_INFO_IRQ_READY |
				 (fa->weight_ready ? CAM_FACE_ACCEL_INFO_WEIGHT_READY : 0),
		};
		if (copy_to_user(argp, &info, sizeof(info)))
			ret = -EFAULT;
		break;
	}
	case CAM_FACE_ACCEL_IOC_GET_CAPS: {
		struct cam_face_accel_caps caps = {
			.uapi_version = CAM_FACE_ACCEL_UAPI_VERSION,
			.flags = fa->pre.cpu ? CAM_FACE_ACCEL_CAP_DMABUF_INPUT : 0,
			.hw_revision = fa->hw_revision,
			.hw_features = fa->hw_features,
			.queue_depth = CAM_FA_QUEUE_DEPTH,
			.max_imports = CAM_FACE_ACCEL_MAX_IMPORTS,
		};
		memcpy(caps.weight_sha256, cam_fa_weight_sha256,
		       sizeof(caps.weight_sha256));
		if (copy_to_user(argp, &caps, sizeof(caps)))
			ret = -EFAULT;
		break;
	}
	case CAM_FACE_ACCEL_IOC_RUN: {
		struct cam_face_accel_run run;

		if (copy_from_user(&run, argp, sizeof(run)))
			return -EFAULT;
		memset(&run.result, 0, sizeof(run.result));
		ret = mutex_lock_killable(&fa->job_lock);
		if (ret)
			return ret;
		ret = cam_fa_run_locked(fa, &run);
		mutex_unlock(&fa->job_lock);
		if (copy_to_user(argp, &run, sizeof(run)))
			return -EFAULT;
		break;
	}
	case CAM_FACE_ACCEL_IOC_RESET:
		ret = mutex_lock_killable(&fa->job_lock);
		if (ret)
			return ret;
		if (fa->job_owner && fa->job_owner != cfile) {
			ret = -EBUSY;
		} else {
			ret = cam_fa_reset_locked(fa);
			fa->job_owner = NULL;
		}
		mutex_unlock(&fa->job_lock);
		break;
	case CAM_FACE_ACCEL_IOC_IMPORT: {
		struct cam_face_accel_import req;

		if (copy_from_user(&req, argp, sizeof(req)))
			return -EFAULT;
		ret = mutex_lock_killable(&fa->job_lock);
		if (ret)
			return ret;
		ret = cam_fa_import_locked(cfile, &req);
		mutex_unlock(&fa->job_lock);
		if (!ret && copy_to_user(argp, &req, sizeof(req)))
			return -EFAULT;
		break;
	}
	case CAM_FACE_ACCEL_IOC_SUBMIT: {
		struct cam_face_accel_submit sub;

		if (copy_from_user(&sub, argp, sizeof(sub)))
			return -EFAULT;
		ret = mutex_lock_killable(&fa->job_lock);
		if (ret)
			return ret;
		ret = cam_fa_submit_locked(cfile, &sub);
		mutex_unlock(&fa->job_lock);
		break;
	}
	case CAM_FACE_ACCEL_IOC_WAIT: {
		struct cam_face_accel_wait wait;

		if (copy_from_user(&wait, argp, sizeof(wait)))
			return -EFAULT;
		memset(&wait.result, 0, sizeof(wait.result));
		ret = mutex_lock_killable(&fa->job_lock);
		if (ret)
			return ret;
		ret = cam_fa_wait_locked(cfile, &wait);
		mutex_unlock(&fa->job_lock);
		if (copy_to_user(argp, &wait, sizeof(wait)))
			return -EFAULT;
		break;
	}
	default:
		ret = -ENOTTY;
	}
	return ret;
}

static const struct file_operations cam_fa_fops = {
	.owner = THIS_MODULE,
	.open = cam_fa_open,
	.release = cam_fa_release,
	.unlocked_ioctl = cam_fa_ioctl,
	.compat_ioctl = compat_ptr_ioctl,
	.llseek = no_llseek,
};

static int cam_fa_probe(struct platform_device *pdev)
{
	struct cam_fa_dev *fa;
	u32 debug_dma, ip_id, status;
	int ret;

	fa = devm_kzalloc(&pdev->dev, sizeof(*fa), GFP_KERNEL);
	if (!fa)
		return -ENOMEM;
	fa->dev = &pdev->dev;
	fa->regs = devm_platform_ioremap_resource(pdev, 0);
	if (IS_ERR(fa->regs))
		return PTR_ERR(fa->regs);
	fa->irq = platform_get_irq(pdev, 0);
	if (fa->irq < 0)
		return fa->irq;
	fa->clk = devm_clk_get(&pdev->dev, "clk");
	if (IS_ERR(fa->clk))
		return dev_err_probe(&pdev->dev, PTR_ERR(fa->clk), "missing clk\n");
	ret = clk_prepare_enable(fa->clk);
	if (ret)
		return ret;
	ret = dma_set_mask_and_coherent(&pdev->dev, DMA_BIT_MASK(32));
	if (ret)
		goto err_clk;

	ip_id = cam_fa_read(fa, FA_IP_ID);
	status = cam_fa_read(fa, FA_STATUS);
	debug_dma = cam_fa_read(fa, FA_DEBUG_DMA);
	if (ip_id != FACE_ACCEL_IP_ID_MAGIC ||
	    cam_fa_read(fa, FA_IN_W_REG) != FACE_ACCEL_IN_W ||
	    cam_fa_read(fa, FA_IN_H_REG) != FACE_ACCEL_IN_H ||
	    !(status & FA_STATUS_IDLE) ||
	    !(debug_dma & FA_DEBUG_DMA_ARESETN) ||
	    (debug_dma & FA_DEBUG_DMA_RESET_REQ)) {
		dev_err(&pdev->dev,
			"contract mismatch id=%#x status=%#x dma=%#x w=%u h=%u\n",
			ip_id, status, debug_dma, cam_fa_read(fa, FA_IN_W_REG),
			cam_fa_read(fa, FA_IN_H_REG));
		ret = -ENODEV;
		goto err_clk;
	}
	fa->hw_revision = cam_fa_read(fa, FA_HW_REVISION);
	fa->hw_features = cam_fa_read(fa, FA_HW_FEATURES);
	ret = cam_fa_buffer_alloc(fa, &fa->input, FACE_ACCEL_FRAME_SIZE);
	if (ret)
		goto err_clk;
	ret = cam_fa_buffer_alloc(fa, &fa->weight, FACE_ACCEL_WEIGHT_SIZE);
	if (ret)
		goto err_clk;
	ret = cam_fa_buffer_alloc(fa, &fa->scratch, FACE_ACCEL_SCRATCH_SIZE);
	if (ret)
		goto err_clk;
	ret = cam_fa_output_alloc(fa);
	if (ret)
		goto err_clk;
	if (fa->hw_features & FA_FEATURE_PRE_BUF) {
		/* Every job uses it: an R10 accelerator never writes its input. */
		ret = cam_fa_buffer_alloc(fa, &fa->pre, FACE_ACCEL_PRE_SIZE);
		if (ret)
			goto err_clk;
	}

	mutex_init(&fa->job_lock);
	init_completion(&fa->done);
	ret = devm_request_irq(&pdev->dev, fa->irq, cam_fa_irq, 0,
			       dev_name(&pdev->dev), fa);
	if (ret)
		goto err_clk;
	ret = cam_fa_load_weight(fa);
	if (ret)
		goto err_clk;

	fa->misc.minor = MISC_DYNAMIC_MINOR;
	fa->misc.name = "face_accel";
	fa->misc.fops = &cam_fa_fops;
	fa->misc.parent = &pdev->dev;
	ret = misc_register(&fa->misc);
	if (ret)
		goto err_clk;
	platform_set_drvdata(pdev, fa);
	dev_info(&pdev->dev,
		 "accelerator ready id=%#x revision=%u features=%#x irq=%d dma=%#x\n",
		 ip_id, fa->hw_revision, fa->hw_features, fa->irq, debug_dma);
	return 0;

err_clk:
	clk_disable_unprepare(fa->clk);
	return ret;
}

static int cam_fa_remove(struct platform_device *pdev)
{
	struct cam_fa_dev *fa = platform_get_drvdata(pdev);

	misc_deregister(&fa->misc);
	mutex_lock(&fa->job_lock);
	cam_fa_reset_locked(fa);
	mutex_unlock(&fa->job_lock);
	devm_free_irq(&pdev->dev, fa->irq, fa);
	clk_disable_unprepare(fa->clk);
	return 0;
}

static const struct of_device_id cam_fa_of_match[] = {
	{ .compatible = "xlnx,face-accel-1.0" },
	{ }
};
MODULE_DEVICE_TABLE(of, cam_fa_of_match);

static struct platform_driver cam_fa_driver = {
	.probe = cam_fa_probe,
	.remove = cam_fa_remove,
	.driver = {
		.name = CAM_FA_DRIVER_NAME,
		.of_match_table = cam_fa_of_match,
		.suppress_bind_attrs = true,
	},
};
module_platform_driver(cam_fa_driver);

MODULE_DESCRIPTION("cam_system face accelerator driver");
MODULE_AUTHOR("cam_system project");
MODULE_LICENSE("GPL");
MODULE_VERSION("0.2.0");
