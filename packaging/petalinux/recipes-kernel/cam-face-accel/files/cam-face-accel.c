// SPDX-License-Identifier: GPL-2.0-only
/* Zynq-7020 face accelerator platform/misc driver. */

#include <crypto/algapi.h>
#include <crypto/hash.h>
#include <linux/clk.h>
#include <linux/completion.h>
#include <linux/crypto.h>
#include <linux/delay.h>
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
#include <linux/slab.h>
#include <linux/uaccess.h>

#include "cam_face_accel_uapi.h"
#include "face_accel_regs.h"

#define CAM_FA_DRIVER_NAME "cam-face-accel"
#define CAM_FA_FIRMWARE "cam-system/face-accel/unified128_dw32_weights_v1.bin"
#define CAM_FA_DEFAULT_TIMEOUT_MS 250u
#define CAM_FA_MAX_TIMEOUT_MS 2000u
#define CAM_FA_WEIGHT_MAGIC 0x42574146u /* little-endian "FAWB" */

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

struct cam_fa_dev {
	struct device *dev;
	void __iomem *regs;
	struct clk *clk;
	int irq;
	struct miscdevice misc;
	struct mutex job_lock;
	struct completion done;
	struct cam_fa_buffer input;
	struct cam_fa_buffer weight;
	struct cam_fa_buffer scratch;
	struct cam_fa_buffer output;
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

static int cam_fa_buffer_alloc(struct cam_fa_dev *fa,
			       struct cam_fa_buffer *buf, size_t size)
{
	buf->size = size;
	buf->cpu = dmam_alloc_coherent(fa->dev, size, &buf->dma, GFP_KERNEL);
	if (!buf->cpu)
		return -ENOMEM;
	if (!buf->dma || upper_32_bits(buf->dma) ||
	    !IS_ALIGNED(buf->dma, FACE_ACCEL_BUF_ALIGN)) {
		dev_err(fa->dev, "unusable DMA address %pad size=%zu\n",
			&buf->dma, size);
		return -ERANGE;
	}
	return 0;
}

static void cam_fa_collect_result(struct cam_fa_dev *fa,
				  struct cam_face_accel_run *run)
{
	run->status = cam_fa_read(fa, FA_STATUS);
	run->error_code = (run->status & FA_STATUS_ERRCODE_MASK) >>
			  FA_STATUS_ERRCODE_SHIFT;
	run->frame_id_done = cam_fa_read(fa, FA_FRAME_ID_DONE);
	run->output_size = cam_fa_read(fa, FA_OUT_LEN);
	run->perf_cycles = cam_fa_read(fa, FA_PERF_CYCLES);
	run->perf_dep_wait = cam_fa_read(fa, FA_PERF_DEP_WAIT);
	run->perf_cmd_wait = cam_fa_read(fa, FA_PERF_CMD_WAIT);
	run->perf_dma_active = cam_fa_read(fa, FA_PERF_DMA_ACTIVE);
	run->perf_pre_active = cam_fa_read(fa, FA_PERF_PRE_ACTIVE);
	run->perf_op_active = cam_fa_read(fa, FA_PERF_OP_ACTIVE);
	run->perf_merge_active = cam_fa_read(fa, FA_PERF_MERGE_ACTIVE);
	run->debug_progress = cam_fa_read(fa, FA_DEBUG_PROGRESS);
	run->debug_dma = cam_fa_read(fa, FA_DEBUG_DMA);
	run->debug_transport = cam_fa_read(fa, FA_DEBUG_TRANSPORT);
	run->debug_dma_read = cam_fa_read(fa, FA_DEBUG_DMA_READ);
	run->debug_dma_write = cam_fa_read(fa, FA_DEBUG_DMA_WRITE);
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
	return cam_fa_wait_dma_ready(fa);
}

static int cam_fa_validate_output(struct cam_fa_dev *fa,
				  const struct cam_face_accel_run *run)
{
	const fa_out_header_t *header = fa->output.cpu;
	const fa_tensor_desc_t *desc = (const void *)(header + 1);

	if ((run->status & FA_STATUS_ERROR) || !(run->status & FA_STATUS_DONE) ||
	    run->error_code != FA_ERR_NONE)
		return -EIO;
	if (run->frame_id_done != run->frame_id || run->output_size != FA_OUT_LEN_V1)
		return -EBADMSG;
	dma_rmb();
	if (header->magic != FA_OUT_MAGIC || header->frame_id != run->frame_id ||
	    header->tensor_count != FA_OUT_TENSOR_COUNT ||
	    header->total_len != FA_OUT_LEN_V1)
		return -EBADMSG;
	if (desc[0].offset != FA_OUT_SCORES_OFFSET ||
	    desc[0].dtype != FA_DTYPE_INT8 ||
	    desc[0].dims[0] != 1 || desc[0].dims[1] != 4420 ||
	    desc[0].dims[2] != 2 || desc[0].dims[3] != 0 ||
	    desc[0].zero_point != FA_OUT_ZERO_POINT ||
	    desc[1].offset != FA_OUT_BOXES_OFFSET ||
	    desc[1].dtype != FA_DTYPE_INT8 ||
	    desc[1].dims[0] != 1 || desc[1].dims[1] != 4420 ||
	    desc[1].dims[2] != 4 || desc[1].dims[3] != 0 ||
	    desc[1].zero_point != FA_OUT_ZERO_POINT)
		return -EBADMSG;
	return 0;
}

static int cam_fa_run_locked(struct cam_fa_dev *fa,
			     struct cam_face_accel_run *run)
{
	unsigned long waited;
	u32 timeout_ms;
	int ret;

	if (!fa->weight_ready)
		return -ENODATA;
	if (!run->input_ptr || !run->output_ptr ||
	    run->input_size != FACE_ACCEL_FRAME_SIZE ||
	    run->output_capacity < FA_OUT_LEN_V1)
		return -EINVAL;
	timeout_ms = run->timeout_ms ? run->timeout_ms : CAM_FA_DEFAULT_TIMEOUT_MS;
	if (timeout_ms > CAM_FA_MAX_TIMEOUT_MS)
		return -EINVAL;
	if (copy_from_user(fa->input.cpu, u64_to_user_ptr(run->input_ptr),
			   FACE_ACCEL_FRAME_SIZE))
		return -EFAULT;

	if (!(cam_fa_read(fa, FA_STATUS) & FA_STATUS_IDLE)) {
		ret = cam_fa_reset_locked(fa);
		if (ret)
			return ret;
	}
	ret = cam_fa_wait_dma_ready(fa);
	if (ret)
		return ret;
	memset(fa->output.cpu, 0, FA_OUT_CAP_MIN);
	reinit_completion(&fa->done);
	cam_fa_write(fa, FA_IRQ_ENABLE, 0);
	cam_fa_write(fa, FA_IRQ_STATUS, FA_IRQ_DONE);
	cam_fa_write(fa, FA_IN_BUF_BASE, lower_32_bits(fa->input.dma));
	cam_fa_write(fa, FA_IN_STRIDE, FACE_ACCEL_IN_STRIDE_DEF);
	cam_fa_write(fa, FA_OUT_BUF_BASE, lower_32_bits(fa->output.dma));
	cam_fa_write(fa, FA_OUT_BUF_CAP, FA_OUT_CAP_MIN);
	cam_fa_write(fa, FA_WT_BUF_BASE, lower_32_bits(fa->weight.dma));
	cam_fa_write(fa, FA_WT_BUF_SIZE, FACE_ACCEL_WEIGHT_SIZE);
	cam_fa_write(fa, FA_SCRATCH_BUF_BASE, lower_32_bits(fa->scratch.dma));
	cam_fa_write(fa, FA_SCRATCH_BUF_SIZE, FACE_ACCEL_SCRATCH_SIZE);
	cam_fa_write(fa, FA_FRAME_ID, run->frame_id);
	cam_fa_write(fa, FA_MODEL_SEL, 0);
	cam_fa_write(fa, FA_TIMEOUT_CYCLES, 10000000u);
	cam_fa_write(fa, FA_IRQ_ENABLE, FA_IRQ_ENABLE_DONE);
	dma_wmb();
	cam_fa_write(fa, FA_CTRL, FA_CTRL_START);

	waited = wait_for_completion_interruptible_timeout(
			&fa->done, msecs_to_jiffies(timeout_ms));
	cam_fa_write(fa, FA_IRQ_ENABLE, 0);
	cam_fa_collect_result(fa, run);
	if ((long)waited < 0) {
		ret = (int)(long)waited;
		goto reset;
	}
	if (!waited) {
		ret = -ETIMEDOUT;
		goto reset;
	}
	ret = cam_fa_validate_output(fa, run);
	if (ret)
		goto reset;
	if (copy_to_user(u64_to_user_ptr(run->output_ptr), fa->output.cpu,
			 run->output_size))
		return -EFAULT;
	return 0;

reset:
	dev_err_ratelimited(fa->dev,
		"job failed ret=%d frame=%u done=%u status=%#x err=%#x out=%u debug=%#x dma=%#x transport=%#x read=%#x write=%#x\n",
		ret, run->frame_id, run->frame_id_done, run->status,
		run->error_code, run->output_size, run->debug_progress,
		run->debug_dma, run->debug_transport, run->debug_dma_read,
		run->debug_dma_write);
	cam_fa_reset_locked(fa);
	return ret;
}

static irqreturn_t cam_fa_irq(int irq, void *data)
{
	struct cam_fa_dev *fa = data;
	u32 irq_status = cam_fa_read(fa, FA_IRQ_STATUS);
	u32 status = cam_fa_read(fa, FA_STATUS);

	if (!(irq_status & FA_IRQ_DONE) &&
	    !(status & (FA_STATUS_DONE | FA_STATUS_ERROR)))
		return IRQ_NONE;
	cam_fa_write(fa, FA_IRQ_STATUS, FA_IRQ_DONE);
	complete(&fa->done);
	return IRQ_HANDLED;
}

static long cam_fa_ioctl(struct file *file, unsigned int cmd, unsigned long arg)
{
	struct miscdevice *misc = file->private_data;
	struct cam_fa_dev *fa = container_of(misc, struct cam_fa_dev, misc);
	void __user *argp = (void __user *)arg;
	int ret = 0;

	if (_IOC_TYPE(cmd) != CAM_FACE_ACCEL_IOC_MAGIC)
		return -ENOTTY;
	switch (cmd) {
	case CAM_FACE_ACCEL_IOC_GET_INFO: {
		struct cam_face_accel_info info = {
			.uapi_version = CAM_FACE_ACCEL_UAPI_VERSION,
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
	case CAM_FACE_ACCEL_IOC_RUN: {
		struct cam_face_accel_run run;
		if (copy_from_user(&run, argp, sizeof(run)))
			return -EFAULT;
		memset(&run.output_size, 0,
		       sizeof(run) - offsetof(struct cam_face_accel_run, output_size));
		mutex_lock(&fa->job_lock);
		ret = cam_fa_run_locked(fa, &run);
		mutex_unlock(&fa->job_lock);
		if (copy_to_user(argp, &run, sizeof(run)))
			return -EFAULT;
		break;
	}
	case CAM_FACE_ACCEL_IOC_RESET:
		mutex_lock(&fa->job_lock);
		ret = cam_fa_reset_locked(fa);
		mutex_unlock(&fa->job_lock);
		break;
	default:
		ret = -ENOTTY;
	}
	return ret;
}

static const struct file_operations cam_fa_fops = {
	.owner = THIS_MODULE,
	.unlocked_ioctl = cam_fa_ioctl,
#ifdef CONFIG_COMPAT
	.compat_ioctl = cam_fa_ioctl,
#endif
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
	ret = cam_fa_buffer_alloc(fa, &fa->input, FACE_ACCEL_FRAME_SIZE);
	if (ret)
		goto err_clk;
	ret = cam_fa_buffer_alloc(fa, &fa->weight, FACE_ACCEL_WEIGHT_SIZE);
	if (ret)
		goto err_clk;
	ret = cam_fa_buffer_alloc(fa, &fa->scratch, FACE_ACCEL_SCRATCH_SIZE);
	if (ret)
		goto err_clk;
	ret = cam_fa_buffer_alloc(fa, &fa->output, FA_OUT_CAP_MIN);
	if (ret)
		goto err_clk;

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
	dev_info(&pdev->dev, "accelerator ready id=%#x irq=%d dma=%#x\n",
		 ip_id, fa->irq, debug_dma);
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
	},
};
module_platform_driver(cam_fa_driver);

MODULE_DESCRIPTION("cam_system face accelerator driver");
MODULE_AUTHOR("cam_system project");
MODULE_LICENSE("GPL");
MODULE_VERSION("0.1.1");
