/* face_accel_driver_test.c - the driver source against a model of the accelerator.
 *
 * tests/kshim/kshim.h stands in for the kernel.  The model checks START parameters as
 * contracts/vm_interface_r10.json specifies and ends a job when the driver waits for it, in the way
 * the scenario asks.  The cached output buffer is modelled: the accelerator writes a shadow that
 * reaches the CPU only through dma_sync_single_for_cpu.  This checks driver logic and resource
 * release; it says nothing about real timing.
 */
#include "cam-face-accel.c"

#include <assert.h>

/* ── results ── */
static int g_failures;

#define CHECK(cond) do { \
	if (!(cond)) { \
		fprintf(stderr, "FAIL %s:%d: %s\n", __func__, __LINE__, #cond); \
		g_failures++; \
	} \
} while (0)

/* ── shim state ── */
int shim_dev_errors;
unsigned long jiffies = 1000;
char shim_regs[256];

enum scenario {
	JOB_OK, JOB_TIMEOUT, JOB_PL_ERROR, JOB_BAD_MAGIC, JOB_STALE_FRAME_ID, JOB_KILLED, JOB_IRQ_LOST
};

#define COHERENT_BASE 0x10000000u
#define OUTPUT_DMA    0x20000000u
#define IMPORT_DMA    0x3F000000u
#define MAX_ALLOCS    16

static struct {
	u32 regs[64];
	u32 revision;
	u32 features;
	enum scenario scenario;
	int running;
	int starts;
	int rejected;
	int soft_resets;
	int soft_resets_at_unmap;
	int pre_buf_writes;
	u32 reset_info;
	u32 last_input;
	u32 last_pre;
} model;

static struct {
	void *alloc[MAX_ALLOCS];
	int alloc_count;
	u32 next_coherent;
	u8 *output_cpu;
	u8 shadow[FA_OUT_CAP_MIN];
	int mapped;
	int sync_device;
	int sync_cpu;
	void (*action)(void *);
	void *action_data;
	irqreturn_t (*handler)(int, void *);
	void *handler_data;
	int completions;
	int mutex_depth;
	int dbuf_refs;
	int attachments;
	int mappings;
	struct dma_buf dbufs[4];
	struct dma_buf_attachment attach;
	struct scatterlist sg;
	struct sg_table sgt;
	unsigned int import_segments;
	dma_addr_t import_dma;
	size_t import_size;
	u8 firmware[FACE_ACCEL_WEIGHT_SIZE];
	int digest_wrong;
} shim;

/* ── the accelerator ── */
static u32 *reg(u32 offset)
{
	assert(offset < sizeof(model.regs) && offset % 4 == 0);
	return &model.regs[offset / 4];
}

static void model_reject(u32 code)
{
	*reg(FA_STATUS) = (*reg(FA_STATUS) & ~FA_STATUS_ERRCODE_MASK) | FA_STATUS_ERROR |
			  (code << FA_STATUS_ERRCODE_SHIFT);
	*reg(FA_IRQ_STATUS) |= FA_IRQ_DONE;
}

static bool base_ok(u32 base, u64 size)
{
	return base != 0 && (base & 63u) == 0 && base >= FACE_ACCEL_BUF_MIN_ADDR &&
	       (u64)base + size <= BIT_ULL(32);
}

static void model_start(void)
{
	u32 pre = *reg(FA_PRE_BUF_BASE);

	if (model.running) {
		model_reject(0x20);
		model.rejected++;
		return;
	}
	if (!base_ok(*reg(FA_IN_BUF_BASE), FACE_ACCEL_FRAME_SIZE) ||
	    !base_ok(*reg(FA_OUT_BUF_BASE), *reg(FA_OUT_BUF_CAP)) ||
	    !base_ok(*reg(FA_WT_BUF_BASE), *reg(FA_WT_BUF_SIZE)) ||
	    !base_ok(*reg(FA_SCRATCH_BUF_BASE), *reg(FA_SCRATCH_BUF_SIZE)) ||
	    *reg(FA_IN_STRIDE) != FACE_ACCEL_IN_STRIDE_DEF || *reg(FA_OUT_BUF_CAP) < FA_OUT_CAP_MIN ||
	    *reg(FA_WT_BUF_SIZE) < FACE_ACCEL_WEIGHT_SIZE ||
	    *reg(FA_SCRATCH_BUF_SIZE) < FACE_ACCEL_SCRATCH_SIZE || *reg(FA_MODEL_SEL) != 0 ||
	    (pre && (!base_ok(pre, *reg(FA_PRE_BUF_SIZE)) ||
		     *reg(FA_PRE_BUF_SIZE) < FACE_ACCEL_PRE_SIZE))) {
		model_reject(0x30);
		model.rejected++;
		return;
	}
	model.running = 1;
	model.starts++;
	model.last_input = *reg(FA_IN_BUF_BASE);
	model.last_pre = pre;
	*reg(FA_STATUS) = FA_STATUS_BUSY;
	*reg(FA_IRQ_STATUS) = 0;
}

static void model_finish_job(void)
{
	fa_out_header_t header;
	fa_tensor_desc_t desc[2];

	memset(shim.shadow, 0, sizeof(shim.shadow));
	memset(&header, 0, sizeof(header));
	memset(desc, 0, sizeof(desc));
	header.magic = FA_OUT_MAGIC;
	header.frame_id = *reg(FA_FRAME_ID);
	header.tensor_count = FA_OUT_TENSOR_COUNT;
	header.total_len = FA_OUT_LEN_V1;
	desc[0].offset = FA_OUT_SCORES_OFFSET;
	desc[0].dtype = FA_DTYPE_INT8;
	desc[0].dims[0] = 1;
	desc[0].dims[1] = 4420;
	desc[0].dims[2] = 2;
	desc[0].zero_point = FA_OUT_ZERO_POINT;
	desc[1].offset = FA_OUT_BOXES_OFFSET;
	desc[1].dtype = FA_DTYPE_INT8;
	desc[1].dims[0] = 1;
	desc[1].dims[1] = 4420;
	desc[1].dims[2] = 4;
	desc[1].zero_point = FA_OUT_ZERO_POINT;
	if (model.scenario == JOB_BAD_MAGIC)
		header.magic = 0;
	memcpy(shim.shadow, &header, sizeof(header));
	memcpy(shim.shadow + sizeof(header), desc, sizeof(desc));

	model.running = 0;
	*reg(FA_FRAME_ID_DONE) = *reg(FA_FRAME_ID);
	if (model.scenario == JOB_STALE_FRAME_ID)
		*reg(FA_FRAME_ID_DONE) += 1;
	*reg(FA_OUT_LEN) = FA_OUT_LEN_V1;
	*reg(FA_PERF_CYCLES) = 2200000;
	*reg(FA_STATUS) = FA_STATUS_IDLE | FA_STATUS_DONE;
	*reg(FA_IRQ_STATUS) |= FA_IRQ_DONE;
}

static void model_fire_irq(void)
{
	if ((*reg(FA_IRQ_ENABLE) & *reg(FA_IRQ_STATUS)) & 1u)
		shim.handler(55, shim.handler_data);
}

u32 model_read(u32 offset)
{
	if (offset == FA_HW_REVISION)
		return model.revision;
	if (offset == FA_HW_FEATURES)
		return model.features;
	if (offset == FA_RESET_INFO)
		return model.reset_info;
	return *reg(offset);
}

void model_write(u32 offset, u32 value)
{
	if (offset == FA_CTRL) {
		if (value & FA_CTRL_START)
			model_start();
		if (value & FA_CTRL_SOFT_RESET) {
			model.running = 0;
			model.soft_resets++;
			*reg(FA_STATUS) = FA_STATUS_IDLE;
			*reg(FA_OUT_LEN) = 0;
			*reg(FA_IRQ_STATUS) = 0;
		}
		return;
	}
	if (offset == FA_IRQ_STATUS) {
		*reg(FA_IRQ_STATUS) &= ~value;
		return;
	}
	if (offset == FA_PRE_BUF_BASE || offset == FA_PRE_BUF_SIZE) {
		/* undefined offsets on R9 ignore writes; the driver should not rely on that */
		CHECK(model.revision >= 10);
		model.pre_buf_writes++;
	}
	*reg(offset) = value;
}

/* ── shim functions ── */
void *shim_devm_zalloc(size_t size)
{
	assert(shim.alloc_count < MAX_ALLOCS);
	shim.alloc[shim.alloc_count] = calloc(1, size);
	return shim.alloc[shim.alloc_count++];
}

void *dmam_alloc_coherent(struct device *dev, size_t size, dma_addr_t *dma, gfp_t gfp)
{
	(void)dev; (void)gfp;
	*dma = shim.next_coherent;
	shim.next_coherent += 0x200000u;
	return shim_devm_zalloc(size);
}

unsigned long devm_get_free_pages(struct device *dev, gfp_t gfp, unsigned int order)
{
	(void)dev; (void)gfp; (void)order;
	shim.output_cpu = shim_devm_zalloc(FA_OUT_CAP_MIN);
	return (unsigned long)(uintptr_t)shim.output_cpu;
}

int devm_add_action_or_reset(struct device *dev, void (*action)(void *), void *data)
{
	(void)dev;
	shim.action = action;
	shim.action_data = data;
	return 0;
}

dma_addr_t dma_map_single(struct device *dev, void *cpu, size_t size, enum dma_data_direction dir)
{
	(void)dev; (void)cpu; (void)size;
	CHECK(dir == DMA_FROM_DEVICE);
	shim.mapped++;
	return OUTPUT_DMA;
}

void dma_unmap_single(struct device *dev, dma_addr_t dma, size_t size, enum dma_data_direction dir)
{
	(void)dev; (void)dma; (void)size; (void)dir;
	shim.mapped--;
}

int dma_mapping_error(struct device *dev, dma_addr_t dma)
{
	(void)dev; (void)dma;
	return 0;
}

void dma_sync_single_for_device(struct device *dev, dma_addr_t dma, size_t size,
				enum dma_data_direction dir)
{
	(void)dev; (void)size; (void)dir;
	CHECK(dma == OUTPUT_DMA);
	CHECK(!model.running);
	shim.sync_device++;
}

void dma_sync_single_for_cpu(struct device *dev, dma_addr_t dma, size_t size,
			     enum dma_data_direction dir)
{
	(void)dev; (void)dir;
	CHECK(dma == OUTPUT_DMA);
	memcpy(shim.output_cpu, shim.shadow, size);
	shim.sync_cpu++;
}

struct dma_buf *dma_buf_get(int fd)
{
	if (fd < 0)
		return ERR_PTR(-EBADF);
	shim.dbufs[fd % 4].size = shim.import_size;
	shim.dbuf_refs++;
	return &shim.dbufs[fd % 4];
}

void dma_buf_put(struct dma_buf *dbuf)
{
	(void)dbuf;
	shim.dbuf_refs--;
}

struct dma_buf_attachment *dma_buf_attach(struct dma_buf *dbuf, struct device *dev)
{
	(void)dev;
	shim.attach.dmabuf = dbuf;
	shim.attachments++;
	return &shim.attach;
}

void dma_buf_detach(struct dma_buf *dbuf, struct dma_buf_attachment *attach)
{
	(void)dbuf; (void)attach;
	shim.attachments--;
}

struct sg_table *dma_buf_map_attachment(struct dma_buf_attachment *attach,
					enum dma_data_direction dir)
{
	(void)attach;
	CHECK(dir == DMA_TO_DEVICE);
	shim.sg.address = shim.import_dma;
	shim.sg.length = (unsigned int)shim.import_size;
	shim.sgt.sgl = &shim.sg;
	shim.sgt.nents = shim.import_segments;
	shim.mappings++;
	return &shim.sgt;
}

void dma_buf_unmap_attachment(struct dma_buf_attachment *attach, struct sg_table *sgt,
			      enum dma_data_direction dir)
{
	(void)attach; (void)sgt; (void)dir;
	shim.mappings--;
	model.soft_resets_at_unmap = model.soft_resets;
	CHECK(!model.running);
}

void mutex_init(struct mutex *lock) { lock->locked = 0; }

void mutex_lock(struct mutex *lock)
{
	CHECK(!lock->locked);
	lock->locked = 1;
}

void mutex_unlock(struct mutex *lock)
{
	CHECK(lock->locked);
	lock->locked = 0;
}

int mutex_lock_killable(struct mutex *lock)
{
	mutex_lock(lock);
	return 0;
}

void init_completion(struct completion *x) { x->done = 0; }
void reinit_completion(struct completion *x) { x->done = 0; }

void complete(struct completion *x)
{
	x->done = 1;
	shim.completions++;
}

long wait_for_completion_killable_timeout(struct completion *x, unsigned long timeout)
{
	(void)timeout;
	CHECK(timeout > 0);
	switch (model.scenario) {
	case JOB_OK:
	case JOB_BAD_MAGIC:
	case JOB_STALE_FRAME_ID:
		model_finish_job();
		model_fire_irq();
		break;
	case JOB_PL_ERROR:
		model.running = 0;
		*reg(FA_STATUS) = FA_STATUS_IDLE;
		model_reject(0x23);
		model_fire_irq();
		break;
	case JOB_TIMEOUT:
		return 0;
	case JOB_KILLED:
		return -ERESTARTSYS;
	case JOB_IRQ_LOST:
		model_finish_job();
		return 0;
	}
	return x->done ? 1 : 0;
}

int request_firmware(const struct firmware **fw, const char *name, struct device *dev)
{
	static struct firmware image;

	(void)name; (void)dev;
	memset(shim.firmware, 0, sizeof(shim.firmware));
	memcpy(shim.firmware, "FAWB", 4);
	image.size = sizeof(shim.firmware);
	image.data = shim.firmware;
	*fw = &image;
	return 0;
}

void release_firmware(const struct firmware *fw) { (void)fw; }

struct crypto_shash *crypto_alloc_shash(const char *name, u32 type, u32 mask)
{
	static struct crypto_shash tfm;

	(void)name; (void)type; (void)mask;
	return &tfm;
}

void crypto_free_shash(struct crypto_shash *tfm) { (void)tfm; }

int crypto_shash_digest(struct shash_desc *desc, const u8 *data, unsigned int len, u8 *out)
{
	(void)desc; (void)data; (void)len;
	memcpy(out, cam_fa_weight_sha256, 32);
	if (shim.digest_wrong)
		out[0] ^= 1u;
	return 0;
}

int misc_register(struct miscdevice *misc) { (void)misc; return 0; }
void misc_deregister(struct miscdevice *misc) { (void)misc; }

struct clk *devm_clk_get(struct device *dev, const char *id)
{
	static struct clk clk;

	(void)dev; (void)id;
	return &clk;
}

void __iomem *devm_platform_ioremap_resource(struct platform_device *pdev, unsigned int index)
{
	(void)pdev; (void)index;
	return shim_regs;
}

int devm_request_irq(struct device *dev, unsigned int irq, irqreturn_t (*handler)(int, void *),
		     unsigned long flags, const char *name, void *data)
{
	(void)dev; (void)irq; (void)flags; (void)name;
	shim.handler = handler;
	shim.handler_data = data;
	return 0;
}

/* ── fixture ── */
static struct platform_device g_pdev;

static void boot(u32 revision, u32 features)
{
	int ret;

	memset(&model, 0, sizeof(model));
	memset(&shim, 0, sizeof(shim));
	shim_dev_errors = 0;
	shim.next_coherent = COHERENT_BASE;
	shim.import_segments = 1;
	shim.import_dma = IMPORT_DMA;
	shim.import_size = FACE_ACCEL_FRAME_SIZE;
	model.revision = revision;
	model.features = features;
	*reg(FA_IP_ID) = FACE_ACCEL_IP_ID_MAGIC;
	*reg(FA_IN_W_REG) = FACE_ACCEL_IN_W;
	*reg(FA_IN_H_REG) = FACE_ACCEL_IN_H;
	*reg(FA_STATUS) = FA_STATUS_IDLE;
	*reg(FA_DEBUG_DMA) = FA_DEBUG_DMA_ARESETN;
	memset(&g_pdev, 0, sizeof(g_pdev));
	ret = cam_fa_driver.probe(&g_pdev);
	CHECK(ret == 0);
}

static void shutdown(void)
{
	int i;

	cam_fa_driver.remove(&g_pdev);
	if (shim.action)
		shim.action(shim.action_data);
	CHECK(shim.mapped == 0);
	for (i = 0; i < shim.alloc_count; i++)
		free(shim.alloc[i]);
}

static struct file *open_file(void)
{
	static struct file files[4];
	static int next;
	struct cam_fa_dev *fa = platform_get_drvdata(&g_pdev);
	struct file *file = &files[next++ % 4];

	file->private_data = &fa->misc;
	CHECK(cam_fa_fops.open(NULL, file) == 0);
	return file;
}

static long do_ioctl(struct file *file, unsigned int cmd, void *arg)
{
	return cam_fa_fops.unlocked_ioctl(file, cmd, (unsigned long)(uintptr_t)arg);
}

static long import_fd(struct file *file, int fd, u32 *handle)
{
	struct cam_face_accel_import req = { .fd = fd };
	long ret = do_ioctl(file, CAM_FACE_ACCEL_IOC_IMPORT, &req);

	if (handle)
		*handle = req.handle;
	return ret;
}

static long submit(struct file *file, u32 handle, u32 frame_id)
{
	struct cam_face_accel_submit sub = { .input_handle = handle, .frame_id = frame_id };

	return do_ioctl(file, CAM_FACE_ACCEL_IOC_SUBMIT, &sub);
}

static u8 g_output[CAM_FACE_ACCEL_OUTPUT_CAPACITY];
static u8 g_input[CAM_FACE_ACCEL_INPUT_SIZE];

static long wait_job(struct file *file, struct cam_face_accel_result *result)
{
	struct cam_face_accel_wait wait = {
		.output_ptr = (uintptr_t)g_output,
		.output_capacity = sizeof(g_output),
	};
	long ret = do_ioctl(file, CAM_FACE_ACCEL_IOC_WAIT, &wait);

	if (result)
		*result = wait.result;
	return ret;
}

static long run_job(struct file *file, u32 frame_id, struct cam_face_accel_result *result)
{
	struct cam_face_accel_run run = {
		.input_ptr = (uintptr_t)g_input,
		.output_ptr = (uintptr_t)g_output,
		.input_size = sizeof(g_input),
		.output_capacity = sizeof(g_output),
		.frame_id = frame_id,
	};
	long ret = do_ioctl(file, CAM_FACE_ACCEL_IOC_RUN, &run);

	if (result)
		*result = run.result;
	return ret;
}

static void close_file(struct file *file)
{
	CHECK(cam_fa_fops.release(NULL, file) == 0);
}

#define R10_FEATURES (FA_FEATURE_PRE_BUF | FA_FEATURE_DRAIN_RESET | FA_FEATURE_WDT_DATA_PROGRESS)

/* ── tests ── */
static void test_identity(void)
{
	struct cam_face_accel_info info;
	struct cam_face_accel_caps caps;
	struct file *file;

	boot(10, R10_FEATURES);
	file = open_file();
	CHECK(do_ioctl(file, CAM_FACE_ACCEL_IOC_GET_INFO, &info) == 0);
	CHECK(info.uapi_version == 1);    /* a version 1 client compares for equality */
	CHECK(info.flags & CAM_FACE_ACCEL_INFO_WEIGHT_READY);
	CHECK(do_ioctl(file, CAM_FACE_ACCEL_IOC_GET_CAPS, &caps) == 0);
	CHECK(caps.uapi_version == 2 && caps.hw_revision == 10 && caps.hw_features == R10_FEATURES);
	CHECK(caps.flags & CAM_FACE_ACCEL_CAP_DMABUF_INPUT);
	CHECK(caps.queue_depth == 1 && caps.max_imports == 8);
	CHECK(memcmp(caps.weight_sha256, cam_fa_weight_sha256, 32) == 0);
	CHECK(do_ioctl(file, 0x7A00u, &caps) == -ENOTTY);
	close_file(file);
	shutdown();

	boot(0, 0);
	file = open_file();
	CHECK(do_ioctl(file, CAM_FACE_ACCEL_IOC_GET_CAPS, &caps) == 0);
	CHECK(caps.hw_revision == 0 && !(caps.flags & CAM_FACE_ACCEL_CAP_DMABUF_INPUT));
	close_file(file);
	shutdown();
}

static void test_v1_run(void)
{
	struct cam_face_accel_result result;
	struct cam_fa_dev *fa;
	struct file *file;

	/* R9: no preprocess buffer; the model fails the test if the driver writes 0x88 or 0x8C. */
	boot(0, 0);
	fa = platform_get_drvdata(&g_pdev);
	file = open_file();
	CHECK(run_job(file, 7, &result) == 0);
	CHECK(model.starts == 1 && model.last_input == fa->input.dma && model.last_pre == 0);
	CHECK(model.pre_buf_writes == 0);
	CHECK(result.frame_id_done == 7 && result.output_size == FA_OUT_LEN_V1);
	CHECK(((fa_out_header_t *)g_output)->frame_id == 7);
	CHECK(shim.sync_device == 1 && shim.sync_cpu == 1);
	close_file(file);
	shutdown();

	/* R10: the same path, reading through the preprocess buffer. */
	boot(10, R10_FEATURES);
	fa = platform_get_drvdata(&g_pdev);
	file = open_file();
	CHECK(run_job(file, 8, &result) == 0);
	CHECK(model.last_pre == fa->pre.dma && model.pre_buf_writes == 2);
	close_file(file);
	shutdown();
}

static void test_import_validation(void)
{
	struct file *file;
	u32 handle = 0;
	int i;

	boot(0, 0);
	file = open_file();
	CHECK(import_fd(file, 3, &handle) == -EOPNOTSUPP);
	CHECK(shim.dbuf_refs == 0);
	close_file(file);
	shutdown();

	boot(10, R10_FEATURES);
	file = open_file();
	CHECK(import_fd(file, -1, NULL) == -EBADF);

	shim.import_segments = 2;
	CHECK(import_fd(file, 3, NULL) == -EINVAL);
	shim.import_segments = 1;
	shim.import_size = FACE_ACCEL_FRAME_SIZE - 1;
	CHECK(import_fd(file, 3, NULL) == -EINVAL);
	shim.import_size = FACE_ACCEL_FRAME_SIZE;
	shim.import_dma = IMPORT_DMA + 8;
	CHECK(import_fd(file, 3, NULL) == -EINVAL);
	shim.import_dma = FACE_ACCEL_BUF_MIN_ADDR - 64;
	CHECK(import_fd(file, 3, NULL) == -EINVAL);
	shim.import_dma = 0xFFFFFFC0u;
	CHECK(import_fd(file, 3, NULL) == -EINVAL);
	CHECK(shim.dbuf_refs == 0 && shim.attachments == 0 && shim.mappings == 0);
	shim.import_dma = IMPORT_DMA;

	{
		struct cam_face_accel_import req = { .fd = 3, .reserved = { 1, 0 } };

		CHECK(do_ioctl(file, CAM_FACE_ACCEL_IOC_IMPORT, &req) == -EINVAL);
	}
	for (i = 0; i < 8; i++) {
		CHECK(import_fd(file, 3, &handle) == 0);
		CHECK(handle == (u32)i + 1);
	}
	CHECK(import_fd(file, 3, NULL) == -ENOSPC);
	CHECK(shim.dbuf_refs == 8);
	close_file(file);
	CHECK(shim.dbuf_refs == 0 && shim.attachments == 0 && shim.mappings == 0);
	shutdown();
}

static void test_submit_wait(void)
{
	struct cam_face_accel_result result;
	struct cam_fa_dev *fa;
	struct file *file;
	struct file *other;
	u32 handle;

	boot(10, R10_FEATURES);
	fa = platform_get_drvdata(&g_pdev);
	file = open_file();
	other = open_file();
	CHECK(import_fd(file, 1, &handle) == 0);

	CHECK(wait_job(file, NULL) == -ENODATA);        /* nothing submitted */
	CHECK(submit(file, 0, 1) == -EINVAL);
	CHECK(submit(file, 9, 1) == -EINVAL);
	CHECK(submit(file, 2, 1) == -EINVAL);           /* a free slot */
	CHECK(submit(other, handle, 1) == -EINVAL);     /* handles belong to the file */

	CHECK(submit(file, handle, 42) == 0);
	CHECK(model.starts == 1 && model.running);
	CHECK(model.last_input == IMPORT_DMA && model.last_pre == fa->pre.dma);
	CHECK(submit(file, handle, 43) == -EBUSY);
	CHECK(run_job(other, 44, NULL) == -EBUSY);
	CHECK(do_ioctl(other, CAM_FACE_ACCEL_IOC_RESET, NULL) == -EBUSY);
	CHECK(wait_job(other, NULL) == -ENODATA);
	CHECK(model.starts == 1 && model.rejected == 0);

	CHECK(wait_job(file, &result) == 0);
	CHECK(result.frame_id_done == 42 && result.output_size == FA_OUT_LEN_V1);
	CHECK(((fa_out_header_t *)g_output)->frame_id == 42);
	CHECK(fa->job_owner == NULL);
	CHECK(wait_job(file, NULL) == -ENODATA);        /* taken once */

	CHECK(submit(file, handle, 45) == 0);
	CHECK(wait_job(file, NULL) == 0);
	CHECK(model.soft_resets == 0 && shim_dev_errors == 0);
	close_file(other);
	close_file(file);
	shutdown();
}

/* After a failed job the accelerator is usable again and no resource is held by the job. */
static void check_failure(enum scenario scenario, long expected)
{
	struct cam_face_accel_result result;
	struct cam_fa_dev *fa;
	struct file *file;
	u32 handle;

	boot(10, R10_FEATURES);
	fa = platform_get_drvdata(&g_pdev);
	file = open_file();
	CHECK(import_fd(file, 1, &handle) == 0);
	CHECK(submit(file, handle, 50) == 0);
	model.scenario = scenario;
	CHECK(wait_job(file, &result) == expected);
	CHECK(model.soft_resets == 1 && !model.running);
	CHECK(fa->job_owner == NULL);
	CHECK(*reg(FA_IRQ_ENABLE) == 0);

	model.scenario = JOB_OK;
	CHECK(submit(file, handle, 51) == 0);
	CHECK(wait_job(file, &result) == 0);
	CHECK(result.frame_id_done == 51);

	/* the same failure through RUN */
	model.scenario = scenario;
	model.soft_resets = 0;
	CHECK(run_job(file, 52, NULL) == expected);
	CHECK(model.soft_resets == 1 && !model.running);
	close_file(file);
	shutdown();
}

static void test_failures(void)
{
	check_failure(JOB_TIMEOUT, -ETIMEDOUT);
	check_failure(JOB_IRQ_LOST, -ETIMEDOUT);
	check_failure(JOB_PL_ERROR, -EIO);
	check_failure(JOB_BAD_MAGIC, -EBADMSG);
	check_failure(JOB_STALE_FRAME_ID, -EBADMSG);
	check_failure(JOB_KILLED, -ERESTARTSYS);
}

static void test_failure_reports_the_error(void)
{
	struct cam_face_accel_result result;
	struct file *file;

	boot(10, R10_FEATURES);
	file = open_file();
	model.scenario = JOB_PL_ERROR;
	CHECK(run_job(file, 60, &result) == -EIO);
	CHECK(result.error_code == 0x23 && (result.status & FA_STATUS_ERROR));
	close_file(file);
	shutdown();
}

static void test_close_with_job_running(void)
{
	struct file *file;
	u32 handle;

	boot(10, R10_FEATURES);
	file = open_file();
	CHECK(import_fd(file, 1, &handle) == 0);
	CHECK(submit(file, handle, 70) == 0);
	CHECK(model.running);
	close_file(file);
	/* dma_buf_unmap_attachment checks the model has stopped */
	CHECK(model.soft_resets == 1 && model.soft_resets_at_unmap == 1);
	CHECK(shim.dbuf_refs == 0 && shim.mappings == 0);

	/* the next user starts clean */
	file = open_file();
	CHECK(run_job(file, 71, NULL) == 0);
	close_file(file);
	shutdown();
}

static void test_reset(void)
{
	struct cam_fa_dev *fa;
	struct file *file;
	u32 handle;

	boot(10, R10_FEATURES);
	fa = platform_get_drvdata(&g_pdev);
	file = open_file();
	CHECK(import_fd(file, 1, &handle) == 0);
	CHECK(submit(file, handle, 80) == 0);
	CHECK(do_ioctl(file, CAM_FACE_ACCEL_IOC_RESET, NULL) == 0);
	CHECK(fa->job_owner == NULL && !model.running);

	model.reset_info = FA_RESET_INFO_UNCLEAN | (0x100000u << FA_RESET_INFO_DRAIN_CYCLES_SHIFT);
	CHECK(do_ioctl(file, CAM_FACE_ACCEL_IOC_RESET, NULL) == -EIO);
	CHECK(shim_dev_errors == 1);
	close_file(file);
	shutdown();

	/* R9 has no RESET_INFO; a stale read there must not fail the reset */
	boot(0, 0);
	model.reset_info = FA_RESET_INFO_UNCLEAN;
	file = open_file();
	CHECK(do_ioctl(file, CAM_FACE_ACCEL_IOC_RESET, NULL) == 0);
	close_file(file);
	shutdown();
}

static void test_irq_source(void)
{
	struct cam_fa_dev *fa;

	boot(10, R10_FEATURES);
	fa = platform_get_drvdata(&g_pdev);

	/* DONE is sticky; alone it does not mean this interrupt is ours */
	*reg(FA_STATUS) = FA_STATUS_IDLE | FA_STATUS_DONE;
	*reg(FA_IRQ_STATUS) = 0;
	CHECK(cam_fa_irq(55, fa) == IRQ_NONE);
	CHECK(shim.completions == 0);

	*reg(FA_IRQ_STATUS) = FA_IRQ_DONE;
	CHECK(cam_fa_irq(55, fa) == IRQ_HANDLED);
	CHECK(shim.completions == 1 && *reg(FA_IRQ_STATUS) == 0);
	shutdown();
}

static void test_bad_arguments(void)
{
	struct cam_face_accel_submit sub = { .input_handle = 1, .frame_id = 1, .timeout_ms = 2001 };
	struct cam_face_accel_wait wait = { .output_ptr = (uintptr_t)g_output, .output_capacity = 16 };
	struct cam_face_accel_run run = {
		.input_ptr = (uintptr_t)g_input,
		.output_ptr = (uintptr_t)g_output,
		.input_size = sizeof(g_input),
		.output_capacity = sizeof(g_output),
		.timeout_ms = 2001,
	};
	struct file *file;
	u32 handle;

	boot(10, R10_FEATURES);
	file = open_file();
	CHECK(import_fd(file, 1, &handle) == 0);
	CHECK(do_ioctl(file, CAM_FACE_ACCEL_IOC_SUBMIT, &sub) == -EINVAL);
	sub.timeout_ms = 0;
	sub.reserved = 1;
	CHECK(do_ioctl(file, CAM_FACE_ACCEL_IOC_SUBMIT, &sub) == -EINVAL);
	CHECK(do_ioctl(file, CAM_FACE_ACCEL_IOC_RUN, &run) == -EINVAL);
	CHECK(model.starts == 0);

	/* a bad WAIT leaves the job to a good one */
	CHECK(submit(file, handle, 90) == 0);
	CHECK(do_ioctl(file, CAM_FACE_ACCEL_IOC_WAIT, &wait) == -EINVAL);
	CHECK(wait_job(file, NULL) == 0);
	close_file(file);
	shutdown();
}

static void test_probe_rejects_wrong_weights(void)
{
	boot(0, 0);
	shutdown();
	memset(&shim, 0, sizeof(shim));
	shim.next_coherent = COHERENT_BASE;
	shim.digest_wrong = 1;
	memset(&g_pdev, 0, sizeof(g_pdev));
	CHECK(cam_fa_driver.probe(&g_pdev) == -EBADMSG);
	if (shim.action)
		shim.action(shim.action_data);
	for (int i = 0; i < shim.alloc_count; i++)
		free(shim.alloc[i]);
}

int main(void)
{
	test_identity();
	test_v1_run();
	test_import_validation();
	test_submit_wait();
	test_failures();
	test_failure_reports_the_error();
	test_close_with_job_running();
	test_reset();
	test_irq_source();
	test_bad_arguments();
	test_probe_rejects_wrong_weights();

	if (g_failures) {
		fprintf(stderr, "face_accel_driver_test: %d failure(s)\n", g_failures);
		return 1;
	}
	puts("face_accel_driver_test: OK");
	return 0;
}
