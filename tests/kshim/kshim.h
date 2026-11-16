/* kshim.h - just enough of the kernel API to build cam-face-accel.c as a host program.
 *
 * tests/face_accel_driver_test.c includes the driver source and provides what is declared here
 * without a body: register access goes to its model of the accelerator.  scripts/build/
 * test_face_accel_driver.sh generates empty files for the driver's kernel includes, so this
 * header is the whole kernel the driver sees.
 */
#ifndef KSHIM_H
#define KSHIM_H

#include <errno.h>
#include <limits.h>
#include <stdarg.h>
#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

typedef uint8_t u8;
typedef uint16_t u16;
typedef uint32_t u32;
typedef int32_t s32;
typedef uint64_t u64;
typedef uint32_t dma_addr_t;
typedef unsigned int gfp_t;
typedef int irqreturn_t;

#define __iomem
#define __user
#define IRQ_NONE 0
#define IRQ_HANDLED 1
#define GFP_KERNEL 1u
#define __GFP_ZERO 2u
#define ERESTARTSYS 512
#define PAGE_SIZE 4096u
#define BIT_ULL(n) (1ULL << (n))
#define IS_ALIGNED(x, a) (((x) & ((a) - 1)) == 0)
#define ARRAY_SIZE(a) (sizeof(a) / sizeof((a)[0]))
#define lower_32_bits(n) ((u32)(n))
#define container_of(ptr, type, member) ((type *)(void *)((char *)(ptr) - offsetof(type, member)))
#define THIS_MODULE NULL
#define no_llseek NULL
#define compat_ptr_ioctl NULL
#define dma_wmb() do { } while (0)
#define MODULE_DESCRIPTION(x)
#define MODULE_AUTHOR(x)
#define MODULE_LICENSE(x)
#define MODULE_VERSION(x)
#define MODULE_DEVICE_TABLE(type, name)
#define module_platform_driver(drv)

#define MAX_ERRNO 4095
#define IS_ERR(p) ((uintptr_t)(p) >= (uintptr_t)-MAX_ERRNO)
#define PTR_ERR(p) ((long)(intptr_t)(p))
#define ERR_PTR(e) ((void *)(intptr_t)(e))
#define u64_to_user_ptr(v) ((void *)(uintptr_t)(v))
#define copy_from_user(to, from, n) (memcpy((to), (from), (n)), 0UL)
#define copy_to_user(to, from, n) (memcpy((to), (from), (n)), 0UL)

extern int shim_dev_errors;
#define dev_err(dev, ...) do { (void)(dev); shim_dev_errors++; } while (0)
#define dev_err_ratelimited(dev, ...) dev_err(dev, __VA_ARGS__)
#define dev_info(dev, ...) do { (void)(dev); } while (0)
#define dev_err_probe(dev, err, ...) (err)
#define dev_name(dev) "face-accel"

struct device { int unused; };

/* ── time ── */
extern unsigned long jiffies;
static inline unsigned long msecs_to_jiffies(unsigned int ms) { return ms; }

/* ── registers: the test's accelerator model ── */
u32 model_read(u32 offset);
void model_write(u32 offset, u32 value);
extern char shim_regs[256];
#define ioread32(addr) model_read((u32)((char *)(addr) - shim_regs))
#define iowrite32(val, addr) model_write((u32)((char *)(addr) - shim_regs), (val))
#define readl_poll_timeout(addr, val, cond, sleep_us, timeout_us) \
	({ (void)(sleep_us); (void)(timeout_us); (val) = ioread32(addr); (cond) ? 0 : -ETIMEDOUT; })

/* ── allocation ── */
#define kzalloc(size, gfp) calloc(1, (size))
#define kmalloc(size, gfp) malloc(size)
#define kfree(p) free(p)
#define devm_kzalloc(dev, size, gfp) shim_devm_zalloc(size)
void *shim_devm_zalloc(size_t size);
void *dmam_alloc_coherent(struct device *dev, size_t size, dma_addr_t *dma, gfp_t gfp);
unsigned long devm_get_free_pages(struct device *dev, gfp_t gfp, unsigned int order);
#define get_order(size) 3
int devm_add_action_or_reset(struct device *dev, void (*action)(void *), void *data);

/* ── streaming DMA ── */
enum dma_data_direction { DMA_BIDIRECTIONAL, DMA_TO_DEVICE, DMA_FROM_DEVICE };
dma_addr_t dma_map_single(struct device *dev, void *cpu, size_t size, enum dma_data_direction dir);
void dma_unmap_single(struct device *dev, dma_addr_t dma, size_t size, enum dma_data_direction dir);
int dma_mapping_error(struct device *dev, dma_addr_t dma);
void dma_sync_single_for_device(struct device *dev, dma_addr_t dma, size_t size,
				enum dma_data_direction dir);
void dma_sync_single_for_cpu(struct device *dev, dma_addr_t dma, size_t size,
			     enum dma_data_direction dir);
#define DMA_BIT_MASK(n) (((n) == 64) ? ~0ULL : ((1ULL << (n)) - 1))
#define dma_set_mask_and_coherent(dev, mask) 0

/* ── dma-buf ── */
struct scatterlist { dma_addr_t address; unsigned int length; };
struct sg_table { struct scatterlist *sgl; unsigned int nents; };
#define sg_dma_address(sg) ((sg)->address)
#define sg_dma_len(sg) ((sg)->length)
struct dma_buf { size_t size; };
struct dma_buf_attachment { struct dma_buf *dmabuf; };
struct dma_buf *dma_buf_get(int fd);
void dma_buf_put(struct dma_buf *dbuf);
struct dma_buf_attachment *dma_buf_attach(struct dma_buf *dbuf, struct device *dev);
void dma_buf_detach(struct dma_buf *dbuf, struct dma_buf_attachment *attach);
struct sg_table *dma_buf_map_attachment(struct dma_buf_attachment *attach,
					enum dma_data_direction dir);
void dma_buf_unmap_attachment(struct dma_buf_attachment *attach, struct sg_table *sgt,
			      enum dma_data_direction dir);

/* ── locking and completion ── */
struct mutex { int locked; };
void mutex_init(struct mutex *lock);
void mutex_lock(struct mutex *lock);
void mutex_unlock(struct mutex *lock);
int mutex_lock_killable(struct mutex *lock);
struct completion { int done; };
void init_completion(struct completion *x);
void reinit_completion(struct completion *x);
void complete(struct completion *x);
long wait_for_completion_killable_timeout(struct completion *x, unsigned long timeout);

/* ── firmware and hash ── */
struct firmware { size_t size; const u8 *data; };
int request_firmware(const struct firmware **fw, const char *name, struct device *dev);
void release_firmware(const struct firmware *fw);
static inline u32 get_unaligned_le32(const void *p) { u32 v; memcpy(&v, p, sizeof(v)); return v; }
struct crypto_shash { int unused; };
struct shash_desc { struct crypto_shash *tfm; };
struct crypto_shash *crypto_alloc_shash(const char *name, u32 type, u32 mask);
void crypto_free_shash(struct crypto_shash *tfm);
#define crypto_shash_descsize(tfm) 0u
int crypto_shash_digest(struct shash_desc *desc, const u8 *data, unsigned int len, u8 *out);
#define crypto_memneq(a, b, n) memcmp((a), (b), (n))

/* ── character and platform device ── */
struct inode { int unused; };
struct file { void *private_data; };
struct file_operations {
	void *owner;
	int (*open)(struct inode *inode, struct file *file);
	int (*release)(struct inode *inode, struct file *file);
	long (*unlocked_ioctl)(struct file *file, unsigned int cmd, unsigned long arg);
	long (*compat_ioctl)(struct file *file, unsigned int cmd, unsigned long arg);
	void *llseek;
};
#define MISC_DYNAMIC_MINOR 255
struct miscdevice {
	int minor;
	const char *name;
	const struct file_operations *fops;
	struct device *parent;
};
int misc_register(struct miscdevice *misc);
void misc_deregister(struct miscdevice *misc);

struct clk { int unused; };
struct clk *devm_clk_get(struct device *dev, const char *id);
#define clk_prepare_enable(clk) 0
#define clk_disable_unprepare(clk) do { } while (0)

struct platform_device { struct device dev; void *drvdata; };
void __iomem *devm_platform_ioremap_resource(struct platform_device *pdev, unsigned int index);
#define platform_get_irq(pdev, index) 55
#define platform_set_drvdata(pdev, data) ((pdev)->drvdata = (data))
#define platform_get_drvdata(pdev) ((pdev)->drvdata)
int devm_request_irq(struct device *dev, unsigned int irq, irqreturn_t (*handler)(int, void *),
		     unsigned long flags, const char *name, void *data);
#define devm_free_irq(dev, irq, data) do { } while (0)

struct of_device_id { char compatible[64]; };
struct device_driver {
	const char *name;
	const struct of_device_id *of_match_table;
	bool suppress_bind_attrs;
};
struct platform_driver {
	int (*probe)(struct platform_device *pdev);
	int (*remove)(struct platform_device *pdev);
	struct device_driver driver;
};

#endif
