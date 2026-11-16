/* kshim_v4l2.h - just enough of the kernel API to build cam-ov5640-control.c as a host program.
 *
 * tests/ov5640_driver_test.c includes the driver source and provides what is declared here
 * without a body: I2C, the clock and the GPIO lines go to its model of the sensor.
 * scripts/build/test_ov5640_driver.sh generates empty files for the driver's kernel includes, so
 * this header is the whole kernel the driver sees.  The V4L2 framework is reduced to what the
 * driver touches; the test calls the driver's operations the way the framework would.
 */
#ifndef KSHIM_V4L2_H
#define KSHIM_V4L2_H

#include <errno.h>
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

#define S32_MAX INT32_MAX
#define ARRAY_SIZE(a) (sizeof(a) / sizeof((a)[0]))
#define container_of(ptr, type, member) ((type *)(void *)((char *)(ptr) - offsetof(type, member)))
#define MODULE_DESCRIPTION(x)
#define MODULE_AUTHOR(x)
#define MODULE_LICENSE(x)
#define MODULE_DEVICE_TABLE(type, name)
#define module_i2c_driver(drv)

#define MAX_ERRNO 4095
#define IS_ERR(p) ((uintptr_t)(p) >= (uintptr_t)-MAX_ERRNO)
#define PTR_ERR(p) ((long)(intptr_t)(p))

extern int shim_warnings;
#define WARN_ON(cond) ({ int __c = !!(cond); if (__c) shim_warnings++; __c; })

struct device { int unused; };

extern int shim_dev_errors;
#define dev_err(dev, ...) do { (void)(dev); shim_dev_errors++; } while (0)
#define dev_err_ratelimited(dev, ...) dev_err(dev, __VA_ARGS__)
#define dev_info(dev, ...) do { (void)(dev); } while (0)

/* ── time: sleeps cost nothing; jiffies moves only when the test moves it ── */
extern unsigned long jiffies;
#define time_after(a, b) ((long)((b) - (a)) < 0)
static inline unsigned long msecs_to_jiffies(unsigned int ms) { return ms; }
static inline void usleep_range(unsigned long min, unsigned long max) { (void)min; (void)max; }
static inline void msleep(unsigned int ms) { (void)ms; }

/* ── locking: the test is single-threaded; it checks that the driver pairs its calls ── */
struct mutex { int held; int initialized; };
static inline void mutex_init(struct mutex *m) { m->held = 0; m->initialized = 1; }
static inline void mutex_destroy(struct mutex *m) { m->initialized = 0; }
void mutex_lock(struct mutex *m);
void mutex_unlock(struct mutex *m);

void *devm_kzalloc(struct device *dev, size_t size, unsigned int flags);
#define GFP_KERNEL 1u

/* ── the sensor's clock, lines and bus: the test's model ── */
struct clk { int enabled; int fail_enable; };
struct clk *devm_clk_get_optional(struct device *dev, const char *id);
int clk_prepare_enable(struct clk *clk);
void clk_disable_unprepare(struct clk *clk);

enum gpiod_flags { GPIOD_OUT_LOW = 2, GPIOD_OUT_HIGH = 3 };
struct gpio_desc { int value; };
struct gpio_desc *devm_gpiod_get_optional(struct device *dev, const char *con_id,
                                          enum gpiod_flags flags);
void gpiod_set_value_cansleep(struct gpio_desc *desc, int value);

#define I2C_M_RD 0x0001
struct i2c_adapter { int unused; };
struct i2c_client {
    struct device dev;
    unsigned short addr;
    struct i2c_adapter *adapter;
    void *clientdata;
};
struct i2c_msg { u16 addr; u16 flags; u16 len; u8 *buf; };
int i2c_master_send(const struct i2c_client *client, const char *buf, int count);
int i2c_transfer(struct i2c_adapter *adap, struct i2c_msg *msgs, int num);
static inline void *i2c_get_clientdata(const struct i2c_client *client) { return client->clientdata; }

struct of_device_id { char compatible[128]; };
struct i2c_device_id { char name[32]; unsigned long driver_data; };
struct device_driver {
    const char *name;
    bool suppress_bind_attrs;
    const struct of_device_id *of_match_table;
};
struct i2c_driver {
    struct device_driver driver;
    const struct i2c_device_id *id_table;
    int (*probe_new)(struct i2c_client *client);
    int (*remove)(struct i2c_client *client);
};

/* ── media bus and media controller ── */
#define MEDIA_BUS_FMT_RGB565_2X8_LE 0x1008
#define V4L2_FIELD_NONE 1
#define V4L2_COLORSPACE_SRGB 8
struct v4l2_mbus_framefmt { u32 width, height, code, field, colorspace; };

#define MEDIA_PAD_FL_SOURCE 2
#define MEDIA_ENT_F_CAM_SENSOR 0x20001
struct media_pad { unsigned long flags; };
struct media_entity_operations { int (*link_validate)(void *link); };
struct media_entity { u32 function; const struct media_entity_operations *ops; };
int media_entity_pads_init(struct media_entity *entity, u16 num_pads, struct media_pad *pads);
void media_entity_cleanup(struct media_entity *entity);
int v4l2_subdev_link_validate(void *link);

/* ── controls ── */
#define V4L2_CID_USER_BASE 0x00980900
#define V4L2_CTRL_TYPE_INTEGER 1
#define V4L2_CTRL_TYPE_BOOLEAN 2
#define V4L2_CTRL_TYPE_BUTTON 4
#define V4L2_CTRL_FLAG_READ_ONLY 0x0004
#define V4L2_CTRL_FLAG_VOLATILE 0x0080
#define V4L2_CTRL_FLAG_EXECUTE_ON_WRITE 0x0200

struct v4l2_ctrl;
struct v4l2_ctrl_ops {
    int (*g_volatile_ctrl)(struct v4l2_ctrl *ctrl);
    int (*s_ctrl)(struct v4l2_ctrl *ctrl);
};
struct v4l2_ctrl_config {
    const struct v4l2_ctrl_ops *ops;
    u32 id;
    const char *name;
    int type;
    s32 min, max;
    u32 step;
    s32 def;
    u32 flags;
};
struct v4l2_ctrl_handler;
struct v4l2_ctrl {
    struct v4l2_ctrl_handler *handler;
    const struct v4l2_ctrl_ops *ops;
    u32 id;
    s32 val;
};
#define SHIM_MAX_CTRLS 16
struct v4l2_ctrl_handler {
    struct mutex *lock;
    int error;
    unsigned int count;
    struct v4l2_ctrl ctrls[SHIM_MAX_CTRLS];
};
int v4l2_ctrl_handler_init(struct v4l2_ctrl_handler *hdl, unsigned int nr_of_controls_hint);
struct v4l2_ctrl *v4l2_ctrl_new_custom(struct v4l2_ctrl_handler *hdl,
                                       const struct v4l2_ctrl_config *cfg, void *priv);
void v4l2_ctrl_handler_free(struct v4l2_ctrl_handler *hdl);

/* ── subdev ── */
#define V4L2_SUBDEV_FL_HAS_DEVNODE (1U << 2)
#define V4L2_SUBDEV_FORMAT_TRY 0
#define V4L2_SUBDEV_FORMAT_ACTIVE 1

struct v4l2_subdev;
struct v4l2_subdev_state { struct v4l2_mbus_framefmt try_fmt; };
struct v4l2_subdev_fh { struct v4l2_subdev_state *state; };
struct v4l2_subdev_mbus_code_enum { u32 pad, index, code; };
struct v4l2_subdev_format { u32 which, pad; struct v4l2_mbus_framefmt format; };

struct v4l2_subdev_video_ops { int (*s_stream)(struct v4l2_subdev *sd, int enable); };
struct v4l2_subdev_pad_ops {
    int (*init_cfg)(struct v4l2_subdev *sd, struct v4l2_subdev_state *state);
    int (*enum_mbus_code)(struct v4l2_subdev *sd, struct v4l2_subdev_state *state,
                          struct v4l2_subdev_mbus_code_enum *code);
    int (*get_fmt)(struct v4l2_subdev *sd, struct v4l2_subdev_state *state,
                   struct v4l2_subdev_format *fmt);
    int (*set_fmt)(struct v4l2_subdev *sd, struct v4l2_subdev_state *state,
                   struct v4l2_subdev_format *fmt);
};
struct v4l2_subdev_ops {
    const struct v4l2_subdev_video_ops *video;
    const struct v4l2_subdev_pad_ops *pad;
};
struct v4l2_subdev_internal_ops {
    int (*open)(struct v4l2_subdev *sd, struct v4l2_subdev_fh *fh);
    int (*close)(struct v4l2_subdev *sd, struct v4l2_subdev_fh *fh);
};
struct v4l2_subdev {
    struct media_entity entity;
    u32 flags;
    const struct v4l2_subdev_ops *ops;
    const struct v4l2_subdev_internal_ops *internal_ops;
    struct v4l2_ctrl_handler *ctrl_handler;
};

static inline struct v4l2_mbus_framefmt *
v4l2_subdev_get_try_format(struct v4l2_subdev *sd, struct v4l2_subdev_state *state, unsigned int pad)
{
    (void)sd;
    (void)pad;
    return &state->try_fmt;
}

void v4l2_i2c_subdev_init(struct v4l2_subdev *sd, struct i2c_client *client,
                          const struct v4l2_subdev_ops *ops);
int v4l2_async_register_subdev_sensor(struct v4l2_subdev *sd);
void v4l2_async_unregister_subdev(struct v4l2_subdev *sd);

#endif /* KSHIM_V4L2_H */
