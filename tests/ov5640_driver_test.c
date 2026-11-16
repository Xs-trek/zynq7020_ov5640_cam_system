/* ov5640_driver_test.c - cam-ov5640-control.c against a model of the sensor's power and bus.
 *
 * The driver source is compiled in (tests/kshim/kshim_v4l2.h is the kernel it sees).  The model
 * answers I2C only while the sensor is powered: clock on, power-down and reset released.  The test
 * drives the driver the way the V4L2 framework does: file handle open and close through the
 * internal operations, controls with the handler lock held, and s_stream from the capture
 * pipeline.  What it checks is when the sensor has power, above all after the process holding
 * it ends without cleaning up.
 */
#include "cam-ov5640-control.c"

int shim_dev_errors;
int shim_warnings;
unsigned long jiffies = 1000;

static int g_checks;

static void require(int cond, const char *what)
{
    g_checks++;
    if (!cond) {
        fprintf(stderr, "ov5640_driver_test: FAIL: %s\n", what);
        exit(1);
    }
}

/* ── sensor model ── */

static struct {
    struct clk clk;
    struct gpio_desc pwdn;      /* logical values: 1 = asserted */
    struct gpio_desc reset;
    struct gpio_desc *lines[2];
    unsigned int lines_given;
    u16 chip_id;
    u8 regs[0x10000];
    unsigned int writes;        /* answered */
    unsigned int dead_writes;   /* sent while the sensor had no power */
    unsigned int reads;
    int fail_i2c;               /* the bus fails while set */
    u16 last_reg;
    u8 last_val;
    unsigned int locks_held;
} m;

static int model_powered(void)
{
    return m.clk.enabled && !m.pwdn.value && !m.reset.value;
}

/* What a sensor that lost its power has forgotten. */
static void model_forget(void)
{
    if (!model_powered())
        memset(m.regs, 0, sizeof(m.regs));
}

void mutex_lock(struct mutex *mx)
{
    require(mx->initialized && !mx->held, "the lock is free when taken");
    mx->held = 1;
    m.locks_held++;
}

void mutex_unlock(struct mutex *mx)
{
    require(mx->held, "the lock is held when released");
    mx->held = 0;
    m.locks_held--;
}

static unsigned char g_heap[sizeof(struct cam_ov5640) + 64];

void *devm_kzalloc(struct device *dev, size_t size, unsigned int flags)
{
    (void)dev;
    (void)flags;
    require(size <= sizeof(g_heap), "one sensor fits");
    memset(g_heap, 0, sizeof(g_heap));
    return g_heap;
}

struct clk *devm_clk_get_optional(struct device *dev, const char *id)
{
    (void)dev;
    (void)id;
    return &m.clk;
}

int clk_prepare_enable(struct clk *clk)
{
    if (clk->fail_enable)
        return -EINVAL;
    require(!clk->enabled, "the clock is not enabled twice");
    clk->enabled = 1;
    return 0;
}

void clk_disable_unprepare(struct clk *clk)
{
    require(clk->enabled, "the clock is enabled when disabled");
    clk->enabled = 0;
    model_forget();
}

struct gpio_desc *devm_gpiod_get_optional(struct device *dev, const char *con_id,
                                          enum gpiod_flags flags)
{
    struct gpio_desc *desc = strcmp(con_id, "powerdown") == 0 ? &m.pwdn : &m.reset;

    (void)dev;
    desc->value = flags == GPIOD_OUT_HIGH;
    m.lines[m.lines_given++ % 2] = desc;
    return desc;
}

void gpiod_set_value_cansleep(struct gpio_desc *desc, int value)
{
    desc->value = value;
    model_forget();
}

int i2c_master_send(const struct i2c_client *client, const char *buf, int count)
{
    (void)client;
    if (!model_powered()) {
        m.dead_writes++;
        return -EREMOTEIO;
    }
    require(count == 3, "a register write is address and value");
    if (m.fail_i2c)
        return -EREMOTEIO;
    m.last_reg = (u16)(((u8)buf[0] << 8) | (u8)buf[1]);
    m.last_val = (u8)buf[2];
    m.regs[m.last_reg] = m.last_val;
    m.writes++;
    return count;
}

int i2c_transfer(struct i2c_adapter *adap, struct i2c_msg *msgs, int num)
{
    u16 reg;

    (void)adap;
    require(num == 2 && msgs[1].flags == I2C_M_RD, "a register read is address then data");
    if (!model_powered())
        return -EREMOTEIO;
    reg = (u16)((msgs[0].buf[0] << 8) | msgs[0].buf[1]);
    if (reg == OV5640_CHIP_ID_REG)
        msgs[1].buf[0] = m.chip_id >> 8;
    else if (reg == OV5640_CHIP_ID_REG + 1)
        msgs[1].buf[0] = m.chip_id & 0xff;
    else
        msgs[1].buf[0] = m.regs[reg];
    m.reads++;
    return num;
}

int media_entity_pads_init(struct media_entity *entity, u16 num_pads, struct media_pad *pads)
{
    (void)entity;
    (void)num_pads;
    (void)pads;
    return 0;
}

void media_entity_cleanup(struct media_entity *entity)
{
    (void)entity;
}

int v4l2_subdev_link_validate(void *link)
{
    (void)link;
    return 0;
}

int v4l2_ctrl_handler_init(struct v4l2_ctrl_handler *hdl, unsigned int nr_of_controls_hint)
{
    (void)nr_of_controls_hint;
    memset(hdl, 0, sizeof(*hdl));
    return 0;
}

struct v4l2_ctrl *v4l2_ctrl_new_custom(struct v4l2_ctrl_handler *hdl,
                                       const struct v4l2_ctrl_config *cfg, void *priv)
{
    struct v4l2_ctrl *ctrl;

    (void)priv;
    if (hdl->count >= SHIM_MAX_CTRLS) {
        hdl->error = -ENOMEM;
        return NULL;
    }
    ctrl = &hdl->ctrls[hdl->count++];
    ctrl->handler = hdl;
    ctrl->ops = cfg->ops;
    ctrl->id = cfg->id;
    ctrl->val = cfg->def;
    return ctrl;
}

void v4l2_ctrl_handler_free(struct v4l2_ctrl_handler *hdl)
{
    hdl->count = 0;
}

static struct i2c_client g_client;

void v4l2_i2c_subdev_init(struct v4l2_subdev *sd, struct i2c_client *client,
                          const struct v4l2_subdev_ops *ops)
{
    memset(sd, 0, sizeof(*sd));
    sd->ops = ops;
    client->clientdata = sd;
}

int v4l2_async_register_subdev_sensor(struct v4l2_subdev *sd)
{
    (void)sd;
    return 0;
}

void v4l2_async_unregister_subdev(struct v4l2_subdev *sd)
{
    (void)sd;
}

/* ── the framework's side ── */

static struct cam_ov5640 *sensor(void)
{
    return to_cam_ov5640(i2c_get_clientdata(&g_client));
}

static int probe(u16 chip_id)
{
    memset(&m, 0, sizeof(m));
    memset(&g_client, 0, sizeof(g_client));
    m.chip_id = chip_id;
    shim_dev_errors = 0;
    shim_warnings = 0;
    return cam_ov5640_i2c_driver.probe_new(&g_client);
}

static int fh_open(struct v4l2_subdev_fh *fh)
{
    struct v4l2_subdev *sd = &sensor()->sd;

    return sd->internal_ops->open(sd, fh);
}

static void fh_close(struct v4l2_subdev_fh *fh)
{
    struct v4l2_subdev *sd = &sensor()->sd;

    require(sd->internal_ops->close(sd, fh) == 0, "close succeeds");
}

static int set_ctrl(u32 id, s32 val)
{
    struct v4l2_ctrl_handler *hdl = &sensor()->ctrls;
    int ret = -EINVAL;

    for (unsigned int i = 0; i < hdl->count; i++) {
        if (hdl->ctrls[i].id != id)
            continue;
        hdl->ctrls[i].val = val;
        mutex_lock(hdl->lock);
        ret = hdl->ctrls[i].ops->s_ctrl(&hdl->ctrls[i]);
        mutex_unlock(hdl->lock);
    }
    return ret;
}

static int s_stream(int enable)
{
    struct v4l2_subdev *sd = &sensor()->sd;

    return sd->ops->video->s_stream(sd, enable);
}

/* As cam_system starts: open, initialize, leave standby, then STREAMON reaches the sensor. */
static void app_start(struct v4l2_subdev_fh *fh)
{
    require(fh_open(fh) == 0, "open powers the sensor up");
    require(set_ctrl(CAM_OV5640_CID_INITIALIZE, 1) == 0, "initialize");
    require(set_ctrl(CAM_OV5640_CID_STANDBY, 0) == 0, "leave standby");
    require(s_stream(1) == 0, "stream on");
}

static void require_off(const char *what)
{
    require(!model_powered() && !m.clk.enabled && m.pwdn.value && m.reset.value, what);
    require(!sensor()->powered && !sensor()->initialized, "the driver knows the sensor is off");
}

static void require_clean(void)
{
    require(m.dead_writes == 0, "nothing was written to a sensor without power");
    require(m.locks_held == 0, "every lock was released");
    require(shim_warnings == 0, "no warnings");
}

static void test_probe(void)
{
    require(probe(0x5640) == 0, "probe finds the sensor");
    require(m.reads == 2, "probe reads the chip id");
    require_off("after probe the sensor is powered down and held in reset");
    require(sensor()->sd.internal_ops != NULL, "file handles reach the driver");
    require(cam_ov5640_i2c_driver.driver.suppress_bind_attrs, "no unbinding under an open handle");
    require_clean();

    require(probe(0x2640) == -ENODEV, "a different chip is refused");
    require(!model_powered() && !m.clk.enabled, "and left without power");
}

/* An orderly exit: STREAMOFF, standby, close. */
static void test_orderly_exit(void)
{
    struct v4l2_subdev_fh fh;

    probe(0x5640);
    app_start(&fh);
    require(model_powered() && sensor()->streaming, "running");
    require(m.regs[0x4300] == 0x61, "the table is written");
    require(set_ctrl(CAM_OV5640_CID_ISP_OFFSET, (100 << 16) | 50) == 0, "an offset is written");
    require(m.regs[0x3811] == 100 && m.regs[0x3813] == 50 && m.regs[OV5640_REG_GROUP_ACCESS] == OV5640_GROUP3_LAUNCH,
            "inside a launched group");

    require(s_stream(0) == 0, "stream off");
    require(model_powered(), "the open handle keeps the power");
    require(set_ctrl(CAM_OV5640_CID_STANDBY, 1) == 0, "standby");
    fh_close(&fh);
    require_off("the last close powers the sensor down");
    require_clean();
}

/* kill -9: the kernel closes the files in descriptor order, the subdev before the video node,
 * so the sensor's handle goes while the stream still runs. */
static void test_killed_process(void)
{
    struct v4l2_subdev_fh fh;
    unsigned int writes;

    probe(0x5640);
    app_start(&fh);
    fh_close(&fh);
    require(model_powered(), "the running stream keeps the power");
    writes = m.writes;
    require(s_stream(0) == 0, "the video node's release stops the stream");
    require(m.writes == writes + 1 && m.last_reg == OV5640_REG_SYS_CTRL0 &&
            m.last_val == OV5640_REG_SYS_CTRL0_SW_PWDN, "with a standby write first");
    require_off("then the sensor is powered down");
    require_clean();

    /* the other order: the video node first */
    app_start(&fh);
    require(s_stream(0) == 0, "stream off");
    require(model_powered(), "the handle keeps the power");
    fh_close(&fh);
    require_off("the close powers the sensor down");
    require_clean();

    /* the restarted process starts from a sensor that forgot everything */
    app_start(&fh);
    require(m.regs[0x4300] == 0x61, "the table is written again");
    require(model_powered() && sensor()->streaming, "and the sensor runs");
    require(s_stream(0) == 0, "stream off");
    fh_close(&fh);
    require_off("off again");
    require_clean();
}

static void test_two_handles(void)
{
    struct v4l2_subdev_fh a;
    struct v4l2_subdev_fh b;

    probe(0x5640);
    require(fh_open(&a) == 0 && fh_open(&b) == 0, "two handles");
    require(sensor()->open_count == 2, "both counted");
    fh_close(&a);
    require(model_powered(), "one handle is enough");
    fh_close(&b);
    require_off("the last one powers down");
    require_clean();
}

/* Without a handle: what streams powers up, initializes and powers down again. */
static void test_stream_alone(void)
{
    probe(0x5640);
    require(s_stream(0) == 0 && m.writes == 0, "stopping a stream that never ran touches nothing");
    require(s_stream(1) == 0, "stream on without a handle");
    require(model_powered() && m.regs[0x4300] == 0x61 && m.regs[OV5640_REG_SYS_CTRL0] == 0x02,
            "powered, initialized and out of standby");
    require(s_stream(0) == 0, "stream off");
    require_off("nothing holds the power any more");
    require_clean();
}

static void test_without_power(void)
{
    unsigned int writes;

    probe(0x5640);
    writes = m.writes;
    require(set_ctrl(CAM_OV5640_CID_INITIALIZE, 1) == -EIO, "no initialize without a user");
    require(set_ctrl(CAM_OV5640_CID_ISP_OFFSET, 0) == -EIO, "no offset without a user");
    require(m.writes == writes && !model_powered(), "and the sensor stays off");
    require_clean();
}

static void test_failures(void)
{
    struct v4l2_subdev_fh fh;

    probe(0x5640);
    m.clk.fail_enable = 1;
    require(fh_open(&fh) == -EINVAL, "an open fails when the clock does not start");
    require(sensor()->open_count == 0 && !sensor()->powered, "and holds nothing");
    require(s_stream(1) == -EINVAL && !sensor()->streaming, "a stream cannot start either");
    m.clk.fail_enable = 0;

    /* the bus fails during the table: the stream does not start and keeps no power */
    m.fail_i2c = 1;
    require(s_stream(1) != 0, "a stream cannot start on a dead bus");
    require(!sensor()->streaming, "it does not count as running");
    require_off("and holds no power");

    /* an open handle keeps the power through a failed initialize */
    require(fh_open(&fh) == 0, "open");
    require(set_ctrl(CAM_OV5640_CID_INITIALIZE, 1) != 0, "initialize fails");
    require(sensor()->powered, "the handle still holds the power");
    fh_close(&fh);
    require_off("the close releases it");
    m.fail_i2c = 0;
    require(sensor()->write_failures > 0 && shim_dev_errors > 0, "the failures were counted");
    require_clean();
}

int main(void)
{
    test_probe();
    test_orderly_exit();
    test_killed_process();
    test_two_handles();
    test_stream_alone();
    test_without_power();
    test_failures();
    printf("ov5640_driver_test: OK (%d checks)\n", g_checks);
    return 0;
}
