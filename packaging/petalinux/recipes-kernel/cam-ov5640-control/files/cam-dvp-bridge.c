// SPDX-License-Identifier: GPL-2.0-only
/*
 * Minimal V4L2 subdev for the project PL DVP bridge.
 *
 * The hardware samples OV5640 8-bit DVP RGB565, expands it to RGB888 native
 * video, and feeds the Xilinx AXI4-Stream/video DMA path. The driver is kept
 * deliberately small: it models the media graph and fixed pad formats, while
 * the already validated PL datapath remains configured by hardware design.
 * The bridge's frame counters are the one thing read from the hardware: a frame
 * of the wrong size is otherwise invisible to software.
 */

#include <linux/io.h>
#include <linux/module.h>
#include <linux/mutex.h>
#include <linux/of.h>
#include <linux/platform_device.h>
#include <media/media-entity.h>
#include <media/v4l2-ctrls.h>
#include <media/v4l2-subdev.h>
#include <linux/v4l2-mediabus.h>

#define CAM_DVP_BRIDGE_PADS 2
#define CAM_DVP_BRIDGE_SINK 0
#define CAM_DVP_BRIDGE_SOURCE 1

/* Snapshots the PL takes at each VSYNC; see ov5640_to_lcd_v1_0_S00_AXI.v. */
#define CAM_DVP_REG_FRAME_COUNT 0x14
#define CAM_DVP_REG_SYNC_ERROR_COUNT 0x24

#define CAM_DVP_CID_BASE (V4L2_CID_USER_BASE + 0x2010)
#define CAM_DVP_CID_FRAMES (CAM_DVP_CID_BASE + 0)
#define CAM_DVP_CID_BAD_FRAMES (CAM_DVP_CID_BASE + 1)

struct cam_dvp_bridge {
	struct device *dev;
	void __iomem *regs;
	struct mutex lock;	/* pad formats and controls */
	struct v4l2_ctrl_handler ctrls;
	struct v4l2_subdev sd;
	struct media_pad pads[CAM_DVP_BRIDGE_PADS];
	struct v4l2_mbus_framefmt formats[CAM_DVP_BRIDGE_PADS];
};

static inline struct cam_dvp_bridge *to_cam_dvp_bridge(struct v4l2_subdev *sd)
{
	return container_of(sd, struct cam_dvp_bridge, sd);
}

static void cam_dvp_bridge_set_defaults(struct cam_dvp_bridge *bridge)
{
	struct v4l2_mbus_framefmt *sink = &bridge->formats[CAM_DVP_BRIDGE_SINK];
	struct v4l2_mbus_framefmt *source = &bridge->formats[CAM_DVP_BRIDGE_SOURCE];

	memset(sink, 0, sizeof(*sink));
	sink->width = 640;
	sink->height = 480;
	sink->code = MEDIA_BUS_FMT_RGB565_2X8_LE;
	sink->field = V4L2_FIELD_NONE;
	sink->colorspace = V4L2_COLORSPACE_SRGB;

	*source = *sink;
	source->code = MEDIA_BUS_FMT_RGB888_1X24;
}

static struct v4l2_mbus_framefmt *
cam_dvp_bridge_get_pad_format(struct cam_dvp_bridge *bridge,
			      struct v4l2_subdev_state *state,
			      unsigned int pad, u32 which)
{
	if (pad >= CAM_DVP_BRIDGE_PADS)
		return NULL;

	if (which == V4L2_SUBDEV_FORMAT_TRY)
		return v4l2_subdev_get_try_format(&bridge->sd, state, pad);

	return &bridge->formats[pad];
}

static int cam_dvp_bridge_init_cfg(struct v4l2_subdev *sd,
				   struct v4l2_subdev_state *state)
{
	struct cam_dvp_bridge *bridge = to_cam_dvp_bridge(sd);
	unsigned int pad;

	for (pad = 0; pad < CAM_DVP_BRIDGE_PADS; pad++)
		*v4l2_subdev_get_try_format(sd, state, pad) = bridge->formats[pad];
	return 0;
}

static int cam_dvp_bridge_enum_mbus_code(struct v4l2_subdev *sd,
					 struct v4l2_subdev_state *state,
					 struct v4l2_subdev_mbus_code_enum *code)
{
	(void)sd;
	(void)state;

	if (code->index > 0)
		return -EINVAL;

	if (code->pad == CAM_DVP_BRIDGE_SINK)
		code->code = MEDIA_BUS_FMT_RGB565_2X8_LE;
	else if (code->pad == CAM_DVP_BRIDGE_SOURCE)
		code->code = MEDIA_BUS_FMT_RGB888_1X24;
	else
		return -EINVAL;

	return 0;
}

static int cam_dvp_bridge_get_fmt(struct v4l2_subdev *sd,
				  struct v4l2_subdev_state *state,
				  struct v4l2_subdev_format *fmt)
{
	struct cam_dvp_bridge *bridge = to_cam_dvp_bridge(sd);
	struct v4l2_mbus_framefmt *format;
	int ret = 0;

	mutex_lock(&bridge->lock);
	format = cam_dvp_bridge_get_pad_format(bridge, state, fmt->pad,
					       fmt->which);
	if (!format)
		ret = -EINVAL;
	else
		fmt->format = *format;
	mutex_unlock(&bridge->lock);

	return ret;
}

static int cam_dvp_bridge_set_fmt(struct v4l2_subdev *sd,
				  struct v4l2_subdev_state *state,
				  struct v4l2_subdev_format *fmt)
{
	struct cam_dvp_bridge *bridge = to_cam_dvp_bridge(sd);
	struct v4l2_mbus_framefmt *format;
	struct v4l2_mbus_framefmt *source;
	struct v4l2_mbus_framefmt fixed;
	int ret = 0;

	mutex_lock(&bridge->lock);

	format = cam_dvp_bridge_get_pad_format(bridge, state, fmt->pad,
					       fmt->which);
	if (!format) {
		ret = -EINVAL;
		goto out;
	}

	fixed = fmt->format;
	fixed.width = 640;
	fixed.height = 480;
	fixed.field = V4L2_FIELD_NONE;
	fixed.colorspace = V4L2_COLORSPACE_SRGB;

	if (fmt->pad == CAM_DVP_BRIDGE_SINK)
		fixed.code = MEDIA_BUS_FMT_RGB565_2X8_LE;
	else
		fixed.code = MEDIA_BUS_FMT_RGB888_1X24;

	*format = fixed;
	fmt->format = fixed;

	source = cam_dvp_bridge_get_pad_format(bridge, state,
					       CAM_DVP_BRIDGE_SOURCE,
					       fmt->which);
	if (source) {
		source->width = fixed.width;
		source->height = fixed.height;
		source->field = fixed.field;
		source->colorspace = fixed.colorspace;
		source->code = MEDIA_BUS_FMT_RGB888_1X24;
	}

out:
	mutex_unlock(&bridge->lock);
	return ret;
}

static int cam_dvp_bridge_s_stream(struct v4l2_subdev *sd, int enable)
{
	struct cam_dvp_bridge *bridge = to_cam_dvp_bridge(sd);

	dev_dbg(bridge->dev, "stream %s\n", enable ? "on" : "off");
	return 0;
}

static const struct v4l2_subdev_video_ops cam_dvp_bridge_video_ops = {
	.s_stream = cam_dvp_bridge_s_stream,
};

static const struct v4l2_subdev_pad_ops cam_dvp_bridge_pad_ops = {
	.init_cfg = cam_dvp_bridge_init_cfg,
	.enum_mbus_code = cam_dvp_bridge_enum_mbus_code,
	.get_fmt = cam_dvp_bridge_get_fmt,
	.set_fmt = cam_dvp_bridge_set_fmt,
};

static const struct v4l2_subdev_ops cam_dvp_bridge_subdev_ops = {
	.video = &cam_dvp_bridge_video_ops,
	.pad = &cam_dvp_bridge_pad_ops,
};

static const struct media_entity_operations cam_dvp_bridge_media_ops = {
	.link_validate = v4l2_subdev_link_validate,
};

static int cam_dvp_bridge_g_volatile_ctrl(struct v4l2_ctrl *ctrl)
{
	struct cam_dvp_bridge *bridge =
		container_of(ctrl->handler, struct cam_dvp_bridge, ctrls);

	switch (ctrl->id) {
	case CAM_DVP_CID_FRAMES:
		ctrl->val = ioread32(bridge->regs + CAM_DVP_REG_FRAME_COUNT) & S32_MAX;
		return 0;
	case CAM_DVP_CID_BAD_FRAMES:
		ctrl->val = ioread32(bridge->regs + CAM_DVP_REG_SYNC_ERROR_COUNT) & S32_MAX;
		return 0;
	default:
		return -EINVAL;
	}
}

static const struct v4l2_ctrl_ops cam_dvp_bridge_ctrl_ops = {
	.g_volatile_ctrl = cam_dvp_bridge_g_volatile_ctrl,
};

static const struct v4l2_ctrl_config cam_dvp_bridge_ctrls[] = {
	{
		.ops = &cam_dvp_bridge_ctrl_ops,
		.id = CAM_DVP_CID_FRAMES,
		.name = "DVP Frames",
		.type = V4L2_CTRL_TYPE_INTEGER,
		.flags = V4L2_CTRL_FLAG_READ_ONLY | V4L2_CTRL_FLAG_VOLATILE,
		.max = S32_MAX,
		.step = 1,
	},
	{
		.ops = &cam_dvp_bridge_ctrl_ops,
		.id = CAM_DVP_CID_BAD_FRAMES,
		.name = "DVP Bad Frames",
		.type = V4L2_CTRL_TYPE_INTEGER,
		.flags = V4L2_CTRL_FLAG_READ_ONLY | V4L2_CTRL_FLAG_VOLATILE,
		.max = S32_MAX,
		.step = 1,
	},
};

static int cam_dvp_bridge_init_controls(struct cam_dvp_bridge *bridge)
{
	struct v4l2_ctrl_handler *hdl = &bridge->ctrls;
	unsigned int i;
	int ret;

	v4l2_ctrl_handler_init(hdl, ARRAY_SIZE(cam_dvp_bridge_ctrls));
	hdl->lock = &bridge->lock;
	for (i = 0; i < ARRAY_SIZE(cam_dvp_bridge_ctrls); i++)
		v4l2_ctrl_new_custom(hdl, &cam_dvp_bridge_ctrls[i], NULL);

	ret = hdl->error;
	if (ret) {
		v4l2_ctrl_handler_free(hdl);
		return ret;
	}
	bridge->sd.ctrl_handler = hdl;
	return 0;
}

static int cam_dvp_bridge_probe(struct platform_device *pdev)
{
	struct cam_dvp_bridge *bridge;
	int ret;

	bridge = devm_kzalloc(&pdev->dev, sizeof(*bridge), GFP_KERNEL);
	if (!bridge)
		return -ENOMEM;

	bridge->dev = &pdev->dev;
	bridge->regs = devm_platform_ioremap_resource(pdev, 0);
	if (IS_ERR(bridge->regs))
		return PTR_ERR(bridge->regs);
	mutex_init(&bridge->lock);
	cam_dvp_bridge_set_defaults(bridge);

	v4l2_subdev_init(&bridge->sd, &cam_dvp_bridge_subdev_ops);
	bridge->sd.dev = &pdev->dev;
	bridge->sd.fwnode = of_fwnode_handle(pdev->dev.of_node);
	bridge->sd.flags |= V4L2_SUBDEV_FL_HAS_DEVNODE;
	bridge->sd.entity.function = MEDIA_ENT_F_PROC_VIDEO_PIXEL_ENC_CONV;
	strscpy(bridge->sd.name, dev_name(&pdev->dev), sizeof(bridge->sd.name));

	ret = cam_dvp_bridge_init_controls(bridge);
	if (ret)
		goto destroy_mutex;

	bridge->pads[CAM_DVP_BRIDGE_SINK].flags = MEDIA_PAD_FL_SINK;
	bridge->pads[CAM_DVP_BRIDGE_SOURCE].flags = MEDIA_PAD_FL_SOURCE;
	bridge->sd.entity.ops = &cam_dvp_bridge_media_ops;

	ret = media_entity_pads_init(&bridge->sd.entity, CAM_DVP_BRIDGE_PADS,
				     bridge->pads);
	if (ret)
		goto free_ctrls;

	platform_set_drvdata(pdev, bridge);

	ret = v4l2_async_register_subdev(&bridge->sd);
	if (ret)
		goto entity_cleanup;

	dev_info(&pdev->dev, "cam DVP RGB565 to RGB888 bridge registered\n");
	return 0;

entity_cleanup:
	media_entity_cleanup(&bridge->sd.entity);
free_ctrls:
	v4l2_ctrl_handler_free(&bridge->ctrls);
destroy_mutex:
	mutex_destroy(&bridge->lock);
	return ret;
}

static int cam_dvp_bridge_remove(struct platform_device *pdev)
{
	struct cam_dvp_bridge *bridge = platform_get_drvdata(pdev);

	v4l2_async_unregister_subdev(&bridge->sd);
	media_entity_cleanup(&bridge->sd.entity);
	v4l2_ctrl_handler_free(&bridge->ctrls);
	mutex_destroy(&bridge->lock);
	return 0;
}

static const struct of_device_id cam_dvp_bridge_of_match[] = {
	{ .compatible = "cam-system,dvp-rgb565-rgb888-bridge" },
	{ }
};
MODULE_DEVICE_TABLE(of, cam_dvp_bridge_of_match);

static struct platform_driver cam_dvp_bridge_driver = {
	.driver = {
		.name = "cam-dvp-bridge",
		.of_match_table = cam_dvp_bridge_of_match,
	},
	.probe = cam_dvp_bridge_probe,
	.remove = cam_dvp_bridge_remove,
};

module_platform_driver(cam_dvp_bridge_driver);

MODULE_DESCRIPTION("Project DVP RGB565 to RGB888 V4L2 bridge subdev");
MODULE_LICENSE("GPL");
