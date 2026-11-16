/* OV5640 control adapter.
 *
 * Product path: Linux V4L2 subdev private controls owned by the
 * cam-ov5640-control kernel driver. Removed userspace I2C/GPIO control code is
 * available only through git history.
 */
#define LOG_TAG "ov5640"

#include "platform/hal_ov5640.h"
#include "cam_system/config.h"
#include "common/app_config.h"
#include "common/log.h"

#include <dirent.h>
#include <errno.h>
#include <fcntl.h>
#include <linux/videodev2.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/ioctl.h>
#include <unistd.h>

#define CAM_OV5640_CID_BASE          (V4L2_CID_USER_BASE + 0x2000)
#define CAM_OV5640_CID_INITIALIZE    (CAM_OV5640_CID_BASE + 0)
#define CAM_OV5640_CID_STANDBY       (CAM_OV5640_CID_BASE + 1)
#define CAM_OV5640_CID_ISP_OFFSET    (CAM_OV5640_CID_BASE + 2)
#define CAM_OV5640_CID_SCALER_ENABLE (CAM_OV5640_CID_BASE + 3)
#define CAM_OV5640_CID_WRITE_FAILURES (CAM_OV5640_CID_BASE + 4)
#define CAM_OV5640_CID_GROUP_ABORTS  (CAM_OV5640_CID_BASE + 5)
#define CAM_OV5640_CID_TEST_PATTERN  (CAM_OV5640_CID_BASE + 6)

static int v4l2_fd = -1;

static int clamp_int(int val, int lo, int hi)
{
    if (val < lo) return lo;
    if (val > hi) return hi;
    return val;
}

static int v4l2_set_ctrl(uint32_t id, int32_t value)
{
    struct v4l2_control ctrl;

    if (v4l2_fd < 0) return -1;

    memset(&ctrl, 0, sizeof(ctrl));
    ctrl.id = id;
    ctrl.value = value;
    if (ioctl(v4l2_fd, VIDIOC_S_CTRL, &ctrl) < 0) {
        /* The ISP offset is written once per frame; a failing bus must not flood the log. */
        LOG_W_RL(5.0, 3, "VIDIOC_S_CTRL 0x%08x=%d failed: %s", id, value, strerror(errno));
        return -1;
    }
    return 0;
}

static int v4l2_get_ctrl(uint32_t id, int32_t *value)
{
    struct v4l2_control ctrl;

    if (v4l2_fd < 0) return -1;

    memset(&ctrl, 0, sizeof(ctrl));
    ctrl.id = id;
    if (ioctl(v4l2_fd, VIDIOC_G_CTRL, &ctrl) < 0)
        return -1;
    *value = ctrl.value;
    return 0;
}

static int v4l2_name_matches(const char *entry)
{
    char name_path[512];
    char name[128];
    int fd;
    ssize_t n;

    snprintf(name_path, sizeof(name_path), "/sys/class/video4linux/%s/name",
             entry);
    fd = open(name_path, O_RDONLY);
    if (fd < 0) return 0;
    n = read(fd, name, sizeof(name) - 1);
    close(fd);
    if (n <= 0) return 0;
    name[n] = '\0';

    return strstr(name, "cam-ov5640-control") != NULL;
}

static int v4l2_open_subdev(void)
{
    const char *forced = app_config()->ov5640_subdev;
    DIR *dir;
    struct dirent *de;
    char path[512];

    if (forced && forced[0] != '\0') {
        v4l2_fd = open(forced, O_RDWR);
        if (v4l2_fd >= 0) {
            LOG_I("Using V4L2 control subdev %s", forced);
            return 0;
        }
        LOG_E("open %s failed: %s",
      forced, strerror(errno));
        return -1;
    }

    dir = opendir("/sys/class/video4linux");
    if (!dir) return -1;

    while ((de = readdir(dir)) != NULL) {
        if (strncmp(de->d_name, "v4l-subdev", 10) != 0)
            continue;
        if (!v4l2_name_matches(de->d_name))
            continue;

        snprintf(path, sizeof(path), "/dev/%s", de->d_name);
        v4l2_fd = open(path, O_RDWR);
        if (v4l2_fd >= 0) {
            LOG_I("Using V4L2 control subdev %s", path);
            closedir(dir);
            return 0;
        }
    }

    closedir(dir);
    return -1;
}

int ov5640_init(void)
{
    if (v4l2_open_subdev() < 0)
        return -1;

    /* Finds a broken sensor at start-up.  The table leaves the sensor running; it waits in
     * standby until the camera is enabled, which initializes it again. */
    if (ov5640_initialize() < 0 ||
        v4l2_set_ctrl(CAM_OV5640_CID_STANDBY, 1) < 0) {
        close(v4l2_fd);
        v4l2_fd = -1;
        return -1;
    }

    LOG_I("V4L2 control backend initialized");
    return 0;
}

int ov5640_initialize(void)
{
    return v4l2_set_ctrl(CAM_OV5640_CID_INITIALIZE, 1);
}

int ov5640_standby(int standby)
{
    return v4l2_set_ctrl(CAM_OV5640_CID_STANDBY, standby ? 1 : 0);
}

int ov5640_apply_isp_offset(int x_offset, int y_offset)
{
    x_offset = clamp_int(x_offset, 0, PAN_MAX_X);
    y_offset = clamp_int(y_offset, 0, PAN_MAX_Y);

    int32_t packed = ((x_offset & 0x0FFF) << 16) |
                     (y_offset & 0x0FFF);
    return v4l2_set_ctrl(CAM_OV5640_CID_ISP_OFFSET, packed);
}

int ov5640_set_scaler_enabled(int enabled)
{
    return v4l2_set_ctrl(CAM_OV5640_CID_SCALER_ENABLE, enabled ? 1 : 0);
}

int ov5640_set_test_pattern(int enabled)
{
    return v4l2_set_ctrl(CAM_OV5640_CID_TEST_PATTERN, enabled ? 1 : 0);
}

int ov5640_get_error_counts(uint32_t *write_failures, uint32_t *group_aborts)
{
    int32_t writes = 0;
    int32_t aborts = 0;

    if (v4l2_get_ctrl(CAM_OV5640_CID_WRITE_FAILURES, &writes) < 0 ||
        v4l2_get_ctrl(CAM_OV5640_CID_GROUP_ABORTS, &aborts) < 0)
        return -1;
    *write_failures = (uint32_t)writes;
    *group_aborts = (uint32_t)aborts;
    return 0;
}

void ov5640_close(void)
{
    if (v4l2_fd >= 0) {
        close(v4l2_fd);
        v4l2_fd = -1;
    }
}
