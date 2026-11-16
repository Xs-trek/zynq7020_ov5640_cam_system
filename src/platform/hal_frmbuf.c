/* V4L2 capture backend adapter.
 *
 * Product path: Xilinx VIPP/frmbuf through /dev/videoX mmap buffers.
 * Removed direct-register fallback code is available only through git history.
 */
#define LOG_TAG "capture-backend"

#include "platform/hal_frmbuf.h"
#include "cam_system/config.h"
#include "common/app_config.h"
#include "common/log.h"

#include <dirent.h>
#include <errno.h>
#include <fcntl.h>
#include <linux/media-bus-format.h>
#include <linux/v4l2-subdev.h>
#include <linux/videodev2.h>
#include <poll.h>
#include <stdatomic.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/ioctl.h>
#include <sys/mman.h>
#include <unistd.h>

struct v4l2_slot {
    void *addr;
    size_t len;
    int queued;
};

static int video_fd = -1;

/* Read-only controls of cam-dvp-bridge. */
#define CAM_DVP_CID_BASE       (V4L2_CID_USER_BASE + 0x2010)
#define CAM_DVP_CID_FRAMES     (CAM_DVP_CID_BASE + 0)
#define CAM_DVP_CID_BAD_FRAMES (CAM_DVP_CID_BASE + 1)

static int bridge_fd = -1;
static int bridge_open_tried;
static struct v4l2_slot v4l2_slots[CAPTURE_BUF_CNT];
static int v4l2_streaming = 0;
static enum v4l2_buf_type v4l2_type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
static int v4l2_mplane = 0;
static atomic_ulong v4l2_error_frames;

static int xioctl(int fd, unsigned long request, void *arg)
{
    int ret;

    do {
        ret = ioctl(fd, request, arg);
    } while (ret < 0 && errno == EINTR);

    return ret;
}

static int cap_subdev_name_matches(const char *entry, const char *needle)
{
    char name_path[512];
    char name[128];
    int fd;
    ssize_t n;

    snprintf(name_path, sizeof(name_path), "/sys/class/video4linux/%s/name",
             entry);
    fd = open(name_path, O_RDONLY);
    if (fd < 0)
        return 0;

    n = read(fd, name, sizeof(name) - 1);
    close(fd);
    if (n <= 0)
        return 0;

    name[n] = '\0';
    return strstr(name, needle) != NULL;
}

/* Opens the first subdev whose name contains needle.  Returns the fd, or -1. */
static int cap_open_subdev(const char *needle, char *path, size_t path_size)
{
    DIR *dir = opendir("/sys/class/video4linux");
    struct dirent *de;
    int fd = -1;

    if (!dir)
        return -1;

    while ((de = readdir(dir)) != NULL) {
        if (strncmp(de->d_name, "v4l-subdev", 10) != 0)
            continue;
        if (!cap_subdev_name_matches(de->d_name, needle))
            continue;

        snprintf(path, path_size, "/dev/%s", de->d_name);
        fd = open(path, O_RDWR);
        if (fd < 0)
            LOG_E("open %s failed: %s", path, strerror(errno));
        break;
    }

    closedir(dir);
    return fd;
}

static int cap_set_subdev_format(const char *needle, unsigned int pad,
                                 uint32_t code)
{
    struct v4l2_subdev_format fmt;
    char path[512];
    int fd = cap_open_subdev(needle, path, sizeof(path));

    if (fd < 0) {
        LOG_E("subdev '%s' pad=%u not configured", needle, pad);
        return -1;
    }

    memset(&fmt, 0, sizeof(fmt));
    fmt.which = V4L2_SUBDEV_FORMAT_ACTIVE;
    fmt.pad = pad;
    fmt.format.width = IMG_W;
    fmt.format.height = IMG_H;
    fmt.format.code = code;
    fmt.format.field = V4L2_FIELD_NONE;
    fmt.format.colorspace = V4L2_COLORSPACE_SRGB;

    if (xioctl(fd, VIDIOC_SUBDEV_S_FMT, &fmt) < 0) {
        LOG_E("VIDIOC_SUBDEV_S_FMT %s pad=%u code=0x%04x failed: %s",
              path, pad, code, strerror(errno));
        close(fd);
        return -1;
    }

    LOG_I("media format %s pad=%u code=0x%04x %ux%u",
          path, pad, fmt.format.code, fmt.format.width, fmt.format.height);
    close(fd);
    return 0;
}

static int cap_configure_media_graph(void)
{
    if (app_config()->capture_skip_media_setup)
        return 0;

    if (cap_set_subdev_format("cam-ov5640-control", 0,
                              MEDIA_BUS_FMT_RGB565_2X8_LE) < 0)
        return -1;
    if (cap_set_subdev_format("ov5640_to_lcd", 0,
                              MEDIA_BUS_FMT_RGB565_2X8_LE) < 0)
        return -1;
    if (cap_set_subdev_format("ov5640_to_lcd", 1,
                              MEDIA_BUS_FMT_RGB888_1X24) < 0)
        return -1;
    if (cap_set_subdev_format("axis-subset-converter", 0,
                              MEDIA_BUS_FMT_RGB888_1X24) < 0)
        return -1;
    if (cap_set_subdev_format("axis-subset-converter", 1,
                              MEDIA_BUS_FMT_RBG888_1X24) < 0)
        return -1;

    return 0;
}

static int cap_select_v4l2_type(int fd, const char *path)
{
    struct v4l2_capability cap;
    uint32_t caps;

    memset(&cap, 0, sizeof(cap));
    if (xioctl(fd, VIDIOC_QUERYCAP, &cap) < 0) {
        LOG_E("VIDIOC_QUERYCAP %s failed: %s",
      path, strerror(errno));
        return -1;
    }

    caps = cap.device_caps ? cap.device_caps : cap.capabilities;
    if (!(caps & V4L2_CAP_STREAMING)) {
        LOG_E("%s does not support streaming",
      path);
        return -1;
    }

    if (caps & V4L2_CAP_VIDEO_CAPTURE_MPLANE) {
        v4l2_type = V4L2_BUF_TYPE_VIDEO_CAPTURE_MPLANE;
        v4l2_mplane = 1;
    } else if (caps & V4L2_CAP_VIDEO_CAPTURE) {
        v4l2_type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
        v4l2_mplane = 0;
    } else {
        LOG_E("%s is not a capture device caps=0x%08x",
      path, caps);
        return -1;
    }

    LOG_I("using V4L2 device %s (%s, %s)",
          path, (const char *)cap.card,
          v4l2_mplane ? "mplane" : "single-plane");
    return 0;
}

static int cap_open_video_device(void)
{
    const char *forced = app_config()->capture_device;
    char path[32];

    if (forced && forced[0] != '\0') {
        video_fd = open(forced, O_RDWR | O_NONBLOCK);
        if (video_fd >= 0 && cap_select_v4l2_type(video_fd, forced) == 0)
            return 0;
        LOG_E("open %s failed: %s",
      forced, video_fd < 0 ? strerror(errno) : "unsupported device");
        if (video_fd >= 0) {
            close(video_fd);
            video_fd = -1;
        }
        return -1;
    }

    for (int i = 0; i < 10; i++) {
        snprintf(path, sizeof(path), "/dev/video%d", i);
        video_fd = open(path, O_RDWR | O_NONBLOCK);
        if (video_fd < 0)
            continue;

        if (cap_select_v4l2_type(video_fd, path) == 0)
            return 0;

        close(video_fd);
        video_fd = -1;
    }

    LOG_E("no V4L2 capture device found");
    return -1;
}

int frmbuf_init(void)
{
    struct v4l2_format fmt;
    struct v4l2_requestbuffers req;

    if (video_fd >= 0)
        return 0;
    if (cap_open_video_device() < 0)
        return -1;
    if (cap_configure_media_graph() < 0)
        return -1;

    memset(&fmt, 0, sizeof(fmt));
    fmt.type = v4l2_type;
    if (v4l2_mplane) {
        fmt.fmt.pix_mp.width = IMG_W;
        fmt.fmt.pix_mp.height = IMG_H;
        fmt.fmt.pix_mp.pixelformat = V4L2_PIX_FMT_RGB24;
        fmt.fmt.pix_mp.field = V4L2_FIELD_NONE;
        fmt.fmt.pix_mp.num_planes = 1;
        fmt.fmt.pix_mp.plane_fmt[0].bytesperline = IMG_W * IMG_BPP;
        fmt.fmt.pix_mp.plane_fmt[0].sizeimage = FRAME_SIZE;
    } else {
        fmt.fmt.pix.width = IMG_W;
        fmt.fmt.pix.height = IMG_H;
        fmt.fmt.pix.pixelformat = V4L2_PIX_FMT_RGB24;
        fmt.fmt.pix.field = V4L2_FIELD_NONE;
        fmt.fmt.pix.bytesperline = IMG_W * IMG_BPP;
        fmt.fmt.pix.sizeimage = FRAME_SIZE;
    }

    if (xioctl(video_fd, VIDIOC_S_FMT, &fmt) < 0) {
        LOG_E("VIDIOC_S_FMT RGB24 %dx%d failed: %s",
      IMG_W, IMG_H, strerror(errno));
        return -1;
    }

    if (v4l2_mplane) {
        if (fmt.fmt.pix_mp.width != IMG_W || fmt.fmt.pix_mp.height != IMG_H ||
            fmt.fmt.pix_mp.pixelformat != V4L2_PIX_FMT_RGB24 ||
            fmt.fmt.pix_mp.num_planes < 1 ||
            fmt.fmt.pix_mp.plane_fmt[0].bytesperline != IMG_W * IMG_BPP ||
            fmt.fmt.pix_mp.plane_fmt[0].sizeimage < FRAME_SIZE) {
            LOG_E("unexpected mplane format %ux%u fourcc=0x%08x planes=%u stride=%u size=%u",
                  fmt.fmt.pix_mp.width, fmt.fmt.pix_mp.height,
                  fmt.fmt.pix_mp.pixelformat, fmt.fmt.pix_mp.num_planes,
                  fmt.fmt.pix_mp.plane_fmt[0].bytesperline,
                  fmt.fmt.pix_mp.plane_fmt[0].sizeimage);
            return -1;
        }
    } else {
        if (fmt.fmt.pix.width != IMG_W || fmt.fmt.pix.height != IMG_H ||
            fmt.fmt.pix.pixelformat != V4L2_PIX_FMT_RGB24 ||
            fmt.fmt.pix.bytesperline != IMG_W * IMG_BPP ||
            fmt.fmt.pix.sizeimage < FRAME_SIZE) {
            LOG_E("unexpected format %ux%u fourcc=0x%08x stride=%u size=%u",
                  fmt.fmt.pix.width, fmt.fmt.pix.height,
                  fmt.fmt.pix.pixelformat, fmt.fmt.pix.bytesperline,
                  fmt.fmt.pix.sizeimage);
            return -1;
        }
    }

    memset(&req, 0, sizeof(req));
    req.count = CAPTURE_BUF_CNT;
    req.type = v4l2_type;
    req.memory = V4L2_MEMORY_MMAP;
    if (xioctl(video_fd, VIDIOC_REQBUFS, &req) < 0) {
        LOG_E("VIDIOC_REQBUFS failed: %s",
      strerror(errno));
        return -1;
    }
    if (req.count < CAPTURE_BUF_CNT) {
        LOG_E("insufficient V4L2 buffers: %u",
      req.count);
        return -1;
    }

    for (int i = 0; i < CAPTURE_BUF_CNT; i++) {
        struct v4l2_buffer buf;
        struct v4l2_plane planes[VIDEO_MAX_PLANES];
        unsigned int length;
        unsigned int offset;

        memset(&buf, 0, sizeof(buf));
        memset(planes, 0, sizeof(planes));
        buf.type = v4l2_type;
        buf.memory = V4L2_MEMORY_MMAP;
        buf.index = i;
        if (v4l2_mplane) {
            buf.length = 1;
            buf.m.planes = planes;
        }

        if (xioctl(video_fd, VIDIOC_QUERYBUF, &buf) < 0) {
            LOG_E("VIDIOC_QUERYBUF %d failed: %s",
          i, strerror(errno));
            return -1;
        }

        length = v4l2_mplane ? planes[0].length : buf.length;
        offset = v4l2_mplane ? planes[0].m.mem_offset : buf.m.offset;
        v4l2_slots[i].len = length;
        v4l2_slots[i].addr = mmap(NULL, length, PROT_READ | PROT_WRITE,
                                  MAP_SHARED, video_fd, offset);
        if (v4l2_slots[i].addr == MAP_FAILED) {
            LOG_E("mmap buffer %d failed: %s",
          i, strerror(errno));
            v4l2_slots[i].addr = NULL;
            return -1;
        }
        v4l2_slots[i].queued = 0;
    }

    /* The stream starts when the camera is enabled (frmbuf_start). */
    LOG_I("V4L2 capture ready: %dx%d RGB24 buffers=%d",
          IMG_W, IMG_H, CAPTURE_BUF_CNT);
    return 0;
}

uint8_t *frmbuf_get_buffer(int buf_idx)
{
    if (buf_idx < 0 || buf_idx >= CAPTURE_BUF_CNT)
        return NULL;
    return (uint8_t *)v4l2_slots[buf_idx].addr;
}

static int cap_queue_v4l2_index(int buf_idx)
{
    struct v4l2_buffer buf;
    struct v4l2_plane planes[VIDEO_MAX_PLANES];

    if (buf_idx < 0 || buf_idx >= CAPTURE_BUF_CNT || v4l2_slots[buf_idx].queued)
        return -1;

    memset(&buf, 0, sizeof(buf));
    memset(planes, 0, sizeof(planes));
    buf.type = v4l2_type;
    buf.memory = V4L2_MEMORY_MMAP;
    buf.index = (unsigned int)buf_idx;
    if (v4l2_mplane) {
        buf.length = 1;
        buf.m.planes = planes;
    }

    if (xioctl(video_fd, VIDIOC_QBUF, &buf) < 0) {
        LOG_E_RL(5.0, 3, "VIDIOC_QBUF release %d failed: %s",
                 buf_idx, strerror(errno));
        return -1;
    }

    v4l2_slots[buf_idx].queued = 1;
    return 0;
}

static void cap_queue_v4l2_index_best_effort(int buf_idx)
{
    if (buf_idx < 0 || buf_idx >= CAPTURE_BUF_CNT)
        return;
    if (v4l2_slots[buf_idx].queued)
        return;

    if (cap_queue_v4l2_index(buf_idx) < 0) {
        LOG_E_RL(5.0, 3, "best-effort QBUF cleanup %d failed",
                         buf_idx);
    }
}

int frmbuf_capture_frame(int timeout_ms, frmbuf_frame_info_t *info)
{
    struct pollfd pfd;
    int latest_idx = -1;
    int dropped = 0;
    frmbuf_frame_info_t latest_info;
    int ret;

    memset(&latest_info, 0, sizeof(latest_info));
    if (info)
        memset(info, 0, sizeof(*info));

    if (video_fd < 0) {
        LOG_E("not initialized");
        return -1;
    }

    pfd.fd = video_fd;
    pfd.events = POLLIN;
    pfd.revents = 0;
    ret = poll(&pfd, 1, timeout_ms > 0 ? timeout_ms : 2000);
    if (ret <= 0) {
        LOG_W_RL(5.0, 3, "V4L2 frame %s",
                 ret == 0 ? "timeout" : strerror(errno));
        return -1;
    }

    for (int drained = 0; drained < CAPTURE_BUF_CNT; drained++) {
        struct v4l2_buffer buf;
        struct v4l2_plane planes[VIDEO_MAX_PLANES];
        unsigned int bytesused;

        memset(&buf, 0, sizeof(buf));
        memset(planes, 0, sizeof(planes));
        buf.type = v4l2_type;
        buf.memory = V4L2_MEMORY_MMAP;
        if (v4l2_mplane) {
            buf.length = 1;
            buf.m.planes = planes;
        }

        if (xioctl(video_fd, VIDIOC_DQBUF, &buf) < 0) {
            if ((errno == EAGAIN || errno == EWOULDBLOCK) && latest_idx >= 0)
                break;
            if ((errno == EAGAIN || errno == EWOULDBLOCK) && dropped)
                return FRMBUF_ERR_NO_FRAME;
            LOG_E_RL(5.0, 3, "VIDIOC_DQBUF failed: %s",
                     strerror(errno));
            cap_queue_v4l2_index_best_effort(latest_idx);
            return -1;
        }

        bytesused = v4l2_mplane ? planes[0].bytesused : buf.bytesused;
        if (buf.index >= CAPTURE_BUF_CNT) {
            LOG_E_RL(5.0, 3, "invalid buffer index=%u bytes=%u",
                     buf.index, bytesused);
            cap_queue_v4l2_index_best_effort(latest_idx);
            return -1;
        }

        v4l2_slots[buf.index].queued = 0;
        if (buf.flags & V4L2_BUF_FLAG_ERROR) {
            /* The driver completed the buffer with an error; its content is undefined. */
            atomic_fetch_add_explicit(&v4l2_error_frames, 1, memory_order_relaxed);
            dropped = 1;
            if (cap_queue_v4l2_index((int)buf.index) < 0) {
                cap_queue_v4l2_index_best_effort(latest_idx);
                return -1;
            }
            continue;
        }
        if (bytesused < FRAME_SIZE) {
            LOG_E_RL(5.0, 3, "invalid buffer index=%u bytes=%u",
                     buf.index, bytesused);
            cap_queue_v4l2_index_best_effort((int)buf.index);
            cap_queue_v4l2_index_best_effort(latest_idx);
            return -1;
        }

        if (latest_idx >= 0 && cap_queue_v4l2_index(latest_idx) < 0) {
            cap_queue_v4l2_index_best_effort((int)buf.index);
            cap_queue_v4l2_index_best_effort(latest_idx);
            return -1;
        }
        latest_idx = (int)buf.index;
        latest_info.sequence = buf.sequence;
        latest_info.timestamp_sec = (double)buf.timestamp.tv_sec +
                                    (double)buf.timestamp.tv_usec / 1000000.0;
        latest_info.flags = buf.flags;
        latest_info.timestamp_monotonic =
            (buf.flags & V4L2_BUF_FLAG_TIMESTAMP_MASK) ==
            V4L2_BUF_FLAG_TIMESTAMP_MONOTONIC;
    }

    if (latest_idx < 0)
        return FRMBUF_ERR_NO_FRAME;  /* every buffer drained this time was an error frame */
    if (info)
        *info = latest_info;
    return latest_idx;
}

/* Open on first use and kept: the status is refreshed once a second. */
int frmbuf_get_bridge_counts(int32_t *frames, int32_t *bad_frames)
{
    struct v4l2_control frame_ctrl = { .id = CAM_DVP_CID_FRAMES };
    struct v4l2_control bad_ctrl = { .id = CAM_DVP_CID_BAD_FRAMES };

    if (bridge_fd < 0 && !bridge_open_tried) {
        char path[512];

        bridge_open_tried = 1;
        bridge_fd = cap_open_subdev("ov5640_to_lcd", path, sizeof(path));
    }
    if (bridge_fd < 0)
        return -1;
    if (xioctl(bridge_fd, VIDIOC_G_CTRL, &frame_ctrl) < 0 ||
        xioctl(bridge_fd, VIDIOC_G_CTRL, &bad_ctrl) < 0)
        return -1;
    *frames = frame_ctrl.value;
    *bad_frames = bad_ctrl.value;
    return 0;
}

unsigned long frmbuf_error_frame_count(void)
{
    return atomic_load_explicit(&v4l2_error_frames, memory_order_relaxed);
}

int frmbuf_release_frame(int buf_idx)
{
    return cap_queue_v4l2_index(buf_idx);
}

int frmbuf_export_dmabuf(int buf_idx)
{
    struct v4l2_exportbuffer request;

    if (video_fd < 0 || buf_idx < 0 || buf_idx >= CAPTURE_BUF_CNT)
        return -EINVAL;
    memset(&request, 0, sizeof(request));
    request.type = v4l2_type;
    request.index = (unsigned int)buf_idx;
    request.flags = O_RDONLY | O_CLOEXEC;
    if (xioctl(video_fd, VIDIOC_EXPBUF, &request) < 0)
        return -errno;
    return request.fd;
}

static int cap_stream_off(void)
{
    if (!v4l2_streaming)
        return 0;
    if (xioctl(video_fd, VIDIOC_STREAMOFF, &v4l2_type) < 0) {
        LOG_E("VIDIOC_STREAMOFF failed: %s", strerror(errno));
        return -1;
    }
    v4l2_streaming = 0;
    /* STREAMOFF took every buffer out of the queue. */
    for (int i = 0; i < CAPTURE_BUF_CNT; i++)
        v4l2_slots[i].queued = 0;
    return 0;
}

int frmbuf_start(const int held[CAPTURE_BUF_CNT])
{
    if (video_fd < 0)
        return -1;
    if (v4l2_streaming)
        return 0;
    for (int i = 0; i < CAPTURE_BUF_CNT; i++) {
        if ((held && held[i]) || v4l2_slots[i].queued)
            continue;
        if (cap_queue_v4l2_index(i) < 0)
            return -1;
    }
    if (xioctl(video_fd, VIDIOC_STREAMON, &v4l2_type) < 0) {
        LOG_E("VIDIOC_STREAMON failed: %s", strerror(errno));
        return -1;
    }
    v4l2_streaming = 1;
    return 0;
}

int frmbuf_restart(const int held[CAPTURE_BUF_CNT])
{
    if (video_fd < 0 || cap_stream_off() < 0)
        return -1;
    return frmbuf_start(held);
}

void frmbuf_stop(void)
{
    if (video_fd >= 0)
        (void)cap_stream_off();
}

void frmbuf_close(void)
{
    frmbuf_stop();

    if (bridge_fd >= 0) {
        close(bridge_fd);
        bridge_fd = -1;
    }

    for (int i = 0; i < CAPTURE_BUF_CNT; i++) {
        if (v4l2_slots[i].addr) {
            munmap(v4l2_slots[i].addr, v4l2_slots[i].len);
            v4l2_slots[i].addr = NULL;
            v4l2_slots[i].len = 0;
            v4l2_slots[i].queued = 0;
        }
    }
    if (video_fd >= 0) {
        close(video_fd);
        video_fd = -1;
    }
}
