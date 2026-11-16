/* frmbuf_capture_test.c - frmbuf_capture_frame against a scripted V4L2 device.
 *
 * The capture backend is compiled into this test with ioctl() and poll() redirected to fakes, so
 * the drain policy (newest frame wins, error frames dropped and requeued) runs without hardware.
 */
#define _GNU_SOURCE

#include <errno.h>
#include <fcntl.h>
#include <poll.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/ioctl.h>
#include <linux/videodev2.h>

int fake_ioctl(int fd, unsigned long request, ...);
int fake_poll(struct pollfd *fds, nfds_t nfds, int timeout);

#define ioctl fake_ioctl
#define poll fake_poll
#include "../src/platform/hal_frmbuf.c"
#undef ioctl
#undef poll

#include <stdarg.h>

typedef struct {
    unsigned int index;
    unsigned int bytesused;
    unsigned int flags;
    unsigned int sequence;
} scripted_buf_t;

static scripted_buf_t g_script[8];
static int g_script_len;
static int g_script_pos;
static int g_poll_ready = 1;
static int g_qbuf_log[16];
static int g_qbuf_count;
static int g_checks;
static int g_expbuf_errno;
static int g_streamoffs;
static int g_streamons;

static void require(int cond, const char *what)
{
    g_checks++;
    if (!cond) {
        fprintf(stderr, "frmbuf_capture_test: FAIL: %s\n", what);
        exit(1);
    }
}

int fake_poll(struct pollfd *fds, nfds_t nfds, int timeout)
{
    (void)nfds;
    (void)timeout;
    fds[0].revents = g_poll_ready ? POLLIN : 0;
    return g_poll_ready;
}

int fake_ioctl(int fd, unsigned long request, ...)
{
    struct v4l2_buffer *buf;
    void *arg;
    va_list ap;

    (void)fd;
    va_start(ap, request);
    arg = va_arg(ap, void *);
    va_end(ap);
    buf = arg;

    if (request == VIDIOC_STREAMOFF) {
        g_streamoffs++;
        return 0;
    }
    if (request == VIDIOC_STREAMON) {
        g_streamons++;
        return 0;
    }
    if (request == VIDIOC_EXPBUF) {
        struct v4l2_exportbuffer *export_request = arg;

        if (g_expbuf_errno) {
            errno = g_expbuf_errno;
            return -1;
        }
        require(export_request->type == V4L2_BUF_TYPE_VIDEO_CAPTURE, "export type");
        require(export_request->plane == 0u, "export plane");
        require((export_request->flags & O_ACCMODE) == O_RDONLY, "exported read-only");
        require((export_request->flags & O_CLOEXEC) != 0u, "exported close-on-exec");
        export_request->fd = 100 + (int)export_request->index;
        return 0;
    }
    if (request == VIDIOC_QBUF) {
        g_qbuf_log[g_qbuf_count++] = (int)buf->index;
        return 0;
    }
    if (request == VIDIOC_DQBUF) {
        const scripted_buf_t *s;

        if (g_script_pos >= g_script_len) {
            errno = EAGAIN;
            return -1;
        }
        s = &g_script[g_script_pos++];
        buf->index = s->index;
        buf->bytesused = s->bytesused;
        buf->flags = s->flags | V4L2_BUF_FLAG_TIMESTAMP_MONOTONIC;
        buf->sequence = s->sequence;
        return 0;
    }
    errno = ENOTTY;
    return -1;
}

static void reset_device(void)
{
    video_fd = 3;
    v4l2_type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    v4l2_mplane = 0;
    for (int i = 0; i < CAPTURE_BUF_CNT; i++)
        v4l2_slots[i].queued = 1;
    v4l2_streaming = 1;
    g_streamoffs = 0;
    g_streamons = 0;
    g_script_len = 0;
    g_script_pos = 0;
    g_qbuf_count = 0;
    g_poll_ready = 1;
}

static void script(unsigned int index, unsigned int bytesused, unsigned int flags, unsigned int sequence)
{
    g_script[g_script_len].index = index;
    g_script[g_script_len].bytesused = bytesused;
    g_script[g_script_len].flags = flags;
    g_script[g_script_len].sequence = sequence;
    g_script_len++;
}

int main(void)
{
    frmbuf_frame_info_t info;
    unsigned long errors_before;
    int idx;

    _Static_assert(CAPTURE_BUF_CNT >= 3, "scenarios use three buffers");

    reset_device();
    require(frmbuf_export_dmabuf(CAPTURE_BUF_CNT - 1) == 100 + CAPTURE_BUF_CNT - 1, "export the last buffer");
    require(frmbuf_export_dmabuf(CAPTURE_BUF_CNT) == -EINVAL, "export out of range");
    require(frmbuf_export_dmabuf(-1) == -EINVAL, "export negative index");
    g_expbuf_errno = ENOTTY;
    require(frmbuf_export_dmabuf(0) == -ENOTTY, "export not supported");
    g_expbuf_errno = 0;

    /* a restart requeues what the mailbox does not hold; the rest comes back later */
    {
        int held[CAPTURE_BUF_CNT] = {0};

        reset_device();
        held[1] = 1;
        held[4] = 1;
        require(frmbuf_restart(held) == 0, "restart");
        require(g_streamoffs == 1 && g_streamons == 1, "stream off and on");
        require(g_qbuf_count == CAPTURE_BUF_CNT - 2, "free buffers requeued");
        require(!v4l2_slots[1].queued && !v4l2_slots[4].queued, "held buffers stay out");
        require(v4l2_slots[0].queued && v4l2_slots[5].queued, "free buffers are in");
        require(frmbuf_release_frame(1) == 0 && v4l2_slots[1].queued, "a held buffer comes back");
        require(frmbuf_release_frame(1) < 0, "not twice");
    }

    /* The camera off and on again: stop empties the queue, start refills it with every buffer
     * nobody holds, so the first frame after a start is a new one. */
    {
        int held[CAPTURE_BUF_CNT] = {0};

        reset_device();
        frmbuf_stop();
        require(g_streamoffs == 1 && !v4l2_streaming, "stop streams off");
        for (int i = 0; i < CAPTURE_BUF_CNT; i++)
            require(!v4l2_slots[i].queued, "stop takes every buffer out");
        frmbuf_stop();
        require(g_streamoffs == 1, "stopping a stopped stream does nothing");
        require(frmbuf_release_frame(2) == 0 && v4l2_slots[2].queued,
                "a buffer handed back while stopped is queued for the next start");

        held[4] = 1;
        require(frmbuf_start(held) == 0 && v4l2_streaming && g_streamons == 1, "start streams on");
        require(g_qbuf_count == CAPTURE_BUF_CNT - 1, "start queues the rest, not twice");
        require(!v4l2_slots[4].queued && v4l2_slots[0].queued && v4l2_slots[5].queued,
                "a held buffer stays out");
        require(frmbuf_start(held) == 0 && g_streamons == 1, "starting a running stream does nothing");

        frmbuf_stop();
        g_qbuf_count = 0;
        require(frmbuf_start(NULL) == 0 && g_qbuf_count == CAPTURE_BUF_CNT, "start without holds");
    }

    /* One good frame. */
    reset_device();
    script(1, FRAME_SIZE, 0, 10);
    idx = frmbuf_capture_frame(100, &info);
    require(idx == 1 && info.sequence == 10 && info.timestamp_monotonic, "single good frame returned");
    require(g_qbuf_count == 0 && !v4l2_slots[1].queued, "returned frame stays with the caller");

    /* Two good frames: the newest wins, the older one is requeued. */
    reset_device();
    script(0, FRAME_SIZE, 0, 20);
    script(2, FRAME_SIZE, 0, 21);
    idx = frmbuf_capture_frame(100, &info);
    require(idx == 2 && info.sequence == 21, "newest frame returned");
    require(g_qbuf_count == 1 && g_qbuf_log[0] == 0, "older frame requeued");

    /* Only an error frame: dropped, requeued, counted, distinct return code. */
    reset_device();
    errors_before = frmbuf_error_frame_count();
    script(1, 1000, V4L2_BUF_FLAG_ERROR, 30);
    idx = frmbuf_capture_frame(100, &info);
    require(idx == FRMBUF_ERR_NO_FRAME, "error frame alone yields FRMBUF_ERR_NO_FRAME");
    require(g_qbuf_count == 1 && g_qbuf_log[0] == 1 && v4l2_slots[1].queued, "error frame requeued");
    require(frmbuf_error_frame_count() == errors_before + 1, "error frame counted");

    /* Error frame followed by a good frame: the good frame is returned. */
    reset_device();
    errors_before = frmbuf_error_frame_count();
    script(0, FRAME_SIZE, V4L2_BUF_FLAG_ERROR, 40);
    script(1, FRAME_SIZE, 0, 41);
    idx = frmbuf_capture_frame(100, &info);
    require(idx == 1 && info.sequence == 41, "good frame after error frame returned");
    require(g_qbuf_count == 1 && g_qbuf_log[0] == 0, "only the error frame requeued");
    require(frmbuf_error_frame_count() == errors_before + 1, "one error frame counted");

    /* Good frame followed by an error frame: the earlier good frame is kept. */
    reset_device();
    script(0, FRAME_SIZE, 0, 50);
    script(2, FRAME_SIZE, V4L2_BUF_FLAG_ERROR, 51);
    idx = frmbuf_capture_frame(100, &info);
    require(idx == 0 && info.sequence == 50, "good frame kept when a later frame errors");
    require(g_qbuf_count == 1 && g_qbuf_log[0] == 2, "later error frame requeued");

    /* Every drained buffer is an error frame. */
    reset_device();
    for (unsigned int i = 0; i < CAPTURE_BUF_CNT; i++)
        script(i, FRAME_SIZE, V4L2_BUF_FLAG_ERROR, 60 + i);
    idx = frmbuf_capture_frame(100, &info);
    require(idx == FRMBUF_ERR_NO_FRAME && g_qbuf_count == CAPTURE_BUF_CNT, "all-error drain");

    /* A short buffer without the error flag is still a device fault. */
    reset_device();
    script(1, FRAME_SIZE - 1, 0, 70);
    idx = frmbuf_capture_frame(100, &info);
    require(idx == -1 && v4l2_slots[1].queued, "short frame is an error and is requeued");

    /* Poll timeout. */
    reset_device();
    g_poll_ready = 0;
    require(frmbuf_capture_frame(100, &info) == -1, "poll timeout returns -1");

    printf("frmbuf_capture_test: OK (%d checks)\n", g_checks);
    return 0;
}
