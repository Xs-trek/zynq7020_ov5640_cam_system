/* face_accel_client_test.c - the client's protocol with the driver, against a fake device. */
#define _GNU_SOURCE
#include <errno.h>
#include <fcntl.h>
#include <stdarg.h>
#include <stddef.h>
#include <stdio.h>
#include <stdint.h>
#include <string.h>
#include <sys/ioctl.h>
#include <unistd.h>

#include "cam_system/face_accel_uapi.h"
#include "face_accel_regs.h"

/* Version 1 clients are already deployed: these sizes are in their ioctl numbers. */
_Static_assert(sizeof(struct cam_face_accel_info) == 64, "version 1 GET_INFO layout");
_Static_assert(sizeof(struct cam_face_accel_run) == 96, "version 1 RUN layout");
_Static_assert(offsetof(struct cam_face_accel_run, frame_id) == 24, "version 1 RUN layout");
_Static_assert(offsetof(struct cam_face_accel_run, result) == 32, "version 1 RUN layout");

static struct {
    int has_queue_interface;
    int submit_errno;
    int wait_errno;
    int import_errno;
    int stale_output;
    unsigned int runs;
    unsigned int submits;
    unsigned int waits;
    unsigned int last_handle;
    unsigned int last_frame;
    int last_fd;
} dev;

static void fill_output(void *output, uint32_t frame_id)
{
    fa_out_header_t header;
    fa_tensor_desc_t desc[2];
    const float score_scale = FA_OUT_SCORES_SCALE;
    const float box_scale = FA_OUT_BOXES_SCALE;

    memset(output, 0, CAM_FACE_ACCEL_OUTPUT_CAPACITY);
    memset(&header, 0, sizeof(header));
    memset(desc, 0, sizeof(desc));
    header.magic = FA_OUT_MAGIC;
    header.frame_id = frame_id;
    header.tensor_count = FA_OUT_TENSOR_COUNT;
    header.total_len = FA_OUT_LEN_V1;
    desc[0].offset = FA_OUT_SCORES_OFFSET;
    desc[0].dtype = FA_DTYPE_INT8;
    desc[0].dims[0] = 1;
    desc[0].dims[1] = 4420;
    desc[0].dims[2] = 2;
    memcpy(&desc[0].scale, &score_scale, sizeof(score_scale));
    desc[1].offset = FA_OUT_BOXES_OFFSET;
    desc[1].dtype = FA_DTYPE_INT8;
    desc[1].dims[0] = 1;
    desc[1].dims[1] = 4420;
    desc[1].dims[2] = 4;
    memcpy(&desc[1].scale, &box_scale, sizeof(box_scale));
    memcpy(output, &header, sizeof(header));
    memcpy((uint8_t *)output + sizeof(header), desc, sizeof(desc));
}

static void fill_result(struct cam_face_accel_result *result, uint32_t frame_id)
{
    memset(result, 0, sizeof(*result));
    result->output_size = CAM_FACE_ACCEL_OUTPUT_SIZE;
    result->frame_id_done = dev.stale_output ? frame_id + 1u : frame_id;
    result->perf_cycles = 2200000;
}

static int fake_open(const char *path, int flags)
{
    (void)path;
    (void)flags;
    return 7;
}

static int fake_close(int fd)
{
    (void)fd;
    return 0;
}

static int fake_ioctl(int fd, unsigned long request, ...)
{
    va_list ap;
    void *arg;

    (void)fd;
    va_start(ap, request);
    arg = va_arg(ap, void *);
    va_end(ap);
    switch (request) {
    case CAM_FACE_ACCEL_IOC_GET_INFO: {
        struct cam_face_accel_info *info = arg;

        memset(info, 0, sizeof(*info));
        info->uapi_version = CAM_FACE_ACCEL_INFO_VERSION;
        info->ip_id = CAM_FACE_ACCEL_IP_ID;
        info->input_width = CAM_FACE_ACCEL_INPUT_WIDTH;
        info->input_height = CAM_FACE_ACCEL_INPUT_HEIGHT;
        info->input_stride = CAM_FACE_ACCEL_INPUT_STRIDE;
        info->input_size = CAM_FACE_ACCEL_INPUT_SIZE;
        info->output_size = CAM_FACE_ACCEL_OUTPUT_SIZE;
        info->output_capacity = CAM_FACE_ACCEL_OUTPUT_CAPACITY;
        info->weight_size = CAM_FACE_ACCEL_WEIGHT_SIZE;
        info->scratch_size = CAM_FACE_ACCEL_SCRATCH_SIZE;
        info->flags = CAM_FACE_ACCEL_INFO_WEIGHT_READY | CAM_FACE_ACCEL_INFO_IRQ_READY;
        return 0;
    }
    case CAM_FACE_ACCEL_IOC_GET_CAPS: {
        struct cam_face_accel_caps *caps = arg;

        if (!dev.has_queue_interface) {
            errno = ENOTTY;
            return -1;
        }
        memset(caps, 0, sizeof(*caps));
        caps->uapi_version = CAM_FACE_ACCEL_UAPI_VERSION;
        caps->flags = CAM_FACE_ACCEL_CAP_DMABUF_INPUT;
        caps->hw_revision = 10;
        return 0;
    }
    case CAM_FACE_ACCEL_IOC_RUN: {
        struct cam_face_accel_run *run = arg;

        dev.runs++;
        fill_output((void *)(uintptr_t)run->output_ptr, run->frame_id);
        fill_result(&run->result, run->frame_id);
        return 0;
    }
    case CAM_FACE_ACCEL_IOC_IMPORT: {
        struct cam_face_accel_import *req = arg;

        if (dev.import_errno) {
            errno = dev.import_errno;
            return -1;
        }
        dev.last_fd = req->fd;
        req->handle = 3;
        return 0;
    }
    case CAM_FACE_ACCEL_IOC_SUBMIT: {
        const struct cam_face_accel_submit *sub = arg;

        dev.submits++;
        if (dev.submit_errno) {
            errno = dev.submit_errno;
            return -1;
        }
        dev.last_handle = sub->input_handle;
        dev.last_frame = sub->frame_id;
        return 0;
    }
    case CAM_FACE_ACCEL_IOC_WAIT: {
        struct cam_face_accel_wait *wait = arg;

        dev.waits++;
        if (dev.wait_errno) {
            wait->result.status = 8u;
            errno = dev.wait_errno;
            return -1;
        }
        fill_output((void *)(uintptr_t)wait->output_ptr, dev.last_frame);
        fill_result(&wait->result, dev.last_frame);
        return 0;
    }
    case CAM_FACE_ACCEL_IOC_RESET:
        return 0;
    }
    errno = ENOTTY;
    return -1;
}

#define open fake_open
#define close fake_close
#define ioctl fake_ioctl
#include "vision/face_accel_client.c"
#undef open
#undef close
#undef ioctl

static int g_failures;

#define CHECK(cond) do { \
    if (!(cond)) { \
        fprintf(stderr, "FAIL %s:%d: %s\n", __func__, __LINE__, #cond); \
        g_failures++; \
    } \
} while (0)

#define RUN_ARGS 0.7f, 0.3f, faces, UF_CANDIDATE_LIMIT, &metrics

static void test_old_driver(void)
{
    static uint8_t rgb[CAM_FACE_ACCEL_INPUT_SIZE];
    uf_face_t faces[UF_CANDIDATE_LIMIT];
    face_accel_metrics_t metrics;
    face_accel_client_t *client = NULL;

    memset(&dev, 0, sizeof(dev));
    CHECK(face_accel_client_open(&client, NULL) == 0);
    CHECK(face_accel_client_caps(client)->uapi_version == 1);
    CHECK(!(face_accel_client_caps(client)->flags & CAM_FACE_ACCEL_CAP_DMABUF_INPUT));
    CHECK(face_accel_client_run(client, rgb, sizeof(rgb), 5, 250, RUN_ARGS) == 0);
    CHECK(dev.runs == 1 && metrics.perf_cycles == 2200000);
    face_accel_client_close(client);
}

static void test_imported_job(void)
{
    uf_face_t faces[UF_CANDIDATE_LIMIT];
    face_accel_metrics_t metrics;
    face_accel_client_t *client = NULL;
    uint32_t handle = 0;

    memset(&dev, 0, sizeof(dev));
    dev.has_queue_interface = 1;
    CHECK(face_accel_client_open(&client, NULL) == 0);
    CHECK(face_accel_client_caps(client)->uapi_version == 2);
    CHECK(face_accel_client_caps(client)->hw_revision == 10);
    CHECK(face_accel_client_import(client, 11, &handle) == 0);
    CHECK(handle == 3 && dev.last_fd == 11);

    CHECK(face_accel_client_run_imported(client, handle, 9, 250, RUN_ARGS) == 0);
    CHECK(dev.submits == 1 && dev.waits == 1 && dev.last_handle == 3 && dev.last_frame == 9);
    CHECK(metrics.perf_cycles == 2200000);
    CHECK(memcmp(face_accel_client_last_output(client), &(uint32_t){FA_OUT_MAGIC}, 4) == 0);

    /* a refused SUBMIT does not wait */
    dev.submit_errno = EBUSY;
    CHECK(face_accel_client_run_imported(client, handle, 10, 250, RUN_ARGS) == -EBUSY);
    CHECK(dev.submits == 2 && dev.waits == 1);
    dev.submit_errno = 0;

    dev.wait_errno = ETIMEDOUT;
    CHECK(face_accel_client_run_imported(client, handle, 11, 250, RUN_ARGS) == -ETIMEDOUT);
    dev.wait_errno = 0;

    dev.stale_output = 1;
    CHECK(face_accel_client_run_imported(client, handle, 12, 250, RUN_ARGS) == -EBADMSG);
    dev.stale_output = 0;

    CHECK(face_accel_client_run_imported(client, 0, 13, 250, RUN_ARGS) == -EINVAL);

    dev.import_errno = EOPNOTSUPP;
    CHECK(face_accel_client_import(client, 11, &handle) == -EOPNOTSUPP);
    CHECK(face_accel_client_import(client, -1, &handle) == -EINVAL);
    face_accel_client_close(client);
}

int main(void)
{
    test_old_driver();
    test_imported_job();
    if (g_failures) {
        fprintf(stderr, "face_accel_client_test: %d failure(s)\n", g_failures);
        return 1;
    }
    puts("face_accel_client_test: OK");
    return 0;
}
