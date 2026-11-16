#define LOG_TAG "face-accel"

#include "vision/face_accel_client.h"

#include "cam_system/face_accel_uapi.h"
#include "detect/face_decode_fast.h"
#include "face_accel_regs.h"
#include "common/log.h"

#include <errno.h>
#include <fcntl.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/ioctl.h>
#include <unistd.h>

struct face_accel_client {
    int fd;
    struct cam_face_accel_caps caps;
    uint8_t output[CAM_FACE_ACCEL_OUTPUT_CAPACITY];
};

static int validate_info(const struct cam_face_accel_info *info)
{
    const uint32_t required_flags = CAM_FACE_ACCEL_INFO_WEIGHT_READY |
                                    CAM_FACE_ACCEL_INFO_IRQ_READY;
    return info->uapi_version == CAM_FACE_ACCEL_INFO_VERSION &&
           info->ip_id == CAM_FACE_ACCEL_IP_ID &&
           info->input_width == CAM_FACE_ACCEL_INPUT_WIDTH &&
           info->input_height == CAM_FACE_ACCEL_INPUT_HEIGHT &&
           info->input_stride == CAM_FACE_ACCEL_INPUT_STRIDE &&
           info->input_size == CAM_FACE_ACCEL_INPUT_SIZE &&
           info->output_size == CAM_FACE_ACCEL_OUTPUT_SIZE &&
           info->output_capacity >= CAM_FACE_ACCEL_OUTPUT_CAPACITY &&
           info->weight_size == CAM_FACE_ACCEL_WEIGHT_SIZE &&
           info->scratch_size >= CAM_FACE_ACCEL_SCRATCH_SIZE &&
           (info->flags & required_flags) == required_flags;
}

static int validate_output(const uint8_t *output, uint32_t frame_id)
{
    fa_out_header_t header;
    fa_tensor_desc_t desc[2];
    const float score_scale = FA_OUT_SCORES_SCALE;
    const float box_scale = FA_OUT_BOXES_SCALE;

    memcpy(&header, output, sizeof(header));
    memcpy(desc, output + sizeof(header), sizeof(desc));

    if (header.magic != FA_OUT_MAGIC || header.frame_id != frame_id ||
        header.tensor_count != FA_OUT_TENSOR_COUNT ||
        header.total_len != FA_OUT_LEN_V1)
        return -1;
    if (desc[0].offset != FA_OUT_SCORES_OFFSET ||
        desc[0].dtype != FA_DTYPE_INT8 ||
        desc[0].dims[0] != 1u || desc[0].dims[1] != UF_PRIOR_COUNT ||
        desc[0].dims[2] != 2u || desc[0].dims[3] != 0u ||
        memcmp(&desc[0].scale, &score_scale, sizeof(score_scale)) != 0 ||
        desc[0].zero_point != FA_OUT_ZERO_POINT ||
        desc[1].offset != FA_OUT_BOXES_OFFSET ||
        desc[1].dtype != FA_DTYPE_INT8 ||
        desc[1].dims[0] != 1u || desc[1].dims[1] != UF_PRIOR_COUNT ||
        desc[1].dims[2] != 4u || desc[1].dims[3] != 0u ||
        memcmp(&desc[1].scale, &box_scale, sizeof(box_scale)) != 0 ||
        desc[1].zero_point != FA_OUT_ZERO_POINT)
        return -1;
    return 0;
}

int face_accel_client_open(face_accel_client_t **client, const char *device)
{
    struct cam_face_accel_info info;
    face_accel_client_t *ctx;

    if (!client)
        return -EINVAL;
    *client = NULL;
    ctx = (face_accel_client_t *)calloc(1, sizeof(*ctx));
    if (!ctx)
        return -ENOMEM;
    ctx->fd = open(device ? device : CAM_FACE_ACCEL_DEVICE,
                   O_RDWR | O_CLOEXEC);
    if (ctx->fd < 0) {
        int error = -errno;
        free(ctx);
        return error;
    }
    memset(&info, 0, sizeof(info));
    if (ioctl(ctx->fd, CAM_FACE_ACCEL_IOC_GET_INFO, &info) < 0) {
        int error = -errno;
        close(ctx->fd);
        free(ctx);
        return error;
    }
    if (!validate_info(&info)) {
        LOG_E("incompatible driver uapi=%u ip=%#x flags=%#x",
              info.uapi_version, info.ip_id, info.flags);
        close(ctx->fd);
        free(ctx);
        return -EPROTO;
    }
    if (ioctl(ctx->fd, CAM_FACE_ACCEL_IOC_GET_CAPS, &ctx->caps) < 0) {
        int error = errno;

        memset(&ctx->caps, 0, sizeof(ctx->caps));
        ctx->caps.uapi_version = CAM_FACE_ACCEL_INFO_VERSION;
        if (error != ENOTTY)
            LOG_W("GET_CAPS failed: %s", strerror(error));
    }
    LOG_I("ready device=%s ip=%#x input=%ux%u RGB24 uapi=%u hw_revision=%u",
          device ? device : CAM_FACE_ACCEL_DEVICE, info.ip_id,
          info.input_width, info.input_height,
          ctx->caps.uapi_version, ctx->caps.hw_revision);
    *client = ctx;
    return 0;
}

const struct cam_face_accel_caps *face_accel_client_caps(const face_accel_client_t *client)
{
    return &client->caps;
}

/* After the driver returned the job's result: checks the output and decodes it. */
static int finish_job(face_accel_client_t *client,
                      const struct cam_face_accel_result *result,
                      uint32_t frame_id,
                      float score_threshold,
                      float nms_threshold,
                      uf_face_t *faces,
                      size_t face_capacity,
                      face_accel_metrics_t *metrics)
{
    int face_count;

    if (result->output_size != CAM_FACE_ACCEL_OUTPUT_SIZE ||
        result->frame_id_done != frame_id ||
        validate_output(client->output, frame_id) != 0)
        return -EBADMSG;
    if (metrics) {
        memset(metrics, 0, sizeof(*metrics));
        metrics->status = result->status;
        metrics->error_code = result->error_code;
        metrics->perf_cycles = result->perf_cycles;
        metrics->debug_progress = result->debug_progress;
    }
    face_count = face_decode_nms_s8(
        (const int8_t *)(client->output + FA_OUT_SCORES_OFFSET),
        (const int8_t *)(client->output + FA_OUT_BOXES_OFFSET),
        score_threshold, nms_threshold, faces, face_capacity);
    return face_count < 0 ? -EBADMSG : face_count;
}

static void log_job_failure(const char *what, int error, uint32_t frame_id,
                            const struct cam_face_accel_result *result)
{
    LOG_E_RL(1.0, 5, "%s failed: %s frame=%u done=%u status=%#x "
                     "error=%#x debug=%#x dma=%#x transport=%#x "
                     "read=%#x write=%#x",
                     what, strerror(error), frame_id, result->frame_id_done, result->status,
                     result->error_code, result->debug_progress, result->debug_dma,
                     result->debug_transport, result->debug_dma_read,
                     result->debug_dma_write);
}

int face_accel_client_run(face_accel_client_t *client,
                          const uint8_t *rgb,
                          size_t rgb_size,
                          uint32_t frame_id,
                          uint32_t timeout_ms,
                          float score_threshold,
                          float nms_threshold,
                          uf_face_t *faces,
                          size_t face_capacity,
                          face_accel_metrics_t *metrics)
{
    struct cam_face_accel_run run;

    if (!client || !rgb || rgb_size != CAM_FACE_ACCEL_INPUT_SIZE ||
        !faces || face_capacity == 0u)
        return -EINVAL;
    memset(&run, 0, sizeof(run));
    run.input_ptr = (uintptr_t)rgb;
    run.output_ptr = (uintptr_t)client->output;
    run.input_size = (uint32_t)rgb_size;
    run.output_capacity = sizeof(client->output);
    run.frame_id = frame_id;
    run.timeout_ms = timeout_ms;
    if (ioctl(client->fd, CAM_FACE_ACCEL_IOC_RUN, &run) < 0) {
        int error = errno;

        log_job_failure("RUN", error, frame_id, &run.result);
        return -error;
    }
    return finish_job(client, &run.result, frame_id, score_threshold, nms_threshold,
                      faces, face_capacity, metrics);
}

int face_accel_client_import(face_accel_client_t *client, int dmabuf_fd, uint32_t *handle)
{
    struct cam_face_accel_import req;

    if (!client || dmabuf_fd < 0 || !handle)
        return -EINVAL;
    memset(&req, 0, sizeof(req));
    req.fd = dmabuf_fd;
    if (ioctl(client->fd, CAM_FACE_ACCEL_IOC_IMPORT, &req) < 0)
        return -errno;
    *handle = req.handle;
    return 0;
}

int face_accel_client_run_imported(face_accel_client_t *client,
                                   uint32_t handle,
                                   uint32_t frame_id,
                                   uint32_t timeout_ms,
                                   float score_threshold,
                                   float nms_threshold,
                                   uf_face_t *faces,
                                   size_t face_capacity,
                                   face_accel_metrics_t *metrics)
{
    struct cam_face_accel_submit submit;
    struct cam_face_accel_wait wait;

    if (!client || handle == 0u || !faces || face_capacity == 0u)
        return -EINVAL;
    memset(&submit, 0, sizeof(submit));
    submit.input_handle = handle;
    submit.frame_id = frame_id;
    submit.timeout_ms = timeout_ms;
    if (ioctl(client->fd, CAM_FACE_ACCEL_IOC_SUBMIT, &submit) < 0) {
        int error = errno;

        LOG_E_RL(1.0, 5, "SUBMIT failed: %s frame=%u", strerror(error), frame_id);
        return -error;
    }
    memset(&wait, 0, sizeof(wait));
    wait.output_ptr = (uintptr_t)client->output;
    wait.output_capacity = sizeof(client->output);
    if (ioctl(client->fd, CAM_FACE_ACCEL_IOC_WAIT, &wait) < 0) {
        int error = errno;

        log_job_failure("WAIT", error, frame_id, &wait.result);
        return -error;
    }
    return finish_job(client, &wait.result, frame_id, score_threshold, nms_threshold,
                      faces, face_capacity, metrics);
}

const uint8_t *face_accel_client_last_output(const face_accel_client_t *client)
{
    return client->output;
}

int face_accel_client_summarize_last(face_accel_client_t *client,
                                     uf_tensor_summary_t *summary)
{
    if (!client || !summary)
        return -EINVAL;
    if (uf_summarize_tensors_s8(
            (const int8_t *)(client->output + FA_OUT_SCORES_OFFSET),
            (const int8_t *)(client->output + FA_OUT_BOXES_OFFSET),
            summary) != 0)
        return -EBADMSG;
    return 0;
}

int face_accel_client_reset(face_accel_client_t *client)
{
    if (!client)
        return -EINVAL;
    if (ioctl(client->fd, CAM_FACE_ACCEL_IOC_RESET) < 0)
        return -errno;
    return 0;
}

void face_accel_client_close(face_accel_client_t *client)
{
    if (!client)
        return;
    if (client->fd >= 0)
        close(client->fd);
    free(client);
}
