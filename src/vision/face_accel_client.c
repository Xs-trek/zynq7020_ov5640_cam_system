#include "vision/face_accel_client.h"

#include "cam_system/face_accel_uapi.h"
#include "face_accel_regs.h"

#include <errno.h>
#include <fcntl.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/ioctl.h>
#include <unistd.h>

struct face_accel_client {
    int fd;
    uint8_t output[CAM_FACE_ACCEL_OUTPUT_CAPACITY];
};

static int validate_info(const struct cam_face_accel_info *info)
{
    const uint32_t required_flags = CAM_FACE_ACCEL_INFO_WEIGHT_READY |
                                    CAM_FACE_ACCEL_INFO_IRQ_READY;
    return info->uapi_version == CAM_FACE_ACCEL_UAPI_VERSION &&
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
        fprintf(stderr,
                "[face-accel] incompatible driver uapi=%u ip=%#x flags=%#x\n",
                info.uapi_version, info.ip_id, info.flags);
        close(ctx->fd);
        free(ctx);
        return -EPROTO;
    }
    printf("[face-accel] ready device=%s ip=%#x input=%ux%u RGB24\n",
           device ? device : CAM_FACE_ACCEL_DEVICE, info.ip_id,
           info.input_width, info.input_height);
    *client = ctx;
    return 0;
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
    int face_count;

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
        int error = -errno;
        fprintf(stderr,
                "[face-accel] RUN failed: %s frame=%u done=%u status=%#x "
                "error=%#x debug=%#x dma=%#x transport=%#x "
                "read=%#x write=%#x\n",
                strerror(errno), frame_id, run.frame_id_done, run.status,
                run.error_code, run.debug_progress, run.debug_dma,
                run.debug_transport, run.debug_dma_read,
                run.debug_dma_write);
        return error;
    }
    if (run.output_size != CAM_FACE_ACCEL_OUTPUT_SIZE ||
        run.frame_id_done != frame_id ||
        validate_output(client->output, frame_id) != 0)
        return -EBADMSG;
    if (metrics) {
        memset(metrics, 0, sizeof(*metrics));
        metrics->status = run.status;
        metrics->error_code = run.error_code;
        metrics->perf_cycles = run.perf_cycles;
        metrics->debug_progress = run.debug_progress;
    }
    face_count = uf_decode_nms_s8(
        (const int8_t *)(client->output + FA_OUT_SCORES_OFFSET),
        (const int8_t *)(client->output + FA_OUT_BOXES_OFFSET),
        score_threshold, nms_threshold, faces, face_capacity);
    return face_count < 0 ? -EBADMSG : face_count;
}

void face_accel_client_close(face_accel_client_t *client)
{
    if (!client)
        return;
    if (client->fd >= 0)
        close(client->fd);
    free(client);
}
