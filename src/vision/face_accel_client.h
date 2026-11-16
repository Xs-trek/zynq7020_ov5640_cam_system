#ifndef CAM_SYSTEM_FACE_ACCEL_CLIENT_H
#define CAM_SYSTEM_FACE_ACCEL_CLIENT_H

#include <stddef.h>
#include <stdint.h>

#include "cam_system/face_accel_uapi.h"
#include "ultraface_decode.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef struct face_accel_client face_accel_client_t;

typedef struct {
    uint32_t status;
    uint32_t error_code;
    uint32_t perf_cycles;
    uint32_t debug_progress;
} face_accel_metrics_t;

int face_accel_client_open(face_accel_client_t **client, const char *device);

/* The driver's capabilities.  Against a driver without the queue interface, uapi_version is 1 and
 * everything else is zero; face_accel_client_run still works. */
const struct cam_face_accel_caps *face_accel_client_caps(const face_accel_client_t *client);

/* Runs one job on a frame copied through the driver. */
int face_accel_client_run(face_accel_client_t *client,
                          const uint8_t *rgb,
                          size_t rgb_size,
                          uint32_t frame_id,
                          uint32_t timeout_ms,
                          float score_threshold,
                          float nms_threshold,
                          uf_face_t *faces,
                          size_t face_capacity,
                          face_accel_metrics_t *metrics);

/* Imports a dma-buf holding input frames.  The handle lasts until the client closes. */
int face_accel_client_import(face_accel_client_t *client, int dmabuf_fd, uint32_t *handle);

/* Runs one job on an imported buffer, which must stay untouched until this returns. */
int face_accel_client_run_imported(face_accel_client_t *client,
                                   uint32_t handle,
                                   uint32_t frame_id,
                                   uint32_t timeout_ms,
                                   float score_threshold,
                                   float nms_threshold,
                                   uf_face_t *faces,
                                   size_t face_capacity,
                                   face_accel_metrics_t *metrics);

/* The output of the last successful job, CAM_FACE_ACCEL_OUTPUT_SIZE bytes. */
const uint8_t *face_accel_client_last_output(const face_accel_client_t *client);

int face_accel_client_summarize_last(face_accel_client_t *client,
                                     uf_tensor_summary_t *summary);
int face_accel_client_reset(face_accel_client_t *client);
void face_accel_client_close(face_accel_client_t *client);

#ifdef __cplusplus
}
#endif

#endif
