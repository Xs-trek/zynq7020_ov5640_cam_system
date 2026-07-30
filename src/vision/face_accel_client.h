#ifndef CAM_SYSTEM_FACE_ACCEL_CLIENT_H
#define CAM_SYSTEM_FACE_ACCEL_CLIENT_H

#include <stddef.h>
#include <stdint.h>

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
void face_accel_client_close(face_accel_client_t *client);

#ifdef __cplusplus
}
#endif

#endif
