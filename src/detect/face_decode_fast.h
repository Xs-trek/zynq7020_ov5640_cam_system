/* face_decode_fast.h - decode of the accelerator output with an integer prefilter.
 *
 * Same contract and bit-identical output as uf_decode_nms_s8 (third_party/face_accel_ref), which
 * stays the reference.  Most priors are far below the score threshold, so the logistic function
 * is evaluated only for priors whose quantized logit difference can reach it.
 */
#ifndef CAM_FACE_DECODE_FAST_H
#define CAM_FACE_DECODE_FAST_H

#include <stddef.h>
#include <stdint.h>

#include "ultraface_decode.h"

#ifdef __cplusplus
extern "C" {
#endif

/* Priors with q_face - q_background below this value cannot reach score_threshold. */
int face_decode_min_score_delta(float score_threshold);

int face_decode_nms_s8(const int8_t *scores,
                       const int8_t *boxes,
                       float score_threshold,
                       float nms_threshold,
                       uf_face_t *faces,
                       size_t face_capacity);

#ifdef __cplusplus
}
#endif

#endif /* CAM_FACE_DECODE_FAST_H */
