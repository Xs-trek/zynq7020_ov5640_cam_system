#ifndef ULTRAFACE_DECODE_H
#define ULTRAFACE_DECODE_H

#include <stddef.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

#define UF_PRIOR_COUNT 4420u
#define UF_SCORE_BYTES (UF_PRIOR_COUNT * 2u)
#define UF_BOX_BYTES (UF_PRIOR_COUNT * 4u)
#define UF_CANDIDATE_LIMIT 200u

#define UF_SCORE_SCALE 0.06540081650018692f
#define UF_BOX_SCALE 0.062080398201942444f
#define UF_DEFAULT_SCORE_THRESHOLD 0.80f
#define UF_DEFAULT_NMS_THRESHOLD 0.30f

typedef struct {
    float x;
    float y;
    float w;
    float h;
    float confidence;
    int32_t class_id;
} uf_face_t;

/*
 * Compact, image-free diagnostics for one raw accelerator result.  The hash
 * covers all score and box tensor bytes (but not the frame-dependent output
 * header), so identical inputs can be checked for bit-exact repeatability.
 */
typedef struct {
    uint64_t tensor_hash;
    float top_confidence;
    uint16_t top_prior_index;
    int8_t top_background_q;
    int8_t top_face_q;
    int8_t top_box_q[4];
} uf_tensor_summary_t;

int uf_summarize_tensors_s8(const int8_t *scores,
                            const int8_t *boxes,
                            uf_tensor_summary_t *summary);

/*
 * Decode row-major signed-int8 tensors:
 *   scores[4420][2], boxes[4420][4].
 * Returned x/y/w/h are clipped and normalized to the 640x480 input frame.
 * Candidate order is score descending, then prior index ascending.
 * Returns face count, or -1 for invalid arguments.
 */
int uf_decode_nms_s8(const int8_t *scores,
                     const int8_t *boxes,
                     float score_threshold,
                     float nms_threshold,
                     uf_face_t *faces,
                     size_t face_capacity);

#ifdef __cplusplus
}
#endif

#endif
