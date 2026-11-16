#include "ultraface_decode.h"

#include <math.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>

static int expect(int condition, const char *message)
{
    if (!condition) {
        fprintf(stderr, "FAIL: %s\n", message);
        return 1;
    }
    return 0;
}

int main(void)
{
    int8_t scores[UF_SCORE_BYTES];
    int8_t boxes[UF_BOX_BYTES];
    uf_tensor_summary_t zero_summary;
    uf_tensor_summary_t face_summary;
    uf_tensor_summary_t changed_summary;
    int failed = 0;

    memset(scores, 0, sizeof(scores));
    memset(boxes, 0, sizeof(boxes));
    failed |= expect(uf_summarize_tensors_s8(scores, boxes, &zero_summary) == 0,
                     "zero tensor summarizes");
    failed |= expect(zero_summary.top_prior_index == 0u,
                     "equal scores use the first prior");
    failed |= expect(fabsf(zero_summary.top_confidence - 0.5f) < 1.0e-6f,
                     "zero logits decode to confidence 0.5");

    scores[2u * 17u] = -10;
    scores[2u * 17u + 1u] = 10;
    boxes[4u * 17u] = 1;
    boxes[4u * 17u + 1u] = -2;
    boxes[4u * 17u + 2u] = 3;
    boxes[4u * 17u + 3u] = -4;
    failed |= expect(uf_summarize_tensors_s8(scores, boxes, &face_summary) == 0,
                     "nonzero tensor summarizes");
    failed |= expect(face_summary.top_prior_index == 17u,
                     "highest face logit is selected");
    failed |= expect(face_summary.top_background_q == -10 &&
                     face_summary.top_face_q == 10,
                     "top quantized scores are preserved");
    failed |= expect(face_summary.top_box_q[0] == 1 &&
                     face_summary.top_box_q[1] == -2 &&
                     face_summary.top_box_q[2] == 3 &&
                     face_summary.top_box_q[3] == -4,
                     "top quantized box is preserved");
    failed |= expect(face_summary.tensor_hash != zero_summary.tensor_hash,
                     "score/box mutation changes tensor hash");

    boxes[0] = 1;
    failed |= expect(uf_summarize_tensors_s8(scores, boxes, &changed_summary) == 0,
                     "changed tensor summarizes");
    failed |= expect(changed_summary.top_prior_index == face_summary.top_prior_index,
                     "unrelated box does not alter top prior");
    failed |= expect(changed_summary.tensor_hash != face_summary.tensor_hash,
                     "full tensor hash includes every box byte");

    failed |= expect(uf_summarize_tensors_s8(NULL, boxes, &changed_summary) < 0,
                     "null scores are rejected");

    if (!failed)
        printf("face accelerator diagnostics tests passed\n");
    return failed ? 1 : 0;
}
