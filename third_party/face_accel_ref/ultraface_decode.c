#include "ultraface_decode.h"

#include <math.h>

typedef struct {
    float x1;
    float y1;
    float x2;
    float y2;
    float score;
    uint16_t prior_index;
} uf_candidate_t;

static float uf_clip01(float value)
{
    if (value < 0.0f) {
        return 0.0f;
    }
    if (value > 1.0f) {
        return 1.0f;
    }
    return value;
}

static int uf_candidate_precedes(float score, uint16_t prior_index,
                                 const uf_candidate_t *other)
{
    return score > other->score ||
           (score == other->score && prior_index < other->prior_index);
}

static void uf_insert_candidate(uf_candidate_t *items, size_t *count,
                                const uf_candidate_t *candidate)
{
    size_t position = 0u;
    size_t limit = *count;

    if (limit > UF_CANDIDATE_LIMIT) {
        limit = UF_CANDIDATE_LIMIT;
    }
    for (position = 0; position < limit; ++position) {
        if (uf_candidate_precedes(candidate->score,
                                  candidate->prior_index,
                                  &items[position])) {
            break;
        }
    }
    if (position >= UF_CANDIDATE_LIMIT) {
        return;
    }
    if (*count < UF_CANDIDATE_LIMIT) {
        ++(*count);
    }
    for (size_t index = *count - 1u; index > position; --index) {
        items[index] = items[index - 1u];
    }
    items[position] = *candidate;
}

static float uf_iou(const uf_candidate_t *lhs, const uf_candidate_t *rhs)
{
    const float ix1 = lhs->x1 > rhs->x1 ? lhs->x1 : rhs->x1;
    const float iy1 = lhs->y1 > rhs->y1 ? lhs->y1 : rhs->y1;
    const float ix2 = lhs->x2 < rhs->x2 ? lhs->x2 : rhs->x2;
    const float iy2 = lhs->y2 < rhs->y2 ? lhs->y2 : rhs->y2;
    const float iw = ix2 > ix1 ? ix2 - ix1 : 0.0f;
    const float ih = iy2 > iy1 ? iy2 - iy1 : 0.0f;
    const float lw = lhs->x2 > lhs->x1 ? lhs->x2 - lhs->x1 : 0.0f;
    const float lh = lhs->y2 > lhs->y1 ? lhs->y2 - lhs->y1 : 0.0f;
    const float rw = rhs->x2 > rhs->x1 ? rhs->x2 - rhs->x1 : 0.0f;
    const float rh = rhs->y2 > rhs->y1 ? rhs->y2 - rhs->y1 : 0.0f;
    const float intersection = iw * ih;
    return intersection / (lw * lh + rw * rh - intersection + 1.0e-6f);
}

int uf_decode_nms_s8(const int8_t *scores,
                     const int8_t *boxes,
                     float score_threshold,
                     float nms_threshold,
                     uf_face_t *faces,
                     size_t face_capacity)
{
    static const unsigned int strides[4] = {8u, 16u, 32u, 64u};
    static const unsigned int size_counts[4] = {3u, 2u, 2u, 3u};
    static const float min_boxes[4][3] = {
        {10.0f, 16.0f, 24.0f},
        {32.0f, 48.0f, 0.0f},
        {64.0f, 96.0f, 0.0f},
        {128.0f, 192.0f, 256.0f},
    };
    uf_candidate_t candidates[UF_CANDIDATE_LIMIT];
    uint16_t kept_indices[UF_CANDIDATE_LIMIT];
    size_t candidate_count = 0u;
    uint16_t prior_index = 0u;

    if (scores == NULL || boxes == NULL || faces == NULL ||
        face_capacity == 0u || score_threshold < 0.0f ||
        score_threshold > 1.0f || nms_threshold < 0.0f ||
        nms_threshold > 1.0f) {
        return -1;
    }

    for (unsigned int level = 0u; level < 4u; ++level) {
        const unsigned int feature_w = (320u + strides[level] - 1u) / strides[level];
        const unsigned int feature_h = (240u + strides[level] - 1u) / strides[level];
        for (unsigned int row = 0u; row < feature_h; ++row) {
            for (unsigned int col = 0u; col < feature_w; ++col) {
                for (unsigned int size_index = 0u;
                     size_index < size_counts[level]; ++size_index, ++prior_index) {
                    const float score0 = (float)scores[2u * prior_index] * UF_SCORE_SCALE;
                    const float score1 = (float)scores[2u * prior_index + 1u] * UF_SCORE_SCALE;
                    const float confidence = 1.0f / (1.0f + expf(score0 - score1));
                    uf_candidate_t candidate;
                    float center_x;
                    float center_y;
                    float width;
                    float height;

                    if (confidence < score_threshold) {
                        continue;
                    }
                    center_x = ((float)col + 0.5f) / (float)feature_w;
                    center_y = ((float)row + 0.5f) / (float)feature_h;
                    width = min_boxes[level][size_index] / 320.0f;
                    height = min_boxes[level][size_index] / 240.0f;
                    center_x += (float)boxes[4u * prior_index] * UF_BOX_SCALE *
                                0.1f * width;
                    center_y += (float)boxes[4u * prior_index + 1u] * UF_BOX_SCALE *
                                0.1f * height;
                    width *= expf((float)boxes[4u * prior_index + 2u] *
                                  UF_BOX_SCALE * 0.2f);
                    height *= expf((float)boxes[4u * prior_index + 3u] *
                                   UF_BOX_SCALE * 0.2f);
                    candidate.x1 = center_x - width * 0.5f;
                    candidate.y1 = center_y - height * 0.5f;
                    candidate.x2 = center_x + width * 0.5f;
                    candidate.y2 = center_y + height * 0.5f;
                    candidate.score = confidence;
                    candidate.prior_index = prior_index;
                    uf_insert_candidate(candidates, &candidate_count, &candidate);
                }
            }
        }
    }

    if (prior_index != UF_PRIOR_COUNT) {
        return -1;
    }

    size_t face_count = 0u;
    for (size_t index = 0u;
         index < candidate_count && face_count < face_capacity;
         ++index) {
        int keep = 1;
        for (size_t accepted = 0u; accepted < face_count; ++accepted) {
            if (uf_iou(&candidates[index],
                       &candidates[kept_indices[accepted]]) > nms_threshold) {
                keep = 0;
                break;
            }
        }
        if (keep) {
            const float x1 = uf_clip01(candidates[index].x1);
            const float y1 = uf_clip01(candidates[index].y1);
            const float x2 = uf_clip01(candidates[index].x2);
            const float y2 = uf_clip01(candidates[index].y2);
            faces[face_count].x = x1;
            faces[face_count].y = y1;
            faces[face_count].w = x2 > x1 ? x2 - x1 : 0.0f;
            faces[face_count].h = y2 > y1 ? y2 - y1 : 0.0f;
            faces[face_count].confidence = candidates[index].score;
            faces[face_count].class_id = 0;
            kept_indices[face_count] = (uint16_t)index;
            ++face_count;
        }
    }
    return (int)face_count;
}
