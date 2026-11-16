/* face_decode_fast_test.c - face_decode_nms_s8 against the reference uf_decode_nms_s8, byte for byte.
 *
 * Covers the prefilter over every (q_background, q_face) pair, the three delivery vectors and
 * random tensors shaped to reach the candidate limit, score ties and saturated values.
 * Usage: face_decode_fast_test path/to/face_accel_vectors
 */
#define _POSIX_C_SOURCE 200809L

#include "detect/face_decode_fast.h"

#include <limits.h>
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#define OUT_SCORES_OFFSET 128u
#define OUT_BOXES_OFFSET 9024u
#define OUT_LEN 26704u
#define DELTA_COUNT 511

static int g_checks;
static unsigned long long g_compared_faces;
static uint32_t g_rng = 0x2f6e2b1u;

static void require(int cond, const char *what)
{
    g_checks++;
    if (!cond) {
        fprintf(stderr, "face_decode_fast_test: FAIL: %s\n", what);
        exit(1);
    }
}

static uint32_t rng(void)
{
    /* xorshift32: reproducible across hosts and targets */
    g_rng ^= g_rng << 13;
    g_rng ^= g_rng >> 17;
    g_rng ^= g_rng << 5;
    return g_rng;
}

static int rng_range(int lo, int hi)
{
    return lo + (int)(rng() % (uint32_t)(hi - lo + 1));
}

/* Largest confidence the reference computes for each q_face - q_background. */
static float g_max_conf[DELTA_COUNT];

static void build_max_conf(void)
{
    for (int i = 0; i < DELTA_COUNT; i++)
        g_max_conf[i] = -1.0f;
    for (int q0 = -128; q0 <= 127; q0++) {
        for (int q1 = -128; q1 <= 127; q1++) {
            const float score0 = (float)q0 * UF_SCORE_SCALE;
            const float score1 = (float)q1 * UF_SCORE_SCALE;
            const float confidence = 1.0f / (1.0f + expf(score0 - score1));
            const int index = q1 - q0 + 255;

            if (confidence > g_max_conf[index])
                g_max_conf[index] = confidence;
        }
    }
}

/* The cut must never reject a pair the reference keeps. */
static void check_cut(float threshold)
{
    const int cut = face_decode_min_score_delta(threshold);

    for (int delta = -255; delta < cut && delta <= 255; delta++) {
        if (g_max_conf[delta + 255] >= threshold) {
            fprintf(stderr, "threshold %.9g cut %d rejects delta %d (conf %.9g)\n", (double)threshold, cut, delta,
                    (double)g_max_conf[delta + 255]);
            require(0, "prefilter rejected a passing pair");
        }
    }
    g_checks++;
}

static void test_prefilter(void)
{
    static const float edges[] = {0.0f, 1.0e-40f, 1.0e-6f, 0.1f, 0.5f, 0.6f, 0.8f, 0.9f, 0.99f, 0.999f, 0.9999f,
                                  0.99999f, 0.999995f, 0.999999f, 0.9999999f, 1.0f};

    build_max_conf();
    for (size_t i = 0; i < sizeof(edges) / sizeof(edges[0]); i++) {
        check_cut(edges[i]);
        check_cut(nextafterf(edges[i], 0.0f));
        check_cut(nextafterf(edges[i], 1.0f));
    }
    for (int i = 0; i < 200000; i++)
        check_cut((float)rng() / 4294967296.0f);
    /* The 1 - t region where rounding matters. */
    for (int i = 0; i < 200000; i++)
        check_cut(1.0f - ldexpf((float)rng_range(1, 1 << 20), -24));

    require(face_decode_min_score_delta(0.0f) == INT_MIN, "threshold 0 keeps everything");
    require(face_decode_min_score_delta(nanf("")) == INT_MIN, "NaN threshold keeps everything");

    /* The filter must also be useful: at most two steps below the exact cut. */
    for (float t = 0.05f; t < 0.995f; t += 0.01f) {
        int exact = 256;

        for (int delta = 255; delta >= -255; delta--)
            if (g_max_conf[delta + 255] >= t)
                exact = delta;
        require(face_decode_min_score_delta(t) >= exact - 2, "prefilter stays tight");
    }
}

static void compare(const int8_t *scores, const int8_t *boxes, float score, float nms, size_t capacity)
{
    uf_face_t ref[UF_CANDIDATE_LIMIT];
    uf_face_t fast[UF_CANDIDATE_LIMIT];
    int ref_count;
    int fast_count;

    memset(ref, 0x5a, sizeof(ref));
    memset(fast, 0x5a, sizeof(fast));
    ref_count = uf_decode_nms_s8(scores, boxes, score, nms, ref, capacity);
    fast_count = face_decode_nms_s8(scores, boxes, score, nms, fast, capacity);
    if (ref_count != fast_count || memcmp(ref, fast, sizeof(ref)) != 0) {
        fprintf(stderr, "mismatch score=%.9g nms=%.9g capacity=%zu counts %d %d\n", (double)score, (double)nms,
                capacity, ref_count, fast_count);
        require(0, "decode differs from the reference");
    }
    g_compared_faces += ref_count > 0 ? (unsigned long long)ref_count : 0u;
    g_checks++;
}

static void compare_grid(const int8_t *scores, const int8_t *boxes)
{
    static const float score_grid[] = {0.0f, 0.1f, 0.5f, 0.6f, 0.8f, 0.95f, 0.99999f, 0.9999999f, 1.0f};
    static const float nms_grid[] = {0.0f, 0.3f, 0.5f, 1.0f};
    static const size_t capacity_grid[] = {1u, 2u, 10u, UF_CANDIDATE_LIMIT};

    for (size_t s = 0; s < sizeof(score_grid) / sizeof(score_grid[0]); s++)
        for (size_t n = 0; n < sizeof(nms_grid) / sizeof(nms_grid[0]); n++)
            for (size_t c = 0; c < sizeof(capacity_grid) / sizeof(capacity_grid[0]); c++)
                compare(scores, boxes, score_grid[s], nms_grid[n], capacity_grid[c]);
    compare(scores, boxes, (float)rng() / 4294967296.0f, (float)rng() / 4294967296.0f,
            (size_t)rng_range(1, (int)UF_CANDIDATE_LIMIT));
}

static void test_vectors(const char *dir)
{
    static const int expected[] = {3, 1, 1};

    for (int i = 0; i < 3; i++) {
        uint8_t output[OUT_LEN];
        uf_face_t faces[UF_CANDIDATE_LIMIT];
        char path[512];
        FILE *file;

        snprintf(path, sizeof(path), "%s/vector_%02d_output.bin", dir, i);
        file = fopen(path, "rb");
        require(file != NULL, "open delivery vector");
        require(fread(output, 1, sizeof(output), file) == sizeof(output) && fgetc(file) == EOF,
                "delivery vector length");
        fclose(file);

        /* The vectors define their expected faces at score 0.70, NMS 0.30 (expected.json). */
        require(face_decode_nms_s8((const int8_t *)(output + OUT_SCORES_OFFSET),
                                   (const int8_t *)(output + OUT_BOXES_OFFSET), 0.70f, UF_DEFAULT_NMS_THRESHOLD,
                                   faces, UF_CANDIDATE_LIMIT) == expected[i],
                "delivery vector face count");
        compare_grid((const int8_t *)(output + OUT_SCORES_OFFSET), (const int8_t *)(output + OUT_BOXES_OFFSET));
    }
}

typedef enum { SHAPE_UNIFORM, SHAPE_SPARSE, SHAPE_DENSE, SHAPE_TIES, SHAPE_SATURATED, SHAPE_COUNT } shape_t;

static void fill_tensors(int8_t *scores, int8_t *boxes, shape_t shape)
{
    for (unsigned int i = 0; i < UF_BOX_BYTES; i++)
        boxes[i] = (int8_t)rng_range(-128, 127);
    for (unsigned int prior = 0; prior < UF_PRIOR_COUNT; prior++) {
        int q0 = rng_range(-128, 127);
        int q1 = rng_range(-128, 127);

        switch (shape) {
        case SHAPE_SPARSE:
            q0 = rng_range(40, 120);
            q1 = rng_range(-120, -40);
            break;
        case SHAPE_DENSE:
            q0 = rng_range(-60, -20);
            q1 = rng_range(20, 60);
            break;
        case SHAPE_TIES:
            q0 = 0;
            q1 = 16 * rng_range(0, 3);
            break;
        case SHAPE_SATURATED:
            q0 = (rng() & 1u) ? -128 : 127;
            q1 = (rng() & 1u) ? -128 : 127;
            break;
        default:
            break;
        }
        scores[2u * prior] = (int8_t)q0;
        scores[2u * prior + 1u] = (int8_t)q1;
    }
    if (shape == SHAPE_SPARSE) {
        /* A few clusters of neighbouring priors around the thresholds, so NMS has work. */
        for (int cluster = 0; cluster < 8; cluster++) {
            const unsigned int base = (unsigned int)rng_range(0, (int)UF_PRIOR_COUNT - 12);

            for (unsigned int k = 0; k < 12u; k++) {
                const int delta = rng_range(0, 90);

                scores[2u * (base + k)] = (int8_t)(-delta / 2);
                scores[2u * (base + k) + 1u] = (int8_t)(delta - delta / 2);
            }
        }
    }
}

static void test_random(void)
{
    static int8_t scores[UF_SCORE_BYTES];
    static int8_t boxes[UF_BOX_BYTES];

    for (int trial = 0; trial < 40; trial++) {
        fill_tensors(scores, boxes, (shape_t)(trial % SHAPE_COUNT));
        compare_grid(scores, boxes);
    }
}

static void test_invalid_arguments(void)
{
    static int8_t scores[UF_SCORE_BYTES];
    static int8_t boxes[UF_BOX_BYTES];
    uf_face_t faces[4];

    require(face_decode_nms_s8(NULL, boxes, 0.5f, 0.5f, faces, 4u) == -1, "NULL scores");
    require(face_decode_nms_s8(scores, NULL, 0.5f, 0.5f, faces, 4u) == -1, "NULL boxes");
    require(face_decode_nms_s8(scores, boxes, 0.5f, 0.5f, NULL, 4u) == -1, "NULL faces");
    require(face_decode_nms_s8(scores, boxes, 0.5f, 0.5f, faces, 0u) == -1, "zero capacity");
    require(face_decode_nms_s8(scores, boxes, -0.1f, 0.5f, faces, 4u) == -1, "negative score threshold");
    require(face_decode_nms_s8(scores, boxes, 1.1f, 0.5f, faces, 4u) == -1, "score threshold above 1");
    require(face_decode_nms_s8(scores, boxes, 0.5f, -0.1f, faces, 4u) == -1, "negative nms threshold");
    require(face_decode_nms_s8(scores, boxes, 0.5f, 1.1f, faces, 4u) == -1, "nms threshold above 1");
    compare(scores, boxes, nanf(""), 0.5f, 4u);
    compare(scores, boxes, 0.5f, nanf(""), 4u);
}

int main(int argc, char **argv)
{
    if (argc != 2) {
        fprintf(stderr, "usage: %s path/to/face_accel_vectors\n", argv[0]);
        return 2;
    }
    test_prefilter();
    test_vectors(argv[1]);
    test_random();
    test_invalid_arguments();
    require(g_compared_faces > 1000u, "comparisons reached decoded faces");
    printf("face_decode_fast_test: OK (%d checks, %llu faces compared)\n", g_checks, g_compared_faces);
    return 0;
}
