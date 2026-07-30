/* detector.cpp - detector thread.
 *
 * The v0.1.1 launcher selects the PL face accelerator.  The frozen
 * YOLOv6Lite-S backend remains available for v0.1.0 compatibility.  REST and
 * WebSocket fields containing "yolo" are compatibility protocol names only.
 */

#include "vision/detector.h"

#include "ncnn/net.h"
#include "ncnn/mat.h"
#include "vision/face_accel_client.h"
#include "vision/face_temporal_tracker.h"

extern "C" {
#include "cam_system/app_state.h"
#include "cam_system/config.h"
#include "cam_system/frame_mailbox.h"
}

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <string>
#include <sys/time.h>
#include <time.h>
#include <unistd.h>
#include <vector>

typedef FaceTemporalBox BBox;

enum DetectorBackend {
    DETECTOR_YOLOV6 = 0,
    DETECTOR_FACE_ACCEL,
};

struct DetectorSpec {
    DetectorBackend backend;
    const char *name;
    const char *param_path;
    const char *bin_path;
};

static const DetectorSpec g_specs[] = {
    {DETECTOR_YOLOV6, "yolov6lite_s", MODEL_PARAM_PATH, MODEL_BIN_PATH},
    {DETECTOR_FACE_ACCEL, "face_accel", NULL, NULL},
};

static ncnn::Net *g_net = NULL;
static face_accel_client_t *g_face_accel = NULL;
static const DetectorSpec *g_spec = &g_specs[0];
static int g_log_every = 0;
static float g_face_score_threshold = UF_DEFAULT_SCORE_THRESHOLD;
static float g_face_track_score_threshold = 0.60f;
static float g_face_nms_threshold = UF_DEFAULT_NMS_THRESHOLD;
static unsigned int g_face_timeout_ms = 250u;
static float g_detector_fps = 30.0f;
static int g_face_confirm_frames = 3;
static int g_face_max_misses = 2;
static float g_face_temporal_iou = 0.15f;
static double g_face_hw_ms = 0.0;
static FaceTemporalTracker g_face_tracker;

static double get_time_ms(void)
{
    static double t0 = 0;
    struct timeval tv;
    gettimeofday(&tv, NULL);
    double now = tv.tv_sec * 1000.0 + tv.tv_usec / 1000.0;
    if (t0 == 0) t0 = now;
    return now - t0;
}

static double monotonic_sec(void)
{
    struct timespec ts;
    clock_gettime(CLOCK_MONOTONIC, &ts);
    return (double)ts.tv_sec + (double)ts.tv_nsec / 1000000000.0;
}

static int clamp_int(int v, int lo, int hi)
{
    if (v < lo) return lo;
    if (v > hi) return hi;
    return v;
}

static float clamp_float(float v, float lo, float hi)
{
    if (v < lo) return lo;
    if (v > hi) return hi;
    return v;
}

static const DetectorSpec *find_spec(DetectorBackend backend)
{
    for (size_t i = 0; i < sizeof(g_specs) / sizeof(g_specs[0]); i++)
        if (g_specs[i].backend == backend)
            return &g_specs[i];
    return &g_specs[0];
}

static const DetectorSpec *select_spec_from_env(void)
{
    const char *name = getenv("CAM_DETECTOR_BACKEND");
    if (name && (strcmp(name, "yolo") == 0 ||
                 strcmp(name, "yolov6") == 0))
        return find_spec(DETECTOR_YOLOV6);

    if (!name || name[0] == '\0' || strcmp(name, "face_accel") == 0 ||
        strcmp(name, "pl_face") == 0 || strcmp(name, "pl") == 0)
        return find_spec(DETECTOR_FACE_ACCEL);

    fprintf(stderr,
            "[detector] Unknown CAM_DETECTOR_BACKEND=%s, using face_accel\n",
            name);
    return find_spec(DETECTOR_FACE_ACCEL);
}

static float compute_iou(const BBox &a, const BBox &b)
{
    float x1 = std::max(a.x1, b.x1);
    float y1 = std::max(a.y1, b.y1);
    float x2 = std::min(a.x2, b.x2);
    float y2 = std::min(a.y2, b.y2);
    float inter = std::max(0.f, x2 - x1) * std::max(0.f, y2 - y1);
    float area_a = std::max(0.f, a.x2 - a.x1) * std::max(0.f, a.y2 - a.y1);
    float area_b = std::max(0.f, b.x2 - b.x1) * std::max(0.f, b.y2 - b.y1);
    return inter / (area_a + area_b - inter + 1e-6f);
}

static void nms_sorted(std::vector<BBox> &bboxes, float threshold, int max_det)
{
    if (max_det <= 0) {
        bboxes.clear();
        return;
    }

    std::sort(bboxes.begin(), bboxes.end(),
        [](const BBox &a, const BBox &b) { return a.score > b.score; });

    std::vector<bool> removed(bboxes.size(), false);
    for (size_t i = 0; i < bboxes.size(); i++) {
        if (removed[i]) continue;
        for (size_t j = i + 1; j < bboxes.size(); j++) {
            if (removed[j]) continue;
            if (compute_iou(bboxes[i], bboxes[j]) > threshold)
                removed[j] = true;
        }
    }

    std::vector<BBox> result;
    result.reserve(bboxes.size());
    for (size_t i = 0; i < bboxes.size(); i++)
        if (!removed[i]) result.push_back(bboxes[i]);

    if ((int)result.size() > max_det)
        result.resize(max_det);
    bboxes.swap(result);
}

static void decode_yolo_output(const ncnn::Mat &out, float conf_thresh,
                               const int *enabled_classes, int enabled_count,
                               std::vector<BBox> &proposals)
{
    for (int i = 0; i < out.h; i++) {
        const float *row = out.row(i);
        float obj_conf = row[4];

        float max_cls_score = -1.f;
        int max_cls_id = -1;
        for (int c = 0; c < 80; c++) {
            float s = row[5 + c];
            if (s > max_cls_score) {
                max_cls_score = s;
                max_cls_id = c;
            }
        }

        float score = obj_conf * max_cls_score;
        if (score <= conf_thresh) continue;

        if (enabled_count > 0 && enabled_count < 80) {
            bool found = false;
            for (int k = 0; k < enabled_count; k++) {
                if (enabled_classes[k] == max_cls_id) {
                    found = true;
                    break;
                }
            }
            if (!found) continue;
        }

        float cx = row[0];
        float cy = row[1];
        float w = row[2];
        float h = row[3];
        proposals.push_back({cx - w / 2.f, cy - h / 2.f,
                             cx + w / 2.f, cy + h / 2.f,
                             score, max_cls_id});
    }
}

static void detect_yolo(const uint8_t *rgb, std::vector<BBox> &proposals,
                        int max_det, float conf_thresh,
                        const int *enabled_classes, int enabled_count,
                        double *pre_ms, double *infer_ms, double *post_ms)
{
    const float mean_vals[3] = {0.f, 0.f, 0.f};
    const float norm_vals[3] = {1 / 255.f, 1 / 255.f, 1 / 255.f};

    double t0 = get_time_ms();
    ncnn::Mat in = ncnn::Mat::from_pixels_resize(
        rgb, ncnn::Mat::PIXEL_RGB2BGR,
        IMG_W, IMG_H, YOLO_INPUT_W, YOLO_INPUT_H);
    in.substract_mean_normalize(mean_vals, norm_vals);
    double t1 = get_time_ms();

    ncnn::Extractor ex = g_net->create_extractor();
    ex.input("in0", in);
    ncnn::Mat out;
    ex.extract("out0", out);
    double t2 = get_time_ms();

    decode_yolo_output(out, conf_thresh, enabled_classes, enabled_count, proposals);
    nms_sorted(proposals, YOLO_NMS_THRESH, max_det);
    double t3 = get_time_ms();

    *pre_ms = t1 - t0;
    *infer_ms = t2 - t1;
    *post_ms = t3 - t2;
}

static int detect_face_accel(const uint8_t *rgb, unsigned int frame_id,
                             std::vector<BBox> &proposals, int max_det,
                             double *pre_ms, double *infer_ms,
                             double *post_ms)
{
    uf_face_t faces[UF_CANDIDATE_LIMIT];
    face_accel_metrics_t metrics;
    double t0 = get_time_ms();
    int count = face_accel_client_run(g_face_accel, rgb, FRAME_SIZE, frame_id,
                                      g_face_timeout_ms,
                                      g_face_track_score_threshold,
                                      g_face_nms_threshold,
                                      faces, UF_CANDIDATE_LIMIT, &metrics);
    double t1 = get_time_ms();

    *pre_ms = 0.0;
    *infer_ms = t1 - t0;
    *post_ms = 0.0;
    if (count < 0)
        return count;
    g_face_hw_ms = (double)metrics.perf_cycles / 100000.0;
    if (count > max_det)
        count = max_det;
    for (int i = 0; i < count; i++) {
        const uf_face_t &face = faces[i];
        const float x1 = face.x * (float)IMG_W;
        const float y1 = face.y * (float)IMG_H;
        proposals.push_back({x1, y1,
                             x1 + face.w * (float)IMG_W,
                             y1 + face.h * (float)IMG_H,
                             face.confidence, DETECTION_CLASS_FACE});
    }
    return 0;
}

static int load_ncnn_model(const char *param_path, const char *bin_path)
{
    g_net = new ncnn::Net();
    g_net->opt.use_vulkan_compute = false;
    g_net->opt.num_threads = 1;
    g_net->opt.use_packing_layout = true;

    if (g_net->load_param(param_path) != 0) {
        fprintf(stderr, "[detector] Failed to load param: %s\n", param_path);
        delete g_net;
        g_net = NULL;
        return -1;
    }
    if (g_net->load_model(bin_path) != 0) {
        fprintf(stderr, "[detector] Failed to load bin: %s\n", bin_path);
        delete g_net;
        g_net = NULL;
        return -1;
    }
    return 0;
}

extern "C" {

int detector_init(void)
{
    g_spec = select_spec_from_env();
    printf("[detector] backend=%s\n", g_spec->name);

    const char *log_every = getenv("CAM_DETECTOR_LOG_EVERY");
    /* Keep the v0.1.0 variable as a compatibility fallback. */
    if (!log_every)
        log_every = getenv("CAM_YOLO_LOG_EVERY");
    if (log_every) {
        g_log_every = atoi(log_every);
        if (g_log_every < 0) g_log_every = 0;
        if (g_log_every > 0)
            printf("[detector] Timing log every %d frames\n", g_log_every);
    }

    const char *detector_fps = getenv("CAM_DETECTOR_FPS");
    if (detector_fps) {
        float parsed = strtof(detector_fps, NULL);
        if (parsed >= 1.0f && parsed <= 30.0f)
            g_detector_fps = parsed;
    }
    printf("[detector] target frequency=%.1f fps\n", g_detector_fps);

    if (g_spec->backend == DETECTOR_FACE_ACCEL) {
        const char *value = getenv("CAM_FACE_SCORE_THRESH");
        if (value) {
            float parsed = strtof(value, NULL);
            if (parsed >= 0.0f && parsed <= 1.0f)
                g_face_score_threshold = parsed;
        }
        value = getenv("CAM_FACE_TRACK_SCORE_THRESH");
        if (value) {
            float parsed = strtof(value, NULL);
            if (parsed >= 0.0f && parsed <= 1.0f)
                g_face_track_score_threshold = parsed;
        }
        if (g_face_track_score_threshold > g_face_score_threshold) {
            fprintf(stderr,
                    "[detector] tracker score %.3f exceeds public score %.3f; "
                    "clamping tracker score\n",
                    g_face_track_score_threshold, g_face_score_threshold);
            g_face_track_score_threshold = g_face_score_threshold;
        }
        value = getenv("CAM_FACE_NMS_THRESH");
        if (value) {
            float parsed = strtof(value, NULL);
            if (parsed >= 0.0f && parsed <= 1.0f)
                g_face_nms_threshold = parsed;
        }
        value = getenv("CAM_FACE_TIMEOUT_MS");
        if (value) {
            long parsed = strtol(value, NULL, 10);
            if (parsed > 0 && parsed <= 2000)
                g_face_timeout_ms = (unsigned int)parsed;
        }
        value = getenv("CAM_FACE_CONFIRM_FRAMES");
        if (value) {
            long parsed = strtol(value, NULL, 10);
            if (parsed >= 1 && parsed <= 10)
                g_face_confirm_frames = (int)parsed;
        }
        value = getenv("CAM_FACE_MAX_MISSES");
        if (value) {
            long parsed = strtol(value, NULL, 10);
            if (parsed >= 0 && parsed <= 10)
                g_face_max_misses = (int)parsed;
        }
        value = getenv("CAM_FACE_TEMPORAL_IOU");
        if (value) {
            float parsed = strtof(value, NULL);
            if (parsed >= 0.0f && parsed <= 1.0f)
                g_face_temporal_iou = parsed;
        }
        int ret = face_accel_client_open(&g_face_accel, NULL);
        if (ret < 0) {
            fprintf(stderr, "[detector] face accelerator init failed: %s\n",
                    strerror(-ret));
            return -1;
        }
        printf("[detector] face thresholds public=%.3f tracker=%.3f "
               "nms=%.3f timeout=%ums "
               "confirm=%d max_misses=%d temporal_iou=%.2f\n",
               g_face_score_threshold, g_face_track_score_threshold,
               g_face_nms_threshold,
               g_face_timeout_ms, g_face_confirm_frames, g_face_max_misses,
               g_face_temporal_iou);
    }

    if (g_spec->backend != DETECTOR_FACE_ACCEL) {
        printf("[detector] Loading model: %s\n", g_spec->param_path);
        if (load_ncnn_model(g_spec->param_path, g_spec->bin_path) < 0)
            return -1;
    }

    printf("[detector] Warming up...\n");
    std::vector<uint8_t> dummy(FRAME_SIZE, 0);
    std::vector<BBox> warmup;
    int enabled_classes[1] = {0};
    double pre_ms = 0.0, infer_ms = 0.0, post_ms = 0.0;
    for (int w = 0; w < 2; w++) {
        warmup.clear();
        if (g_spec->backend == DETECTOR_YOLOV6) {
            detect_yolo(dummy.data(), warmup, DETECTOR_MAX_DETECTIONS,
                        YOLO_CONF_THRESH,
                        enabled_classes, 0, &pre_ms, &infer_ms, &post_ms);
        } else if (g_spec->backend == DETECTOR_FACE_ACCEL) {
            if (detect_face_accel(dummy.data(), 0xFFFFFF00u + (unsigned int)w,
                                  warmup, DETECTOR_MAX_DETECTIONS,
                                  &pre_ms, &infer_ms, &post_ms) < 0) {
                fprintf(stderr,
                        "[detector] face accelerator warmup failed; disabling backend\n");
                face_accel_client_close(g_face_accel);
                g_face_accel = NULL;
                return -1;
            }
        }
    }

    printf("[detector] Model loaded and warmed up\n");
    return 0;
}

void *detector_thread(void *arg)
{
    (void)arg;
    pthread_setname_np(pthread_self(), "detector");
    printf("[detector] Thread started on Core %d backend=%s\n",
           CAM_CPU_DETECTOR_CORE, g_spec->name);

    if (!g_net && !g_face_accel) {
        fprintf(stderr, "[detector] Model not loaded, thread exiting\n");
        return NULL;
    }

    unsigned int last_seq = 0;
    int log_count = 0;
    double pre_sum_ms = 0.0;
    double infer_sum_ms = 0.0;
    double post_sum_ms = 0.0;
    double total_sum_ms = 0.0;
    double pickup_sum_ms = 0.0;
    double source_age_sum_ms = 0.0;
    double result_age_sum_ms = 0.0;
    double result_age_max_ms = 0.0;
    double publish_sum_ms = 0.0;
    double lease_hold_sum_ms = 0.0;
    double hw_sum_ms = 0.0;
    double ps_overhead_sum_ms = 0.0;
    unsigned long skipped_frames = 0;
    double raw_det_sum = 0.0;
    double confirmed_det_sum = 0.0;
    double max_conf_seen = 0.0;
    int raw_positive_count = 0;
    double next_run_ms = 0.0;
    std::vector<BBox> proposals;
    proposals.reserve(128);

    while (!app_state_load_int(&g_state.quit)) {
        if (!app_state_load_int(&g_state.detector_enabled)) {
            g_face_tracker.clear();
            next_run_ms = 0.0;
            usleep(100000);
            continue;
        }

        if (g_detector_fps < 29.5f && next_run_ms > 0.0) {
            double wait_ms = next_run_ms - get_time_ms();
            if (wait_ms > 0.0) {
                useconds_t wait_us = (useconds_t)(wait_ms * 1000.0);
                if (wait_us > 5000u) wait_us = 5000u;
                usleep(wait_us);
                continue;
            }
        }

        frame_lease_t frame_lease = {};
        int acquire_ret = frame_mailbox_wait_latest(&g_state, g_cached_pool,
                                                     last_seq, 100,
                                                     &frame_lease);
        if (acquire_ret < 0) {
            fprintf(stderr, "[detector] mailbox wait failed: %d\n", acquire_ret);
            break;
        }
        if (acquire_ret == 0)
            continue;

        const double lease_acquire_sec = monotonic_sec();
        const unsigned int cur_seq = frame_lease.frame_seq;
        const frame_view_t frame_view = frame_lease.view;
        const uint8_t *rgb_data = frame_lease.data;
        const double pickup_ms =
            (lease_acquire_sec - frame_lease.publish_time_sec) * 1000.0;
        const double source_age_ms = frame_view.capture_time_sec > 0.0 ?
            (lease_acquire_sec - frame_view.capture_time_sec) * 1000.0 : 0.0;
        if (last_seq != 0U) {
            unsigned int gap = cur_seq - last_seq;
            if (gap > 1U && gap < 0x80000000U)
                skipped_frames += (unsigned long)(gap - 1U);
        }
        last_seq = cur_seq;
        if (g_detector_fps < 29.5f) {
            double now_ms = get_time_ms();
            double interval_ms = 1000.0 / (double)g_detector_fps;
            if (next_run_ms <= 0.0)
                next_run_ms = now_ms;
            do {
                next_run_ms += interval_ms;
            } while (next_run_ms <= now_ms);
        }

        int enabled_classes[YOLO_NUM_CLASSES];
        int enabled_count = 0;
        int max_det = DETECTOR_MAX_DETECTIONS;
        pthread_mutex_lock(&g_state.settings_mutex);
        enabled_count = g_state.enabled_class_count;
        if (enabled_count < 0 || enabled_count > YOLO_NUM_CLASSES)
            enabled_count = YOLO_NUM_CLASSES;
        if (enabled_count > 0 && enabled_count < YOLO_NUM_CLASSES)
            memcpy(enabled_classes, g_state.enabled_classes,
                   enabled_count * sizeof(int));
        max_det = clamp_int(g_state.max_det, 0,
                            DETECTOR_MAX_DETECTIONS);
        pthread_mutex_unlock(&g_state.settings_mutex);

        proposals.clear();

        double pre_ms = 0.0;
        double infer_ms = 0.0;
        double post_ms = 0.0;
        double t0 = get_time_ms();

        int detect_ret = 0;
        if (g_spec->backend == DETECTOR_YOLOV6) {
            detect_yolo(rgb_data, proposals, max_det, YOLO_CONF_THRESH,
                        enabled_classes, enabled_count,
                        &pre_ms, &infer_ms, &post_ms);
        } else if (g_spec->backend == DETECTOR_FACE_ACCEL) {
            detect_ret = detect_face_accel(rgb_data, cur_seq,
                                           proposals, max_det,
                                           &pre_ms, &infer_ms, &post_ms);
        }

        if (detect_ret < 0) {
            frame_mailbox_release(&g_state, &frame_lease);
            if (g_spec->backend == DETECTOR_FACE_ACCEL) {
                fprintf(stderr,
                        "[detector] face accelerator job failed; disabling backend\n");
                face_accel_client_close(g_face_accel);
                g_face_accel = NULL;
                break;
            }
            usleep(20000);
            continue;
        }

        int raw_det_count = (int)proposals.size();
        double raw_max_conf = 0.0;
        for (size_t i = 0; i < proposals.size(); i++)
            raw_max_conf = std::max(raw_max_conf, (double)proposals[i].score);
        if (g_spec->backend == DETECTOR_FACE_ACCEL)
            g_face_tracker.update(proposals, frame_view,
                                  g_face_score_threshold,
                                  g_face_temporal_iou,
                                  g_face_confirm_frames,
                                  g_face_max_misses);

        double total_ms = get_time_ms() - t0;

        if (!app_state_load_int(&g_state.detector_enabled)) {
            pthread_mutex_lock(&g_state.det_mutex);
            g_state.det_count = 0;
            g_state.det_frame_seq = cur_seq;
            g_state.det_frame_view = frame_view;
            g_state.det_publish_time_sec = monotonic_sec();
            app_state_inc_u32(&g_state.det_seq);
            pthread_mutex_unlock(&g_state.det_mutex);
            frame_mailbox_release(&g_state, &frame_lease);
            continue;
        }

        const double publish_t0_sec = monotonic_sec();
        pthread_mutex_lock(&g_state.det_mutex);
        g_state.det_count = 0;
        for (size_t i = 0;
             i < proposals.size() &&
             i < (size_t)DETECTOR_MAX_DETECTIONS; i++) {
            const BBox &b = proposals[i];
            detection_t *d = &g_state.detections[g_state.det_count];

            float x1 = clamp_float(b.x1, 0.f, (float)IMG_W);
            float y1 = clamp_float(b.y1, 0.f, (float)IMG_H);
            float x2 = clamp_float(b.x2, 0.f, (float)IMG_W);
            float y2 = clamp_float(b.y2, 0.f, (float)IMG_H);
            if (x2 <= x1 || y2 <= y1)
                continue;

            d->bbox[0] = x1 / (float)IMG_W;
            d->bbox[1] = y1 / (float)IMG_H;
            d->bbox[2] = (x2 - x1) / (float)IMG_W;
            d->bbox[3] = (y2 - y1) / (float)IMG_H;
            d->class_id = b.label;
            d->confidence = b.score;
            /* -1 is intentionally preserved for an unassociated private
             * candidate; inventing an array-index ID can create false
             * identity continuity in the control tracker. */
            d->track_id = b.track_id;
            d->is_target = 0;
            d->publish = b.publish;
            g_state.det_count++;
        }

        g_state.det_frame_seq = cur_seq;
        g_state.det_frame_view = frame_view;
        g_state.det_publish_time_sec = monotonic_sec();
        app_state_inc_u32(&g_state.det_seq);
        pthread_mutex_unlock(&g_state.det_mutex);
        const double publish_done_sec = monotonic_sec();
        const double publish_ms = (publish_done_sec - publish_t0_sec) * 1000.0;
        const double result_age_ms = frame_view.capture_time_sec > 0.0 ?
            (publish_done_sec - frame_view.capture_time_sec) * 1000.0 : 0.0;
        const double lease_hold_ms =
            (publish_done_sec - lease_acquire_sec) * 1000.0;
        frame_mailbox_release(&g_state, &frame_lease);

        if (g_log_every > 0) {
            pre_sum_ms += pre_ms;
            infer_sum_ms += infer_ms;
            post_sum_ms += post_ms;
            total_sum_ms += total_ms;
            pickup_sum_ms += pickup_ms;
            source_age_sum_ms += source_age_ms;
            result_age_sum_ms += result_age_ms;
            result_age_max_ms = std::max(result_age_max_ms, result_age_ms);
            publish_sum_ms += publish_ms;
            lease_hold_sum_ms += lease_hold_ms;
            hw_sum_ms += g_spec->backend == DETECTOR_FACE_ACCEL ?
                g_face_hw_ms : 0.0;
            ps_overhead_sum_ms += std::max(
                0.0, infer_ms - (g_spec->backend == DETECTOR_FACE_ACCEL ?
                                 g_face_hw_ms : 0.0));
            raw_det_sum += (double)raw_det_count;
            for (size_t i = 0; i < proposals.size(); i++)
                if (proposals[i].publish)
                    confirmed_det_sum += 1.0;
            if (raw_det_count > 0)
                raw_positive_count++;
            max_conf_seen = std::max(max_conf_seen, raw_max_conf);
            log_count++;
            if (log_count >= g_log_every) {
                printf("[detector] backend=%s avg%d pre=%.1fms infer=%.1fms "
                       "post=%.1fms total=%.1fms hw=%.1fms ps=%.1fms "
                       "pickup=%.2fms source_age=%.1fms "
                       "result_age=%.1f/%.1fms publish=%.2fms hold=%.1fms "
                       "skipped=%lu raw=%.2f "
                       "confirmed=%.2f positive=%.1f%% max_conf=%.3f\n",
                       g_spec->name, log_count,
                       pre_sum_ms / (double)log_count,
                       infer_sum_ms / (double)log_count,
                       post_sum_ms / (double)log_count,
                       total_sum_ms / (double)log_count,
                       hw_sum_ms / (double)log_count,
                       ps_overhead_sum_ms / (double)log_count,
                       pickup_sum_ms / (double)log_count,
                       source_age_sum_ms / (double)log_count,
                       result_age_sum_ms / (double)log_count,
                       result_age_max_ms,
                       publish_sum_ms / (double)log_count,
                       lease_hold_sum_ms / (double)log_count,
                       skipped_frames,
                       raw_det_sum / (double)log_count,
                       confirmed_det_sum / (double)log_count,
                       100.0 * (double)raw_positive_count / (double)log_count,
                       max_conf_seen);
                log_count = 0;
                pre_sum_ms = 0.0;
                infer_sum_ms = 0.0;
                post_sum_ms = 0.0;
                total_sum_ms = 0.0;
                pickup_sum_ms = 0.0;
                source_age_sum_ms = 0.0;
                result_age_sum_ms = 0.0;
                result_age_max_ms = 0.0;
                publish_sum_ms = 0.0;
                lease_hold_sum_ms = 0.0;
                hw_sum_ms = 0.0;
                ps_overhead_sum_ms = 0.0;
                skipped_frames = 0;
                raw_det_sum = 0.0;
                confirmed_det_sum = 0.0;
                max_conf_seen = 0.0;
                raw_positive_count = 0;
            }
        }
    }

    printf("[detector] Thread exiting\n");
    return NULL;
}

void detector_close(void)
{
    if (g_face_accel) {
        face_accel_client_close(g_face_accel);
        g_face_accel = NULL;
    }
    if (g_net) {
        delete g_net;
        g_net = NULL;
    }
    g_face_tracker.clear();
}

} /* extern "C" */
