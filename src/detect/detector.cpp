/* detector.cpp - detection thread (see detector.h).
 *
 * Per frame: lease the newest frame from the mailbox, run the backend, publish the detections
 * with the frame's sequence and view, release the lease.  A hardware backend runs under the
 * recovery policy of detector_recovery.h: a failed job publishes an empty detection for its
 * frame, so the control tracker sees a miss and follows its HOLD/LOST rules, and the thread
 * keeps going instead of exiting.
 */
#define LOG_TAG "detector"

#include "detect/detector.h"

#include "detect/detector_backend.h"

extern "C" {
#include "cam_system/app_state.h"
#include "cam_system/config.h"
#include "cam_system/frame_mailbox.h"
#include "common/app_config.h"
#include "common/fps_meter.h"
#include "common/log.h"
#include "common/time_util.h"
#include "detect/detector_recovery.h"
}

#include <algorithm>
#include <cstring>
#include <pthread.h>
#include <unistd.h>

namespace {

DetectorBackend *g_backend = NULL;
drec_t g_recovery;
int g_log_every = 0;
float g_detector_fps = 30.0f;
pthread_mutex_t g_status_lock = PTHREAD_MUTEX_INITIALIZER;
detector_status_t g_status = {DETECTOR_HEALTH_UNAVAILABLE, 0, 0, 0, 0.0f, 0, {0, 0, ""}};
fps_meter_t g_fps;

int clamp_int(int v, int lo, int hi)
{
    return v < lo ? lo : (v > hi ? hi : v);
}

float clamp_float(float v, float lo, float hi)
{
    return v < lo ? lo : (v > hi ? hi : v);
}

bool is_face_accel_name(const char *name)
{
    return strcmp(name, "face_accel") == 0;
}

void update_status(void)
{
    detector_status_t s;

    s.health = (detector_health_t)drec_health(&g_recovery);
    s.failures = g_recovery.total_failures;
    s.recoveries = g_recovery.recoveries;
    s.last_error = g_recovery.last_error;
    s.fps = 0.0f;
    s.direct_input = g_backend->reads_capture_buffers();
    memset(&s.identity, 0, sizeof(s.identity));
    g_backend->identity(&s.identity);
    pthread_mutex_lock(&g_status_lock);
    g_status = s;
    pthread_mutex_unlock(&g_status_lock);
}

/* An empty result for frame_seq: the control tracker treats it as a frame without faces. */
void publish_empty(unsigned int frame_seq, const frame_view_t &view)
{
    pthread_mutex_lock(&g_state.det_mutex);
    g_state.det_count = 0;
    g_state.det_frame_seq = frame_seq;
    g_state.det_frame_view = view;
    g_state.det_publish_time_sec = monotonic_sec();
    app_state_inc_u32(&g_state.det_seq);
    pthread_cond_broadcast(&g_state.det_cond);
    pthread_mutex_unlock(&g_state.det_mutex);
}

/* Performs a pending recovery action.  Returns true when the next step is a detection job. */
bool recovery_step(void)
{
    const double now = monotonic_sec();
    int ret;

    switch (drec_next_action(&g_recovery, now)) {
    case DREC_ACT_RUN:
        return true;
    case DREC_ACT_WAIT: {
        double wait = std::min(drec_wait_remaining(&g_recovery, now), 0.1);
        usleep((useconds_t)(wait * 1000000.0));
        return false;
    }
    case DREC_ACT_RESET:
        g_backend->clear_temporal_state();
        ret = g_backend->reset();
        drec_on_reset(&g_recovery, ret, monotonic_sec());
        LOG_W("backend reset %s%s", ret < 0 ? "failed: " : "done", ret < 0 ? strerror(-ret) : "");
        break;
    case DREC_ACT_REOPEN:
        g_backend->clear_temporal_state();
        g_backend->close();
        ret = g_backend->open();
        drec_on_reopen(&g_recovery, ret, monotonic_sec());
        if (ret < 0)
            LOG_W("backend reopen failed: %s; next attempt in %.1f s", strerror(-ret), g_recovery.backoff_sec);
        else
            LOG_I("backend reopened");
        break;
    }
    update_status();
    return false;
}

void recovery_after_job(int ret)
{
    const drec_state_t before = g_recovery.state;

    drec_on_job(&g_recovery, ret, monotonic_sec());
    if (ret < 0) {
        LOG_W_RL(1.0, 5, "job failed: %s (%s -> %s)", strerror(-ret), drec_state_name(before),
                 drec_state_name(g_recovery.state));
        update_status();
    } else if (before != DREC_RUNNING) {
        LOG_I("recovered from %s (recoveries=%lu failures=%lu)", drec_state_name(before),
              g_recovery.recoveries, g_recovery.total_failures);
        update_status();
    }
}

} /* namespace */

extern "C" {

int detector_init(void)
{
    const app_config_t *cfg = app_config();
    drec_config_t rcfg;
    int ret;

    g_log_every = cfg->detector_log_every;
    if (g_log_every > 0)
        LOG_I("Timing log every %d frames", g_log_every);
    g_detector_fps = cfg->detector_fps;
    LOG_I("target frequency=%.1f fps", g_detector_fps);

    if (is_face_accel_name(cfg->detector_backend))
        g_backend = detector_backend_face_accel_create(cfg);
#ifdef CAM_SYSTEM_ENABLE_CPU_DETECTORS
    else
        g_backend = detector_backend_cpu_create(cfg->detector_backend);
#endif
    if (!g_backend) {
        LOG_E("detector.backend=%s is not available in this build", cfg->detector_backend);
        return -1;
    }
    LOG_I("backend=%s", g_backend->name());

    rcfg.retry_limit = cfg->detector_retry_limit;
    rcfg.backoff_initial_sec = 0.5;
    rcfg.backoff_max_sec = cfg->detector_backoff_max_sec;
    drec_init(&g_recovery, &rcfg);

    ret = g_backend->open();
    if (ret < 0) {
        if (!g_backend->recoverable()) {
            delete g_backend;
            g_backend = NULL;
            return -1;
        }
        drec_start_failed(&g_recovery, ret, monotonic_sec());
        LOG_W("backend not ready (%s); retrying in the detector thread", strerror(-ret));
    }
    update_status();
    return 0;
}

namespace {

/* Timing of the detector loop, logged every detector.log_every frames. */
struct DetectorPerf {
    int frames;
    double pre_ms;
    double infer_ms;
    double post_ms;
    double total_ms;
    double hw_ms;
    double ps_overhead_ms;
    double pickup_ms;
    double source_age_ms;
    double result_age_ms;
    double result_age_max_ms;
    double publish_ms;
    double lease_hold_ms;
    unsigned long skipped_frames;
    double raw_detections;
    double confirmed_detections;
    double max_conf;
    int raw_positive_frames;
};

void detector_perf_log(DetectorPerf *perf)
{
    const double n = (double)perf->frames;

    LOG_I("backend=%s avg%d pre=%.1fms infer=%.1fms "
          "post=%.1fms total=%.1fms hw=%.1fms ps=%.1fms "
          "pickup=%.2fms source_age=%.1fms "
          "result_age=%.1f/%.1fms publish=%.2fms hold=%.1fms "
          "skipped=%lu raw=%.2f "
          "confirmed=%.2f positive=%.1f%% max_conf=%.3f",
          g_backend->name(), perf->frames,
          perf->pre_ms / n, perf->infer_ms / n,
          perf->post_ms / n, perf->total_ms / n,
          perf->hw_ms / n, perf->ps_overhead_ms / n,
          perf->pickup_ms / n, perf->source_age_ms / n,
          perf->result_age_ms / n, perf->result_age_max_ms,
          perf->publish_ms / n, perf->lease_hold_ms / n,
          perf->skipped_frames, perf->raw_detections / n,
          perf->confirmed_detections / n,
          100.0 * (double)perf->raw_positive_frames / n,
          perf->max_conf);
    *perf = DetectorPerf();
}

void publish_detections(const std::vector<BBox> &proposals, unsigned int frame_seq,
                        const frame_view_t &frame_view)
{
    pthread_mutex_lock(&g_state.det_mutex);
    g_state.det_count = 0;
    for (size_t i = 0; i < proposals.size() && i < (size_t)DETECTOR_MAX_DET; i++) {
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
    g_state.det_frame_seq = frame_seq;
    g_state.det_frame_view = frame_view;
    g_state.det_publish_time_sec = monotonic_sec();
    app_state_inc_u32(&g_state.det_seq);
    pthread_cond_broadcast(&g_state.det_cond);
    pthread_mutex_unlock(&g_state.det_mutex);
}

}  // namespace

void *detector_thread(void *arg)
{
    (void)arg;
    pthread_setname_np(pthread_self(), "detector");
    LOG_I("Thread started on Core %d backend=%s", CAM_CPU_DETECTOR_CORE, g_backend ? g_backend->name() : "none");

    if (!g_backend) {
        LOG_E("Model not loaded, thread exiting");
        return NULL;
    }

    const bool recoverable = g_backend->recoverable();
    unsigned int last_seq = 0;
    DetectorPerf perf = {};
    double next_run_ms = 0.0;
    bool reported_direct = g_backend->reads_capture_buffers();
    std::vector<BBox> proposals;
    proposals.reserve(128);

    while (!app_state_load_int(&g_state.quit)) {
        if (!app_state_load_int(&g_state.detector_enabled)) {
            g_backend->clear_temporal_state();
            next_run_ms = 0.0;
            usleep(100000);
            continue;
        }
        if (recoverable && !recovery_step())
            continue;

        if (g_detector_fps < 29.5f && next_run_ms > 0.0) {
            double wait_ms = next_run_ms - monotonic_ms();
            if (wait_ms > 0.0) {
                useconds_t wait_us = (useconds_t)(wait_ms * 1000.0);
                if (wait_us > 5000u)
                    wait_us = 5000u;
                usleep(wait_us);
                continue;
            }
        }

        const bool direct = g_backend->reads_capture_buffers();
        if (direct != reported_direct) {
            reported_direct = direct;
            update_status();
        }
        frame_lease_t frame_lease = {};
        int acquire_ret = direct ?
            frame_mailbox_wait_direct(&g_state, g_frame_pool, last_seq, 100, &frame_lease) :
            frame_mailbox_wait_latest(&g_state, g_cached_pool, last_seq, 100, &frame_lease);
        if (acquire_ret < 0) {
            LOG_E("mailbox wait failed: %d", acquire_ret);
            break;
        }
        if (acquire_ret == 0)
            continue;

        const double lease_acquire_sec = monotonic_sec();
        const unsigned int cur_seq = frame_lease.frame_seq;
        const frame_view_t frame_view = frame_lease.view;
        const double pickup_ms = (lease_acquire_sec - frame_lease.publish_time_sec) * 1000.0;
        const double source_age_ms = frame_view.capture_time_sec > 0.0 ?
            (lease_acquire_sec - frame_view.capture_time_sec) * 1000.0 : 0.0;
        if (last_seq != 0U) {
            unsigned int gap = cur_seq - last_seq;
            if (gap > 1U && gap < 0x80000000U)
                perf.skipped_frames += (unsigned long)(gap - 1U);
        }
        last_seq = cur_seq;
        if (g_detector_fps < 29.5f) {
            double t = monotonic_ms();
            double interval_ms = 1000.0 / (double)g_detector_fps;
            if (next_run_ms <= 0.0)
                next_run_ms = t;
            do {
                next_run_ms += interval_ms;
            } while (next_run_ms <= t);
        }

        int enabled_classes[DETECTOR_MAX_CLASSES];
        DetectorJob job = {};
        DetectorJobInfo info = {};

        pthread_mutex_lock(&g_state.settings_mutex);
        job.enabled_count = g_state.enabled_class_count;
        if (job.enabled_count < 0 || job.enabled_count > DETECTOR_MAX_CLASSES)
            job.enabled_count = DETECTOR_MAX_CLASSES;
        if (job.enabled_count > 0 && job.enabled_count < DETECTOR_MAX_CLASSES)
            memcpy(enabled_classes, g_state.enabled_classes, (size_t)job.enabled_count * sizeof(int));
        job.max_det = clamp_int(g_state.max_det, 0, DETECTOR_MAX_DET);
        pthread_mutex_unlock(&g_state.settings_mutex);
        job.rgb = frame_lease.data;
        job.capture_idx = frame_lease.capture_idx;
        job.frame_seq = cur_seq;
        job.view = frame_view;
        job.conf_thresh = DETECTOR_CONF_THRESH;
        job.enabled_classes = enabled_classes;

        const double t0 = monotonic_ms();
        const int detect_ret = g_backend->run(job, proposals, &info);
        if (recoverable)
            recovery_after_job(detect_ret);
        if (detect_ret < 0) {
            publish_empty(cur_seq, frame_view);
            frame_mailbox_release(&g_state, &frame_lease);
            continue;
        }
        const double total_ms = monotonic_ms() - t0;

        if (!app_state_load_int(&g_state.detector_enabled)) {
            publish_empty(cur_seq, frame_view);
            frame_mailbox_release(&g_state, &frame_lease);
            continue;
        }

        const double publish_t0_sec = monotonic_sec();
        publish_detections(proposals, cur_seq, frame_view);
        const double publish_done_sec = monotonic_sec();
        fps_meter_tick(&g_fps, publish_done_sec);
        const double publish_ms = (publish_done_sec - publish_t0_sec) * 1000.0;
        const double result_age_ms = frame_view.capture_time_sec > 0.0 ?
            (publish_done_sec - frame_view.capture_time_sec) * 1000.0 : 0.0;
        const double lease_hold_ms = (publish_done_sec - lease_acquire_sec) * 1000.0;
        frame_mailbox_release(&g_state, &frame_lease);

        if (g_log_every > 0) {
            perf.pre_ms += info.pre_ms;
            perf.infer_ms += info.infer_ms;
            perf.post_ms += info.post_ms;
            perf.total_ms += total_ms;
            perf.pickup_ms += pickup_ms;
            perf.source_age_ms += source_age_ms;
            perf.result_age_ms += result_age_ms;
            perf.result_age_max_ms = std::max(perf.result_age_max_ms, result_age_ms);
            perf.publish_ms += publish_ms;
            perf.lease_hold_ms += lease_hold_ms;
            perf.hw_ms += info.hw_ms;
            perf.ps_overhead_ms += std::max(0.0, info.infer_ms - info.hw_ms);
            perf.raw_detections += (double)info.raw_count;
            for (size_t i = 0; i < proposals.size(); i++)
                if (proposals[i].publish)
                    perf.confirmed_detections += 1.0;
            if (info.raw_count > 0)
                perf.raw_positive_frames++;
            perf.max_conf = std::max(perf.max_conf, info.raw_max_conf);
            if (++perf.frames >= g_log_every)
                detector_perf_log(&perf);
        }
    }

    LOG_I("Thread exiting");
    return NULL;
}

void detector_close(void)
{
    delete g_backend;
    g_backend = NULL;
}

void detector_get_status(detector_status_t *status)
{
    pthread_mutex_lock(&g_status_lock);
    *status = g_status;
    pthread_mutex_unlock(&g_status_lock);
    status->fps = fps_meter_read(&g_fps, monotonic_sec());
}

const char *detector_health_name(detector_health_t health)
{
    static const char *const names[] = {"running", "recovering", "failed", "unavailable"};
    return (unsigned)health < sizeof(names) / sizeof(names[0]) ? names[health] : "unknown";
}

} /* extern "C" */
