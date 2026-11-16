#define _GNU_SOURCE
#define LOG_TAG "capture"

#include "capture/capture.h"
#include "capture/camera_switch.h"
#include "capture/capture_recovery.h"
#include "capture/control_tick.h"
#include "capture/dvp_counts.h"
#include "capture/stream_control.h"
#include "control/track_trace.h"
#include "detect/detector.h"
#include "media/jpeg_worker.h"
#include "platform/hal_frmbuf.h"
#include "platform/hal_ov5640.h"
#include "cam_system/tracker.h"
#include "cam_system/app_state.h"
#include "cam_system/config.h"
#include "common/app_config.h"
#include "common/fps_meter.h"
#include "common/log.h"
#include "common/time_util.h"
#include "cam_system/frame_mailbox.h"
#include "cam_system/frame_view_timeline.h"
#include <limits.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sysexits.h>
#include <unistd.h>
#include <time.h>

extern tracker_t g_tracker;

static fps_meter_t g_capture_fps;
#define CAPTURE_STALL_SEC 15.0

static unsigned long g_capture_restarts;
static unsigned int g_capture_heartbeat_sec;   /* 0 until the thread runs */
static unsigned long g_isp_slips;

#define CAPTURE_HEALTH_PERIOD_FRAMES 30
static capture_health_t g_health = { -1, -1, -1, 0 };
static dvp_counts_t g_dvp_counts;   /* capture thread only */

/* For the status interface.  Whole monotonic seconds of the stream's start plus 1; 0 when it is
 * stopped. */
static unsigned int g_stream_start_sec;
static int g_stream_stopped_by_limit;

float capture_get_fps(double now)
{
    return fps_meter_read(&g_capture_fps, now);
}

unsigned long capture_get_isp_slips(void)
{
    return __atomic_load_n(&g_isp_slips, __ATOMIC_RELAXED);
}

static void notify_status_changed(void)
{
    app_state_inc_u32(&g_state.status_seq);
}

static int capture_pop_control_cmd(track_cmd_t *cmd, int *target_id)
{
    pthread_mutex_lock(&g_state.cmd_mutex);

    if (g_state.cmd_count > 0) {
        int idx = g_state.cmd_head;
        *cmd = g_state.cmd_queue[idx];
        *target_id = g_state.cmd_target_queue[idx];
        g_state.cmd_head = (idx + 1) % CONTROL_CMD_QUEUE_LEN;
        g_state.cmd_count--;
    } else {
        *cmd = g_state.track_cmd;
        *target_id = g_state.track_target_id;
        g_state.track_cmd = TRACK_CMD_NONE;
    }

    pthread_mutex_unlock(&g_state.cmd_mutex);
    return *cmd != TRACK_CMD_NONE;
}

static void capture_handle_control_cmd(void)
{
    track_cmd_t cmd = TRACK_CMD_NONE;
    int tid = -1;

    if (!capture_pop_control_cmd(&cmd, &tid)) return;

    tracker_apply_cmd(&g_tracker, cmd, tid);
}

static void capture_snapshot_current_view(frame_view_t *view)
{
    int xoff;
    int yoff;

    tracker_get_isp_offset(&xoff, &yoff);
    tracker_snapshot_view(view, xoff, yoff, 0, 0.0);
}

static void capture_schedule_current_view(frame_view_timeline_t *timeline,
                                          uint32_t captured_sequence)
{
    frame_view_t view;

    capture_snapshot_current_view(&view);
    frame_view_timeline_schedule(timeline, captured_sequence, &view);
}

/* Blocks until the tick of this frame may run (its detection is published or the deadline passed),
 * or until a detection the control thread has not seen yet is published.  In the second case it
 * returns CONTROL_TICK_WAIT. */
static control_tick_action_t capture_wait_for_tick(const control_tick_t *tick, unsigned int last_det_seq)
{
    const double deadline = control_tick_deadline(tick);
    struct timespec abs_deadline;
    control_tick_action_t action;

    abs_deadline.tv_sec = (time_t)deadline;
    abs_deadline.tv_nsec = (long)((deadline - (double)abs_deadline.tv_sec) * 1000000000.0);

    pthread_mutex_lock(&g_state.det_mutex);
    for (;;) {
        action = control_tick_poll(tick, g_state.det_frame_seq, monotonic_sec());
        if (action != CONTROL_TICK_WAIT || app_state_load_u32(&g_state.det_seq) != last_det_seq)
            break;
        pthread_cond_timedwait(&g_state.det_cond, &g_state.det_mutex, &abs_deadline);
    }
    pthread_mutex_unlock(&g_state.det_mutex);
    return action;
}

typedef struct {
    unsigned long on_time;
    unsigned long late;
    unsigned long observations;
    double age_sum;
    double age_max;
    double dispatch_sum;
    double dispatch_max;
} capture_det_stats_t;

/* tracker.trace: the row of the current control tick.  Filled only while the trace is on. */
static track_trace_row_t g_trace_row;
static unsigned long g_trace_slips;

static void trace_begin(unsigned int frame_seq, uint32_t v4l2_seq, double capture_time)
{
    g_trace_row.frame_seq = frame_seq;
    g_trace_row.v4l2_seq = v4l2_seq;
    g_trace_row.capture_time = capture_time;
    g_trace_row.n_det = 0;
    g_trace_row.adopted = 0;
}

/* The detections a tick took in, after the tracker marked the one it adopted. */
static void trace_detections(const detection_t *dets, int count, const frame_view_t *view, double control_now)
{
    for (int i = 0; i < count && g_trace_row.n_det < TRACK_TRACE_MAX_DETS; i++) {
        track_trace_det_t *d = &g_trace_row.det[g_trace_row.n_det++];

        tracker_det_to_abs(&dets[i], view, &d->x, &d->y, &d->w, &d->h);
        d->id = dets[i].track_id;
        d->score = dets[i].confidence;
        d->publish = dets[i].publish;
        d->target = dets[i].is_target;
        if (dets[i].is_target) {
            g_trace_row.adopted = 1;
            g_trace_row.obs_x = d->x;
            g_trace_row.obs_y = d->y;
            g_trace_row.obs_h = d->h;
            g_trace_row.obs_age_ms = (control_now - view->capture_time_sec) * 1000.0;
        }
    }
}

static void trace_end(control_tick_action_t action, int isp_x, int isp_y, unsigned long slips_total)
{
    const tracker_t *trk = &g_tracker;
    track_trace_row_t *row = &g_trace_row;

    row->tick = action == CONTROL_TICK_ON_DETECTION ? TRACK_TRACE_TICK_DETECTION :
                action == CONTROL_TICK_ON_DEADLINE ? TRACK_TRACE_TICK_DEADLINE : TRACK_TRACE_TICK_IMMEDIATE;
    row->state = tracker_state_name(tracker_get_state(trk));
    row->active = tracker_is_active(trk) && trk->kf.initialized;
    if (row->active) {
        row->kf_x = trk->kf.x.pos;
        row->kf_y = trk->kf.y.pos;
        row->kf_vx = trk->kf.x.vel;
        row->kf_vy = trk->kf.y.vel;
        row->kf_log_h = trk->kf.log_h.pos;
        row->ref_x = trk->view.reference_x.pos;
        row->ref_y = trk->view.reference_y.pos;
        row->ref_vx = trk->view.reference_x.vel;
        row->ref_vy = trk->view.reference_y.vel;
        row->path_x = trk->view.x.pos;
        row->path_y = trk->view.y.pos;
        row->path_vx = trk->view.x.vel;
        row->path_vy = trk->view.y.vel;
        row->path_ax = trk->view.x.accel;
        row->path_ay = trk->view.y.accel;
    }
    row->isp_x = isp_x;
    row->isp_y = isp_y;
    row->prev_slip = slips_total != g_trace_slips;
    g_trace_slips = slips_total;
    track_trace_write(row);
}

/* Hands a newly published detection to the control tracker.  Does nothing if there is none. */
static void capture_consume_detection(unsigned int *last_processed_det_seq, unsigned int captured_seq,
                                       double control_now, int collect_stats, capture_det_stats_t *stats)
{
    unsigned int current_det_seq = 0;
    int have_new_det = 0;
    int det_count = 0;
    unsigned int det_frame_seq = 0;
    double det_publish_time = 0.0;
    frame_view_t det_view;
    detection_t det_snapshot[DETECTOR_MAX_DET];

    memset(&det_view, 0, sizeof(det_view));
    pthread_mutex_lock(&g_state.det_mutex);
    current_det_seq = app_state_load_u32(&g_state.det_seq);

    if (current_det_seq != 0 && current_det_seq != *last_processed_det_seq) {
        *last_processed_det_seq = current_det_seq;
        have_new_det = 1;
        det_count = g_state.det_count;
        if (det_count < 0) det_count = 0;
        if (det_count > DETECTOR_MAX_DET) det_count = DETECTOR_MAX_DET;
        det_view = g_state.det_frame_view;
        det_frame_seq = g_state.det_frame_seq;
        det_publish_time = g_state.det_publish_time_sec;
        memcpy(det_snapshot, g_state.detections,
               (size_t)det_count * sizeof(detection_t));
    }

    pthread_mutex_unlock(&g_state.det_mutex);
    if (!have_new_det)
        return;

    if (collect_stats && det_frame_seq != 0) {
        double wall_now = monotonic_sec();
        unsigned int lag = captured_seq - det_frame_seq;
        double age_ms = (wall_now - det_view.capture_time_sec) * 1000.0;
        double dispatch_ms = det_publish_time > 0.0 ?
            (wall_now - det_publish_time) * 1000.0 : 0.0;
        if (lag <= 1U)
            stats->on_time++;
        else
            stats->late++;
        if (age_ms >= 0.0 && age_ms < 5000.0) {
            stats->age_sum += age_ms;
            if (age_ms > stats->age_max)
                stats->age_max = age_ms;
        }
        if (dispatch_ms >= 0.0 && dispatch_ms < 5000.0) {
            stats->dispatch_sum += dispatch_ms;
            if (dispatch_ms > stats->dispatch_max)
                stats->dispatch_max = dispatch_ms;
        }
        stats->observations++;
    }
    tracker_update_detections(&g_tracker, det_snapshot, det_count, &det_view, control_now);
    if (track_trace_on())
        trace_detections(det_snapshot, det_count, &det_view, control_now);
    int tracking_active = tracker_is_active(&g_tracker);
    int tracking_target_id = tracker_get_target_id(&g_tracker);

    pthread_mutex_lock(&g_state.det_mutex);
    int prev_tracking_active = app_state_load_int(&g_state.tracking_active);
    int prev_tracking_target_id = app_state_load_int(&g_state.tracking_target_id);
    app_state_store_int(&g_state.tracking_active, tracking_active);
    app_state_store_int(&g_state.tracking_target_id, tracking_target_id);
    if (app_state_load_u32(&g_state.det_seq) == current_det_seq) {
        memcpy(g_state.detections, det_snapshot, (size_t)det_count * sizeof(detection_t));
        g_state.det_count = det_count;
    }
    pthread_mutex_unlock(&g_state.det_mutex);

    if (prev_tracking_active != tracking_active ||
        prev_tracking_target_id != tracking_target_id)
        notify_status_changed();
}

/* Waiting is worthwhile only when a tick can use this frame's detection. */
static int capture_expect_detection(int published)
{
    detector_status_t detector;

    if (!published || !app_state_load_int(&g_state.tracking_active) ||
        !app_state_load_int(&g_state.detector_enabled))
        return 0;
    detector_get_status(&detector);
    return detector.health == DETECTOR_HEALTH_RUNNING;
}

/* Requeues the capture buffers the direct channel has finished with. */
static int capture_requeue_reclaimed(void)
{
    int indices[CAPTURE_BUF_CNT];
    int count = frame_mailbox_reclaim(&g_state, indices);

    for (int i = 0; i < count; i++) {
        if (frmbuf_release_frame(indices[i]) < 0) {
            LOG_E("Failed to release frame %d", indices[i]);
            app_state_fail(&g_state, EX_SOFTWARE);
            return -1;
        }
    }
    return 0;
}

/* A capture attempt gave no good frame.  was_enabled goes to 0 after a restart: V4L2 sequences
 * start over, so everything keyed on them starts over too. */
static void capture_handle_no_frame(caprec_t *recovery, int *was_enabled)
{
    switch (caprec_on_no_frame(recovery, monotonic_sec())) {
    case CAPREC_WAIT:
        break;
    case CAPREC_RESTART: {
        int held[CAPTURE_BUF_CNT];

        LOG_W("no frames for %.0f s; restarting the stream (%d of %d)", CAPREC_STALL_SEC,
              recovery->restarts, CAPREC_RESTART_LIMIT);
        frame_mailbox_held(&g_state, held);
        if (frmbuf_restart(held) < 0)
            LOG_E("stream restart failed");
        /* The restart's transition frames are not this run's either. */
        dvp_counts_restart(&g_dvp_counts);
        __atomic_add_fetch(&g_capture_restarts, 1UL, __ATOMIC_RELAXED);
        *was_enabled = 0;
        break;
    }
    case CAPREC_GIVE_UP:
        LOG_E("stream did not recover after %d restarts", CAPREC_RESTART_LIMIT);
        app_state_fail(&g_state, EX_SOFTWARE);
        break;
    }
}

/* The sensor's counters share a lock with its I2C writes, which this thread issues; the network
 * thread must not wait on them, so the counters are read here and published. */
static void capture_poll_health(void)
{
    int32_t frames;
    int32_t bad_frames;
    uint32_t write_failures;
    uint32_t group_aborts;

    if (frmbuf_get_bridge_counts(&frames, &bad_frames) == 0) {
        long before;

        if (dvp_counts_update(&g_dvp_counts, frames, bad_frames))
            LOG_I("DVP bridge after stream start: %d frame(s), %d of the wrong size, since boot",
                  frames, bad_frames);
        before = __atomic_exchange_n(&g_health.dvp_bad_frames, g_dvp_counts.run_bad, __ATOMIC_RELAXED);
        __atomic_store_n(&g_health.dvp_frames, g_dvp_counts.run_frames, __ATOMIC_RELAXED);
        if (before >= 0 && g_dvp_counts.run_bad > before)
            LOG_W_RL(5.0, 3, "DVP bridge: %ld frame(s) of the wrong size, %ld this run",
                     g_dvp_counts.run_bad - before, g_dvp_counts.run_bad);
    }
    if (ov5640_get_error_counts(&write_failures, &group_aborts) == 0)
        __atomic_store_n(&g_health.i2c_write_failures, (long)write_failures, __ATOMIC_RELAXED);
    __atomic_store_n(&g_health.v4l2_error_frames, frmbuf_error_frame_count(), __ATOMIC_RELAXED);
}

void capture_get_health(capture_health_t *health)
{
    health->dvp_frames = __atomic_load_n(&g_health.dvp_frames, __ATOMIC_RELAXED);
    health->dvp_bad_frames = __atomic_load_n(&g_health.dvp_bad_frames, __ATOMIC_RELAXED);
    health->i2c_write_failures = __atomic_load_n(&g_health.i2c_write_failures, __ATOMIC_RELAXED);
    health->v4l2_error_frames = __atomic_load_n(&g_health.v4l2_error_frames, __ATOMIC_RELAXED);
}

void capture_get_stream_state(double now, unsigned int *run_sec, int *stopped_by_limit)
{
    unsigned int start = __atomic_load_n(&g_stream_start_sec, __ATOMIC_RELAXED);
    unsigned int now_sec = (unsigned int)now + 1u;

    *run_sec = start != 0u && now_sec > start ? now_sec - start : 0u;
    *stopped_by_limit = __atomic_load_n(&g_stream_stopped_by_limit, __ATOMIC_RELAXED);
}

/* The camera was enabled: camera_on's steps.  The sensor is initialized again rather than woken
 * from standby (camera_switch.h).  The register table leaves software standby itself
 * (0x3008=0x02 right after its software reset), so the sensor runs even when STREAMON skips its
 * s_stream(1): when the pipeline's s_stream(0) fails, the Xilinx graph code reports no error,
 * marks the sensor as still streaming and skips its s_stream(1) at every later STREAMON. */
static int capture_sensor_initialize(void)
{
    return ov5640_initialize();
}

static int capture_test_pattern(void)
{
    return ov5640_set_test_pattern(1);
}

/* The stream starts from empty buffers, so its first frame is a new one; the bridge counts take
 * a new base. */
static int capture_stream_start(void)
{
    int held[CAPTURE_BUF_CNT];

    if (capture_requeue_reclaimed() < 0)
        return -1;
    frame_mailbox_held(&g_state, held);
    if (frmbuf_start(held) < 0)
        return -1;
    dvp_counts_restart(&g_dvp_counts);
    return 0;
}

static const camera_on_stream_ops_t capture_on_ops = {
    .initialize = capture_sensor_initialize,
    .test_pattern = capture_test_pattern,
    .start_stream = capture_stream_start,
};

/* The camera was disabled: camera_off's stream steps.  This stream's last bridge reading comes
 * before the stop transition. */
static void capture_last_reading(void)
{
    capture_poll_health();
    if (g_dvp_counts.have_base)
        LOG_I("DVP bridge at stream stop: %d frame(s), %d of the wrong size, since boot",
              g_dvp_counts.last_frames, g_dvp_counts.last_bad);
}

/* STREAMOFF stops the pipeline's subdevices, the sensor first (s_stream(0) writes its standby),
 * then terminates the DMA, which ends with a reset of v_frmbuf_wr.  Our own standby write comes
 * after that and also covers a sensor the pipeline did not reach. */
static void capture_stop_stream(void)
{
    frmbuf_stop();
}

static void capture_sensor_standby(void)
{
    if (ov5640_standby(1) < 0)
        LOG_W("the sensor did not enter standby");
}

static const camera_off_stream_ops_t capture_off_ops = {
    .last_reading = capture_last_reading,
    .stop_stream = capture_stop_stream,
    .standby = capture_sensor_standby,
};

/* Starts or stops the stream as the camera state asks.  Returns 0, or -1 when a start failed. */
static int capture_follow_camera(stream_control_t *stream)
{
    const double now = monotonic_sec();
    const stream_action_t action =
        stream_control_poll(stream, app_state_load_int(&g_state.cam_enabled), now);

    switch (action) {
    case STREAM_KEEP:
        return 0;
    case STREAM_START:
        if (camera_on(&g_tracker, app_config()->capture_test_pattern, &capture_on_ops) < 0)
            return -1;
        stream_control_started(stream, now);
        __atomic_store_n(&g_stream_stopped_by_limit, 0, __ATOMIC_RELAXED);
        __atomic_store_n(&g_stream_start_sec, (unsigned int)now + 1u, __ATOMIC_RELAXED);
        LOG_I("Camera on: stream started; initialization and STREAMON took %.0f ms",
              (monotonic_sec() - now) * 1000.0);
        notify_status_changed();
        return 0;
    case STREAM_STOP_LIMIT:
        LOG_W("Camera on for %.0f min, the limit (capture.max_stream_min); turning it off",
              stream->limit_sec / 60.0);
        __atomic_store_n(&g_stream_stopped_by_limit, 1, __ATOMIC_RELAXED);
        break;
    case STREAM_STOP:
        LOG_I("Camera off: stream stopped after %.0f s", stream_control_run_sec(stream, now));
        break;
    }
    /* One ending for both: the user's request and the limit. */
    camera_off(&g_tracker, &capture_off_ops);
    stream_control_stopped(stream, action);
    __atomic_store_n(&g_stream_start_sec, 0u, __ATOMIC_RELAXED);
    notify_status_changed();
    return 0;
}

unsigned long capture_get_restarts(void)
{
    return __atomic_load_n(&g_capture_restarts, __ATOMIC_RELAXED);
}

int capture_stalled(double now)
{
    unsigned int heartbeat = __atomic_load_n(&g_capture_heartbeat_sec, __ATOMIC_RELAXED);

    return heartbeat != 0u && now > (double)heartbeat + CAPTURE_STALL_SEC;
}

/* Timing of the capture loop, logged every capture.log_every frames. */
typedef struct {
    int frames;
    double wait_sum;
    double publish_sum;
    double tick_wait_sum;
    double tracking_sum;
    double total_sum;
    double age_sum;
    double age_max;
    int monotonic_timestamps;
    unsigned long sequence_skips;
    unsigned long mailbox_drops;
    capture_det_stats_t det;
} capture_perf_t;

static void capture_perf_log(capture_perf_t *perf, control_tick_t *tick)
{
    const double n = (double)perf->frames;
    const double avg_total = perf->total_sum / n;
    const capture_det_stats_t *det = &perf->det;
    const control_tick_stats_t *ticks = &tick->stats;

    LOG_I("avg%d wait=%.2fms publish=%.2fms "
          "track=%.2fms total=%.2fms fps=%.2f "
          "capture_age=%.2f/%.2fms ts_mono=%d/%d seq_skip=%lu "
          "mailbox_drop=%lu det_on_time=%lu det_late=%lu "
          "det_age=%.2f/%.2fms det_dispatch=%.2f/%.2fms",
          perf->frames,
          perf->wait_sum / n, perf->publish_sum / n,
          perf->tracking_sum / n,
          avg_total, avg_total > 0.0 ? 1000.0 / avg_total : 0.0,
          perf->age_sum / n, perf->age_max,
          perf->monotonic_timestamps, perf->frames,
          perf->sequence_skips, perf->mailbox_drops,
          det->on_time, det->late,
          det->observations > 0 ? det->age_sum / (double)det->observations : 0.0,
          det->age_max,
          det->observations > 0 ? det->dispatch_sum / (double)det->observations : 0.0,
          det->dispatch_max);
    LOG_I("tick avg%d wait=%.2fms on_detection=%lu on_deadline=%lu immediate=%lu "
          "writes=%lu isp_slip=%lu margin=%.2f/%.2fms",
          perf->frames, perf->tick_wait_sum / n, ticks->on_detection, ticks->on_deadline,
          ticks->immediate, ticks->writes, ticks->isp_slips,
          ticks->writes > 0 ? ticks->margin_sum_ms / (double)ticks->writes : 0.0,
          ticks->margin_min_ms);
    control_tick_stats_reset(tick);
    memset(perf, 0, sizeof(*perf));
}

void *capture_thread(void *arg)
{
    (void)arg;
    pthread_setname_np(pthread_self(), "capture");
    LOG_I("Thread started");

    int capture_log_every = app_config()->capture_log_every;
    capture_perf_t perf;

    memset(&perf, 0, sizeof(perf));
    frame_view_timeline_t view_timeline;
    frame_view_timeline_init(&view_timeline);
    control_tick_t control_tick;
    frame_view_t tick_view;
    memset(&tick_view, 0, sizeof(tick_view));
    control_tick_init(&control_tick, app_config()->control_deadline_ms);
    LOG_I("ISP view metadata delay=%d frame(s)",
          ISP_APPLY_DELAY_FRAMES);

    int warmup = 0;
    unsigned int health_frames = 0;
    int was_enabled = 0;
    unsigned int last_processed_det_seq = 0;
    uint32_t last_v4l2_sequence = 0;
    int have_last_v4l2_sequence = 0;
    caprec_t recovery;
    stream_control_t stream;

    caprec_init(&recovery, monotonic_sec());
    stream_control_init(&stream, 60.0 * app_config()->capture_max_stream_min);

    while (!app_state_load_int(&g_state.quit)) {
        __atomic_store_n(&g_capture_heartbeat_sec, (unsigned int)monotonic_sec() + 1u, __ATOMIC_RELAXED);

        if (capture_follow_camera(&stream) < 0) {
            LOG_E("the stream did not start");
            app_state_fail(&g_state, EX_SOFTWARE);
            break;
        }
        if (!stream.streaming) {
            capture_handle_control_cmd();
            was_enabled = 0;
            usleep(100000);
            continue;
        }

        if (!was_enabled) {
            was_enabled = 1;
            warmup = 5;
            frame_view_timeline_init(&view_timeline);
            control_tick_reset(&control_tick);
            have_last_v4l2_sequence = 0;
            memset(&perf, 0, sizeof(perf));
            caprec_touch(&recovery, monotonic_sec());
            LOG_I("Camera enabled, warming up");
        }

        /* Fallback is used only before the warm-up timeline has enough
         * history.  Published frames are resolved by V4L2 sequence. */
        int snap_x, snap_y;
        tracker_get_isp_offset(&snap_x, &snap_y);
        frame_view_t fallback_view;
        tracker_snapshot_view(&fallback_view, snap_x, snap_y, 0, 0.0);

        double perf_t0 = capture_log_every > 0 ? monotonic_ms() : 0.0;
        if (capture_requeue_reclaimed() < 0)
            break;
        frmbuf_frame_info_t frame_info;
        int completed_idx = frmbuf_capture_frame(2000, &frame_info);
        double perf_t1 = capture_log_every > 0 ? monotonic_ms() : 0.0;
        if (completed_idx == FRMBUF_ERR_NO_FRAME) {
            capture_handle_no_frame(&recovery, &was_enabled);
            continue;  /* error frames only; counted by the capture backend */
        }
        if (completed_idx < 0) {
            LOG_W_RL(5.0, 3, "Frame timeout");
            capture_handle_no_frame(&recovery, &was_enabled);
            continue;
        }
        caprec_on_frame(&recovery, monotonic_sec());

        /* ══ 帧间隙操作 ══ */
        double capture_now = monotonic_sec();
        double capture_time = capture_now;
        int used_monotonic_timestamp = 0;
        frame_view_t captured_view;
        uint32_t tick_issued_sequence;

        if (frame_info.timestamp_monotonic && frame_info.timestamp_sec > 0.0) {
            double age = capture_now - frame_info.timestamp_sec;
            if (age >= -0.005 && age <= 2.0) {
                capture_time = frame_info.timestamp_sec;
                used_monotonic_timestamp = 1;
            }
        }
        /* This frame's end tells whether the previous tick's write made it. */
        if (control_tick_settle(&control_tick, capture_time, &tick_issued_sequence)) {
            frame_view_timeline_schedule(&view_timeline, tick_issued_sequence, &tick_view);
            __atomic_store_n(&g_isp_slips, control_tick.isp_slips_total, __ATOMIC_RELAXED);
        }
        fps_meter_tick(&g_capture_fps, capture_now);
        frame_view_timeline_resolve(&view_timeline, frame_info.sequence,
                                    &fallback_view, &captured_view);
        if (capture_log_every > 0) {
            double age_ms = (capture_now - capture_time) * 1000.0;
            uint32_t delta = frame_info.sequence - last_v4l2_sequence;

            perf.age_sum += age_ms;
            if (age_ms > perf.age_max)
                perf.age_max = age_ms;
            if (used_monotonic_timestamp)
                perf.monotonic_timestamps++;
            if (have_last_v4l2_sequence && delta > 1u && delta < INT_MAX)
                perf.sequence_skips += (unsigned long)(delta - 1u);
        }
        last_v4l2_sequence = frame_info.sequence;
        have_last_v4l2_sequence = 1;

        tracker_reset_frame_flag();  /* 每帧重置写入标志 */
        capture_handle_control_cmd();

        /* warmup 期间保持原有“跳过后续处理”的语义 */
        if (warmup > 0) {
            warmup--;
            capture_schedule_current_view(&view_timeline,
                                          frame_info.sequence);
            if (warmup == 0)
                memset(&perf, 0, sizeof(perf));
            if (frmbuf_release_frame(completed_idx) < 0) {
                LOG_E("Failed to release warmup frame %d",
              completed_idx);
                app_state_fail(&g_state, EX_SOFTWARE);
                break;
            }
            continue;
        }

        /* A detector that reads capture buffers directly gets the frame first, so a copy for
         * other consumers does not delay it.  The copy is made only for a consumer that needs
         * the frame in cached memory; a slow one may pin a slot, but capture never waits. */
        captured_view.capture_time_sec = capture_time;
        captured_view.frame_seq = frame_mailbox_begin_frame(&g_state);
        unsigned int captured_seq = captured_view.frame_seq;
        int direct = frame_mailbox_publish_direct(&g_state, completed_idx, &captured_view);
        int published = direct;
        if (jpeg_worker_has_demand() ||
            (!direct && app_state_load_int(&g_state.detector_enabled))) {
            if (frame_mailbox_publish(&g_state, g_cached_pool,
                                      g_frame_pool[completed_idx], &captured_view) < 0)
                perf.mailbox_drops++;
            else
                published = 1;
        }

        if (!direct && frmbuf_release_frame(completed_idx) < 0) {
            LOG_E("Failed to release frame %d",
          completed_idx);
            app_state_fail(&g_state, EX_SOFTWARE);
            break;
        }
        double perf_t2 = capture_log_every > 0 ? monotonic_ms() : 0.0;

        /* The tracker's clock is the capture time of this frame, wherever the
         * tick really runs: tick + TRACK_COMMAND_LEAD_SEC is then exactly the
         * capture time of the frame the command lands on, and every tick is
         * one frame period after the last. */
        double control_now = capture_time;
        int xoff_before;
        int yoff_before;
        int xoff_after;
        int yoff_after;
        control_tick_action_t tick_action;

        if (track_trace_on())
            trace_begin(captured_seq, frame_info.sequence, capture_time);
        tracker_get_isp_offset(&xoff_before, &yoff_before);
        control_tick_begin(&control_tick, frame_info.sequence, captured_seq, capture_time,
                           capture_expect_detection(published));
        /* A detection that missed the previous tick is still a measurement, and
         * the next publish overwrites it: feed it to the estimator now and keep
         * waiting for this frame's detection. */
        for (;;) {
            capture_consume_detection(&last_processed_det_seq, captured_seq, control_now,
                                      capture_log_every > 0, &perf.det);
            tick_action = capture_wait_for_tick(&control_tick, last_processed_det_seq);
            if (tick_action != CONTROL_TICK_WAIT)
                break;
        }
        capture_consume_detection(&last_processed_det_seq, captured_seq, control_now,
                                  capture_log_every > 0, &perf.det);
        double perf_tick = capture_log_every > 0 ? monotonic_ms() : 0.0;

        if (app_state_load_int(&g_state.tracking_active)) {
            tracker_predict(&g_tracker, control_now);
        }
        /* Registered when the next frame shows whether this write made it. */
        capture_snapshot_current_view(&tick_view);
        tracker_get_isp_offset(&xoff_after, &yoff_after);
        control_tick_end(&control_tick, tick_action, xoff_after != xoff_before || yoff_after != yoff_before,
                         monotonic_sec());
        if (track_trace_on())
            trace_end(tick_action, xoff_after, yoff_after, control_tick.isp_slips_total);
        {
            unsigned int late;

            if (control_tick_watch(&control_tick, &late))
                LOG_W_RL(60.0, 1, "control.deadline_ms = %.0f: %u of %u waiting ticks ran on the "
                         "deadline; detections arrive later than that",
                         control_tick.deadline_sec * 1000.0, late, CONTROL_TICK_WATCH_WAITS);
        }
        double perf_t3 = capture_log_every > 0 ? monotonic_ms() : 0.0;

        if (capture_log_every > 0) {
            perf.wait_sum += perf_t1 - perf_t0;
            perf.publish_sum += perf_t2 - perf_t1;
            perf.tick_wait_sum += perf_tick - perf_t2;
            perf.tracking_sum += perf_t3 - perf_tick;
            perf.total_sum += perf_t3 - perf_t0;
            if (++perf.frames >= capture_log_every)
                capture_perf_log(&perf, &control_tick);
        }

        if (++health_frames >= CAPTURE_HEALTH_PERIOD_FRAMES) {
            health_frames = 0;
            capture_poll_health();
        }
    }

    LOG_I("Thread exiting isp_slip=%lu", control_tick.isp_slips_total);
    return NULL;
}
