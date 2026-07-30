#define _GNU_SOURCE
#include "capture/capture.h"
#include "platform/hal_frmbuf.h"
#include "platform/hal_ov5640.h"
#include "cam_system/tracker.h"
#include "cam_system/app_state.h"
#include "cam_system/config.h"
#include "cam_system/frame_mailbox.h"
#include "cam_system/frame_view_timeline.h"
#include <limits.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <time.h>

extern tracker_t g_tracker;

static double get_time_sec(void)
{
    struct timespec ts;
    clock_gettime(CLOCK_MONOTONIC, &ts);
    return (double)ts.tv_sec + (double)ts.tv_nsec / 1000000000.0;
}

static double monotonic_ms(void)
{
    struct timespec ts;
    clock_gettime(CLOCK_MONOTONIC, &ts);
    return (double)ts.tv_sec * 1000.0 + (double)ts.tv_nsec / 1000000.0;
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

static void capture_schedule_current_view(frame_view_timeline_t *timeline,
                                          uint32_t captured_sequence)
{
    frame_view_t view;
    int xoff;
    int yoff;

    tracker_get_isp_offset(&xoff, &yoff);
    tracker_snapshot_view(&view, xoff, yoff, 0, 0.0);
    frame_view_timeline_schedule(timeline, captured_sequence, &view);
}

void *capture_thread(void *arg)
{
    (void)arg;
    pthread_setname_np(pthread_self(), "capture");
    printf("[capture] Thread started\n");

    int capture_log_every = 0;
    const char *capture_log_env = getenv("CAM_CAPTURE_LOG_EVERY");
    if (capture_log_env) {
        capture_log_every = atoi(capture_log_env);
        if (capture_log_every < 0) capture_log_every = 0;
    }
    int capture_log_count = 0;
    double capture_wait_sum = 0.0;
    double publish_sum = 0.0;
    double tracking_sum = 0.0;
    double jpeg_submit_sum = 0.0;
    double frame_total_sum = 0.0;
    double capture_age_sum = 0.0;
    double capture_age_max = 0.0;
    unsigned long v4l2_sequence_skips = 0;
    unsigned long mailbox_publish_drops = 0;
    unsigned long det_on_time = 0;
    unsigned long det_late = 0;
    unsigned long det_observations = 0;
    double det_age_sum = 0.0;
    double det_age_max = 0.0;
    double det_dispatch_sum = 0.0;
    double det_dispatch_max = 0.0;
    int monotonic_timestamp_count = 0;

    frame_view_timeline_t view_timeline;
    frame_view_timeline_init(&view_timeline);
    printf("[capture] ISP view metadata delay=%d frame(s)\n",
           ISP_APPLY_DELAY_FRAMES);

    int write_idx = 0;
    int warmup = 0;
    int was_enabled = 0;
    unsigned int last_processed_det_seq = 0;
    uint32_t last_v4l2_sequence = 0;
    int have_last_v4l2_sequence = 0;

    while (!app_state_load_int(&g_state.quit)) {

        if (!app_state_load_int(&g_state.cam_enabled)) {
            capture_handle_control_cmd();
            was_enabled = 0;
            usleep(100000);
            continue;
        }

        if (!was_enabled) {
            was_enabled = 1;
            warmup = 5;
            write_idx = 0;
            frame_view_timeline_init(&view_timeline);
            have_last_v4l2_sequence = 0;
            capture_log_count = 0;
            capture_wait_sum = 0.0;
            publish_sum = 0.0;
            tracking_sum = 0.0;
            jpeg_submit_sum = 0.0;
            frame_total_sum = 0.0;
            capture_age_sum = 0.0;
            capture_age_max = 0.0;
            monotonic_timestamp_count = 0;
            v4l2_sequence_skips = 0;
            mailbox_publish_drops = 0;
            det_on_time = 0;
            det_late = 0;
            det_observations = 0;
            det_age_sum = 0.0;
            det_age_max = 0.0;
            det_dispatch_sum = 0.0;
            det_dispatch_max = 0.0;
            printf("[capture] Camera enabled, warming up\n");
        }

        /* Fallback is used only before the warm-up timeline has enough
         * history.  Published frames are resolved by V4L2 sequence. */
        int snap_x, snap_y;
        tracker_get_isp_offset(&snap_x, &snap_y);
        frame_view_t fallback_view;
        tracker_snapshot_view(&fallback_view, snap_x, snap_y, 0, 0.0);

        double perf_t0 = capture_log_every > 0 ? monotonic_ms() : 0.0;
        frmbuf_frame_info_t frame_info;
        int completed_idx = frmbuf_capture_frame(write_idx, 2000, &frame_info);
        double perf_t1 = capture_log_every > 0 ? monotonic_ms() : 0.0;
        if (completed_idx < 0) {
            fprintf(stderr, "[capture] Frame timeout\n");
            continue;
        }

        /* ══ 帧间隙操作 ══ */
        double capture_now = get_time_sec();
        double capture_time = capture_now;
        int used_monotonic_timestamp = 0;
        frame_view_t captured_view;

        frame_view_timeline_resolve(&view_timeline, frame_info.sequence,
                                    &fallback_view, &captured_view);
        if (frame_info.timestamp_monotonic && frame_info.timestamp_sec > 0.0) {
            double age = capture_now - frame_info.timestamp_sec;
            if (age >= -0.005 && age <= 2.0) {
                capture_time = frame_info.timestamp_sec;
                used_monotonic_timestamp = 1;
            }
        }
        if (capture_log_every > 0) {
            double age_ms = (capture_now - capture_time) * 1000.0;
            uint32_t delta = frame_info.sequence - last_v4l2_sequence;

            capture_age_sum += age_ms;
            if (age_ms > capture_age_max)
                capture_age_max = age_ms;
            if (used_monotonic_timestamp)
                monotonic_timestamp_count++;
            if (have_last_v4l2_sequence && delta > 1u && delta < INT_MAX)
                v4l2_sequence_skips += (unsigned long)(delta - 1u);
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
            if (warmup == 0) {
                capture_age_sum = 0.0;
                capture_age_max = 0.0;
                monotonic_timestamp_count = 0;
                v4l2_sequence_skips = 0;
            }
            if (frmbuf_release_frame(completed_idx) < 0) {
                fprintf(stderr, "[capture] Failed to release warmup frame %d\n",
                        completed_idx);
                app_state_store_int(&g_state.quit, 1);
                break;
            }
            write_idx = (completed_idx + 1) % FRAME_BUF_CNT;
            continue;
        }

        /* Publish one immutable cached slot.  A slow detector/JPEG consumer
         * may pin a slot, but capture never waits for it. */
        captured_view.frame_seq = 0;
        captured_view.capture_time_sec = capture_time;
        unsigned int captured_seq = app_state_load_u32(&g_state.frame_seq);
        int publish_ret = frame_mailbox_publish(&g_state, g_cached_pool,
                                                g_frame_pool[completed_idx],
                                                &captured_view,
                                                &captured_seq, NULL);
        if (publish_ret < 0)
            mailbox_publish_drops++;

        if (frmbuf_release_frame(completed_idx) < 0) {
            fprintf(stderr, "[capture] Failed to release frame %d\n",
                    completed_idx);
            app_state_store_int(&g_state.quit, 1);
            break;
        }
        double perf_t2 = capture_log_every > 0 ? monotonic_ms() : 0.0;

        unsigned int current_det_seq = 0;
        int have_new_det = 0;
        int det_count = 0;
        unsigned int det_frame_seq = 0;
        double det_publish_time = 0.0;
        frame_view_t det_view;
        memset(&det_view, 0, sizeof(det_view));
        detection_t det_snapshot[DETECTOR_MAX_DETECTIONS];
        pthread_mutex_lock(&g_state.det_mutex);
        current_det_seq = app_state_load_u32(&g_state.det_seq);

        if (current_det_seq != 0 && current_det_seq != last_processed_det_seq) {
            last_processed_det_seq = current_det_seq;
            have_new_det = 1;
            det_count = g_state.det_count;
            if (det_count < 0) det_count = 0;
            if (det_count > DETECTOR_MAX_DETECTIONS)
                det_count = DETECTOR_MAX_DETECTIONS;
            det_view = g_state.det_frame_view;
            det_frame_seq = g_state.det_frame_seq;
            det_publish_time = g_state.det_publish_time_sec;
            memcpy(det_snapshot, g_state.detections,
                   (size_t)det_count * sizeof(detection_t));
        }

        pthread_mutex_unlock(&g_state.det_mutex);

        /* The capture copy above is measurable work.  Control timestamps must
         * describe when the measurement is actually consumed, not when DQBUF
         * returned several milliseconds earlier. */
        double control_now = get_time_sec();

        if (have_new_det) {
            if (capture_log_every > 0 && det_frame_seq != 0) {
                unsigned int lag = captured_seq - det_frame_seq;
                double age_ms = (control_now - det_view.capture_time_sec) * 1000.0;
                double dispatch_ms = det_publish_time > 0.0 ?
                    (control_now - det_publish_time) * 1000.0 : 0.0;
                if (lag <= 1U)
                    det_on_time++;
                else
                    det_late++;
                if (age_ms >= 0.0 && age_ms < 5000.0) {
                    det_age_sum += age_ms;
                    if (age_ms > det_age_max)
                        det_age_max = age_ms;
                }
                if (dispatch_ms >= 0.0 && dispatch_ms < 5000.0) {
                    det_dispatch_sum += dispatch_ms;
                    if (dispatch_ms > det_dispatch_max)
                        det_dispatch_max = dispatch_ms;
                }
                det_observations++;
            }
            tracker_update_detections(&g_tracker,
                                      det_snapshot,
                                      det_count,
                                      &det_view,
                                      control_now);
            int tracking_active = tracker_is_active(&g_tracker);
            int tracking_target_id = tracker_get_target_id(&g_tracker);

            pthread_mutex_lock(&g_state.det_mutex);
            int prev_tracking_active = app_state_load_int(&g_state.tracking_active);
            int prev_tracking_target_id = app_state_load_int(&g_state.tracking_target_id);
            app_state_store_int(&g_state.tracking_active, tracking_active);
            app_state_store_int(&g_state.tracking_target_id, tracking_target_id);
            if (app_state_load_u32(&g_state.det_seq) == current_det_seq) {
                memcpy(g_state.detections, det_snapshot,
                       (size_t)det_count * sizeof(detection_t));
                g_state.det_count = det_count;
            }
            pthread_mutex_unlock(&g_state.det_mutex);

            if (prev_tracking_active != tracking_active ||
                prev_tracking_target_id != tracking_target_id)
                notify_status_changed();
        }

        if (app_state_load_int(&g_state.tracking_active)) {
            tracker_predict(&g_tracker, control_now);
        }
        capture_schedule_current_view(&view_timeline, frame_info.sequence);
        double perf_t3 = capture_log_every > 0 ? monotonic_ms() : 0.0;

        /* JPEG and detector are mailbox consumers.  No capture-thread JPEG
         * submission copy remains; keep the field in logs for compatibility. */
        double perf_t4 = perf_t3;

        if (capture_log_every > 0) {
            capture_wait_sum += perf_t1 - perf_t0;
            publish_sum += perf_t2 - perf_t1;
            tracking_sum += perf_t3 - perf_t2;
            jpeg_submit_sum += perf_t4 - perf_t3;
            frame_total_sum += perf_t4 - perf_t0;
            capture_log_count++;
            if (capture_log_count >= capture_log_every) {
                double n = (double)capture_log_count;
                double avg_total = frame_total_sum / n;
                printf("[capture] avg%d wait=%.2fms publish=%.2fms "
                       "track=%.2fms jpeg_submit=%.2fms total=%.2fms fps=%.2f "
                       "capture_age=%.2f/%.2fms ts_mono=%d/%d seq_skip=%lu "
                       "mailbox_drop=%lu det_on_time=%lu det_late=%lu "
                       "det_age=%.2f/%.2fms det_dispatch=%.2f/%.2fms\n",
                       capture_log_count,
                       capture_wait_sum / n, publish_sum / n,
                       tracking_sum / n, jpeg_submit_sum / n,
                       avg_total, avg_total > 0.0 ? 1000.0 / avg_total : 0.0,
                       capture_age_sum / n, capture_age_max,
                       monotonic_timestamp_count, capture_log_count,
                       v4l2_sequence_skips, mailbox_publish_drops,
                       det_on_time, det_late,
                       det_observations > 0 ?
                           det_age_sum / (double)det_observations : 0.0,
                       det_age_max,
                       det_observations > 0 ?
                           det_dispatch_sum / (double)det_observations : 0.0,
                       det_dispatch_max);
                capture_log_count = 0;
                capture_wait_sum = 0.0;
                publish_sum = 0.0;
                tracking_sum = 0.0;
                jpeg_submit_sum = 0.0;
                frame_total_sum = 0.0;
                capture_age_sum = 0.0;
                capture_age_max = 0.0;
                monotonic_timestamp_count = 0;
                v4l2_sequence_skips = 0;
                mailbox_publish_drops = 0;
                det_on_time = 0;
                det_late = 0;
                det_observations = 0;
                det_age_sum = 0.0;
                det_age_max = 0.0;
                det_dispatch_sum = 0.0;
                det_dispatch_max = 0.0;
            }
        }

        write_idx = (completed_idx + 1) % FRAME_BUF_CNT;
    }

    printf("[capture] Thread exiting\n");
    return NULL;
}
