#include "cam_system/app_state.h"
#include "cam_system/tracker.h"

#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

app_state_t g_state;
uint8_t *g_frame_pool[FRAME_BUF_CNT];
uint8_t *g_cached_pool[FRAME_BUF_CNT];
volatile sig_atomic_t g_app_signal_exit_requested;

static int applied_x;
static int applied_y;
static int scaler_enabled = 1;
static int offset_write_fail;
static int scaler_write_fail;
static int record_offset_steps;
static int previous_applied_x;
static int previous_applied_y;
static int maximum_offset_step_x;
static int maximum_offset_step_y;

int ov5640_apply_isp_offset(int x_offset, int y_offset)
{
    int step_x;
    int step_y;

    if (offset_write_fail)
        return -1;
    step_x = abs(x_offset - previous_applied_x);
    step_y = abs(y_offset - previous_applied_y);
    if (record_offset_steps && step_x > maximum_offset_step_x)
        maximum_offset_step_x = step_x;
    if (record_offset_steps && step_y > maximum_offset_step_y)
        maximum_offset_step_y = step_y;
    applied_x = x_offset;
    applied_y = y_offset;
    previous_applied_x = x_offset;
    previous_applied_y = y_offset;
    return 0;
}

int ov5640_set_scaler_enabled(int enabled)
{
    if (scaler_write_fail)
        return -1;
    scaler_enabled = enabled;
    return 0;
}

static int nearly_equal(float actual, float expected, float tolerance)
{
    return fabsf(actual - expected) <= tolerance;
}

static detection_t make_detection(int id, float cx, float cy,
                                  float width, float height,
                                  float confidence, int publish)
{
    detection_t det;
    memset(&det, 0, sizeof(det));
    det.bbox[0] = cx - width * 0.5f;
    det.bbox[1] = cy - height * 0.5f;
    det.bbox[2] = width;
    det.bbox[3] = height;
    det.class_id = DETECTION_CLASS_FACE;
    det.confidence = confidence;
    det.track_id = id;
    det.publish = publish;
    return det;
}

static detection_t make_abs_detection(int id, float abs_x, float abs_y,
                                      float abs_width, float abs_height,
                                      float confidence, int publish,
                                      const frame_view_t *view)
{
    float raw_x = (abs_x - view->roi_x) / view->roi_w;
    float width = abs_width / view->roi_w;
    float height = abs_height / view->roi_h;
    float raw_anchor_y = (abs_y - view->roi_y) / view->roi_h;
    float display_anchor_y = view->flip_y ? 1.0f - raw_anchor_y
                                          : raw_anchor_y;
    float display_center_y = display_anchor_y +
        (0.5f - TRACK_FACE_ANCHOR_Y_RATIO) * height;
    float display_center_x = view->flip_x ? 1.0f - raw_x : raw_x;

    return make_detection(id, display_center_x, display_center_y,
                          width, height, confidence, publish);
}

static void init_state(void)
{
    memset(&g_state, 0, sizeof(g_state));
    pthread_mutex_init(&g_state.det_mutex, NULL);
    pthread_mutex_init(&g_state.pred_mutex, NULL);
    g_state.tracking_target_id = -1;
}

int main(void)
{
    tracker_t tracker;
    frame_view_t full_view;
    frame_view_t crop_view;
    detection_t detections[2];
    int fast_start_x;
    int fast_start_y;

    init_state();
    tracker_init(&tracker);

    /* Failed HAL writes must not advance the view reported to capture. */
    offset_write_fail = 1;
    if (tracker_write_isp_offset(100, 100) == 0) {
        fprintf(stderr, "failed ISP write reported success\n");
        return 1;
    }
    tracker_get_isp_offset(&applied_x, &applied_y);
    if (applied_x != ISP_DEFAULT_XOFF || applied_y != ISP_DEFAULT_YOFF) {
        fprintf(stderr, "failed ISP write changed view to (%d,%d)\n",
                applied_x, applied_y);
        return 1;
    }
    offset_write_fail = 0;
    tracker_reset_frame_flag();

    /* Selection is converted with the full-view frame, not current offsets. */
    tracker_snapshot_view(&full_view, ISP_DEFAULT_XOFF, ISP_DEFAULT_YOFF,
                          10, 0.90);
    g_state.det_frame_view = full_view;
    g_state.det_count = 1;
    g_state.detections[0] = make_detection(7, 0.50f, 0.50f,
                                            0.10f, 0.10f, 0.90f, 1);
    tracker_apply_cmd(&tracker, TRACK_CMD_SELECT, 7);
    if (!tracker.select_valid ||
        tracker.state != TRACK_STATE_SELECTED || tracker_is_active(&tracker) ||
        !nearly_equal(tracker.select_abs_x, 656.0f, 0.1f) ||
        !nearly_equal(tracker.select_abs_y, 493.776f, 0.1f)) {
        fprintf(stderr, "full-view transform failed abs=(%.2f,%.2f)\n",
                tracker.select_abs_x, tracker.select_abs_y);
        return 1;
    }

    scaler_write_fail = 1;
    tracker_apply_cmd(&tracker, TRACK_CMD_START, -1);
    if (tracker.state != TRACK_STATE_SELECTED ||
        app_state_load_int(&g_state.tracking_active)) {
        fprintf(stderr, "failed scaler disable lost selection/started tracking\n");
        return 1;
    }
    scaler_write_fail = 0;
    tracker_apply_cmd(&tracker, TRACK_CMD_START, -1);
    if (tracker.state != TRACK_STATE_ENGAGING || scaler_enabled != 0 ||
        applied_x != 336 || applied_y != 215) {
        fprintf(stderr, "start failed state=%d scaler=%d off=(%d,%d)\n",
                tracker.state, scaler_enabled, applied_x, applied_y);
        return 1;
    }

    tracker_snapshot_view(&crop_view, applied_x, applied_y, 11, 1.00);
    detections[0] = make_detection(9, 0.49f, 0.50f,
                                    0.10f, 0.10f, 0.95f, 1);
    detections[1] = make_detection(7, 0.4625f, 0.50f,
                                    0.10f, 0.10f, 0.90f, 1);
    tracker_update_detections(&tracker, detections, 2, &crop_view, 1.05);

    /* Exact selected ID wins even when another candidate is closer. */
    if (!detections[1].is_target || detections[0].is_target ||
        tracker.target_id != 7 || tracker.source_id != 7 ||
        tracker.kf.x.pos <= 656.0f ||
        tracker.kf.x.pos > 690.0f || tracker.kf.x.vel <= 0.0f) {
        fprintf(stderr,
                "association/KF failed target=(%d,%d) id=%d pos=%.2f vel=%.2f\n",
                detections[0].is_target, detections[1].is_target,
                tracker.target_id, tracker.kf.x.pos, tracker.kf.x.vel);
        return 1;
    }

    tracker_reset_frame_flag();
    tracker_predict(&tracker, 1.083333);
    if (tracker.view.x.pos <= 336.0f || applied_x < 336 || applied_x > 354) {
        fprintf(stderr,
                "viewport rate limiting failed command=%.3f applied=%d\n",
                tracker.view.x.pos, applied_x);
        return 1;
    }

    /* A new public ID must establish two consecutive observations on one
     * geometric lineage before it can replace the selected source.  This is
     * confirmation of the new source, not a blind miss-count delay. */
    for (int frame = 0; frame < 3; frame++) {
        double capture_time = 1.10 + 0.04 * frame;
        double receive_time = capture_time + 0.03;
        tracker_snapshot_view(&crop_view, applied_x, applied_y,
                              12u + (unsigned int)frame, capture_time);
        detections[0] = make_detection(12, 0.46f, 0.50f,
                                        0.10f, 0.10f, 0.88f, 1);
        tracker_update_detections(&tracker, detections, 1,
                                  &crop_view, receive_time);
        if ((frame == 0 &&
             (detections[0].is_target || tracker.source_id != 7 ||
              tracker.rebind_hits != 1)) ||
            (frame > 0 &&
             (!detections[0].is_target || tracker.source_id != 12))) {
            fprintf(stderr,
                    "confirmed source rebuild failed frame=%d logical=%d source=%d target=%d provisional=%d\n",
                    frame, tracker.target_id, tracker.source_id,
                    detections[0].is_target, tracker.rebind_hits);
            return 1;
        }
    }
    if (tracker.target_id != 7 || tracker.source_id != 12 ||
        !detections[0].is_target) {
        fprintf(stderr,
                "gated source recovery failed logical=%d source=%d target=%d\n",
                tracker.target_id, tracker.source_id,
                detections[0].is_target);
        return 1;
    }

    /* Unassociated private boxes may provide a high-variance position hint
     * after two consistent samples, but must never erase source_id or reset
     * the strong-observation timeout. */
    {
        double strong_receive_time = tracker.last_receive_time;
        float current_h = expf(tracker.kf.log_h.pos);

        for (int frame = 0; frame < 2; frame++) {
            double capture_time = 1.22 + frame / 30.0;
            double receive_time = capture_time + 0.03;
            tracker_snapshot_view(&crop_view, applied_x, applied_y,
                                  16u + (unsigned int)frame, capture_time);
            detections[0] = make_abs_detection(
                -1, tracker.kf.x.pos + 8.0f, tracker.kf.y.pos,
                current_h * tracker.aspect_ratio, current_h,
                0.68f, 0, &crop_view);
            tracker_update_detections(&tracker, detections, 1,
                                      &crop_view, receive_time);
            if ((frame == 0 && detections[0].is_target) ||
                (frame == 1 && !detections[0].is_target) ||
                tracker.source_id != 12 ||
                !nearly_equal((float)tracker.last_receive_time,
                              (float)strong_receive_time, 1e-5f)) {
                fprintf(stderr,
                        "private trust contract failed frame=%d source=%d target=%d receive=%.3f/%.3f\n",
                        frame, tracker.source_id, detections[0].is_target,
                        tracker.last_receive_time, strong_receive_time);
                return 1;
            }
        }

        /* Alternating private/new IDs are geometry hints, not consecutive
         * positive identity evidence.  They must never churn source_id. */
        for (int frame = 0; frame < 6; frame++) {
            double capture_time = 1.30 + frame / 30.0;
            int candidate_id = frame % 2 == 0 ? -1 : 30 + frame;
            tracker_snapshot_view(&crop_view, applied_x, applied_y,
                                  18u + (unsigned int)frame, capture_time);
            detections[0] = make_abs_detection(
                candidate_id, tracker.kf.x.pos + 6.0f, tracker.kf.y.pos,
                current_h * tracker.aspect_ratio, current_h,
                candidate_id >= 0 ? 0.92f : 0.68f,
                candidate_id >= 0, &crop_view);
            tracker_update_detections(&tracker, detections, 1,
                                      &crop_view, capture_time + 0.03);
            if (tracker.source_id != 12 ||
                !nearly_equal((float)tracker.last_receive_time,
                              (float)strong_receive_time, 1e-5f)) {
                fprintf(stderr,
                        "alternating IDs churned source frame=%d candidate=%d source=%d\n",
                        frame, candidate_id, tracker.source_id);
                return 1;
            }
        }
    }

    /* Timeout lifecycle is time-based and restores full view at 1.5 s. */
    tracker_reset_frame_flag();
    tracker_predict(&tracker, 1.75);
    if (tracker.state != TRACK_STATE_HOLDING) {
        fprintf(stderr, "expected HOLDING, got %d\n", tracker.state);
        return 1;
    }
    tracker_reset_frame_flag();
    tracker_predict(&tracker, 2.05);
    if (tracker.state != TRACK_STATE_LOST) {
        fprintf(stderr, "expected LOST, got %d\n", tracker.state);
        return 1;
    }

    /* A lost target freezes the viewport.  One valid sample enters recovery,
     * an intervening miss returns to LOST, and only two consecutive samples
     * restore camera motion. */
    {
        float frozen_x = tracker.view.x.pos;
        float frozen_y = tracker.view.y.pos;
        float current_h = expf(tracker.kf.log_h.pos);

        tracker_snapshot_view(&crop_view, applied_x, applied_y, 20, 2.06);
        detections[0] = make_abs_detection(
            12, tracker.kf.x.pos, tracker.kf.y.pos,
            current_h * tracker.aspect_ratio, current_h,
            0.95f, 1, &crop_view);
        tracker_update_detections(&tracker, detections, 1,
                                  &crop_view, 2.125);
        if (tracker.state != TRACK_STATE_RECOVERING ||
            !nearly_equal(tracker.view.x.pos, frozen_x, 1e-4f) ||
            !nearly_equal(tracker.view.y.pos, frozen_y, 1e-4f)) {
            fprintf(stderr, "first recovery sample moved viewport/state=%d\n",
                    tracker.state);
            return 1;
        }

        tracker_snapshot_view(&crop_view, applied_x, applied_y, 21, 2.09);
        tracker_update_detections(&tracker, detections, 0,
                                  &crop_view, 2.155);
        if (tracker.state != TRACK_STATE_LOST) {
            fprintf(stderr, "recovery miss did not return LOST state=%d\n",
                    tracker.state);
            return 1;
        }

        for (int recovery = 0; recovery < TRACK_RECOVER_CONFIRM_FRAMES;
             recovery++) {
            double capture_time = 2.12 + recovery / 30.0;
            double receive_time = capture_time + 0.065;
            current_h = expf(tracker.kf.log_h.pos);
            tracker_snapshot_view(&crop_view, applied_x, applied_y,
                                  22u + (unsigned int)recovery,
                                  capture_time);
            detections[0] = make_abs_detection(
                12, tracker.kf.x.pos, tracker.kf.y.pos,
                current_h * tracker.aspect_ratio, current_h,
                0.95f, 1, &crop_view);
            tracker_update_detections(&tracker, detections, 1,
                                      &crop_view, receive_time);
        }
        if (tracker.state != TRACK_STATE_TRACKING ||
            tracker.recovery_hits != 0 ||
            !nearly_equal(tracker.view.x.pos, frozen_x, 1e-4f) ||
            !nearly_equal(tracker.view.y.pos, frozen_y, 1e-4f)) {
            fprintf(stderr,
                    "confirmed recovery failed state=%d hits=%d view=(%.3f,%.3f)\n",
                    tracker.state, tracker.recovery_hits,
                    tracker.view.x.pos, tracker.view.y.pos);
            return 1;
        }
    }

    /* Timeout lifecycle remains time based after a successful recovery. */
    tracker_reset_frame_flag();
    tracker_predict(&tracker, 3.80);
    if (tracker.state != TRACK_STATE_IDLE || scaler_enabled != 1) {
        fprintf(stderr, "timeout stop failed state=%d scaler=%d\n",
                tracker.state, scaler_enabled);
        return 1;
    }

    /* The observation contract is a face anchor, not the box centre.  Equal
     * x/y innovations must receive less vertical gain, and a pure box-height
     * change must not move the reconstructed absolute anchor. */
    tracker_init(&tracker);
    tracker_snapshot_view(&full_view, ISP_DEFAULT_XOFF, ISP_DEFAULT_YOFF,
                          80, 5.00);
    g_state.det_frame_view = full_view;
    g_state.det_count = 1;
    g_state.detections[0] = make_abs_detection(
        31, 500.0f, 400.0f, 100.0f, 100.0f, 0.95f, 1, &full_view);
    tracker_apply_cmd(&tracker, TRACK_CMD_SELECT, 31);
    tracker_apply_cmd(&tracker, TRACK_CMD_START, -1);
    tracker_snapshot_view(&crop_view, applied_x, applied_y, 81, 5.10);
    detections[0] = make_abs_detection(
        31, 540.0f, 440.0f, 100.0f, 100.0f, 0.95f, 1, &crop_view);
    tracker_update_detections(&tracker, detections, 1, &crop_view, 5.10);
    {
        float correction_x = tracker.kf.x.pos - 500.0f;
        float correction_y = tracker.kf.y.pos - 400.0f;
        if (correction_x <= 0.0f || correction_y <= 0.0f ||
            correction_y >= correction_x) {
            fprintf(stderr,
                    "axis measurement weighting failed correction=(%.3f,%.3f)\n",
                    correction_x, correction_y);
            return 1;
        }
    }
    tracker_snapshot_view(&crop_view, applied_x, applied_y, 82, 5.133333);
    detections[0] = make_abs_detection(
        31, 540.0f, 440.0f, 180.0f, 180.0f, 0.95f, 1, &crop_view);
    tracker_update_detections(&tracker, detections, 1, &crop_view, 5.133333);
    if (!detections[0].is_target ||
        !nearly_equal(tracker.last_sub_y, 440.0f, 0.1f)) {
        fprintf(stderr,
                "height-invariant anchor failed target=%d anchor_y=%.3f\n",
                detections[0].is_target, tracker.last_sub_y);
        return 1;
    }
    tracker_stop(&tracker);

    /* A stable temporal ID must survive non-linear 600 px/s motion after the
     * Kalman covariance has converged.  This reproduces the board failure in
     * which an approximately 9 px NIS radius locked out a valid face. */
    tracker_init(&tracker);
    tracker_snapshot_view(&full_view, ISP_DEFAULT_XOFF, ISP_DEFAULT_YOFF,
                          100, 10.00);
    g_state.det_frame_view = full_view;
    g_state.det_count = 1;
    g_state.detections[0] = make_abs_detection(
        21, 400.0f, 300.0f, 100.0f, 100.0f, 0.99f, 1, &full_view);
    tracker_apply_cmd(&tracker, TRACK_CMD_SELECT, 21);
    tracker_apply_cmd(&tracker, TRACK_CMD_START, -1);
    fast_start_x = applied_x;
    fast_start_y = applied_y;
    previous_applied_x = applied_x;
    previous_applied_y = applied_y;
    maximum_offset_step_x = 0;
    maximum_offset_step_y = 0;
    record_offset_steps = 1;
    for (int frame = 0; frame < 24; frame++) {
        double capture_time = 10.10 + frame / 30.0;
        double receive_time = capture_time + 0.065;
        float target_x = 400.0f + 20.0f * frame;
        float target_y = 300.0f + 12.0f * frame;

        tracker_snapshot_view(&crop_view, applied_x, applied_y,
                              101u + (unsigned int)frame, capture_time);
        detections[0] = make_abs_detection(
            21, target_x, target_y, 100.0f, 100.0f,
            0.99f, 1, &crop_view);
        tracker_update_detections(&tracker, detections, 1,
                                  &crop_view, receive_time);
        if (!detections[0].is_target || tracker.target_id != 21) {
            fprintf(stderr,
                    "fast exact-ID motion rejected frame=%d target=%.1f id=%d\n",
                    frame, target_x, tracker.target_id);
            return 1;
        }
        tracker_reset_frame_flag();
        tracker_predict(&tracker, receive_time);
    }
    /* With the fixed 15/10 px path limits, this 0.8 s interval cannot reach
     * the old absolute offsets.  It must still use most of the bounded slew
     * envelope while accepting every observation. */
    if (applied_x - fast_start_x < 180) {
        fprintf(stderr,
                "fast motion under-drove viewport x=%d->%d\n",
                fast_start_x, applied_x);
        return 1;
    }
    if (applied_y - fast_start_y < 120) {
        fprintf(stderr,
                "fast motion under-drove viewport y=%d->%d\n",
                fast_start_y, applied_y);
        return 1;
    }
    if (maximum_offset_step_x > 15 || maximum_offset_step_y > 10) {
        fprintf(stderr, "viewport step is not smooth max=(%d,%d)\n",
                maximum_offset_step_x, maximum_offset_step_y);
        return 1;
    }

    /* A broad but physically bounded exact-ID gate accepts this fast sample;
     * the accepted residual is handled once by the Kalman update. */
    tracker_snapshot_view(&crop_view, applied_x, applied_y, 128, 10.97);
    detections[0] = make_abs_detection(
        21, 810.0f, tracker.kf.y.pos, 100.0f, 100.0f,
        0.99f, 1, &crop_view);
    tracker_update_detections(&tracker, detections, 1,
                              &crop_view, 11.035);
    if (!detections[0].is_target || tracker.kf.x.vel <= 0.0f) {
        fprintf(stderr,
                "accepted exact update reversed trajectory target=%d velocity=%.1f\n",
                detections[0].is_target, tracker.kf.x.vel);
        return 1;
    }
    tracker_reset_frame_flag();
    tracker_predict(&tracker, 11.035);

    /* A physically impossible exact-ID jump must not cause a switch to a
     * nearby different face. */
    for (int frame = 0; frame < 3; frame++) {
        double capture_time = 11.10 + frame / 30.0;
        double receive_time = capture_time + 0.065;

        tracker_snapshot_view(&crop_view, applied_x, applied_y,
                              130u + (unsigned int)frame, capture_time);
        detections[0] = make_abs_detection(
            21, 1250.0f, tracker.kf.y.pos, 100.0f, 100.0f,
            0.99f, 1, &crop_view);
        detections[1] = make_abs_detection(
            22, tracker.kf.x.pos, tracker.kf.y.pos,
            100.0f, 100.0f, 0.99f, 1, &crop_view);
        tracker_update_detections(&tracker, detections, 2,
                                  &crop_view, receive_time);
        if (detections[0].is_target || detections[1].is_target ||
            tracker.target_id != 21) {
            fprintf(stderr,
                    "impossible exact ID caused target switch frame=%d id=%d\n",
                    frame, tracker.target_id);
            return 1;
        }
        tracker_reset_frame_flag();
        tracker_predict(&tracker, receive_time);
    }

    /* Once the exact ID is absent, a nearby published ID still needs two
     * consecutive observations before it becomes the new source. */
    record_offset_steps = 0;
    for (int frame = 0; frame < TRACK_REBIND_CONFIRM_FRAMES; frame++) {
        double capture_time = 11.22 + frame / 30.0;
        tracker_snapshot_view(&crop_view, applied_x, applied_y,
                              140u + (unsigned int)frame, capture_time);
        detections[0] = make_abs_detection(
            23, tracker.kf.x.pos, tracker.kf.y.pos,
            100.0f, 100.0f, 0.95f, 1, &crop_view);
        tracker_update_detections(&tracker, detections, 1,
                                  &crop_view, capture_time + 0.065);
        if (frame == 0 && detections[0].is_target) {
            fprintf(stderr, "rebuilt ID bypassed confirmation\n");
            return 1;
        }
    }
    if (!detections[0].is_target || tracker.target_id != 21 ||
        tracker.source_id != 23) {
        fprintf(stderr, "rebuilt ID did not recover after exact ID vanished\n");
        return 1;
    }

    /* COAST is a bounded absence of observations, not evidence that the
     * subject slowed down.  Decaying the Kalman velocity here created the
     * board's positive feedback: lag -> miss -> slower crop -> larger lag. */
    {
        float velocity_before_coast = tracker.kf.x.vel;
        tracker_reset_frame_flag();
        tracker_predict(&tracker, 11.45);
        if (tracker.state != TRACK_STATE_COASTING ||
            !nearly_equal(tracker.kf.x.vel, velocity_before_coast, 0.5f)) {
            fprintf(stderr,
                    "coast changed target velocity state=%d velocity=%.2f/%.2f\n",
                    tracker.state, tracker.kf.x.vel, velocity_before_coast);
            return 1;
        }
    }
    tracker_stop(&tracker);

    pthread_mutex_destroy(&g_state.pred_mutex);
    pthread_mutex_destroy(&g_state.det_mutex);
    puts("tracker_detection_update_test: OK");
    return 0;
}
