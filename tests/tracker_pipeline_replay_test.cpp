extern "C" {
#include "cam_system/app_state.h"
#include "cam_system/config.h"
#include "cam_system/tracker.h"
}

#include "vision/face_temporal_tracker.h"

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <cstring>
#include <vector>

#define PIPELINE_FRAMES 330
#define PIPELINE_LATENCY_FRAMES 2

extern "C" {
app_state_t g_state;
uint8_t *g_frame_pool[FRAME_BUF_CNT];
uint8_t *g_cached_pool[FRAME_BUF_CNT];
volatile sig_atomic_t g_app_signal_exit_requested;
}

static int hal_x;
static int hal_y;
static int hal_scaler_enabled = 1;

extern "C" int ov5640_apply_isp_offset(int x_offset, int y_offset)
{
    hal_x = x_offset;
    hal_y = y_offset;
    return 0;
}

extern "C" int ov5640_set_scaler_enabled(int enabled)
{
    hal_scaler_enabled = enabled;
    return 0;
}

struct DelayedDetections {
    frame_view_t view;
    detection_t detections[DETECTOR_MAX_DETECTIONS];
    int count;
};

static float clampf_test(float value, float minimum, float maximum)
{
    return value < minimum ? minimum :
           (value > maximum ? maximum : value);
}

static frame_view_t make_view(float x, float y, float width, float height,
                              int scaler, unsigned int sequence,
                              double capture_time)
{
    frame_view_t view = {};
    view.roi_x = x;
    view.roi_y = y;
    view.roi_w = width;
    view.roi_h = height;
    view.output_w = IMG_W;
    view.output_h = IMG_H;
    view.flip_x = 1;
    view.flip_y = 1;
    view.scaler_enabled = scaler;
    view.frame_seq = sequence;
    view.capture_time_sec = capture_time;
    return view;
}

static FaceTemporalBox make_abs_box(float center_x, float center_y,
                                    float width, float height, float score,
                                    const frame_view_t &view)
{
    float left = (center_x - width * 0.5f - view.roi_x) / view.roi_w;
    float right = (center_x + width * 0.5f - view.roi_x) / view.roi_w;
    float top = (center_y - height * 0.5f - view.roi_y) / view.roi_h;
    float bottom = (center_y + height * 0.5f - view.roi_y) / view.roi_h;

    if (view.flip_x) {
        float next_left = 1.0f - right;
        right = 1.0f - left;
        left = next_left;
    }
    if (view.flip_y) {
        float next_top = 1.0f - bottom;
        bottom = 1.0f - top;
        top = next_top;
    }
    left = clampf_test(left, 0.0f, 1.0f);
    right = clampf_test(right, 0.0f, 1.0f);
    top = clampf_test(top, 0.0f, 1.0f);
    bottom = clampf_test(bottom, 0.0f, 1.0f);
    return FaceTemporalBox(left * IMG_W, top * IMG_H,
                           right * IMG_W, bottom * IMG_H,
                           score, DETECTION_CLASS_FACE);
}

static void box_to_detection(const FaceTemporalBox &box, detection_t *det)
{
    std::memset(det, 0, sizeof(*det));
    det->bbox[0] = box.x1 / IMG_W;
    det->bbox[1] = box.y1 / IMG_H;
    det->bbox[2] = (box.x2 - box.x1) / IMG_W;
    det->bbox[3] = (box.y2 - box.y1) / IMG_H;
    det->class_id = box.label;
    det->confidence = box.score;
    det->track_id = box.track_id;
    det->publish = box.publish;
}

static void truth_at(int frame, float *x, float *y,
                     float *width, float *height)
{
    int motion_frame = frame;
    float time;

    /* The subject pauses during the deliberately long detector outage.  This
     * keeps recovery physically observable while still exercising a moving
     * viewport at the transition into the outage. */
    if (frame >= 180 && frame <= 216)
        motion_frame = 179;
    else if (frame > 216)
        motion_frame = frame - 37;
    time = motion_frame / TRACK_CONTROL_FPS;
    *x = 656.0f + 250.0f * std::sin(2.0f * 3.14159265358979323846f *
                                    time / 4.0f);
    *y = 486.0f + 115.0f * std::sin(2.0f * 3.14159265358979323846f *
                                    time / 3.2f + 0.45f);
    /* Sweep the same trajectory through the observed board-test face-height
     * range.  This makes one end-to-end replay exercise both far-face
     * quantization and near-face scale change without separate controller
     * parameter sets. */
    *height = 156.0f + 104.0f * std::sin(
        2.0f * 3.14159265358979323846f * time / 5.1f);
    *width = *height * 0.78f;
}

static void noisy_observation_at(int frame, float truth_x, float truth_y,
                                 float truth_w, float truth_h,
                                 float *x, float *y,
                                 float *width, float *height)
{
    static const float noise_x[] = {
        -2.0f, 1.0f, 3.0f, -1.0f, 2.0f, -3.0f, 0.0f, 2.0f
    };
    static const float noise_y[] = {
        2.0f, -2.0f, 1.0f, 3.0f, -1.0f, 0.0f, -3.0f, 1.0f
    };
    static const float noise_scale[] = {
        -0.025f, 0.015f, 0.030f, -0.010f,
        0.020f, -0.030f, 0.0f, 0.010f
    };
    const size_t count = sizeof(noise_x) / sizeof(noise_x[0]);
    size_t index = (size_t)frame % count;
    /* A fixed pixel error occupies more of a far-face box.  The bounded
     * factor models that scale effect without random input or parameter
     * searching, so failures are exactly reproducible. */
    float far_scale = clampf_test(110.0f / truth_h, 0.65f, 2.0f);
    float box_scale = 1.0f + noise_scale[index];

    *x = truth_x + noise_x[index] * far_scale;
    *y = truth_y + noise_y[index] * far_scale;
    *height = truth_h * box_scale;
    *width = truth_w * box_scale;
}

static int control_anchor_inside_core(float x, float y, float height,
                                      const frame_view_t &view)
{
    float anchor_x = x - view.roi_x;
    float anchor_y = y - view.roi_y +
        (TRACK_FACE_ANCHOR_Y_RATIO - 0.5f) * height;

    return anchor_x >= TRACK_VIEW_CORE_MARGIN_X_PX &&
           anchor_x <= view.roi_w - TRACK_VIEW_CORE_MARGIN_X_PX &&
           anchor_y >= TRACK_VIEW_CORE_MARGIN_Y_PX &&
           anchor_y <= view.roi_h - TRACK_VIEW_CORE_MARGIN_Y_PX;
}

int main()
{
    FaceTemporalTracker temporal;
    tracker_t tracker;
    frame_view_t full = make_view(0.0f, 0.0f,
                                  SUBSAMPLED_1X_W, SUBSAMPLED_1X_H,
                                  1, 1, 10.0);
    DelayedDetections delayed[PIPELINE_LATENCY_FRAMES + 1] = {};
    int issued_x[PIPELINE_FRAMES] = {};
    int issued_y[PIPELINE_FRAMES] = {};
    int initial_id = -1;
    int previous_x;
    int previous_y;
    int previous_dx = 0;
    int previous_dy = 0;
    int previous_ddx = 0;
    int previous_ddy = 0;
    int max_step_x = 0;
    int max_step_y = 0;
    int max_delta_x = 0;
    int max_delta_y = 0;
    int max_jerk_x = 0;
    int max_jerk_y = 0;
    int saw_coast = 0;
    int saw_hold = 0;
    int saw_lost = 0;
    int saw_recover = 0;
    int anchor_outside_core = 0;
    int source_before_distractor = -1;

    std::memset(&g_state, 0, sizeof(g_state));
    pthread_mutex_init(&g_state.det_mutex, NULL);
    pthread_mutex_init(&g_state.pred_mutex, NULL);
    g_state.tracking_target_id = -1;
    tracker_init(&tracker);

    /* Confirm the clickable frontend identity in full view. */
    for (int frame = 0; frame < 3; frame++) {
        std::vector<FaceTemporalBox> boxes;
        full.capture_time_sec = 10.0 + frame / TRACK_CONTROL_FPS;
        full.frame_seq = (unsigned int)(frame + 1);
        boxes.push_back(make_abs_box(656.0f + 4.0f * frame, 486.0f,
                                     90.0f, 115.0f, 0.96f, full));
        temporal.update(boxes, full, 0.80f, 0.15f, 3, 2);
        if (frame == 2) {
            if (!boxes[0].publish || boxes[0].track_id < 0) {
                std::fprintf(stderr, "pipeline selection was not confirmed\n");
                return 1;
            }
            initial_id = boxes[0].track_id;
            g_state.det_frame_view = full;
            g_state.det_count = 1;
            box_to_detection(boxes[0], &g_state.detections[0]);
        }
    }
    tracker_apply_cmd(&tracker, TRACK_CMD_SELECT, initial_id);
    tracker_apply_cmd(&tracker, TRACK_CMD_START, -1);
    if (tracker.state != TRACK_STATE_ENGAGING || hal_scaler_enabled != 0) {
        std::fprintf(stderr, "pipeline tracker did not engage\n");
        return 1;
    }
    previous_x = hal_x;
    previous_y = hal_y;

    for (int frame = 0; frame < PIPELINE_FRAMES; frame++) {
        double capture_time = 10.1 + frame / TRACK_CONTROL_FPS;
        int view_source = frame - ISP_APPLY_DELAY_FRAMES;
        float view_x = view_source >= 0 ? issued_x[view_source] : hal_x;
        float view_y = view_source >= 0 ? issued_y[view_source] : hal_y;
        frame_view_t view = make_view(view_x, view_y, IMG_W, IMG_H, 0,
                                      (unsigned int)(frame + 10),
                                      capture_time);
        DelayedDetections &slot =
            delayed[frame % (PIPELINE_LATENCY_FRAMES + 1)];
        std::vector<FaceTemporalBox> boxes;
        float truth_x;
        float truth_y;
        float truth_w;
        float truth_h;
        float observed_x;
        float observed_y;
        float observed_w;
        float observed_h;
        int detector_outage = frame >= 190 && frame <= 216;

        truth_at(frame, &truth_x, &truth_y, &truth_w, &truth_h);
        noisy_observation_at(frame, truth_x, truth_y, truth_w, truth_h,
                             &observed_x, &observed_y,
                             &observed_w, &observed_h);
        std::memset(&slot, 0, sizeof(slot));
        slot.view = view;

        if (frame == 90)
            temporal.clear();
        if (!detector_outage) {
            float score = ((frame >= 90 && frame <= 91) ||
                           (frame >= 217 && frame <= 218)) ? 0.66f : 0.96f;
            if (!control_anchor_inside_core(truth_x, truth_y,
                                            truth_h, view))
                anchor_outside_core++;
            boxes.push_back(make_abs_box(observed_x, observed_y,
                                         observed_w, observed_h, score, view));
            /* A simultaneous low-score private candidate must not steal the
             * selected source while its exact confirmed observation exists. */
            if (frame >= 140 && frame <= 150)
                boxes.push_back(make_abs_box(observed_x + 105.0f,
                                             observed_y - 35.0f,
                                             observed_w, observed_h,
                                             0.65f, view));
        }
        temporal.update(boxes, view, 0.80f, 0.15f, 3, 2);
        for (size_t i = 0;
             i < boxes.size() && i < DETECTOR_MAX_DETECTIONS; i++) {
            box_to_detection(boxes[i], &slot.detections[slot.count]);
            slot.count++;
        }

        if (frame >= PIPELINE_LATENCY_FRAMES) {
            DelayedDetections &ready = delayed[
                (frame - PIPELINE_LATENCY_FRAMES) %
                (PIPELINE_LATENCY_FRAMES + 1)];
            tracker_update_detections(&tracker,
                                      ready.detections, ready.count,
                                      &ready.view, capture_time);
        }
        tracker_reset_frame_flag();
        tracker_predict(&tracker, capture_time);
        issued_x[frame] = hal_x;
        issued_y[frame] = hal_y;

        if (!tracker_is_active(&tracker) || tracker.target_id != initial_id ||
            tracker.source_id < 0) {
            std::fprintf(stderr,
                         "pipeline identity/state lost frame=%d logical=%d source=%d state=%d\n",
                         frame, tracker.target_id, tracker.source_id,
                         tracker.state);
            return 1;
        }
        if (frame == 139)
            source_before_distractor = tracker.source_id;
        if (frame >= 142 && frame <= 152 &&
            tracker.source_id != source_before_distractor) {
            std::fprintf(stderr,
                         "private distractor stole source frame=%d %d->%d\n",
                         frame, source_before_distractor, tracker.source_id);
            return 1;
        }

        if (tracker.state == TRACK_STATE_COASTING) saw_coast = 1;
        if (tracker.state == TRACK_STATE_HOLDING) saw_hold = 1;
        if (tracker.state == TRACK_STATE_LOST) saw_lost = 1;
        if (tracker.state == TRACK_STATE_RECOVERING) saw_recover = 1;

        {
            int dx = hal_x - previous_x;
            int dy = hal_y - previous_y;
            int ddx = dx - previous_dx;
            int ddy = dy - previous_dy;
            int dddx = ddx - previous_ddx;
            int dddy = ddy - previous_ddy;
            max_step_x = std::max(max_step_x, std::abs(dx));
            max_step_y = std::max(max_step_y, std::abs(dy));
            max_delta_x = std::max(max_delta_x, std::abs(ddx));
            max_delta_y = std::max(max_delta_y, std::abs(ddy));
            max_jerk_x = std::max(max_jerk_x, std::abs(dddx));
            max_jerk_y = std::max(max_jerk_y, std::abs(dddy));
            previous_x = hal_x;
            previous_y = hal_y;
            previous_dx = dx;
            previous_dy = dy;
            previous_ddx = ddx;
            previous_ddy = ddy;
        }
    }

    std::printf("tracker_pipeline_replay: step=(%d,%d) delta=(%d,%d) "
                "jerk=(%d,%d) states=%d/%d/%d/%d source=%d "
                "anchor_outside=%d\n",
                max_step_x, max_step_y, max_delta_x, max_delta_y,
                max_jerk_x, max_jerk_y,
                saw_coast, saw_hold, saw_lost, saw_recover,
                tracker.source_id, anchor_outside_core);
    if (max_step_x > 15 || max_step_y > 10 ||
        max_delta_x > 3 || max_delta_y > 3 ||
        max_jerk_x > 4 || max_jerk_y > 4 ||
        !saw_coast || !saw_hold || !saw_lost || !saw_recover ||
        tracker.source_id == initial_id || anchor_outside_core != 0) {
        std::fprintf(stderr, "tracker pipeline replay acceptance failed\n");
        return 1;
    }

    tracker_stop(&tracker);
    pthread_mutex_destroy(&g_state.pred_mutex);
    pthread_mutex_destroy(&g_state.det_mutex);
    std::puts("tracker_pipeline_replay_test: OK");
    return 0;
}
