#include "cam_system/app_state.h"
#include "cam_system/config.h"
#include "cam_system/tracker.h"

#include <math.h>
#include <signal.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#define SIM_FRAMES 360
#define DETECTOR_LATENCY_FRAMES 3
#define PI_F 3.14159265358979323846f
#define ACCEPT_MAX_STEP_X 15
#define ACCEPT_MAX_STEP_Y 10

app_state_t g_state;
uint8_t *g_frame_pool[FRAME_BUF_CNT];
uint8_t *g_cached_pool[FRAME_BUF_CNT];
volatile sig_atomic_t g_app_signal_exit_requested;

static int hal_x;
static int hal_y;
static int hal_scaler_enabled = 1;

int ov5640_apply_isp_offset(int x_offset, int y_offset)
{
    hal_x = x_offset;
    hal_y = y_offset;
    return 0;
}

int ov5640_set_scaler_enabled(int enabled)
{
    hal_scaler_enabled = enabled;
    return 0;
}

typedef struct {
    frame_view_t view;
    detection_t detections[2];
    int count;
} delayed_observation_t;

typedef struct {
    const char *name;
    float face_height;
    float noise_x;
    float noise_y;
    float motion_scale;
    int long_faults;
} scenario_t;

static float clampf_test(float value, float minimum, float maximum)
{
    return value < minimum ? minimum :
           (value > maximum ? maximum : value);
}

static frame_view_t make_view(float x, float y, float width, float height,
                              int scaler_enabled, unsigned int sequence,
                              double capture_time)
{
    frame_view_t view;

    memset(&view, 0, sizeof(view));
    view.roi_x = x;
    view.roi_y = y;
    view.roi_w = width;
    view.roi_h = height;
    view.output_w = IMG_W;
    view.output_h = IMG_H;
    view.flip_x = 1;
    view.flip_y = 1;
    view.scaler_enabled = scaler_enabled;
    view.frame_seq = sequence;
    view.capture_time_sec = capture_time;
    return view;
}

static detection_t make_abs_detection(int id, float anchor_x, float anchor_y,
                                      float abs_width, float abs_height,
                                      float confidence, int publish,
                                      const frame_view_t *view)
{
    detection_t detection;
    float raw_x = (anchor_x - view->roi_x) / view->roi_w;
    float raw_anchor_y = (anchor_y - view->roi_y) / view->roi_h;
    float width = abs_width / view->roi_w;
    float height = abs_height / view->roi_h;
    float display_x = view->flip_x ? 1.0f - raw_x : raw_x;
    float display_anchor_y = view->flip_y ? 1.0f - raw_anchor_y
                                          : raw_anchor_y;
    float display_center_y = display_anchor_y +
        (0.5f - TRACK_FACE_ANCHOR_Y_RATIO) * height;
    float left = clampf_test(display_x - width * 0.5f, 0.0f, 1.0f);
    float top = clampf_test(display_center_y - height * 0.5f, 0.0f, 1.0f);
    float right = clampf_test(display_x + width * 0.5f, 0.0f, 1.0f);
    float bottom = clampf_test(display_center_y + height * 0.5f, 0.0f, 1.0f);

    memset(&detection, 0, sizeof(detection));
    detection.bbox[0] = left;
    detection.bbox[1] = top;
    detection.bbox[2] = right - left;
    detection.bbox[3] = bottom - top;
    detection.class_id = DETECTION_CLASS_FACE;
    detection.confidence = confidence;
    detection.track_id = id;
    detection.publish = publish;
    return detection;
}

static void truth_at(int frame, const scenario_t *scenario,
                     float *x, float *y, float *width, float *height)
{
    float time = frame / TRACK_CONTROL_FPS;
    float scale_motion = 1.0f + 0.08f * sinf(2.0f * PI_F * time / 3.7f);

    *x = 656.0f + scenario->motion_scale * 380.0f *
        sinf(2.0f * PI_F * time / 5.5f);
    *y = 486.0f + scenario->motion_scale * 200.0f *
        sinf(2.0f * PI_F * time / 4.5f + 0.35f);
    *height = scenario->face_height * scale_motion;
    *width = *height * 0.78f;
}

static int compare_float(const void *left, const void *right)
{
    float a = *(const float *)left;
    float b = *(const float *)right;
    return (a > b) - (a < b);
}

static void init_app_state(void)
{
    memset(&g_state, 0, sizeof(g_state));
    pthread_mutex_init(&g_state.det_mutex, NULL);
    pthread_mutex_init(&g_state.pred_mutex, NULL);
    g_state.tracking_target_id = -1;
}

static int run_scenario(const scenario_t *scenario)
{
    tracker_t tracker;
    delayed_observation_t delayed[DETECTOR_LATENCY_FRAMES + 1];
    int issued_x[SIM_FRAMES];
    int issued_y[SIM_FRAMES];
    float abs_error_x[SIM_FRAMES];
    float abs_error_y[SIM_FRAMES];
    double sum_squared_x = 0.0;
    double sum_squared_y = 0.0;
    int metric_count = 0;
    int max_step_x = 0;
    int max_step_y = 0;
    int max_delta_step_x = 0;
    int max_delta_step_y = 0;
    int max_jerk_step_x = 0;
    int max_jerk_step_y = 0;
    int previous_step_x = 0;
    int previous_step_y = 0;
    int previous_delta_step_x = 0;
    int previous_delta_step_y = 0;
    float max_error_x = 0.0f;
    float max_error_y = 0.0f;
    int max_error_frame_x = -1;
    int max_error_frame_y = -1;
    int offscreen_anchor_frames = 0;
    int outside_core_frames = 0;
    int saw_coasting = 0;
    int saw_holding = 0;
    int saw_lost = 0;
    int saw_recovering = 0;
    int association_rejects = 0;
    float initial_x, initial_y, initial_width, initial_height;
    const double start_time = 100.0;

    memset(delayed, 0, sizeof(delayed));
    memset(issued_x, 0, sizeof(issued_x));
    memset(issued_y, 0, sizeof(issued_y));
    tracker_init(&tracker);
    hal_x = ISP_DEFAULT_XOFF;
    hal_y = ISP_DEFAULT_YOFF;
    hal_scaler_enabled = 1;

    truth_at(0, scenario, &initial_x, &initial_y,
             &initial_width, &initial_height);
    g_state.det_frame_view = make_view(
        0.0f, 0.0f, SUBSAMPLED_1X_W, SUBSAMPLED_1X_H,
        1, 1, start_time);
    g_state.det_count = 1;
    g_state.detections[0] = make_abs_detection(
        100, initial_x, initial_y, initial_width, initial_height,
        0.96f, 1, &g_state.det_frame_view);
    tracker_apply_cmd(&tracker, TRACK_CMD_SELECT, 100);
    tracker_apply_cmd(&tracker, TRACK_CMD_START, -1);
    if (tracker.state != TRACK_STATE_ENGAGING || hal_scaler_enabled != 0) {
        fprintf(stderr, "%s: failed to engage\n", scenario->name);
        return 1;
    }

    for (int frame = 0; frame < SIM_FRAMES; frame++) {
        double capture_time = start_time + frame / TRACK_CONTROL_FPS;
        float truth_x, truth_y, truth_width, truth_height;
        float actual_x;
        float actual_y;
        int crop_visible = frame >= ISP_APPLY_DELAY_FRAMES;
        frame_view_t captured_view;
        delayed_observation_t *current =
            &delayed[frame % (DETECTOR_LATENCY_FRAMES + 1)];
        int first_gap_end = scenario->long_faults ? 118 : 105;
        int second_gap_end = scenario->long_faults ? 257 : 237;

        truth_at(frame, scenario, &truth_x, &truth_y,
                 &truth_width, &truth_height);
        if (crop_visible) {
            int source = frame - ISP_APPLY_DELAY_FRAMES;
            actual_x = (float)issued_x[source];
            actual_y = (float)issued_y[source];
            captured_view = make_view(
                actual_x, actual_y, IMG_W, IMG_H,
                0, (unsigned int)(frame + 1), capture_time);
        } else {
            actual_x = 0.0f;
            actual_y = 0.0f;
            captured_view = make_view(
                0.0f, 0.0f, SUBSAMPLED_1X_W, SUBSAMPLED_1X_H,
                1, (unsigned int)(frame + 1), capture_time);
        }

        memset(current, 0, sizeof(*current));
        current->view = captured_view;
        if ((frame < 100 || frame > first_gap_end) &&
            (frame < 230 || frame > second_gap_end)) {
            int source_id;
            int publish;
            float noise_x = scenario->noise_x *
                (0.65f * sinf(frame * 1.71f) + 0.35f * sinf(frame * 0.37f));
            float noise_y = scenario->noise_y *
                (0.60f * sinf(frame * 1.43f) + 0.40f * sinf(frame * 0.29f));

            /* Board trace regression expressed as a realistic lineage:
             * private boxes bridge a fragmented temporal ID, then the same
             * new positive ID is confirmed.  Arbitrary positive IDs changing
             * every frame are deliberately not considered identity evidence. */
            if (frame < 70)
                source_id = 100;
            else if (frame < 73)
                source_id = -1;
            else if (frame < 180)
                source_id = 200;
            else if (frame < 182)
                source_id = -1;
            else
                source_id = 300;
            publish = ((frame < 70) || (frame >= 75 && frame < 180) ||
                       frame >= 184) && frame % 17 != 0;
            current->detections[0] = make_abs_detection(
                source_id, truth_x + noise_x, truth_y + noise_y,
                truth_width, truth_height, 0.94f,
                publish, &captured_view);
            current->count = 1;
        }

        if (frame >= DETECTOR_LATENCY_FRAMES) {
            delayed_observation_t *ready = &delayed[
                (frame - DETECTOR_LATENCY_FRAMES) %
                (DETECTOR_LATENCY_FRAMES + 1)];
            tracker_update_detections(
                &tracker, ready->detections, ready->count,
                &ready->view, capture_time);
            if (ready->count > 0 && !ready->detections[0].is_target)
                association_rejects++;
        }

        tracker_reset_frame_flag();
        tracker_predict(&tracker, capture_time);
        issued_x[frame] = hal_x;
        issued_y[frame] = hal_y;

        if (frame > 1) {
            int step_x = abs(issued_x[frame] - issued_x[frame - 1]);
            int step_y = abs(issued_y[frame] - issued_y[frame - 1]);
            int signed_step_x = issued_x[frame] - issued_x[frame - 1];
            int signed_step_y = issued_y[frame] - issued_y[frame - 1];
            int delta_step_x = signed_step_x - previous_step_x;
            int delta_step_y = signed_step_y - previous_step_y;
            int jerk_step_x = delta_step_x - previous_delta_step_x;
            int jerk_step_y = delta_step_y - previous_delta_step_y;
            if (frame > 5) {
                if (step_x > max_step_x)
                    max_step_x = step_x;
                if (step_y > max_step_y)
                    max_step_y = step_y;
                if (abs(delta_step_x) > max_delta_step_x)
                    max_delta_step_x = abs(delta_step_x);
                if (abs(delta_step_y) > max_delta_step_y)
                    max_delta_step_y = abs(delta_step_y);
                if (abs(jerk_step_x) > max_jerk_step_x)
                    max_jerk_step_x = abs(jerk_step_x);
                if (abs(jerk_step_y) > max_jerk_step_y)
                    max_jerk_step_y = abs(jerk_step_y);
            }
            previous_step_x = signed_step_x;
            previous_step_y = signed_step_y;
            previous_delta_step_x = delta_step_x;
            previous_delta_step_y = delta_step_y;
        }
        if (tracker.state == TRACK_STATE_COASTING)
            saw_coasting = 1;
        if (tracker.state == TRACK_STATE_HOLDING)
            saw_holding = 1;
        if (tracker.state == TRACK_STATE_LOST)
            saw_lost = 1;
        if (tracker.state == TRACK_STATE_RECOVERING)
            saw_recovering = 1;
        if (tracker.target_id != 100 || tracker.state == TRACK_STATE_IDLE) {
            fprintf(stderr,
                    "%s: logical target/state lost frame=%d id=%d state=%d\n",
                    scenario->name, frame, tracker.target_id, tracker.state);
            return 1;
        }

        if (frame >= 30 && crop_visible) {
            int nominal_tracking_window =
                frame < 95 ||
                (frame >= (scenario->long_faults ? 130 : 125) &&
                 frame < 225) ||
                frame >= (scenario->long_faults ? 270 : 260);
            float ideal_x = clampf_test(
                truth_x - (1.0f - TRACK_VIEW_ANCHOR_X_RATIO) * IMG_W,
                0.0f, (float)PAN_MAX_X);
            float ideal_y = clampf_test(
                truth_y - (1.0f - TRACK_VIEW_ANCHOR_Y_RATIO) * IMG_H,
                0.0f, (float)PAN_MAX_Y);
            float error_x = actual_x - ideal_x;
            float error_y = actual_y - ideal_y;
            float screen_anchor_x = truth_x - actual_x;
            float screen_anchor_y = truth_y - actual_y;

            /* Fault injection is assessed by target retention, safe framing,
             * and path continuity, not folded into nominal RMS.  The 19-frame
             * and 28-frame gaps force HOLD and LOST respectively; output
             * bounds are still checked across every transition. */
            if (nominal_tracking_window) {
                abs_error_x[metric_count] = fabsf(error_x);
                abs_error_y[metric_count] = fabsf(error_y);
                if (fabsf(error_x) > max_error_x) {
                    max_error_x = fabsf(error_x);
                    max_error_frame_x = frame;
                }
                if (fabsf(error_y) > max_error_y) {
                    max_error_y = fabsf(error_y);
                    max_error_frame_y = frame;
                }
                sum_squared_x += error_x * error_x;
                sum_squared_y += error_y * error_y;
                metric_count++;
            }
            if (screen_anchor_x < 0.0f || screen_anchor_x > IMG_W ||
                screen_anchor_y < 0.0f || screen_anchor_y > IMG_H)
                offscreen_anchor_frames++;
            if (screen_anchor_x < TRACK_VIEW_CORE_MARGIN_X_PX ||
                screen_anchor_x > IMG_W - TRACK_VIEW_CORE_MARGIN_X_PX ||
                screen_anchor_y < TRACK_VIEW_CORE_MARGIN_Y_PX ||
                screen_anchor_y > IMG_H - TRACK_VIEW_CORE_MARGIN_Y_PX) {
                outside_core_frames++;
            }
        }
    }

    qsort(abs_error_x, (size_t)metric_count, sizeof(abs_error_x[0]),
          compare_float);
    qsort(abs_error_y, (size_t)metric_count, sizeof(abs_error_y[0]),
          compare_float);
    {
        float rms_x = (float)sqrt(sum_squared_x / metric_count);
        float rms_y = (float)sqrt(sum_squared_y / metric_count);
        float p95_x = abs_error_x[(metric_count * 95) / 100];
        float p95_y = abs_error_y[(metric_count * 95) / 100];

        printf("tracker_closed_loop[%s]: rms=(%.1f,%.1f) p95=(%.1f,%.1f) "
               "max=(%.1f@%d,%.1f@%d) step=(%d,%d) "
               "delta=(%d,%d) jerk=(%d,%d) "
               "offscreen/core=%d/%d state_coast/hold/lost/recover=%d/%d/%d/%d "
               "rejects=%d\n",
               scenario->name, rms_x, rms_y, p95_x, p95_y,
               max_error_x, max_error_frame_x,
               max_error_y, max_error_frame_y,
               max_step_x, max_step_y,
               max_delta_step_x, max_delta_step_y,
               max_jerk_step_x, max_jerk_step_y,
               offscreen_anchor_frames, outside_core_frames,
               saw_coasting, saw_holding, saw_lost, saw_recovering,
               association_rejects);
        if ((scenario->motion_scale == 0.0f &&
             (rms_x > 2.0f || rms_y > 2.0f ||
              p95_x > 2.0f || p95_y > 2.0f)) ||
            (scenario->motion_scale != 0.0f &&
             (rms_x > 55.0f || rms_y > 45.0f ||
              p95_x > 95.0f || p95_y > 80.0f)) ||
            max_step_x > ACCEPT_MAX_STEP_X ||
            max_step_y > ACCEPT_MAX_STEP_Y ||
            max_delta_step_x > 3 || max_delta_step_y > 3 ||
            max_jerk_step_x > 4 || max_jerk_step_y > 4 ||
            offscreen_anchor_frames > 0 || outside_core_frames > 0 ||
            !saw_coasting ||
            (scenario->long_faults &&
             (!saw_holding || !saw_lost || !saw_recovering)) ||
            association_rejects != 2 ||
            tracker.source_id != 300) {
            fprintf(stderr, "%s: closed-loop acceptance failed\n",
                    scenario->name);
            return 1;
        }
    }

    tracker_reset_frame_flag();
    tracker_stop(&tracker);
    return 0;
}

int main(void)
{
    const scenario_t scenarios[] = {
        {"static", 110.0f, 3.0f, 4.0f, 0.0f, 1},
        {"state-fault", 110.0f, 3.0f, 4.0f, 0.25f, 1},
        {"far", 52.0f, 4.0f, 5.0f, 1.0f, 0},
        {"mid", 110.0f, 3.0f, 4.0f, 1.0f, 0},
        {"near", 260.0f, 2.5f, 3.5f, 1.0f, 0},
    };

    init_app_state();
    for (size_t i = 0; i < sizeof(scenarios) / sizeof(scenarios[0]); i++)
        if (run_scenario(&scenarios[i]) != 0)
            return 1;
    pthread_mutex_destroy(&g_state.pred_mutex);
    pthread_mutex_destroy(&g_state.det_mutex);
    puts("tracker_closed_loop_test: OK");
    return 0;
}
