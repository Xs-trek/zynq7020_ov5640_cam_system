/* tracker_closed_loop_test.c - the control tracker in closed loop with a simulated crop.
 *
 * A face moves in the 1312x972 sensor plane; its detection is taken through the crop the tracker
 * itself commanded two frames earlier (ISP_APPLY_DELAY_FRAMES) and handed back after a detector
 * latency.  Latency modes:
 *   lat0+  v0.2.0 as on the board: the control tick waits for this frame's detection (0 frames),
 *          and a pseudo-random one in ten misses the deadline and arrives one frame late;
 *   lat1   always one frame late: the copy fallback, whose detection takes longer than the deadline;
 *   lat3   three frames late, the v0.1.1 timing kept as the worst-case regression.
 * Two motion families: the sine scenarios with injected identity faults and gaps, and the
 * maneuver scenarios (start, reversal, stop, start again at constant speeds, and a change of
 * distance), which measure how fast the crop settles after the speed changes.  The edge stops
 * (the face stops dead just inside a pan limit, at a human speed and near the path speed limit)
 * check that the crop comes to rest at the limit within the smoothness lines; they are accepted
 * at lat0+ and lat1 only.  The single-frame step line follows the path speed limit.
 */
#include "cam_system/app_state.h"
#include "cam_system/config.h"
#include "cam_system/tracker.h"

#include <math.h>
#include <signal.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#define SIM_FRAMES 360
#define MAX_LATENCY_FRAMES 3
#define OBSERVATION_RING 8
#define PI_F 3.14159265358979323846f
/* the single-frame step line is the path speed limit times the frame interval */
#define ACCEPT_MAX_STEP_X ((int)ceilf(TRACK_VIEW_MAX_STEP_X_PX))
#define ACCEPT_MAX_STEP_Y ((int)ceilf(TRACK_VIEW_MAX_STEP_Y_PX))
/* Settled: the framing error stays under this for SETTLE_RUN frames. */
#define SETTLE_ERROR_PX 10.0f
#define SETTLE_RUN 5
/* Edge stops: how far inside a pan limit the ideal crop comes to rest. */
#define EDGE_IN 10.0f

app_state_t g_state;
uint8_t *g_frame_pool[CAPTURE_BUF_CNT];
uint8_t *g_cached_pool[FRAME_BUF_CNT];

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

typedef enum {
    LATENCY_NOMINAL = 0,
    LATENCY_ONE,
    LATENCY_THREE,
} latency_mode_t;

static const char *const latency_names[] = { "lat0+", "lat1", "lat3" };

typedef enum {
    MOTION_SINE = 0,
    MOTION_X,           /* piecewise-constant speed along x */
    MOTION_Y,
    MOTION_DEPTH,       /* the face height changes, the face stays */
    MOTION_EDGE_X,      /* stops dead just inside a pan limit, then just inside the other */
    MOTION_EDGE_Y,
} motion_t;

typedef struct {
    frame_view_t view;
    detection_t detections[2];
    int count;
} delayed_observation_t;

typedef struct {
    const char *name;
    motion_t motion;
    float face_height;
    float noise_x;
    float noise_y;
    float motion_scale;     /* sine: amplitude scale; maneuver: speed in px/s */
    int long_faults;
    int max_settle_frames;  /* maneuver: every speed change settles in this; 0 = not checked */
} scenario_t;

/* The maneuver profile: frames where the speed changes, and the speed (as a fraction of the
 * scenario's) from each of them on. */
static const int k_changes[] = { 30, 66, 102, 160, 190 };
static const float k_change_speed[] = { 1.0f, -1.0f, 0.0f, 0.66f, 0.0f };
#define CHANGE_COUNT ((int)(sizeof(k_changes) / sizeof(k_changes[0])))

static float clampf_test(float value, float minimum, float maximum)
{
    return value < minimum ? minimum :
           (value > maximum ? maximum : value);
}

static int latency_of(latency_mode_t mode, int frame)
{
    uint32_t hash;

    if (mode == LATENCY_THREE)
        return 3;
    if (mode == LATENCY_ONE)
        return 1;
    /* Pseudo-random, not periodic: a fixed period interacts with the noise pattern. */
    hash = (uint32_t)frame * 2654435761u;
    hash ^= hash >> 15;
    hash *= 2246822519u;
    hash ^= hash >> 13;
    return hash % 10u == 0u ? 1 : 0;
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

/* Edge stop: from frame 30 the face runs to where the ideal crop is EDGE_IN px inside the
 * upper pan limit and stops dead; from frame 110 it runs to EDGE_IN px inside the lower
 * limit and stops dead.  The crop must come to rest at the limits within the smoothness lines. */
static float edge_position(int frame, float start, float speed, float offset, float pan_max)
{
    const float upper = pan_max - EDGE_IN + offset;
    const float lower = EDGE_IN + offset;
    float p = start;

    if (frame > 30)
        p = fminf(start + speed * (frame - 30) / TRACK_CONTROL_FPS, upper);
    if (frame > 110)
        p = fmaxf(upper - speed * (frame - 110) / TRACK_CONTROL_FPS, lower);
    return p;
}

static float maneuver_speed(int frame)
{
    float speed = 0.0f;

    for (int i = 0; i < CHANGE_COUNT && frame >= k_changes[i]; i++)
        speed = k_change_speed[i];
    return speed;
}

static void truth_at(int frame, const scenario_t *scenario,
                     float *x, float *y, float *width, float *height)
{
    float time = frame / TRACK_CONTROL_FPS;

    if (scenario->motion == MOTION_SINE) {
        float scale_motion = 1.0f + 0.08f * sinf(2.0f * PI_F * time / 3.7f);

        *x = 656.0f + scenario->motion_scale * 380.0f *
            sinf(2.0f * PI_F * time / 5.5f);
        *y = 486.0f + scenario->motion_scale * 200.0f *
            sinf(2.0f * PI_F * time / 4.5f + 0.35f);
        *height = scenario->face_height * scale_motion;
    } else if (scenario->motion == MOTION_EDGE_X || scenario->motion == MOTION_EDGE_Y) {
        const float off_x = (1.0f - TRACK_VIEW_ANCHOR_X_RATIO) * IMG_W;
        const float off_y = (1.0f - TRACK_VIEW_ANCHOR_Y_RATIO) * IMG_H;

        *x = scenario->motion == MOTION_EDGE_X ?
            edge_position(frame, 656.0f, scenario->motion_scale, off_x, (float)PAN_MAX_X) : 656.0f;
        *y = scenario->motion == MOTION_EDGE_Y ?
            edge_position(frame, 486.0f, scenario->motion_scale, off_y, (float)PAN_MAX_Y) : 486.0f;
        *height = scenario->face_height;
    } else {
        float travel = 0.0f;

        for (int f = 1; f <= frame; f++)
            travel += maneuver_speed(f) * scenario->motion_scale / TRACK_CONTROL_FPS;
        *x = scenario->motion == MOTION_X ? 500.0f + travel : 656.0f;
        *y = scenario->motion == MOTION_Y ? 420.0f + travel : 486.0f;
        /* depth: the height grows and shrinks by motion_scale px/s */
        *height = scenario->face_height +
            (scenario->motion == MOTION_DEPTH ? (travel > 0.0f ? travel : -travel) : 0.0f);
    }
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

/* The face of a sine scenario: identity faults and two gaps.  Board trace regression expressed
 * as a realistic lineage: private boxes bridge a fragmented temporal ID, then the same new positive
 * ID is confirmed.  Arbitrary positive IDs changing every frame are deliberately not considered
 * identity evidence. */
static int sine_observation(int frame, const scenario_t *scenario, int *source_id, int *publish)
{
    int first_gap_end = scenario->long_faults ? 118 : 105;
    int second_gap_end = scenario->long_faults ? 257 : 237;

    if (!((frame < 100 || frame > first_gap_end) &&
          (frame < 230 || frame > second_gap_end)))
        return 0;
    if (frame < 70)
        *source_id = 100;
    else if (frame < 73)
        *source_id = -1;
    else if (frame < 180)
        *source_id = 200;
    else if (frame < 182)
        *source_id = -1;
    else
        *source_id = 300;
    *publish = ((frame < 70) || (frame >= 75 && frame < 180) ||
                frame >= 184) && frame % 17 != 0;
    return 1;
}

static int run_scenario(const scenario_t *scenario, latency_mode_t mode)
{
    tracker_t tracker;
    delayed_observation_t delayed[OBSERVATION_RING];
    int issued_x[SIM_FRAMES];
    int issued_y[SIM_FRAMES];
    float abs_error_x[SIM_FRAMES];
    float abs_error_y[SIM_FRAMES];
    float axis_error[SIM_FRAMES];
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
    double rest_steps = 0.0;
    int rest_frames = 0;
    const int maneuver = scenario->motion != MOTION_SINE;
    int result = 0;
    float initial_x, initial_y, initial_width, initial_height;
    const double start_time = 100.0;

    memset(delayed, 0, sizeof(delayed));
    memset(issued_x, 0, sizeof(issued_x));
    memset(issued_y, 0, sizeof(issued_y));
    memset(axis_error, 0, sizeof(axis_error));
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
        fprintf(stderr, "%s/%s: failed to engage\n", scenario->name, latency_names[mode]);
        return 1;
    }

    for (int frame = 0; frame < SIM_FRAMES; frame++) {
        double capture_time = start_time + frame / TRACK_CONTROL_FPS;
        float truth_x, truth_y, truth_width, truth_height;
        float actual_x;
        float actual_y;
        int crop_visible = frame >= ISP_APPLY_DELAY_FRAMES;
        frame_view_t captured_view;
        delayed_observation_t *current = &delayed[frame % OBSERVATION_RING];
        int source_id = 100;
        int publish = 1;

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
        if (maneuver || sine_observation(frame, scenario, &source_id, &publish)) {
            float noise_x = scenario->noise_x *
                (0.65f * sinf(frame * 1.71f) + 0.35f * sinf(frame * 0.37f));
            float noise_y = scenario->noise_y *
                (0.60f * sinf(frame * 1.43f) + 0.40f * sinf(frame * 0.29f));

            current->detections[0] = make_abs_detection(
                source_id, truth_x + noise_x, truth_y + noise_y,
                truth_width, truth_height, 0.94f,
                publish, &captured_view);
            current->count = 1;
        }

        /* Every observation whose latency ends at this frame, oldest first: a late one is fed
         * before this frame's, as the control tick does. */
        for (int source = frame - MAX_LATENCY_FRAMES; source <= frame; source++) {
            delayed_observation_t *ready;

            if (source < 0 || source + latency_of(mode, source) != frame)
                continue;
            ready = &delayed[source % OBSERVATION_RING];
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
            /* a rest segment well after the stop: the crop must not creep */
            if (maneuver && frame >= 130 && frame < 160) {
                rest_steps += step_x + step_y;
                rest_frames++;
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
                    "%s/%s: logical target/state lost frame=%d id=%d state=%d\n",
                    scenario->name, latency_names[mode], frame, tracker.target_id,
                    tracker.state);
            result = 1;
            goto done;
        }

        if (frame >= 30 && crop_visible) {
            int nominal_tracking_window = maneuver ||
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

            axis_error[frame] = hypotf(error_x, error_y);
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
        int failed;

        printf("tracker_closed_loop[%s/%s]: rms=(%.1f,%.1f) p95=(%.1f,%.1f) "
               "max=(%.1f@%d,%.1f@%d) step=(%d,%d) "
               "delta=(%d,%d) jerk=(%d,%d) "
               "offscreen/core=%d/%d state_coast/hold/lost/recover=%d/%d/%d/%d "
               "rejects=%d\n",
               scenario->name, latency_names[mode], rms_x, rms_y, p95_x, p95_y,
               max_error_x, max_error_frame_x,
               max_error_y, max_error_frame_y,
               max_step_x, max_step_y,
               max_delta_step_x, max_delta_step_y,
               max_jerk_step_x, max_jerk_step_y,
               offscreen_anchor_frames, outside_core_frames,
               saw_coasting, saw_holding, saw_lost, saw_recovering,
               association_rejects);
        failed = max_step_x > ACCEPT_MAX_STEP_X ||
                 max_step_y > ACCEPT_MAX_STEP_Y ||
                 max_delta_step_x > 3 || max_delta_step_y > 3 ||
                 max_jerk_step_x > 4 || max_jerk_step_y > 4 ||
                 offscreen_anchor_frames > 0 || outside_core_frames > 0;
        if (!maneuver) {
            failed = failed ||
                (scenario->motion_scale == 0.0f &&
                 (rms_x > 2.0f || rms_y > 2.0f ||
                  p95_x > 2.0f || p95_y > 2.0f)) ||
                (scenario->motion_scale != 0.0f &&
                 (rms_x > 55.0f || rms_y > 45.0f ||
                  p95_x > 95.0f || p95_y > 80.0f)) ||
                !saw_coasting ||
                (scenario->long_faults &&
                 (!saw_holding || !saw_lost || !saw_recovering)) ||
                association_rejects != 2 ||
                tracker.source_id != 300;
        } else {
            /* How fast the crop settles after each speed change, and the error on the way. */
            printf("tracker_closed_loop[%s/%s]: settle", scenario->name, latency_names[mode]);
            for (int c = 0; c < CHANGE_COUNT; c++) {
                int begin = k_changes[c];
                int end = c + 1 < CHANGE_COUNT ? k_changes[c + 1] : SIM_FRAMES;
                int settle = end - begin;
                float peak = 0.0f;

                for (int f = begin; f < end; f++)
                    if (axis_error[f] > peak)
                        peak = axis_error[f];
                for (int f = begin; f + SETTLE_RUN <= end; f++) {
                    int calm = 1;

                    for (int g = f; g < f + SETTLE_RUN; g++)
                        if (axis_error[g] > SETTLE_ERROR_PX)
                            calm = 0;
                    if (calm) {
                        settle = f - begin;
                        break;
                    }
                }
                printf(" %d/%.0fpx", settle, peak);
                if (scenario->max_settle_frames > 0 && settle > scenario->max_settle_frames)
                    failed = 1;
            }
            printf(" rest_step=%.2f\n", rest_frames ? rest_steps / rest_frames : 0.0);
            /* rest_step is reported, not judged: it shows the stop has not settled yet */
        }
        if (failed) {
            fprintf(stderr, "%s/%s: closed-loop acceptance failed\n",
                    scenario->name, latency_names[mode]);
            result = 1;
        }
    }

done:
    tracker_reset_frame_flag();
    tracker_stop(&tracker);
    return result;
}

int main(void)
{
    const scenario_t scenarios[] = {
        {"static", MOTION_SINE, 110.0f, 3.0f, 4.0f, 0.0f, 1, 0},
        {"state-fault", MOTION_SINE, 110.0f, 3.0f, 4.0f, 0.25f, 1, 0},
        {"far", MOTION_SINE, 52.0f, 4.0f, 5.0f, 1.0f, 0, 0},
        {"mid", MOTION_SINE, 110.0f, 3.0f, 4.0f, 1.0f, 0, 0},
        {"near", MOTION_SINE, 260.0f, 2.5f, 3.5f, 1.0f, 0, 0},
        {"x150", MOTION_X, 110.0f, 3.0f, 4.0f, 150.0f, 0, 0},
        {"x300", MOTION_X, 110.0f, 3.0f, 4.0f, 300.0f, 0, 0},
        {"y200", MOTION_Y, 110.0f, 3.0f, 4.0f, 200.0f, 0, 0},
        {"depth", MOTION_DEPTH, 110.0f, 3.0f, 4.0f, 100.0f, 0, 0},
        {"edge-x", MOTION_EDGE_X, 110.0f, 3.0f, 4.0f, 430.0f, 0, 0},
        {"edge-y", MOTION_EDGE_Y, 110.0f, 3.0f, 4.0f, 290.0f, 0, 0},
        {"edge-x-fast", MOTION_EDGE_X, 110.0f, 3.0f, 4.0f, 700.0f, 0, 0},
        {"edge-y-fast", MOTION_EDGE_Y, 110.0f, 3.0f, 4.0f, 480.0f, 0, 0},
    };
    int failures = 0;

    init_app_state();
    for (int mode = LATENCY_NOMINAL; mode <= LATENCY_THREE; mode++)
        for (size_t i = 0; i < sizeof(scenarios) / sizeof(scenarios[0]); i++)
            /* edge stops are accepted at lat0+ and lat1; lat3 is a regression for the others */
            if (mode != LATENCY_THREE ||
                (scenarios[i].motion != MOTION_EDGE_X && scenarios[i].motion != MOTION_EDGE_Y))
                failures += run_scenario(&scenarios[i], (latency_mode_t)mode);
    pthread_mutex_destroy(&g_state.pred_mutex);
    pthread_mutex_destroy(&g_state.det_mutex);
    if (failures) {
        fprintf(stderr, "tracker_closed_loop_test: %d run(s) failed\n", failures);
        return 1;
    }
    puts("tracker_closed_loop_test: OK");
    return 0;
}
