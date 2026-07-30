/*
 * tracker.c - timestamped single-target Kalman tracker and viewport controller
 *
 * Target state is always expressed in the absolute 2x-subsampled sensor plane.
 * A delayed detector result is transformed with the view metadata captured with
 * that frame, corrected at its observation time, then replayed to the present.
 */
#include "cam_system/tracker.h"
#include "platform/hal_ov5640.h"
#include "cam_system/config.h"
#include "control/tracker_servo.h"

#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#define OX_MIN 0
#define OX_MAX (SUBSAMPLED_1X_W - IMG_W)
#define OY_MIN 0
#define OY_MAX (SUBSAMPLED_1X_H - IMG_H)

static int tracker_verbose_log;
static float tracker_diag_interval_sec;
static float tracker_q_xy;
static float tracker_q_scale;
static float tracker_nis_gate;
static float tracker_exact_nis_gate;
static float tracker_measurement_sigma_base;
static float tracker_exact_gate_base;
static float tracker_exact_gate_speed;
static float tracker_exact_gate_max;
static float tracker_miss_grace_sec;
static float tracker_coast_sec;
static float tracker_hold_sec;
static float tracker_lost_sec;
static float tracker_command_coast_sec;

static int scaler_disabled;
static int current_xoff = ISP_DEFAULT_XOFF;
static int current_yoff = ISP_DEFAULT_YOFF;
static int last_written_xoff = -1;
static int last_written_yoff = -1;
static int isp_written_this_frame;

static int diag_count;
static double diag_mean_x;
static double diag_mean_y;
static double diag_m2_x;
static double diag_m2_y;
static int diag_last_x = -1;
static int diag_last_y = -1;
static int diag_max_jump;
static int diag_last_step_x;
static int diag_last_step_y;
static int diag_last_delta_x;
static int diag_last_delta_y;
static int diag_have_step;
static int diag_have_delta;
static int diag_max_delta_x;
static int diag_max_delta_y;
static int diag_max_jerk_x;
static int diag_max_jerk_y;
static double diag_last_time;

static float clampf_local(float v, float lo, float hi)
{
    return v < lo ? lo : (v > hi ? hi : v);
}

static int clampi(int v, int lo, int hi)
{
    return v < lo ? lo : (v > hi ? hi : v);
}

static int env_flag_enabled(const char *name)
{
    const char *value = getenv(name);
    return value && value[0] != '\0' && strcmp(value, "0") != 0;
}

static void tracker_load_config(void)
{
    const char *diag = getenv("CAM_TRACKER_DIAG_EVERY");

    tracker_verbose_log = env_flag_enabled("CAM_TRACKER_VERBOSE");
    tracker_diag_interval_sec = diag ? strtof(diag, NULL) : 0.0f;
    if (tracker_diag_interval_sec < 0.0f)
        tracker_diag_interval_sec = 0.0f;
    /* The release has one validated behavior.  Environment variables remain
     * diagnostics-only so board testing cannot silently become a parameter
     * sweep with an unrecorded controller/association variant. */
    tracker_q_xy = TRACK_KF_ACCEL_VAR_XY;
    tracker_q_scale = TRACK_KF_ACCEL_VAR_SCALE;
    tracker_nis_gate = TRACK_ASSOC_NIS_GATE;
    tracker_exact_nis_gate = TRACK_ASSOC_EXACT_NIS_GATE;
    tracker_measurement_sigma_base = TRACK_MEAS_SIGMA_BASE_PX;
    tracker_exact_gate_base = TRACK_EXACT_GATE_BASE_PX;
    tracker_exact_gate_speed = TRACK_EXACT_GATE_SPEED_PX_S;
    tracker_exact_gate_max = TRACK_EXACT_GATE_MAX_PX;
    tracker_miss_grace_sec = TRACK_MISS_GRACE_SEC;
    tracker_coast_sec = TRACK_COAST_SEC;
    tracker_hold_sec = TRACK_HOLD_SEC;
    tracker_lost_sec = TRACK_LOST_SEC;
    tracker_command_coast_sec = TRACK_COMMAND_COAST_SEC;
}

static void tracker_reset_diagnostics(void)
{
    diag_count = 0;
    diag_mean_x = diag_mean_y = 0.0;
    diag_m2_x = diag_m2_y = 0.0;
    diag_last_x = diag_last_y = -1;
    diag_max_jump = 0;
    diag_last_step_x = diag_last_step_y = 0;
    diag_last_delta_x = diag_last_delta_y = 0;
    diag_have_step = diag_have_delta = 0;
    diag_max_delta_x = diag_max_delta_y = 0;
    diag_max_jerk_x = diag_max_jerk_y = 0;
    diag_last_time = 0.0;
}

static void notify_status_changed(void)
{
    app_state_inc_u32(&g_state.status_seq);
}

static const char *state_name(track_state_t state)
{
    switch (state) {
    case TRACK_STATE_IDLE: return "IDLE";
    case TRACK_STATE_SELECTED: return "SELECTED";
    case TRACK_STATE_ENGAGING: return "ENGAGING";
    case TRACK_STATE_TRACKING: return "TRACKING";
    case TRACK_STATE_COASTING: return "COASTING";
    case TRACK_STATE_HOLDING: return "HOLDING";
    case TRACK_STATE_LOST: return "LOST";
    case TRACK_STATE_RECOVERING: return "RECOVERING";
    default: return "UNKNOWN";
    }
}

static void set_state(tracker_t *trk, track_state_t state)
{
    if (trk->state == state)
        return;
    trk->state = state;
    if (state == TRACK_STATE_HOLDING || state == TRACK_STATE_LOST) {
        /* State changes may freeze the reference but never own actuator
         * velocity or acceleration.  tracker_predict carries one continuous
         * jerk-limited brake through HOLD, LOST and RECOVER. */
        trk->view.x.response_direction = 0;
        trk->view.y.response_direction = 0;
        trk->view.reference_x.pos = trk->view.x.pos;
        trk->view.reference_x.vel = 0.0f;
        trk->view.reference_y.pos = trk->view.y.pos;
        trk->view.reference_y.vel = 0.0f;
    }
    printf("[tracker] -> %s\n", state_name(state));
    notify_status_changed();
}

static void det_to_abs(const detection_t *det, const frame_view_t *view,
                       float *cx, float *cy, float *width, float *height)
{
    float u = det->bbox[0] + det->bbox[2] * 0.5f;
    /* A point just above the box centre is materially more stable for a face:
     * chin/neck box-height changes no longer look like vertical head motion. */
    float v = det->bbox[1] + det->bbox[3] * TRACK_FACE_ANCHOR_Y_RATIO;

    if (view->flip_x)
        u = 1.0f - u;
    if (view->flip_y)
        v = 1.0f - v;
    *cx = view->roi_x + u * view->roi_w;
    *cy = view->roi_y + v * view->roi_h;
    *width = det->bbox[2] * view->roi_w;
    *height = det->bbox[3] * view->roi_h;
}

static float anchor_to_box_center_y(float anchor_y, float height,
                                    const frame_view_t *view)
{
    float anchor_from_center =
        (0.5f - TRACK_FACE_ANCHOR_Y_RATIO) * height;

    return anchor_y + (view->flip_y ? -anchor_from_center
                                    : anchor_from_center);
}

static float viewport_anchor_offset(float ratio, float extent, int flipped)
{
    return (flipped ? 1.0f - ratio : ratio) * extent;
}

static float bbox_iou(float ax, float ay, float aw, float ah,
                      float bx, float by, float bw, float bh)
{
    float ax0 = ax - aw * 0.5f;
    float ay0 = ay - ah * 0.5f;
    float ax1 = ax + aw * 0.5f;
    float ay1 = ay + ah * 0.5f;
    float bx0 = bx - bw * 0.5f;
    float by0 = by - bh * 0.5f;
    float bx1 = bx + bw * 0.5f;
    float by1 = by + bh * 0.5f;
    float ix0 = fmaxf(ax0, bx0);
    float iy0 = fmaxf(ay0, by0);
    float ix1 = fminf(ax1, bx1);
    float iy1 = fminf(ay1, by1);
    float inter = fmaxf(0.0f, ix1 - ix0) * fmaxf(0.0f, iy1 - iy0);
    float area_a = fmaxf(0.0f, aw) * fmaxf(0.0f, ah);
    float area_b = fmaxf(0.0f, bw) * fmaxf(0.0f, bh);

    return inter / (area_a + area_b - inter + 1e-6f);
}

static void kf_axis_init(tracker_kf_axis_t *axis, float position)
{
    axis->pos = position;
    axis->vel = 0.0f;
    axis->p00 = 64.0f;
    axis->p01 = 0.0f;
    axis->p10 = 0.0f;
    axis->p11 = 4096.0f;
}

static void kf_init(tracker_kf_t *kf, float x, float y, float height)
{
    memset(kf, 0, sizeof(*kf));
    kf_axis_init(&kf->x, x);
    kf_axis_init(&kf->y, y);
    kf_axis_init(&kf->log_h, logf(fmaxf(height, 1.0f)));
    kf->log_h.p00 = 0.04f;
    kf->log_h.p11 = 1.0f;
    kf->initialized = 1;
}

static void kf_axis_predict(tracker_kf_axis_t *axis, float dt, float accel_var)
{
    float dt2 = dt * dt;
    float dt3 = dt2 * dt;
    float dt4 = dt2 * dt2;
    float p00 = axis->p00;
    float p01 = axis->p01;
    float p10 = axis->p10;
    float p11 = axis->p11;

    axis->pos += axis->vel * dt;
    axis->p00 = p00 + dt * (p01 + p10) + dt2 * p11 + accel_var * dt4 * 0.25f;
    axis->p01 = p01 + dt * p11 + accel_var * dt3 * 0.5f;
    axis->p10 = p10 + dt * p11 + accel_var * dt3 * 0.5f;
    axis->p11 = p11 + accel_var * dt2;
}

static void kf_predict_state(tracker_kf_t *kf, double from, double to)
{
    double remaining;

    if (!kf->initialized || to <= from)
        return;
    remaining = to - from;
    while (remaining > 0.0) {
        float dt = (float)fmin(remaining, 0.1);
        kf_axis_predict(&kf->x, dt, tracker_q_xy);
        kf_axis_predict(&kf->y, dt, tracker_q_xy);
        kf_axis_predict(&kf->log_h, dt, tracker_q_scale);
        remaining -= dt;
    }
}

static void kf_axis_update(tracker_kf_axis_t *axis, float measurement,
                           float measurement_var)
{
    float p00 = axis->p00;
    float p01 = axis->p01;
    float p10 = axis->p10;
    float p11 = axis->p11;
    float innovation_var = p00 + fmaxf(measurement_var, 1e-6f);
    float k0 = p00 / innovation_var;
    float k1 = p10 / innovation_var;
    float residual = measurement - axis->pos;

    axis->pos += k0 * residual;
    axis->vel += k1 * residual;
    axis->p00 = (1.0f - k0) * p00;
    axis->p01 = (1.0f - k0) * p01;
    axis->p10 = p10 - k1 * p00;
    axis->p11 = p11 - k1 * p01;
    axis->p01 = axis->p10 = 0.5f * (axis->p01 + axis->p10);
}

static void history_reset(tracker_t *trk, double time_sec)
{
    trk->history_count = 1;
    trk->history[0].time_sec = time_sec;
    trk->history[0].kf = trk->kf;
}

static void history_append(tracker_t *trk, double time_sec)
{
    int index;

    if (trk->history_count > 0 &&
        fabs(trk->history[trk->history_count - 1].time_sec - time_sec) < 1e-6) {
        trk->history[trk->history_count - 1].kf = trk->kf;
        return;
    }
    if (trk->history_count == TRACK_HISTORY_SIZE) {
        memmove(&trk->history[0], &trk->history[1],
                sizeof(trk->history[0]) * (TRACK_HISTORY_SIZE - 1));
        trk->history_count--;
    }
    index = trk->history_count++;
    trk->history[index].time_sec = time_sec;
    trk->history[index].kf = trk->kf;
}

static int state_at_time(const tracker_t *trk, double time_sec,
                         tracker_kf_t *state)
{
    int base = -1;

    if (!trk->kf.initialized)
        return 0;
    for (int i = 0; i < trk->history_count; i++) {
        if (trk->history[i].time_sec <= time_sec)
            base = i;
        else
            break;
    }
    if (base >= 0) {
        *state = trk->history[base].kf;
        kf_predict_state(state, trk->history[base].time_sec, time_sec);
    } else if (trk->history_count > 0) {
        *state = trk->history[0].kf;
    } else {
        *state = trk->kf;
        kf_predict_state(state, trk->kf_time_sec, time_sec);
    }
    return 1;
}

static void measurement_variance(const detection_t *det,
                                 const frame_view_t *view, float abs_height,
                                 float *r_x, float *r_y,
                                 float *r_log_height)
{
    float confidence = clampf_local(det->confidence, 0.0f, 1.0f);
    float output_height = abs_height * (float)view->output_h /
                          fmaxf(view->roi_h, 1.0f);
    float sigma_output = tracker_measurement_sigma_base +
                         4.0f * (1.0f - confidence) +
                         20.0f / fmaxf(output_height, 20.0f);
    float scale = 0.5f * (view->roi_w / (float)view->output_w +
                          view->roi_h / (float)view->output_h);
    float sigma_abs = sigma_output * scale;
    float sigma_log = 0.04f + 0.18f * (1.0f - confidence);

    *r_x = sigma_abs * sigma_abs;
    *r_y = *r_x * TRACK_MEAS_Y_VAR_SCALE;
    *r_log_height = sigma_log * sigma_log;
}

static void delayed_measurement_update(tracker_t *trk,
                                       double measurement_time,
                                       double now_sec,
                                       float x, float y, float height,
                                       float r_x, float r_y,
                                       float r_log_height,
                                       float *innovation_x,
                                       float *innovation_y)
{
    tracker_kf_t corrected;
    double corrected_time;
    int base = -1;

    for (int i = 0; i < trk->history_count; i++) {
        if (trk->history[i].time_sec <= measurement_time)
            base = i;
        else
            break;
    }

    if (base >= 0) {
        corrected = trk->history[base].kf;
        corrected_time = trk->history[base].time_sec;
    } else if (trk->history_count > 0) {
        corrected = trk->history[0].kf;
        corrected_time = trk->history[0].time_sec;
        r_x *= 2.0f;
        r_y *= 2.0f;
        r_log_height *= 2.0f;
        measurement_time = corrected_time;
    } else {
        corrected = trk->kf;
        corrected_time = trk->kf_time_sec;
    }

    kf_predict_state(&corrected, corrected_time, measurement_time);
    corrected_time = measurement_time;
    *innovation_x = x - corrected.x.pos;
    *innovation_y = y - corrected.y.pos;
    /* Association is the outlier decision.  Clipping an accepted residual
     * creates a second, hidden motion model and biases velocity estimates. */
    kf_axis_update(&corrected.x, x, r_x);
    kf_axis_update(&corrected.y, y, r_y);
    kf_axis_update(&corrected.log_h, logf(fmaxf(height, 1.0f)), r_log_height);

    for (int i = base + 1; i < trk->history_count; i++) {
        kf_predict_state(&corrected, corrected_time, trk->history[i].time_sec);
        corrected_time = trk->history[i].time_sec;
        trk->history[i].kf = corrected;
    }
    kf_predict_state(&corrected, corrected_time, now_sec);
    trk->kf = corrected;
    trk->kf_time_sec = now_sec;
}

int tracker_write_isp_offset(int x, int y)
{
    int requested_x = clampi(x, OX_MIN, OX_MAX);
    int requested_y = clampi(y, OY_MIN, OY_MAX);

    if (isp_written_this_frame)
        return -1;
    if (requested_x == last_written_xoff && requested_y == last_written_yoff)
        return 0;
    if (ov5640_apply_isp_offset(requested_x, requested_y) < 0) {
        fprintf(stderr, "[tracker] ISP offset write failed (%d,%d)\n",
                requested_x, requested_y);
        return -1;
    }
    current_xoff = requested_x;
    current_yoff = requested_y;
    last_written_xoff = requested_x;
    last_written_yoff = requested_y;
    isp_written_this_frame = 1;
    return 0;
}

void tracker_reset_frame_flag(void)
{
    isp_written_this_frame = 0;
}

void tracker_get_isp_offset(int *xoff, int *yoff)
{
    *xoff = current_xoff;
    *yoff = current_yoff;
}

void tracker_snapshot_view(frame_view_t *view, int xoff, int yoff,
                           unsigned int frame_seq, double capture_time_sec)
{
    memset(view, 0, sizeof(*view));
    view->frame_seq = frame_seq;
    view->capture_time_sec = capture_time_sec;
    view->output_w = IMG_W;
    view->output_h = IMG_H;
    view->flip_x = 1;
    view->flip_y = 1;
    view->scaler_enabled = !scaler_disabled;
    if (scaler_disabled) {
        view->roi_x = (float)xoff;
        view->roi_y = (float)yoff;
        view->roi_w = (float)IMG_W;
        view->roi_h = (float)IMG_H;
    } else {
        view->roi_x = 0.0f;
        view->roi_y = 0.0f;
        view->roi_w = (float)SUBSAMPLED_1X_W;
        view->roi_h = (float)SUBSAMPLED_1X_H;
    }
}

static int scaler_restore(void)
{
    if (!scaler_disabled)
        return 0;
    if (ov5640_set_scaler_enabled(1) < 0) {
        fprintf(stderr, "[tracker] scaler restore failed\n");
        return -1;
    }
    scaler_disabled = 0;
    return 0;
}

static int scaler_disable(void)
{
    if (scaler_disabled)
        return 0;
    if (ov5640_set_scaler_enabled(0) < 0) {
        fprintf(stderr, "[tracker] scaler disable failed\n");
        return -1;
    }
    scaler_disabled = 1;
    return 0;
}

void tracker_init(tracker_t *trk)
{
    tracker_load_config();
    tracker_reset_diagnostics();
    memset(trk, 0, sizeof(*trk));
    trk->state = TRACK_STATE_IDLE;
    trk->target_id = -1;
    trk->source_id = -1;
    trk->target_class = -1;
    trk->aspect_ratio = 1.0f;
    current_xoff = ISP_DEFAULT_XOFF;
    current_yoff = ISP_DEFAULT_YOFF;
    last_written_xoff = -1;
    last_written_yoff = -1;
    isp_written_this_frame = 0;
    printf("[tracker] Init timestamped-kf q_xy=%.1f nis=%.2f/%.2f "
           "exact_gate=%.0f+%.0fpx/s<=%.0f sigma=%.1f y_var=%.1f "
           "anchor=face%.2f/view(%.2f,%.2f) "
           "timeouts=%.2f/%.2f/%.2f/%.2f command_coast=%.2f "
           "trust=rebind%d private_var=%.1f/%.1f "
           "lead=%.1fms max_step=(%.0f,%.0f) "
           "reference=gh(%.2f,%.2f) "
           "composer_x=band%.0f..%.0f omega%.1f vmax%.0f amax%.0f jmax%.0f "
           "composer_y=band%.0f..%.0f omega%.1f vmax%.0f amax%.0f jmax%.0f\n",
           tracker_q_xy, tracker_nis_gate, tracker_exact_nis_gate,
           tracker_exact_gate_base, tracker_exact_gate_speed,
           tracker_exact_gate_max, tracker_measurement_sigma_base,
           TRACK_MEAS_Y_VAR_SCALE, TRACK_FACE_ANCHOR_Y_RATIO,
           TRACK_VIEW_ANCHOR_X_RATIO, TRACK_VIEW_ANCHOR_Y_RATIO,
           tracker_miss_grace_sec, tracker_coast_sec,
           tracker_hold_sec, tracker_lost_sec,
           tracker_command_coast_sec, TRACK_REBIND_CONFIRM_FRAMES,
           TRACK_ASSOC_PRIVATE_VAR_SCALE,
           TRACK_UNASSOCIATED_VAR_SCALE,
           TRACK_COMMAND_LEAD_SEC * 1000.0f,
           TRACK_VIEW_MAX_STEP_X_PX, TRACK_VIEW_MAX_STEP_Y_PX,
           TRACK_REFERENCE_ALPHA, TRACK_REFERENCE_BETA,
           TRACK_VIEW_NOISE_ENTER_X_MIN_PX,
           TRACK_VIEW_NOISE_ENTER_X_MAX_PX, TRACK_VIEW_PATH_OMEGA_X,
           TRACK_VIEW_MAX_VEL_X, TRACK_VIEW_MAX_ACCEL_X,
           TRACK_VIEW_MAX_JERK_X,
           TRACK_VIEW_NOISE_ENTER_Y_MIN_PX,
           TRACK_VIEW_NOISE_ENTER_Y_MAX_PX, TRACK_VIEW_PATH_OMEGA_Y,
           TRACK_VIEW_MAX_VEL_Y, TRACK_VIEW_MAX_ACCEL_Y,
           TRACK_VIEW_MAX_JERK_Y);
}

void tracker_apply_cmd(tracker_t *trk, track_cmd_t cmd, int tid)
{
    if (cmd == TRACK_CMD_NONE)
        return;

    if (cmd == TRACK_CMD_SELECT) {
        frame_view_t view;
        if (tracker_is_active(trk)) {
            printf("[tracker] SELECT rejected while %s\n",
                   state_name(trk->state));
            return;
        }
        trk->target_id = tid;
        trk->source_id = tid;
        trk->select_valid = 0;
        pthread_mutex_lock(&g_state.det_mutex);
        view = g_state.det_frame_view;
        for (int i = 0; i < g_state.det_count; i++) {
            detection_t *det = &g_state.detections[i];
            float width;
            if (det->track_id != tid || !det->publish)
                continue;
            det_to_abs(det, &view,
                       &trk->select_abs_x, &trk->select_abs_y,
                       &width, &trk->select_abs_h);
            trk->select_aspect = width / fmaxf(trk->select_abs_h, 1.0f);
            trk->select_confidence = det->confidence;
            trk->select_time_sec = view.capture_time_sec;
            trk->target_class = det->class_id;
            trk->select_valid = 1;
            break;
        }
        pthread_mutex_unlock(&g_state.det_mutex);
        if (!trk->select_valid) {
            trk->target_id = -1;
            trk->source_id = -1;
            set_state(trk, TRACK_STATE_IDLE);
            printf("[tracker] SELECT failed id=%d\n", tid);
        } else {
            set_state(trk, TRACK_STATE_SELECTED);
            printf("[tracker] SELECT id=%d abs=(%.1f,%.1f) h=%.1f t=%.3f\n",
                   tid, trk->select_abs_x, trk->select_abs_y,
                   trk->select_abs_h, trk->select_time_sec);
        }
        return;
    }

    if (cmd == TRACK_CMD_START) {
        int initial_x;
        int initial_y;
        if (trk->state != TRACK_STATE_SELECTED ||
            !trk->select_valid || trk->target_id < 0) {
            printf("[tracker] Cannot start: no valid selection\n");
            return;
        }
        initial_x = clampi((int)lroundf(
                               trk->select_abs_x - viewport_anchor_offset(
                                   TRACK_VIEW_ANCHOR_X_RATIO, IMG_W, 1)),
                           OX_MIN, OX_MAX);
        initial_y = clampi((int)lroundf(
                               trk->select_abs_y - viewport_anchor_offset(
                                   TRACK_VIEW_ANCHOR_Y_RATIO, IMG_H, 1)),
                           OY_MIN, OY_MAX);
        if (scaler_disable() < 0)
            return;
        last_written_xoff = -1;
        last_written_yoff = -1;
        isp_written_this_frame = 0;
        if (tracker_write_isp_offset(initial_x, initial_y) < 0) {
            scaler_restore();
            return;
        }
        kf_init(&trk->kf, trk->select_abs_x, trk->select_abs_y,
                trk->select_abs_h);
        trk->kf_time_sec = trk->select_time_sec;
        history_reset(trk, trk->select_time_sec);
        tracker_servo_start(&trk->view, (float)initial_x, (float)initial_y);
        trk->aspect_ratio = clampf_local(trk->select_aspect, 0.4f, 2.5f);
        trk->last_sub_x = trk->select_abs_x;
        trk->last_sub_y = trk->select_abs_y;
        trk->last_bbox_h_norm = trk->select_abs_h / (float)IMG_H;
        trk->last_bbox_w_norm = trk->last_bbox_h_norm * trk->aspect_ratio;
        trk->last_seen_time = trk->select_time_sec;
        trk->last_receive_time = -1.0;
        trk->last_predict_time = 0.0;
        trk->id_miss_count = 0;
        trk->rebind_source_id = -1;
        trk->rebind_hits = 0;
        trk->rebind_positive_hits = 0;
        trk->rebind_last_measurement_time = -1.0;
        trk->recovery_hits = 0;
        trk->recovery_last_measurement_time = -1.0;
        trk->measurement_count = 0;
        trk->strong_measurement_count = 0;
        trk->weak_measurement_count = 0;
        trk->measurement_age_sum_ms = 0.0;
        trk->measurement_age_max_ms = 0.0;
        tracker_reset_diagnostics();
        set_state(trk, TRACK_STATE_ENGAGING);
        app_state_store_int(&g_state.tracking_active, 1);
        app_state_store_int(&g_state.tracking_target_id, trk->target_id);
        notify_status_changed();
        printf("[tracker] START id=%d off=(%d,%d)\n",
               trk->target_id, initial_x, initial_y);
        return;
    }

    if (cmd == TRACK_CMD_STOP) {
        tracker_stop(trk);
        printf("[tracker] Stopped by user\n");
    }
}

typedef struct {
    int exact_seen;
    float exact_nis;
    float exact_distance;
    float exact_distance_gate;
    float exact_scale_ratio;
    int rebuild_seen;
    float rebuild_nis;
    float rebuild_distance;
    float rebuild_iou;
    float rebuild_scale_ratio;
} association_diag_t;

static void reset_rebind_candidate(tracker_t *trk)
{
    trk->rebind_source_id = -1;
    trk->rebind_hits = 0;
    trk->rebind_positive_hits = 0;
    trk->rebind_last_measurement_time = -1.0;
    trk->rebind_x = 0.0f;
    trk->rebind_y = 0.0f;
    trk->rebind_height = 0.0f;
}

static int advance_rebind_candidate(tracker_t *trk, const detection_t *det,
                                    double measurement_time,
                                    float x, float y, float height)
{
    int candidate_id = det->track_id;
    int compatible = 0;

    if (trk->rebind_hits > 0 &&
        measurement_time > trk->rebind_last_measurement_time + 1e-6) {
        float dt = (float)(measurement_time -
                           trk->rebind_last_measurement_time);
        float distance_gate = clampf_local(
            tracker_exact_gate_base + tracker_exact_gate_speed * dt,
            tracker_exact_gate_base, tracker_exact_gate_max);
        float distance = hypotf(x - trk->rebind_x, y - trk->rebind_y);
        float ratio = height / fmaxf(trk->rebind_height, 1.0f);
        int same_lineage = trk->rebind_source_id < 0 || candidate_id < 0 ||
                           trk->rebind_source_id == candidate_id;

        compatible = dt <= tracker_miss_grace_sec && same_lineage &&
                     distance <= distance_gate &&
                     ratio >= 0.67f && ratio <= 1.50f;
    }

    if (!compatible) {
        trk->rebind_hits = 1;
        trk->rebind_source_id = candidate_id;
        trk->rebind_positive_hits = candidate_id >= 0 ? 1 : 0;
    } else {
        if (trk->rebind_hits < TRACK_REBIND_CONFIRM_FRAMES)
            trk->rebind_hits++;
        if (candidate_id < 0) {
            trk->rebind_positive_hits = 0;
        } else if (trk->rebind_source_id == candidate_id) {
            if (trk->rebind_positive_hits < TRACK_REBIND_CONFIRM_FRAMES)
                trk->rebind_positive_hits++;
        } else {
            trk->rebind_source_id = candidate_id;
            trk->rebind_positive_hits = 1;
        }
    }
    trk->rebind_last_measurement_time = measurement_time;
    trk->rebind_x = x;
    trk->rebind_y = y;
    trk->rebind_height = height;
    return trk->rebind_hits >= TRACK_REBIND_CONFIRM_FRAMES;
}

static int find_best_detection(tracker_t *trk, detection_t *dets, int count,
                               const frame_view_t *view,
                               tracker_kf_t *predicted,
                               float *best_x, float *best_y,
                               float *best_w, float *best_h,
                               float *best_r_x, float *best_r_y,
                               float *best_r_log,
                               float *accepted_nis,
                               float *accepted_distance,
                               float *accepted_distance_gate,
                               int *matched_exact,
                               association_diag_t *diag)
{
    int best = -1;
    float best_score = 1e30f;
    float predicted_h = expf(predicted->log_h.pos);
    float predicted_w = predicted_h * trk->aspect_ratio;
    float measurement_gap = (float)fmax(0.0,
        view->capture_time_sec - trk->last_seen_time);
    float exact_distance_gate = clampf_local(
        tracker_exact_gate_base + tracker_exact_gate_speed * measurement_gap,
        tracker_exact_gate_base, tracker_exact_gate_max);
    int best_is_exact = 0;

    *matched_exact = 0;
    memset(diag, 0, sizeof(*diag));
    diag->exact_nis = INFINITY;
    diag->exact_distance = INFINITY;
    diag->exact_distance_gate = exact_distance_gate;
    diag->exact_scale_ratio = 0.0f;
    diag->rebuild_nis = INFINITY;
    diag->rebuild_distance = INFINITY;
    diag->rebuild_iou = 0.0f;
    diag->rebuild_scale_ratio = 0.0f;
    /* Once selected, logical identity is owned by this tracker.  A valid
     * exact source ID still wins to prevent switching between nearby faces.
     * Non-exact candidates are only admitted to the provisional rebind path;
     * tracker_update_detections applies the consecutive-observation trust
     * contract before they may renew identity or recovery state. */
    for (int i = 0; i < count; i++) {
        float x, y, width, height;
        float r_x, r_y, r_log;
        float dx, dy, distance, nis, ratio, iou, score;
        int exact = dets[i].track_id >= 0 &&
                    dets[i].track_id == trk->source_id;

        if (trk->target_class >= 0 && dets[i].class_id != trk->target_class)
            continue;
        if (exact)
            diag->exact_seen = 1;
        det_to_abs(&dets[i], view, &x, &y, &width, &height);
        if (height < 2.0f || width < 2.0f)
            continue;
        measurement_variance(&dets[i], view, height,
                             &r_x, &r_y, &r_log);
        dx = x - predicted->x.pos;
        dy = y - predicted->y.pos;
        distance = hypotf(dx, dy);
        nis = dx * dx / fmaxf(predicted->x.p00 + r_x, 1e-3f) +
              dy * dy / fmaxf(predicted->y.p00 + r_y, 1e-3f);
        ratio = height / fmaxf(predicted_h, 1.0f);
        iou = bbox_iou(x, anchor_to_box_center_y(y, height, view),
                       width, height,
                       predicted->x.pos,
                       anchor_to_box_center_y(predicted->y.pos,
                                              predicted_h, view),
                       predicted_w, predicted_h);
        if (exact && distance < diag->exact_distance) {
            diag->exact_nis = nis;
            diag->exact_distance = distance;
            diag->exact_scale_ratio = ratio;
        } else if (!exact && distance < diag->rebuild_distance) {
            diag->rebuild_seen = 1;
            diag->rebuild_nis = nis;
            diag->rebuild_distance = distance;
            diag->rebuild_iou = iou;
            diag->rebuild_scale_ratio = ratio;
        }
        if (ratio < 0.40f || ratio > 2.50f)
            continue;
        if (exact) {
            if (nis > tracker_exact_nis_gate &&
                distance > exact_distance_gate)
                continue;
        } else if (distance > exact_distance_gate ||
                   (nis > tracker_nis_gate &&
                    iou < TRACK_REBUILD_IOU_GATE)) {
            continue;
        }
        score = nis + 2.0f * fabsf(logf(ratio)) - 2.0f * iou -
                0.2f * dets[i].confidence;
        if ((exact && !best_is_exact) ||
            (exact == best_is_exact && score < best_score)) {
            best_score = score;
            best = i;
            best_is_exact = exact;
            *best_x = x;
            *best_y = y;
            *best_w = width;
            *best_h = height;
            *best_r_x = r_x;
            *best_r_y = r_y;
            *best_r_log = r_log;
            *accepted_nis = nis;
            *accepted_distance = distance;
            *accepted_distance_gate = exact_distance_gate;
            *matched_exact = exact;
        }
    }
    /* A present but physically impossible exact ID is more safely treated as
     * a detector discontinuity than as permission to jump to another face. */
    if (diag->exact_seen && !best_is_exact)
        return -1;
    return best;
}

static void accepted_measurement_transition(tracker_t *trk,
                                            double measurement_time)
{
    if (trk->state == TRACK_STATE_HOLDING ||
        trk->state == TRACK_STATE_LOST) {
        trk->recovery_hits = 1;
        trk->recovery_last_measurement_time = measurement_time;
        set_state(trk, TRACK_STATE_RECOVERING);
        return;
    }
    if (trk->state == TRACK_STATE_RECOVERING) {
        if (measurement_time <= trk->recovery_last_measurement_time + 1e-6)
            return;
        trk->recovery_last_measurement_time = measurement_time;
        trk->recovery_hits++;
        if (trk->recovery_hits < TRACK_RECOVER_CONFIRM_FRAMES)
            return;
        /* HOLD/RECOVER already brake through the jerk-limited path.  Keep
         * that path state continuous: clearing velocity/acceleration here
         * turns a valid recovery into an instantaneous output-step change.
         * Reset only the reference input before following the confirmed
         * measurement again. */
        trk->view.x.response_direction = 0;
        trk->view.y.response_direction = 0;
        trk->view.reference_x.pos = trk->view.x.pos;
        trk->view.reference_x.vel = 0.0f;
        trk->view.reference_y.pos = trk->view.y.pos;
        trk->view.reference_y.vel = 0.0f;
    }
    trk->recovery_hits = 0;
    trk->recovery_last_measurement_time = -1.0;
    set_state(trk, TRACK_STATE_TRACKING);
}

void tracker_update_detections(tracker_t *trk,
                               detection_t *dets, int det_count,
                               const frame_view_t *view,
                               double now_sec)
{
    tracker_kf_t predicted;
    double measurement_time;
    float x = 0.0f, y = 0.0f, width = 0.0f, height = 0.0f;
    float r_x = 0.0f, r_y = 0.0f, r_log = 0.0f;
    float accepted_nis = 0.0f;
    float accepted_distance = 0.0f;
    float accepted_distance_gate = 0.0f;
    float innovation_used_x = 0.0f;
    float innovation_used_y = 0.0f;
    association_diag_t association_diag;
    int matched_exact = 0;
    int strong_identity = 0;
    int weak_measurement = 0;
    int best;

    for (int i = 0; i < det_count; i++)
        dets[i].is_target = 0;
    if (!view || !tracker_is_active(trk) || trk->target_id < 0)
        return;
    measurement_time = view->capture_time_sec;
    if (measurement_time + 0.001 < trk->select_time_sec)
        return;
    if (!state_at_time(trk, measurement_time, &predicted))
        return;

    best = find_best_detection(trk, dets, det_count, view, &predicted,
                               &x, &y, &width, &height,
                               &r_x, &r_y, &r_log, &accepted_nis,
                               &accepted_distance, &accepted_distance_gate,
                               &matched_exact, &association_diag);
    if (best < 0) {
        trk->id_miss_count++;
        if (trk->state == TRACK_STATE_RECOVERING) {
            trk->recovery_hits = 0;
            trk->recovery_last_measurement_time = -1.0;
            set_state(trk, TRACK_STATE_LOST);
        }
        if (tracker_verbose_log) {
            if (association_diag.exact_seen) {
                printf("[tracker] reject exact id_misses=%d candidates=%d "
                       "nis=%.1f/%.1f dist=%.1f/%.1f ratio=%.2f\n",
                       trk->id_miss_count, det_count,
                       association_diag.exact_nis,
                       tracker_exact_nis_gate,
                       association_diag.exact_distance,
                       association_diag.exact_distance_gate,
                       association_diag.exact_scale_ratio);
            } else {
                printf("[tracker] no association id_misses=%d candidates=%d "
                       "rebuild=%d nis=%.1f dist=%.1f iou=%.2f ratio=%.2f\n",
                       trk->id_miss_count, det_count,
                       association_diag.rebuild_seen,
                       association_diag.rebuild_nis,
                       association_diag.rebuild_distance,
                       association_diag.rebuild_iou,
                       association_diag.rebuild_scale_ratio);
            }
        }
        return;
    }

    if (matched_exact) {
        strong_identity = 1;
        reset_rebind_candidate(trk);
    } else {
        int ready = advance_rebind_candidate(
            trk, &dets[best], measurement_time, x, y, height);

        if (!ready) {
            trk->id_miss_count++;
            if (trk->state == TRACK_STATE_RECOVERING) {
                trk->recovery_hits = 0;
                trk->recovery_last_measurement_time = -1.0;
                set_state(trk, TRACK_STATE_LOST);
            }
            if (tracker_verbose_log) {
                printf("[tracker] provisional logical=%d source=%d "
                       "candidate=%d public=%d hits=%d/%d positive=%d/%d "
                       "dist=%.1f/%.1f nis=%.1f\n",
                       trk->target_id, trk->source_id,
                       dets[best].track_id, dets[best].publish,
                       trk->rebind_hits, TRACK_REBIND_CONFIRM_FRAMES,
                       trk->rebind_positive_hits,
                       TRACK_REBIND_CONFIRM_FRAMES,
                       accepted_distance, accepted_distance_gate,
                       accepted_nis);
            }
            return;
        }
        if (dets[best].track_id >= 0 && dets[best].publish &&
            trk->rebind_source_id == dets[best].track_id &&
            trk->rebind_positive_hits >= TRACK_REBIND_CONFIRM_FRAMES) {
            printf("[tracker] detector source rebound logical=%d source=%d->%d "
                   "hits=%d\n",
                   trk->target_id, trk->source_id, dets[best].track_id,
                   trk->rebind_hits);
            trk->source_id = dets[best].track_id;
            strong_identity = 1;
            reset_rebind_candidate(trk);
        } else {
            weak_measurement = 1;
        }
    }
    if (!strong_identity && trk->state == TRACK_STATE_RECOVERING) {
        trk->recovery_hits = 0;
        trk->recovery_last_measurement_time = -1.0;
        set_state(trk, TRACK_STATE_LOST);
    }
    dets[best].is_target = 1;
    if (weak_measurement || !dets[best].publish) {
        float variance_scale = weak_measurement ?
            TRACK_UNASSOCIATED_VAR_SCALE :
            TRACK_ASSOC_PRIVATE_VAR_SCALE;
        r_x *= variance_scale;
        r_y *= variance_scale;
        r_log *= variance_scale;
    }
    delayed_measurement_update(trk, measurement_time, now_sec,
                               x, y, height, r_x, r_y, r_log,
                               &innovation_used_x, &innovation_used_y);
    {
        float aspect_alpha = (!dets[best].publish || weak_measurement) ?
            0.03f : 0.15f;
        trk->aspect_ratio = clampf_local(
            (1.0f - aspect_alpha) * trk->aspect_ratio +
                aspect_alpha * width / fmaxf(height, 1.0f),
                                     0.4f, 2.5f);
    }
    trk->last_sub_x = x;
    trk->last_sub_y = y;
    trk->last_bbox_w_norm = dets[best].bbox[2];
    trk->last_bbox_h_norm = dets[best].bbox[3];
    trk->last_seen_time = measurement_time;
    trk->measurement_count++;
    if (strong_identity) {
        trk->id_miss_count = 0;
        trk->last_receive_time = now_sec;
        trk->strong_measurement_count++;
    } else {
        trk->id_miss_count++;
        trk->weak_measurement_count++;
    }
    {
        double age_ms = fmax(0.0, (now_sec - measurement_time) * 1000.0);
        trk->measurement_age_sum_ms += age_ms;
        if (age_ms > trk->measurement_age_max_ms)
            trk->measurement_age_max_ms = age_ms;
    }
    if (strong_identity)
        accepted_measurement_transition(trk, measurement_time);
    app_state_store_int(&g_state.tracking_target_id, trk->target_id);

    if (tracker_verbose_log) {
        printf("[tracker] update logical=%d source=%d public=%d conf=%.3f "
               "obs=(%.1f,%.1f) est=(%.1f,%.1f) vel=(%.1f,%.1f) "
               "assoc=%s trust=%s nis=%.1f dist=%.1f/%.1f "
               "robust=(%.1f,%.1f) "
               "age=%.1fms\n",
               trk->target_id, trk->source_id,
               dets[best].publish, dets[best].confidence,
               x, y, trk->kf.x.pos, trk->kf.y.pos,
               trk->kf.x.vel, trk->kf.y.vel,
               matched_exact ? "exact" : "provisional",
               strong_identity ? "strong" : "weak",
               accepted_nis, accepted_distance, accepted_distance_gate,
               innovation_used_x, innovation_used_y,
               (now_sec - measurement_time) * 1000.0);
    }
}

static int update_timeout_state(tracker_t *trk, double now_sec)
{
    double age;

    if (trk->last_receive_time < 0.0) {
        trk->last_receive_time = now_sec;
        return 1;
    }
    age = now_sec - trk->last_receive_time;
    if (age <= tracker_miss_grace_sec)
        return 1;
    if (trk->state == TRACK_STATE_RECOVERING) {
        trk->recovery_hits = 0;
        trk->recovery_last_measurement_time = -1.0;
        set_state(trk, TRACK_STATE_LOST);
        return 1;
    }
    if (age > tracker_lost_sec) {
        printf("[tracker] lost timeout %.3fs\n", age);
        tracker_stop(trk);
        return 0;
    }
    /* Timeout states are monotone until a validated observation explicitly
     * starts recovery.  In particular an early failed RECOVER must not turn
     * LOST back into COAST merely because its last sample was recent. */
    if (trk->state == TRACK_STATE_LOST)
        return 1;
    if (age <= tracker_coast_sec) {
        if (trk->state == TRACK_STATE_HOLDING)
            return 1;
        set_state(trk, TRACK_STATE_COASTING);
    } else if (age <= tracker_hold_sec) {
        set_state(trk, TRACK_STATE_HOLDING);
    } else {
        set_state(trk, TRACK_STATE_LOST);
    }
    return 1;
}

void tracker_predict(tracker_t *trk, double time_sec)
{
    tracker_kf_t command_kf;
    float dt;
    float target_x;
    float target_y;
    float noise_enter_x;
    float noise_enter_y;
    double strong_age;
    int follow_unobserved_target;
    int output_x;
    int output_y;
    tracker_view_controller_t previous_view;

    if (!tracker_is_active(trk) || !trk->kf.initialized) {
        pthread_mutex_lock(&g_state.pred_mutex);
        g_state.pred_box.valid = 0;
        pthread_mutex_unlock(&g_state.pred_mutex);
        return;
    }
    if (!update_timeout_state(trk, time_sec))
        return;
    strong_age = trk->last_receive_time >= 0.0 ?
        fmax(0.0, time_sec - trk->last_receive_time) : INFINITY;
    follow_unobserved_target = strong_age <= tracker_command_coast_sec;
    dt = trk->last_predict_time > 0.0 ?
         (float)(time_sec - trk->last_predict_time) : 1.0f / 30.0f;
    dt = clampf_local(dt, 0.001f, 0.1f);
    trk->last_predict_time = time_sec;

    kf_predict_state(&trk->kf, trk->kf_time_sec, time_sec);
    trk->kf_time_sec = time_sec;
    if (trk->state == TRACK_STATE_HOLDING ||
               trk->state == TRACK_STATE_RECOVERING) {
        float decay = expf(-6.0f * dt);
        trk->kf.x.vel *= decay;
        trk->kf.y.vel *= decay;
    } else if (trk->state == TRACK_STATE_LOST) {
        trk->kf.x.vel = 0.0f;
        trk->kf.y.vel = 0.0f;
    }
    history_append(trk, time_sec);

    /* The command written after frame N becomes the image window for N+2.
     * Lead only a copy: the estimator/history remain at real time so delayed
     * detector measurements can still be replayed on one time axis. */
    command_kf = trk->kf;
    kf_predict_state(&command_kf, time_sec,
                     time_sec + TRACK_COMMAND_LEAD_SEC);
    /* Do not discard the stable target trajectory at a pan boundary.  The
     * composer clamps only the physical output; retaining the ideal crop
     * beyond the limit lets it observe a turn-back before the subject has
     * crossed the boundary again. */
    target_x = command_kf.x.pos - viewport_anchor_offset(
        TRACK_VIEW_ANCHOR_X_RATIO, IMG_W, 1);
    target_y = command_kf.y.pos - viewport_anchor_offset(
        TRACK_VIEW_ANCHOR_Y_RATIO, IMG_H, 1);
    /* Covariance defines the no-response band; it does not change the path
     * gain or inject estimator velocity.  During a short coast uncertainty
     * naturally widens the band and suppresses speculative camera motion. */
    noise_enter_x = clampf_local(
        TRACK_VIEW_NOISE_SIGMA_MULT *
            sqrtf(fmaxf(command_kf.x.p00, 0.0f)),
        TRACK_VIEW_NOISE_ENTER_X_MIN_PX,
        TRACK_VIEW_NOISE_ENTER_X_MAX_PX);
    noise_enter_y = clampf_local(
        TRACK_VIEW_NOISE_SIGMA_MULT *
            sqrtf(fmaxf(command_kf.y.p00, 0.0f)),
        TRACK_VIEW_NOISE_ENTER_Y_MIN_PX,
        TRACK_VIEW_NOISE_ENTER_Y_MAX_PX);
    if (trk->state == TRACK_STATE_TRACKING ||
        (trk->state == TRACK_STATE_COASTING && follow_unobserved_target)) {
        previous_view = trk->view;
        tracker_servo_step(&trk->view, target_x, target_y,
                           noise_enter_x, noise_enter_y, dt);
    } else if (trk->state == TRACK_STATE_COASTING ||
               trk->state == TRACK_STATE_HOLDING ||
               trk->state == TRACK_STATE_LOST ||
               trk->state == TRACK_STATE_RECOVERING) {
        previous_view = trk->view;
        tracker_servo_brake(&trk->view, dt);
    } else {
        previous_view = trk->view;
    }
    output_x = clampi((int)lroundf(trk->view.x.pos), OX_MIN, OX_MAX);
    output_y = clampi((int)lroundf(trk->view.y.pos), OY_MIN, OY_MAX);
    if (tracker_write_isp_offset(output_x, output_y) < 0)
        trk->view = previous_view;

    pthread_mutex_lock(&g_state.pred_mutex);
    if (trk->state != TRACK_STATE_LOST) {
        float screen_x = (trk->kf.x.pos - trk->view.x.pos) / (float)IMG_W;
        float screen_y = (trk->kf.y.pos - trk->view.y.pos) / (float)IMG_H;
        float height = expf(trk->kf.log_h.pos);
        g_state.pred_box.cx = clampf_local(1.0f - screen_x, 0.0f, 1.0f);
        g_state.pred_box.cy = clampf_local(
            1.0f - screen_y +
            (0.5f - TRACK_FACE_ANCHOR_Y_RATIO) * height / (float)IMG_H,
            0.0f, 1.0f);
        g_state.pred_box.h = clampf_local(height / (float)IMG_H, 0.0f, 1.0f);
        g_state.pred_box.w = clampf_local(
            height * trk->aspect_ratio / (float)IMG_W, 0.0f, 1.0f);
        g_state.pred_box.class_id = trk->target_class;
        g_state.pred_box.track_id = trk->target_id;
        g_state.pred_box.valid = 1;
    } else {
        g_state.pred_box.valid = 0;
    }
    pthread_mutex_unlock(&g_state.pred_mutex);

    if (tracker_diag_interval_sec > 0.0f) {
        diag_count++;
        {
            double dx = output_x - diag_mean_x;
            double dy = output_y - diag_mean_y;
            diag_mean_x += dx / diag_count;
            diag_mean_y += dy / diag_count;
            diag_m2_x += dx * (output_x - diag_mean_x);
            diag_m2_y += dy * (output_y - diag_mean_y);
        }
        if (diag_last_x >= 0) {
            int step_x = output_x - diag_last_x;
            int step_y = output_y - diag_last_y;
            int jump = abs(step_x);
            int jump_y = abs(step_y);
            if (jump_y > jump)
                jump = jump_y;
            if (jump > diag_max_jump)
                diag_max_jump = jump;
            if (diag_have_step) {
                int delta_x = step_x - diag_last_step_x;
                int delta_y = step_y - diag_last_step_y;
                if (abs(delta_x) > diag_max_delta_x)
                    diag_max_delta_x = abs(delta_x);
                if (abs(delta_y) > diag_max_delta_y)
                    diag_max_delta_y = abs(delta_y);
                if (diag_have_delta) {
                    int jerk_x = delta_x - diag_last_delta_x;
                    int jerk_y = delta_y - diag_last_delta_y;
                    if (abs(jerk_x) > diag_max_jerk_x)
                        diag_max_jerk_x = abs(jerk_x);
                    if (abs(jerk_y) > diag_max_jerk_y)
                        diag_max_jerk_y = abs(jerk_y);
                }
                diag_last_delta_x = delta_x;
                diag_last_delta_y = delta_y;
                diag_have_delta = 1;
            }
            diag_last_step_x = step_x;
            diag_last_step_y = step_y;
            diag_have_step = 1;
        }
        diag_last_x = output_x;
        diag_last_y = output_y;
        if (diag_last_time == 0.0)
            diag_last_time = time_sec;
        if (time_sec - diag_last_time >= tracker_diag_interval_sec) {
            double std_x = diag_count > 1 ?
                           sqrt(diag_m2_x / (diag_count - 1)) : 0.0;
            double std_y = diag_count > 1 ?
                           sqrt(diag_m2_y / (diag_count - 1)) : 0.0;
            double mean_age = trk->measurement_count ?
                              trk->measurement_age_sum_ms /
                              (double)trk->measurement_count : 0.0;
            printf("[tracker] st=%s pos=(%.1f,%.1f) vel=(%.1f,%.1f) "
                   "view=(%d,%d) view_vel=(%.1f,%.1f) "
                   "view_accel=(%.0f,%.0f) jitter=(%.2f,%.2f) jump=%d "
                   "delta=(%d,%d) jerk=(%d,%d) "
                   "measurement_age=%.1f/%.1fms n=%lu strong=%lu weak=%lu\n",
                   state_name(trk->state), trk->kf.x.pos, trk->kf.y.pos,
                   trk->kf.x.vel, trk->kf.y.vel,
                   output_x, output_y,
                   trk->view.x.vel, trk->view.y.vel,
                   trk->view.x.accel, trk->view.y.accel,
                   std_x, std_y, diag_max_jump,
                   diag_max_delta_x, diag_max_delta_y,
                   diag_max_jerk_x, diag_max_jerk_y,
                   mean_age, trk->measurement_age_max_ms,
                   trk->measurement_count,
                   trk->strong_measurement_count,
                   trk->weak_measurement_count);
            diag_last_time = time_sec;
            diag_count = 0;
            diag_mean_x = diag_mean_y = 0.0;
            diag_m2_x = diag_m2_y = 0.0;
            diag_last_x = diag_last_y = -1;
            diag_max_jump = 0;
            diag_last_step_x = diag_last_step_y = 0;
            diag_last_delta_x = diag_last_delta_y = 0;
            diag_have_step = diag_have_delta = 0;
            diag_max_delta_x = diag_max_delta_y = 0;
            diag_max_jerk_x = diag_max_jerk_y = 0;
            trk->measurement_count = 0;
            trk->strong_measurement_count = 0;
            trk->weak_measurement_count = 0;
            trk->measurement_age_sum_ms = 0.0;
            trk->measurement_age_max_ms = 0.0;
        }
    }
}

int tracker_is_active(const tracker_t *trk)
{
    switch (trk->state) {
    case TRACK_STATE_ENGAGING:
    case TRACK_STATE_TRACKING:
    case TRACK_STATE_COASTING:
    case TRACK_STATE_HOLDING:
    case TRACK_STATE_LOST:
    case TRACK_STATE_RECOVERING:
        return 1;
    case TRACK_STATE_IDLE:
    case TRACK_STATE_SELECTED:
    default:
        return 0;
    }
}

int tracker_get_target_id(const tracker_t *trk)
{
    return trk->target_id;
}

track_state_t tracker_get_state(const tracker_t *trk)
{
    return trk->state;
}

void tracker_stop(tracker_t *trk)
{
    trk->state = TRACK_STATE_IDLE;
    trk->target_id = -1;
    trk->source_id = -1;
    trk->target_class = -1;
    trk->select_valid = 0;
    trk->kf.initialized = 0;
    trk->history_count = 0;
    trk->view.initialized = 0;
    reset_rebind_candidate(trk);
    trk->recovery_hits = 0;
    trk->recovery_last_measurement_time = -1.0;
    scaler_restore();
    last_written_xoff = -1;
    isp_written_this_frame = 0;
    tracker_write_isp_offset(ISP_DEFAULT_XOFF, ISP_DEFAULT_YOFF);
    pthread_mutex_lock(&g_state.pred_mutex);
    g_state.pred_box.valid = 0;
    pthread_mutex_unlock(&g_state.pred_mutex);
    app_state_store_int(&g_state.tracking_active, 0);
    app_state_store_int(&g_state.tracking_target_id, -1);
    notify_status_changed();
    printf("[tracker] Stopped, full view restored\n");
}
