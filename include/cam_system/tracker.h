#ifndef TRACKER_H
#define TRACKER_H

#include "cam_system/app_state.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef enum {
    TRACK_STATE_IDLE = 0,
    TRACK_STATE_SELECTED,
    TRACK_STATE_ENGAGING,
    TRACK_STATE_TRACKING,
    TRACK_STATE_COASTING,
    TRACK_STATE_HOLDING,
    TRACK_STATE_LOST,
    TRACK_STATE_RECOVERING,
} track_state_t;

/* Independent constant-velocity Kalman axis: position and velocity. */
typedef struct {
    float pos;
    float vel;
    float p00, p01, p10, p11;
} tracker_kf_axis_t;

typedef struct {
    tracker_kf_axis_t x;
    tracker_kf_axis_t y;
    tracker_kf_axis_t log_h;
    int initialized;
} tracker_kf_t;

typedef struct {
    double time_sec;
    tracker_kf_t kf;
} tracker_history_t;

typedef struct {
    float pos;
    float vel;
    float accel;
    int response_direction;
} tracker_view_axis_t;

typedef struct {
    float pos;
    float vel;
} tracker_reference_axis_t;

typedef struct {
    tracker_view_axis_t x;
    tracker_view_axis_t y;
    tracker_reference_axis_t reference_x;
    tracker_reference_axis_t reference_y;
    int initialized;
} tracker_view_controller_t;

typedef struct {
    track_state_t state;
    /* Stable frontend selection identity; never rewritten by association. */
    int target_id;
    /* Short-lived detector identity currently associated with target_id. */
    int source_id;
    int target_class;

    int select_valid;
    float select_abs_x;
    float select_abs_y;
    float select_abs_h;
    float select_aspect;
    float select_confidence;
    double select_time_sec;

    tracker_kf_t kf;
    double kf_time_sec;
    tracker_history_t history[TRACK_HISTORY_SIZE];
    int history_count;

    tracker_view_controller_t view;
    float aspect_ratio;
    float last_sub_x;
    float last_sub_y;
    float last_bbox_w_norm;
    float last_bbox_h_norm;

    double last_seen_time;
    double last_receive_time;
    double last_predict_time;
    int id_miss_count;
    /* A new detector identity is provisional until consecutive, physically
     * consistent observations establish one lineage.  -1 is a valid
     * provisional key but is never copied into source_id. */
    int rebind_source_id;
    int rebind_hits;
    int rebind_positive_hits;
    double rebind_last_measurement_time;
    float rebind_x;
    float rebind_y;
    float rebind_height;
    int recovery_hits;
    double recovery_last_measurement_time;

    unsigned long measurement_count;
    unsigned long strong_measurement_count;
    unsigned long weak_measurement_count;
    double measurement_age_sum_ms;
    double measurement_age_max_ms;
} tracker_t;

void tracker_init(tracker_t *trk);
/* Diagnostics only: verbose logging and the periodic summary interval (0 = off). */
void tracker_set_diagnostics(int verbose, float diag_every_sec);
void tracker_apply_cmd(tracker_t *trk, track_cmd_t cmd, int tid);
void tracker_update_detections(tracker_t *trk,
                               detection_t *dets, int det_count,
                               const frame_view_t *view,
                               double now_sec);
void tracker_predict(tracker_t *trk, double time_sec);
int tracker_is_active(const tracker_t *trk);
int tracker_get_target_id(const tracker_t *trk);
track_state_t tracker_get_state(const tracker_t *trk);
const char *tracker_state_name(track_state_t state);
/* A detection's control point (the face anchor) and size in the absolute 1312x972 plane. */
void tracker_det_to_abs(const detection_t *det, const frame_view_t *view,
                        float *cx, float *cy, float *width, float *height);
void tracker_stop(tracker_t *trk);
void tracker_reset_frame_flag(void);
void tracker_get_isp_offset(int *xoff, int *yoff);
/* The sensor was initialized: its register table turned the scaler on and set the default
 * offset.  Sets the tracker's copies to match.  Call with the tracker idle. */
void tracker_sensor_initialized(void);
int tracker_write_isp_offset(int x, int y);
void tracker_snapshot_view(frame_view_t *view, int xoff, int yoff,
                           unsigned int frame_seq, double capture_time_sec);

#ifdef __cplusplus
}
#endif

#endif
