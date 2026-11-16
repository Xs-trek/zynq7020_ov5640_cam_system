#ifndef APP_STATE_H
#define APP_STATE_H

#include <stdint.h>
#include <signal.h>
#include <pthread.h>
#include "cam_system/config.h"

typedef struct {
    float bbox[4];
    int   class_id;
    float confidence;
    int   track_id;
    int   is_target;
    int   publish;       /* 1: publish to frontend, 0: tracker-only candidate */
} detection_t;

typedef struct {
    int   valid;
    float cx, cy;      /* 归一化 screen 中心 [0,1] */
    float w, h;        /* 归一化 bbox 宽高 */
    int   class_id;
    int   track_id;
} predict_box_t;

typedef enum {
    TRACK_CMD_NONE = 0,
    TRACK_CMD_START,
    TRACK_CMD_STOP,
    TRACK_CMD_SELECT
} track_cmd_t;

/*
 * Per-captured-frame mapping from normalized display coordinates to the
 * absolute 2x-subsampled sensor plane.  The transform belongs to the frame;
 * consumers must not combine a delayed detection with the current viewport.
 */
typedef struct {
    unsigned int frame_seq;
    double capture_time_sec;
    float roi_x;
    float roi_y;
    float roi_w;
    float roi_h;
    int output_w;
    int output_h;
    int flip_x;
    int flip_y;
    int scaler_enabled;
} frame_view_t;

typedef struct {
    int cam_enabled;
    int detector_enabled;
    int quit;
    int exit_code;      /* what the process exits with; set through app_state_fail */

    int tracking_active;
    int tracking_target_id;

    int latest_frame_idx;
    unsigned int frame_seq;
    unsigned int status_seq;

    /* 每个帧缓冲对应的采集时刻和坐标变换 */
    pthread_mutex_t frame_mutex;
    pthread_cond_t frame_cond;
    frame_view_t frame_views[FRAME_BUF_CNT];
    unsigned int frame_refcount[FRAME_BUF_CNT];
    int frame_writing[FRAME_BUF_CNT];
    double frame_publish_time_sec[FRAME_BUF_CNT];
    int frame_publish_cursor;
    unsigned int frame_wake_seq;
    unsigned long frame_publish_drops;

    /* Direct channel: the newest capture buffer, for a reader that needs no cached copy. */
    int direct_enabled;
    int direct_latest;              /* capture buffer index, or -1 */
    frame_view_t direct_view;
    double direct_publish_time_sec;
    unsigned int direct_refcount[CAPTURE_BUF_CNT];
    int direct_held[CAPTURE_BUF_CNT];   /* not to be requeued until reclaimed */

    /* 预测框互斥锁 */
    pthread_mutex_t pred_mutex;

    /* 跟踪预测框 (tracker 每帧更新 @30fps) */
    predict_box_t pred_box;

    pthread_cond_t det_cond;    /* CLOCK_MONOTONIC; signalled under det_mutex on every publish */

    double start_time_sec;      /* CLOCK_MONOTONIC; set once before the threads start */

    pthread_mutex_t jpeg_mutex;
    uint8_t  *jpeg_buf;
    size_t    jpeg_size;
    unsigned int jpeg_seq;

    pthread_mutex_t det_mutex;
    detection_t detections[DETECTOR_MAX_DET];
    int det_count;
    unsigned int det_seq;
    unsigned int det_frame_seq;
    frame_view_t det_frame_view;
    double det_publish_time_sec;

    pthread_mutex_t cmd_mutex;
    track_cmd_t track_cmd;
    int track_target_id;
    track_cmd_t cmd_queue[CONTROL_CMD_QUEUE_LEN];
    int cmd_target_queue[CONTROL_CMD_QUEUE_LEN];
    int cmd_head;
    int cmd_tail;
    int cmd_count;

    pthread_mutex_t settings_mutex;
    int enabled_classes[DETECTOR_MAX_CLASSES];
    int enabled_class_count;
    int max_det;

} app_state_t;

static inline int app_state_load_int(const int *p)
{
    return __atomic_load_n(p, __ATOMIC_ACQUIRE);
}

static inline void app_state_store_int(int *p, int v)
{
    __atomic_store_n(p, v, __ATOMIC_RELEASE);
}

static inline unsigned int app_state_load_u32(const unsigned int *p)
{
    return __atomic_load_n(p, __ATOMIC_ACQUIRE);
}

static inline void app_state_store_u32(unsigned int *p, unsigned int v)
{
    __atomic_store_n(p, v, __ATOMIC_RELEASE);
}

static inline unsigned int app_state_inc_u32(unsigned int *p)
{
    return __atomic_add_fetch(p, 1U, __ATOMIC_RELEASE);
}

/* Shuts the process down for a fault: it exits non-zero, so its supervisor restarts it.  The
 * first fault decides the status. */
static inline void app_state_fail(app_state_t *s, int exit_code)
{
    int none = 0;

    __atomic_compare_exchange_n(&s->exit_code, &none, exit_code, 0, __ATOMIC_RELEASE, __ATOMIC_RELAXED);
    app_state_store_int(&s->quit, 1);
}

#ifdef __cplusplus
extern "C" {
#endif

extern app_state_t g_state;
extern uint8_t *g_frame_pool[CAPTURE_BUF_CNT];
extern uint8_t *g_cached_pool[FRAME_BUF_CNT];

void app_state_init(app_state_t *s);

#ifdef __cplusplus
}
#endif

#endif
