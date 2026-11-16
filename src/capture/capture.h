#ifndef CAPTURE_H
#define CAPTURE_H

/* 采集线程入口，传入 NULL
 * 持续运行，通过 g_state.quit 退出
 * 通过 g_state.cam_enabled 暂停/恢复 */
void *capture_thread(void *arg);

/* The stream runs only while the camera is enabled, for at most capture.max_stream_min (any
 * thread): how long the current stream has run (0 while stopped), and whether the limit turned
 * the camera off and it has stayed off since. */
void capture_get_stream_state(double now, unsigned int *run_sec, int *stopped_by_limit);

/* Stream restarts since start (any thread). */
unsigned long capture_get_restarts(void);

/* True when the capture thread has not gone round its loop for CAPTURE_STALL_SEC: stuck in a
 * call that does not return, which no orderly shutdown can get past. */
int capture_stalled(double now);

/* 供状态接口读取，任意线程可调用。now 为 CLOCK_MONOTONIC 秒。 */
float capture_get_fps(double now);
unsigned long capture_get_isp_slips(void);

/* What the drivers count, refreshed about once a second while the camera runs.  A negative count
 * has not been read yet.  The bridge counts are this run's, from the first reading after the
 * stream started (capture/dvp_counts.h). */
typedef struct {
    long dvp_frames;                /* frames seen by the PL bridge this run */
    long dvp_bad_frames;            /* of those, not 640x480 */
    long i2c_write_failures;        /* sensor writes that failed after their retries */
    unsigned long v4l2_error_frames;
} capture_health_t;

void capture_get_health(capture_health_t *health);

#endif
