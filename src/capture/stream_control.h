/* stream_control.h - when the capture stream runs.
 *
 * The stream, and with it the sensor, runs only while the camera is enabled: the sensor stays in
 * standby while the application waits with the camera off.  A stream that has run for the limit
 * is stopped and the camera turned off; the time counts from the start of the stream and starts
 * over at the next one.  The board's sensor must not run for longer than that at a time.
 *
 * Pure logic: the caller supplies the time and does the work.
 */
#ifndef CAM_STREAM_CONTROL_H
#define CAM_STREAM_CONTROL_H

typedef enum {
    STREAM_KEEP = 0,        /* nothing to do */
    STREAM_START,           /* the camera was enabled: start the stream */
    STREAM_STOP,            /* the camera was disabled: stop the stream */
    STREAM_STOP_LIMIT       /* the stream ran for the limit: stop it and disable the camera */
} stream_action_t;

typedef struct {
    int streaming;
    double started_at;
    double limit_sec;       /* 0: no limit */
    int stopped_by_limit;   /* the last stop was the limit's, and no start came since */
} stream_control_t;

void stream_control_init(stream_control_t *ctl, double limit_sec);

/* What to do now, for the camera state the user asked for. */
stream_action_t stream_control_poll(const stream_control_t *ctl, int camera_enabled, double now);

/* The caller carried out a start or a stop. */
void stream_control_started(stream_control_t *ctl, double now);
void stream_control_stopped(stream_control_t *ctl, stream_action_t why);

/* Seconds the current stream has run; 0 while stopped. */
double stream_control_run_sec(const stream_control_t *ctl, double now);

#endif /* CAM_STREAM_CONTROL_H */
