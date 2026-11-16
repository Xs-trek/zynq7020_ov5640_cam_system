/* capture_recovery.h - what capture does when good frames stop arriving.
 *
 * A stall is time without a good frame, so a stream that delivers only error frames counts as
 * well as one that delivers nothing.  The stream is restarted a few times; if that does not bring
 * frames back, the process exits for its supervisor to restart.  A frame clears the count.
 *
 * Pure logic: the caller supplies the time and does the work.
 */
#ifndef CAM_CAPTURE_RECOVERY_H
#define CAM_CAPTURE_RECOVERY_H

#define CAPREC_STALL_SEC     4.0
#define CAPREC_RESTART_LIMIT 3

typedef enum {
    CAPREC_WAIT = 0,    /* keep waiting for a frame */
    CAPREC_RESTART,     /* restart the stream */
    CAPREC_GIVE_UP      /* restarting has not helped */
} caprec_action_t;

typedef struct {
    double last_progress;   /* last good frame or restart */
    int restarts;           /* since the last good frame */
} caprec_t;

void caprec_init(caprec_t *rec, double now);

/* Capture (re)started: time without frames counts from now.  The restart count stands, so a
 * stream that never delivers still reaches CAPREC_GIVE_UP. */
void caprec_touch(caprec_t *rec, double now);

void caprec_on_frame(caprec_t *rec, double now);

/* Called when a capture attempt gave no good frame. */
caprec_action_t caprec_on_no_frame(caprec_t *rec, double now);

#endif /* CAM_CAPTURE_RECOVERY_H */
