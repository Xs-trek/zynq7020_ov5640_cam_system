/* control_tick.h - timing of the per-frame control tick (pure logic, no I/O).
 *
 * There is one tick per captured frame, at 30 Hz.  Instead of running right after the frame is
 * dequeued with the previous frame's detection, a tracking tick waits for this frame's detection,
 * at most until frame time + deadline.  An ISP offset written after frame N is dequeued and before
 * frame N+1 ends takes effect on frame N+2, so a tick that finishes in time gains one frame.
 *
 * A write that finishes after frame N+1 has ended takes effect one frame later.  That is known only
 * when frame N+1 arrives, so the view of a tick is registered at that point (control_tick_settle):
 * until then no frame can resolve to it.
 *
 * The caller passes the frame's capture time as the tracker's clock: tick time + 2 frame periods is
 * then exactly the capture time of the frame the command lands on, whenever the tick really ran.
 */
#ifndef CAM_CONTROL_TICK_H
#define CAM_CONTROL_TICK_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef enum {
    CONTROL_TICK_WAIT = 0,      /* keep waiting for this frame's detection */
    CONTROL_TICK_ON_DETECTION,  /* this frame's detection is published */
    CONTROL_TICK_ON_DEADLINE,   /* the deadline came first */
    CONTROL_TICK_IMMEDIATE      /* nothing to wait for */
} control_tick_action_t;

typedef struct {
    unsigned long on_detection;
    unsigned long on_deadline;
    unsigned long immediate;
    unsigned long writes;       /* ticks that changed the ISP offset */
    unsigned long isp_slips;    /* writes that finished after the next frame ended */
    double margin_sum_ms;       /* frame end minus write completion, over writes */
    double margin_min_ms;
} control_tick_stats_t;

typedef struct {
    double deadline_sec;        /* 0: never wait, the baseline cadence */
    uint32_t sequence;          /* V4L2 sequence of the current frame */
    unsigned int frame_seq;     /* mailbox sequence of the current frame */
    double frame_time;
    int expect_detection;
    int pending;                /* a finished tick whose view is not registered yet */
    int pending_wrote;
    uint32_t pending_sequence;
    double pending_write_done;
    control_tick_stats_t stats;     /* since the last control_tick_stats_reset */
    unsigned long isp_slips_total;  /* since capture started */
    unsigned int watch_waits;       /* ticks that waited, in the current watch window */
    unsigned int watch_late;        /* of those, ticks the deadline ended */
} control_tick_t;

void control_tick_init(control_tick_t *tick, double deadline_ms);

/* Forget the pending tick and the statistics (capture restarted). */
void control_tick_reset(control_tick_t *tick);

/* expect_detection: tracking is active and the detector is expected to publish this frame. */
void control_tick_begin(control_tick_t *tick, uint32_t sequence, unsigned int frame_seq, double frame_time,
                        int expect_detection);

/* latest_det_frame_seq: mailbox sequence of the newest published detection. */
control_tick_action_t control_tick_poll(const control_tick_t *tick, unsigned int latest_det_frame_seq, double now);

/* Absolute time at which a waiting tick must run. */
double control_tick_deadline(const control_tick_t *tick);

void control_tick_end(control_tick_t *tick, control_tick_action_t action, int wrote_offset, double write_done);

/* Call when the next frame arrives, with its capture time.  Returns 1 and the sequence to register
 * the finished tick's view under, or 0 if no tick is pending. */
int control_tick_settle(control_tick_t *tick, double next_frame_time, uint32_t *issued_sequence);

void control_tick_stats_reset(control_tick_t *tick);

/* Ticks that waited for their detection, counted CONTROL_TICK_WATCH_WAITS at a time.  At the end of
 * each window returns 1 and the count of late ones if the deadline came first for most of them:
 * the detections take longer than the deadline, so every tick runs on the previous frame's.
 * Returns 0 otherwise. */
#define CONTROL_TICK_WATCH_WAITS 300u
int control_tick_watch(control_tick_t *tick, unsigned int *late);

#ifdef __cplusplus
}
#endif

#endif /* CAM_CONTROL_TICK_H */
