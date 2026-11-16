#ifndef FRAME_MAILBOX_H
#define FRAME_MAILBOX_H

#include <stdint.h>

#include "cam_system/app_state.h"

#ifdef __cplusplus
extern "C" {
#endif

/*
 * A lease pins one frame: an immutable cached slot, or a capture buffer on the
 * direct channel.  The publisher cannot overwrite either until every consumer
 * has released it.
 */
typedef struct {
    int valid;
    int slot_idx;               /* cached slot, or -1 */
    int capture_idx;            /* capture buffer on the direct channel, or -1 */
    unsigned int frame_seq;
    const uint8_t *data;
    frame_view_t view;
    double publish_time_sec;
} frame_lease_t;

void frame_mailbox_init(app_state_t *state);
void frame_mailbox_destroy(app_state_t *state);

/* The sequence of a new frame, never 0.  Both channels publish it under this number, so a
 * detection and a JPEG of the same frame carry the same sequence.  Capture thread only. */
unsigned int frame_mailbox_begin_frame(app_state_t *state);

/*
 * Publish one complete frame using latest-frame semantics; view->frame_seq comes
 * from frame_mailbox_begin_frame.  The large copy is deliberately performed
 * outside frame_mutex.  Returns 0 on success or -EAGAIN when all three slots
 * are leased; capture must never wait for a slow consumer.
 */
int frame_mailbox_publish(app_state_t *state,
                          uint8_t *const buffers[FRAME_BUF_CNT],
                          const uint8_t *source,
                          const frame_view_t *view);

/*
 * Direct channel, for a reader that does not need a cached copy (the accelerator reads the
 * capture buffer by DMA).  The mailbox takes over a published capture buffer: it stays out of the
 * V4L2 queue while it is the newest frame or leased, and frame_mailbox_reclaim hands it back.
 * All of this runs on the capture thread except the set_direct and lease calls, so requeueing
 * stays on one thread.
 */
void frame_mailbox_set_direct(app_state_t *state, int enabled);

/* Returns 1 when the mailbox took over the buffer, 0 when the channel is off and the caller
 * still owns it. */
int frame_mailbox_publish_direct(app_state_t *state, int capture_idx, const frame_view_t *view);

/* Fills indices with the capture buffers that are free to requeue and returns how many. */
int frame_mailbox_reclaim(app_state_t *state, int indices[CAPTURE_BUF_CNT]);

/* Marks in held the capture buffers the mailbox has not handed back. */
void frame_mailbox_held(app_state_t *state, int held[CAPTURE_BUF_CNT]);

/*
 * Wait for and acquire the newest frame after after_seq.  Returns 1 with a
 * valid lease, 0 on timeout/wake/shutdown, or a negative errno value.
 */
int frame_mailbox_wait_latest(app_state_t *state,
                              uint8_t *const buffers[FRAME_BUF_CNT],
                              unsigned int after_seq,
                              int timeout_ms,
                              frame_lease_t *lease);

/* frame_mailbox_wait_latest() on the direct channel; buffers are the capture buffers. */
int frame_mailbox_wait_direct(app_state_t *state,
                              uint8_t *const buffers[CAPTURE_BUF_CNT],
                              unsigned int after_seq,
                              int timeout_ms,
                              frame_lease_t *lease);

/* Non-blocking form of frame_mailbox_wait_latest(). */
int frame_mailbox_acquire_latest(app_state_t *state,
                                 uint8_t *const buffers[FRAME_BUF_CNT],
                                 unsigned int after_seq,
                                 frame_lease_t *lease);

void frame_mailbox_release(app_state_t *state, frame_lease_t *lease);
void frame_mailbox_wake_all(app_state_t *state);

#ifdef __cplusplus
}
#endif

#endif
