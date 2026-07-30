#ifndef FRAME_MAILBOX_H
#define FRAME_MAILBOX_H

#include <stdint.h>

#include "cam_system/app_state.h"

#ifdef __cplusplus
extern "C" {
#endif

/*
 * A lease pins one immutable cached frame slot.  The publisher may continue
 * filling either of the other slots, but it cannot overwrite this slot until
 * every consumer has released it.
 */
typedef struct {
    int valid;
    int slot_idx;
    unsigned int frame_seq;
    const uint8_t *data;
    frame_view_t view;
    double publish_time_sec;
} frame_lease_t;

void frame_mailbox_init(app_state_t *state);
void frame_mailbox_destroy(app_state_t *state);

/*
 * Publish one complete frame using latest-frame semantics.  The large copy is
 * deliberately performed outside frame_mutex.  Returns 0 on success or
 * -EAGAIN when all three slots are leased; capture must never wait for a slow
 * consumer.
 */
int frame_mailbox_publish(app_state_t *state,
                          uint8_t *const buffers[FRAME_BUF_CNT],
                          const uint8_t *source,
                          const frame_view_t *view,
                          unsigned int *published_seq,
                          int *published_slot);

/*
 * Wait for and acquire the newest frame after after_seq.  Returns 1 with a
 * valid lease, 0 on timeout/wake/shutdown, or a negative errno value.
 */
int frame_mailbox_wait_latest(app_state_t *state,
                              uint8_t *const buffers[FRAME_BUF_CNT],
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
