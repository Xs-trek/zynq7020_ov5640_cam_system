#define _GNU_SOURCE
#include "cam_system/frame_mailbox.h"

#include "cam_system/config.h"

#include <errno.h>
#include <pthread.h>
#include <string.h>
#include <time.h>

static double monotonic_sec(void)
{
    struct timespec ts;
    clock_gettime(CLOCK_MONOTONIC, &ts);
    return (double)ts.tv_sec + (double)ts.tv_nsec / 1000000000.0;
}

static void deadline_from_now(struct timespec *deadline, int timeout_ms)
{
    /* pthread_cond_init(NULL) uses CLOCK_REALTIME on every target libc. */
    clock_gettime(CLOCK_REALTIME, deadline);
    deadline->tv_sec += timeout_ms / 1000;
    deadline->tv_nsec += (long)(timeout_ms % 1000) * 1000000L;
    if (deadline->tv_nsec >= 1000000000L) {
        deadline->tv_sec++;
        deadline->tv_nsec -= 1000000000L;
    }
}

void frame_mailbox_init(app_state_t *state)
{
    pthread_mutex_init(&state->frame_mutex, NULL);
    pthread_cond_init(&state->frame_cond, NULL);

    state->latest_frame_idx = -1;
    state->frame_seq = 0;
    state->frame_publish_cursor = 0;
    state->frame_wake_seq = 0;
    state->frame_publish_drops = 0;
    for (int index = 0; index < FRAME_BUF_CNT; ++index) {
        state->frame_refcount[index] = 0;
        state->frame_writing[index] = 0;
        state->frame_publish_time_sec[index] = 0.0;
    }
}

void frame_mailbox_destroy(app_state_t *state)
{
    pthread_cond_destroy(&state->frame_cond);
    pthread_mutex_destroy(&state->frame_mutex);
}

static int select_writable_slot_locked(const app_state_t *state)
{
    int latest = state->latest_frame_idx;

    /* Preserve the current latest slot when another slot is available. */
    for (int pass = 0; pass < 2; ++pass) {
        for (int offset = 0; offset < FRAME_BUF_CNT; ++offset) {
            int index = (state->frame_publish_cursor + offset) % FRAME_BUF_CNT;
            if (state->frame_refcount[index] != 0 || state->frame_writing[index])
                continue;
            if (pass == 0 && index == latest)
                continue;
            return index;
        }
    }
    return -1;
}

int frame_mailbox_publish(app_state_t *state,
                          uint8_t *const buffers[FRAME_BUF_CNT],
                          const uint8_t *source,
                          const frame_view_t *view,
                          unsigned int *published_seq,
                          int *published_slot)
{
    int slot;
    unsigned int sequence;
    frame_view_t published_view;

    if (!state || !buffers || !source || !view)
        return -EINVAL;

    pthread_mutex_lock(&state->frame_mutex);
    slot = select_writable_slot_locked(state);
    if (slot < 0 || !buffers[slot]) {
        state->frame_publish_drops++;
        pthread_mutex_unlock(&state->frame_mutex);
        return -EAGAIN;
    }
    state->frame_writing[slot] = 1;
    state->frame_publish_cursor = (slot + 1) % FRAME_BUF_CNT;
    pthread_mutex_unlock(&state->frame_mutex);

    memcpy(buffers[slot], source, FRAME_SIZE);

    pthread_mutex_lock(&state->frame_mutex);
    sequence = app_state_load_u32(&state->frame_seq) + 1U;
    if (sequence == 0U)
        sequence = 1U;
    published_view = *view;
    published_view.frame_seq = sequence;
    state->frame_views[slot] = published_view;
    state->frame_publish_time_sec[slot] = monotonic_sec();
    app_state_store_int(&state->latest_frame_idx, slot);
    app_state_store_u32(&state->frame_seq, sequence);
    state->frame_writing[slot] = 0;
    pthread_cond_broadcast(&state->frame_cond);
    pthread_mutex_unlock(&state->frame_mutex);

    if (published_seq)
        *published_seq = sequence;
    if (published_slot)
        *published_slot = slot;
    return 0;
}

static int acquire_latest_locked(app_state_t *state,
                                 uint8_t *const buffers[FRAME_BUF_CNT],
                                 unsigned int after_seq,
                                 frame_lease_t *lease)
{
    int slot = app_state_load_int(&state->latest_frame_idx);
    unsigned int sequence = app_state_load_u32(&state->frame_seq);

    if (sequence == after_seq || slot < 0 || slot >= FRAME_BUF_CNT ||
        state->frame_writing[slot] || !buffers[slot])
        return 0;

    state->frame_refcount[slot]++;
    memset(lease, 0, sizeof(*lease));
    lease->valid = 1;
    lease->slot_idx = slot;
    lease->frame_seq = sequence;
    lease->data = buffers[slot];
    lease->view = state->frame_views[slot];
    lease->publish_time_sec = state->frame_publish_time_sec[slot];
    return 1;
}

int frame_mailbox_acquire_latest(app_state_t *state,
                                 uint8_t *const buffers[FRAME_BUF_CNT],
                                 unsigned int after_seq,
                                 frame_lease_t *lease)
{
    int result;

    if (!state || !buffers || !lease)
        return -EINVAL;
    memset(lease, 0, sizeof(*lease));
    lease->slot_idx = -1;

    pthread_mutex_lock(&state->frame_mutex);
    result = acquire_latest_locked(state, buffers, after_seq, lease);
    pthread_mutex_unlock(&state->frame_mutex);
    return result;
}

int frame_mailbox_wait_latest(app_state_t *state,
                              uint8_t *const buffers[FRAME_BUF_CNT],
                              unsigned int after_seq,
                              int timeout_ms,
                              frame_lease_t *lease)
{
    struct timespec deadline;
    unsigned int wake_sequence;
    int result;
    int wait_result = 0;

    if (!state || !buffers || !lease || timeout_ms < 0)
        return -EINVAL;
    memset(lease, 0, sizeof(*lease));
    lease->slot_idx = -1;
    deadline_from_now(&deadline, timeout_ms);

    pthread_mutex_lock(&state->frame_mutex);
    wake_sequence = state->frame_wake_seq;
    for (;;) {
        result = acquire_latest_locked(state, buffers, after_seq, lease);
        if (result != 0)
            break;
        if (app_state_load_int(&state->quit) ||
            state->frame_wake_seq != wake_sequence ||
            wait_result == ETIMEDOUT) {
            result = 0;
            break;
        }
        wait_result = pthread_cond_timedwait(&state->frame_cond,
                                             &state->frame_mutex,
                                             &deadline);
        if (wait_result != 0 && wait_result != EINTR &&
            wait_result != ETIMEDOUT) {
            result = -wait_result;
            break;
        }
    }
    pthread_mutex_unlock(&state->frame_mutex);
    return result;
}

void frame_mailbox_release(app_state_t *state, frame_lease_t *lease)
{
    if (!state || !lease || !lease->valid)
        return;

    pthread_mutex_lock(&state->frame_mutex);
    if (lease->slot_idx >= 0 && lease->slot_idx < FRAME_BUF_CNT &&
        state->frame_refcount[lease->slot_idx] > 0) {
        state->frame_refcount[lease->slot_idx]--;
        pthread_cond_broadcast(&state->frame_cond);
    }
    pthread_mutex_unlock(&state->frame_mutex);
    memset(lease, 0, sizeof(*lease));
    lease->slot_idx = -1;
}

void frame_mailbox_wake_all(app_state_t *state)
{
    if (!state)
        return;
    pthread_mutex_lock(&state->frame_mutex);
    state->frame_wake_seq++;
    pthread_cond_broadcast(&state->frame_cond);
    pthread_mutex_unlock(&state->frame_mutex);
}
