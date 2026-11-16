#define _GNU_SOURCE
#include "cam_system/frame_mailbox.h"

#include "cam_system/config.h"
#include "common/time_util.h"

#include <errno.h>
#include <pthread.h>
#include <string.h>

static void deadline_from_now(struct timespec *deadline, int timeout_ms)
{
    /* frame_cond waits on CLOCK_MONOTONIC (frame_mailbox_init), so wall-clock steps do not
     * stretch or cut short a wait. */
    clock_gettime(CLOCK_MONOTONIC, deadline);
    deadline->tv_sec += timeout_ms / 1000;
    deadline->tv_nsec += (long)(timeout_ms % 1000) * 1000000L;
    if (deadline->tv_nsec >= 1000000000L) {
        deadline->tv_sec++;
        deadline->tv_nsec -= 1000000000L;
    }
}

void frame_mailbox_init(app_state_t *state)
{
    pthread_condattr_t cond_attr;

    pthread_mutex_init(&state->frame_mutex, NULL);
    pthread_condattr_init(&cond_attr);
    pthread_condattr_setclock(&cond_attr, CLOCK_MONOTONIC);
    pthread_cond_init(&state->frame_cond, &cond_attr);
    pthread_condattr_destroy(&cond_attr);

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
    state->direct_enabled = 0;
    state->direct_latest = -1;
    state->direct_publish_time_sec = 0.0;
    for (int index = 0; index < CAPTURE_BUF_CNT; ++index) {
        state->direct_refcount[index] = 0;
        state->direct_held[index] = 0;
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

unsigned int frame_mailbox_begin_frame(app_state_t *state)
{
    unsigned int sequence = app_state_load_u32(&state->frame_seq) + 1U;

    if (sequence == 0U)
        sequence = 1U;
    app_state_store_u32(&state->frame_seq, sequence);
    return sequence;
}

int frame_mailbox_publish(app_state_t *state,
                          uint8_t *const buffers[FRAME_BUF_CNT],
                          const uint8_t *source,
                          const frame_view_t *view)
{
    int slot;

    if (!state || !buffers || !source || !view || view->frame_seq == 0U)
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
    state->frame_views[slot] = *view;
    state->frame_publish_time_sec[slot] = monotonic_sec();
    app_state_store_int(&state->latest_frame_idx, slot);
    state->frame_writing[slot] = 0;
    pthread_cond_broadcast(&state->frame_cond);
    pthread_mutex_unlock(&state->frame_mutex);
    return 0;
}

void frame_mailbox_set_direct(app_state_t *state, int enabled)
{
    pthread_mutex_lock(&state->frame_mutex);
    state->direct_enabled = enabled;
    if (!enabled)
        state->direct_latest = -1;
    pthread_cond_broadcast(&state->frame_cond);
    pthread_mutex_unlock(&state->frame_mutex);
}

int frame_mailbox_publish_direct(app_state_t *state, int capture_idx,
                                 const frame_view_t *view)
{
    int taken;

    pthread_mutex_lock(&state->frame_mutex);
    taken = state->direct_enabled;
    if (taken) {
        state->direct_held[capture_idx] = 1;
        state->direct_latest = capture_idx;
        state->direct_view = *view;
        state->direct_publish_time_sec = monotonic_sec();
        pthread_cond_broadcast(&state->frame_cond);
    }
    pthread_mutex_unlock(&state->frame_mutex);
    return taken;
}

int frame_mailbox_reclaim(app_state_t *state, int indices[CAPTURE_BUF_CNT])
{
    int count = 0;

    pthread_mutex_lock(&state->frame_mutex);
    for (int index = 0; index < CAPTURE_BUF_CNT; ++index) {
        if (state->direct_held[index] && state->direct_refcount[index] == 0 &&
            index != state->direct_latest) {
            state->direct_held[index] = 0;
            indices[count++] = index;
        }
    }
    pthread_mutex_unlock(&state->frame_mutex);
    return count;
}

void frame_mailbox_held(app_state_t *state, int held[CAPTURE_BUF_CNT])
{
    pthread_mutex_lock(&state->frame_mutex);
    for (int index = 0; index < CAPTURE_BUF_CNT; ++index)
        held[index] = state->direct_held[index];
    pthread_mutex_unlock(&state->frame_mutex);
}

/* A cached slot's own sequence decides: frame_seq also counts frames with no cached copy. */
static int acquire_latest_locked(app_state_t *state,
                                 uint8_t *const *buffers,
                                 unsigned int after_seq,
                                 frame_lease_t *lease)
{
    int slot = app_state_load_int(&state->latest_frame_idx);
    unsigned int sequence;

    if (slot < 0 || slot >= FRAME_BUF_CNT ||
        state->frame_writing[slot] || !buffers[slot])
        return 0;
    sequence = state->frame_views[slot].frame_seq;
    if (sequence == after_seq)
        return 0;

    state->frame_refcount[slot]++;
    memset(lease, 0, sizeof(*lease));
    lease->valid = 1;
    lease->slot_idx = slot;
    lease->capture_idx = -1;
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
    lease->capture_idx = -1;

    pthread_mutex_lock(&state->frame_mutex);
    result = acquire_latest_locked(state, buffers, after_seq, lease);
    pthread_mutex_unlock(&state->frame_mutex);
    return result;
}

static int acquire_direct_locked(app_state_t *state,
                                 uint8_t *const *buffers,
                                 unsigned int after_seq,
                                 frame_lease_t *lease)
{
    int index = state->direct_latest;

    if (index < 0 || state->direct_view.frame_seq == after_seq || !buffers[index])
        return 0;

    state->direct_refcount[index]++;
    memset(lease, 0, sizeof(*lease));
    lease->valid = 1;
    lease->slot_idx = -1;
    lease->capture_idx = index;
    lease->frame_seq = state->direct_view.frame_seq;
    lease->data = buffers[index];
    lease->view = state->direct_view;
    lease->publish_time_sec = state->direct_publish_time_sec;
    return 1;
}

typedef int (*acquire_fn)(app_state_t *state, uint8_t *const *buffers,
                          unsigned int after_seq, frame_lease_t *lease);

static int wait_for_frame(app_state_t *state,
                          acquire_fn acquire,
                          uint8_t *const *buffers,
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
    lease->capture_idx = -1;
    deadline_from_now(&deadline, timeout_ms);

    pthread_mutex_lock(&state->frame_mutex);
    wake_sequence = state->frame_wake_seq;
    for (;;) {
        result = acquire(state, buffers, after_seq, lease);
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

int frame_mailbox_wait_latest(app_state_t *state,
                              uint8_t *const buffers[FRAME_BUF_CNT],
                              unsigned int after_seq,
                              int timeout_ms,
                              frame_lease_t *lease)
{
    return wait_for_frame(state, acquire_latest_locked, buffers, after_seq, timeout_ms, lease);
}

int frame_mailbox_wait_direct(app_state_t *state,
                              uint8_t *const buffers[CAPTURE_BUF_CNT],
                              unsigned int after_seq,
                              int timeout_ms,
                              frame_lease_t *lease)
{
    return wait_for_frame(state, acquire_direct_locked, buffers, after_seq, timeout_ms, lease);
}

void frame_mailbox_release(app_state_t *state, frame_lease_t *lease)
{
    if (!state || !lease || !lease->valid)
        return;

    pthread_mutex_lock(&state->frame_mutex);
    if (lease->capture_idx >= 0 && lease->capture_idx < CAPTURE_BUF_CNT) {
        if (state->direct_refcount[lease->capture_idx] > 0)
            state->direct_refcount[lease->capture_idx]--;
    } else if (lease->slot_idx >= 0 && lease->slot_idx < FRAME_BUF_CNT &&
               state->frame_refcount[lease->slot_idx] > 0) {
        state->frame_refcount[lease->slot_idx]--;
        pthread_cond_broadcast(&state->frame_cond);
    }
    pthread_mutex_unlock(&state->frame_mutex);
    memset(lease, 0, sizeof(*lease));
    lease->slot_idx = -1;
    lease->capture_idx = -1;
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
