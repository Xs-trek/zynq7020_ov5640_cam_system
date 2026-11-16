#define _GNU_SOURCE
#include "cam_system/app_state.h"
#include "cam_system/frame_mailbox.h"

#include <errno.h>
#include <pthread.h>
#include <signal.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>
#include <unistd.h>

app_state_t g_state;
uint8_t *g_frame_pool[CAPTURE_BUF_CNT];
uint8_t *g_cached_pool[FRAME_BUF_CNT];

static uint8_t cached[FRAME_BUF_CNT][FRAME_SIZE];
static uint8_t source[FRAME_SIZE];

typedef struct {
    unsigned int after_seq;
    int result;
    frame_lease_t lease;
} waiter_args_t;

static void *waiter_thread(void *opaque)
{
    waiter_args_t *args = (waiter_args_t *)opaque;
    args->result = frame_mailbox_wait_latest(&g_state, g_cached_pool,
                                              args->after_seq, 500,
                                              &args->lease);
    return NULL;
}

static int publish_value(uint8_t value, unsigned int *sequence)
{
    frame_view_t view;
    memset(&view, 0, sizeof(view));
    view.capture_time_sec = (double)value;
    memset(source, value, sizeof(source));
    view.frame_seq = frame_mailbox_begin_frame(&g_state);
    if (sequence)
        *sequence = view.frame_seq;
    return frame_mailbox_publish(&g_state, g_cached_pool, source, &view);
}

static int lease_matches(const frame_lease_t *lease, uint8_t value)
{
    return lease->valid && lease->data &&
           lease->data[0] == value &&
           lease->data[FRAME_SIZE / 2] == value &&
           lease->data[FRAME_SIZE - 1] == value;
}

static uint8_t capture[CAPTURE_BUF_CNT][64];
static uint8_t *capture_pool[CAPTURE_BUF_CNT];

#define REQUIRE(cond) do { \
    if (!(cond)) { \
        fprintf(stderr, "frame_mailbox_test: FAIL line %d: %s\n", __LINE__, #cond); \
        return 1; \
    } \
} while (0)

static int publish_direct(int capture_idx, unsigned int *sequence)
{
    frame_view_t view;

    memset(&view, 0, sizeof(view));
    view.frame_seq = frame_mailbox_begin_frame(&g_state);
    if (sequence)
        *sequence = view.frame_seq;
    return frame_mailbox_publish_direct(&g_state, capture_idx, &view);
}

static int test_direct_channel(void)
{
    frame_lease_t lease;
    frame_lease_t cached_lease;
    frame_view_t view;
    int indices[CAPTURE_BUF_CNT];
    unsigned int cached_seq = 0;
    unsigned int sequence;

    memset(&g_state, 0, sizeof(g_state));
    frame_mailbox_init(&g_state);
    for (int index = 0; index < CAPTURE_BUF_CNT; ++index)
        capture_pool[index] = capture[index];

    /* off: the caller keeps the buffer */
    REQUIRE(publish_direct(0, NULL) == 0);
    REQUIRE(frame_mailbox_reclaim(&g_state, indices) == 0);
    REQUIRE(frame_mailbox_wait_direct(&g_state, capture_pool, 0, 1, &lease) == 0);

    frame_mailbox_set_direct(&g_state, 1);
    REQUIRE(publish_direct(0, &sequence) == 1);
    REQUIRE(frame_mailbox_wait_direct(&g_state, capture_pool, 0, 100, &lease) == 1);
    REQUIRE(lease.capture_idx == 0 && lease.slot_idx == -1 && lease.data == capture[0]);
    REQUIRE(lease.frame_seq == sequence);

    /* nothing newer than what was taken */
    {
        frame_lease_t none;

        REQUIRE(frame_mailbox_wait_direct(&g_state, capture_pool, sequence, 1, &none) == 0);
    }

    /* the leased buffer and the newest one stay out of the queue */
    REQUIRE(publish_direct(1, NULL) == 1);
    REQUIRE(frame_mailbox_reclaim(&g_state, indices) == 0);
    frame_mailbox_release(&g_state, &lease);
    REQUIRE(lease.capture_idx == -1 && lease.slot_idx == -1);
    REQUIRE(frame_mailbox_reclaim(&g_state, indices) == 1 && indices[0] == 0);
    REQUIRE(frame_mailbox_reclaim(&g_state, indices) == 0);
    REQUIRE(publish_direct(2, NULL) == 1);
    REQUIRE(frame_mailbox_reclaim(&g_state, indices) == 1 && indices[0] == 1);

    /* what a stream restart must leave out of the queue */
    {
        int held[CAPTURE_BUF_CNT];

        frame_mailbox_held(&g_state, held);
        REQUIRE(held[2] == 1 && held[0] == 0 && held[1] == 0);
    }

    /* a frame with no cached copy is not a new frame for a cached consumer */
    for (int index = 0; index < FRAME_BUF_CNT; ++index)
        g_cached_pool[index] = cached[index];
    memset(&view, 0, sizeof(view));
    view.frame_seq = frame_mailbox_begin_frame(&g_state);
    cached_seq = view.frame_seq;
    REQUIRE(frame_mailbox_publish_direct(&g_state, 3, &view) == 1);
    REQUIRE(frame_mailbox_publish(&g_state, g_cached_pool, source, &view) == 0);
    REQUIRE(frame_mailbox_acquire_latest(&g_state, g_cached_pool, 0, &cached_lease) == 1);
    REQUIRE(cached_lease.frame_seq == cached_seq && cached_lease.capture_idx == -1);
    frame_mailbox_release(&g_state, &cached_lease);
    REQUIRE(publish_direct(4, &sequence) == 1);
    REQUIRE(sequence == cached_seq + 1U);
    REQUIRE(frame_mailbox_acquire_latest(&g_state, g_cached_pool, cached_seq, &cached_lease) == 0);

    /* turning the channel off gives back the newest buffer too */
    frame_mailbox_set_direct(&g_state, 0);
    {
        int count = frame_mailbox_reclaim(&g_state, indices);

        REQUIRE(count == 3);    /* 2, 3 and 4: the channel held them for no reader */
    }
    REQUIRE(publish_direct(5, NULL) == 0);
    frame_mailbox_destroy(&g_state);
    return 0;
}

/* Runs capture's buffer cycle against a detector that holds a lease for some frames.  The hardware
 * must always keep at least three queued buffers: one being written and two to go to. */
static int test_buffer_accounting(int hold_frames)
{
    int queue[CAPTURE_BUF_CNT];
    int queued = CAPTURE_BUF_CNT;
    int indices[CAPTURE_BUF_CNT];
    frame_lease_t lease;
    int held_for = 0;
    unsigned int last_seq = 0;

    memset(&g_state, 0, sizeof(g_state));
    frame_mailbox_init(&g_state);
    frame_mailbox_set_direct(&g_state, 1);
    memset(&lease, 0, sizeof(lease));
    for (int index = 0; index < CAPTURE_BUF_CNT; ++index)
        queue[index] = index;

    for (int frame = 0; frame < 1000; ++frame) {
        int done = queue[0];
        int count;

        memmove(queue, queue + 1, (size_t)(queued - 1) * sizeof(queue[0]));
        queued--;
        REQUIRE(publish_direct(done, NULL) == 1);

        if (lease.valid && ++held_for > hold_frames) {
            frame_mailbox_release(&g_state, &lease);
            held_for = 0;
        }
        if (!lease.valid) {
            REQUIRE(frame_mailbox_wait_direct(&g_state, capture_pool, last_seq, 1, &lease) == 1);
            last_seq = lease.frame_seq;
        }
        count = frame_mailbox_reclaim(&g_state, indices);
        for (int i = 0; i < count; ++i)
            queue[queued++] = indices[i];
        REQUIRE(queued >= 3 && queued <= CAPTURE_BUF_CNT);
    }
    frame_mailbox_release(&g_state, &lease);
    frame_mailbox_destroy(&g_state);
    return 0;
}

int main(void)
{
    frame_lease_t first = {0};
    frame_lease_t second = {0};
    frame_lease_t third = {0};
    unsigned int sequence = 0;
    waiter_args_t waiter = {0};
    pthread_t thread;

    memset(&g_state, 0, sizeof(g_state));
    frame_mailbox_init(&g_state);
    for (int index = 0; index < FRAME_BUF_CNT; ++index)
        g_cached_pool[index] = cached[index];

    if (publish_value(1, &sequence) != 0 || sequence != 1U ||
        frame_mailbox_acquire_latest(&g_state, g_cached_pool, 0,
                                     &first) != 1 ||
        !lease_matches(&first, 1))
        goto fail;

    if (publish_value(2, &sequence) != 0 || sequence != 2U ||
        frame_mailbox_acquire_latest(&g_state, g_cached_pool, 1,
                                     &second) != 1 ||
        !lease_matches(&second, 2) || !lease_matches(&first, 1))
        goto fail;

    if (publish_value(3, &sequence) != 0 || sequence != 3U ||
        frame_mailbox_acquire_latest(&g_state, g_cached_pool, 2,
                                     &third) != 1 ||
        !lease_matches(&third, 3) || !lease_matches(&first, 1) ||
        !lease_matches(&second, 2))
        goto fail;

    /* Capture must fail fast instead of overwriting any of three leases.  The frame keeps its
     * sequence: it may have been published on the direct channel. */
    if (publish_value(4, &sequence) != -EAGAIN ||
        g_state.frame_publish_drops != 1UL ||
        !lease_matches(&first, 1) || !lease_matches(&second, 2) ||
        !lease_matches(&third, 3))
        goto fail;

    frame_mailbox_release(&g_state, &second);
    if (publish_value(4, &sequence) != 0 || sequence != 5U ||
        !lease_matches(&first, 1) || !lease_matches(&third, 3))
        goto fail;
    frame_mailbox_release(&g_state, &first);
    frame_mailbox_release(&g_state, &third);

    memset(&waiter, 0, sizeof(waiter));
    waiter.after_seq = 5U;
    if (pthread_create(&thread, NULL, waiter_thread, &waiter) != 0)
        goto fail;
    usleep(20000);
    if (publish_value(5, &sequence) != 0) {
        pthread_join(thread, NULL);
        goto fail;
    }
    pthread_join(thread, NULL);
    if (waiter.result != 1 || waiter.lease.frame_seq != 6U ||
        !lease_matches(&waiter.lease, 5))
        goto fail;
    frame_mailbox_release(&g_state, &waiter.lease);

    memset(&waiter, 0, sizeof(waiter));
    waiter.after_seq = 6U;
    if (pthread_create(&thread, NULL, waiter_thread, &waiter) != 0)
        goto fail;
    usleep(20000);
    frame_mailbox_wake_all(&g_state);
    pthread_join(thread, NULL);
    if (waiter.result != 0 || waiter.lease.valid)
        goto fail;

    frame_mailbox_destroy(&g_state);

    if (test_direct_channel() != 0 || test_buffer_accounting(0) != 0 ||
        test_buffer_accounting(1) != 0 || test_buffer_accounting(7) != 0)
        return 1;

    puts("frame_mailbox_test: OK");
    return 0;

fail:
    frame_mailbox_release(&g_state, &first);
    frame_mailbox_release(&g_state, &second);
    frame_mailbox_release(&g_state, &third);
    frame_mailbox_release(&g_state, &waiter.lease);
    frame_mailbox_destroy(&g_state);
    fprintf(stderr, "frame_mailbox_test: FAILED seq=%u drops=%lu\n",
            sequence, g_state.frame_publish_drops);
    return 1;
}
