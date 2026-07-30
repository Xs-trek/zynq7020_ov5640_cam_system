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
uint8_t *g_frame_pool[FRAME_BUF_CNT];
uint8_t *g_cached_pool[FRAME_BUF_CNT];
volatile sig_atomic_t g_app_signal_exit_requested;

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
    return frame_mailbox_publish(&g_state, g_cached_pool, source, &view,
                                 sequence, NULL);
}

static int lease_matches(const frame_lease_t *lease, uint8_t value)
{
    return lease->valid && lease->data &&
           lease->data[0] == value &&
           lease->data[FRAME_SIZE / 2] == value &&
           lease->data[FRAME_SIZE - 1] == value;
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

    /* Capture must fail fast instead of overwriting any of three leases. */
    if (publish_value(4, &sequence) != -EAGAIN ||
        g_state.frame_publish_drops != 1UL ||
        !lease_matches(&first, 1) || !lease_matches(&second, 2) ||
        !lease_matches(&third, 3))
        goto fail;

    frame_mailbox_release(&g_state, &second);
    if (publish_value(4, &sequence) != 0 || sequence != 4U ||
        !lease_matches(&first, 1) || !lease_matches(&third, 3))
        goto fail;
    frame_mailbox_release(&g_state, &first);
    frame_mailbox_release(&g_state, &third);

    memset(&waiter, 0, sizeof(waiter));
    waiter.after_seq = 4U;
    if (pthread_create(&thread, NULL, waiter_thread, &waiter) != 0)
        goto fail;
    usleep(20000);
    if (publish_value(5, &sequence) != 0) {
        pthread_join(thread, NULL);
        goto fail;
    }
    pthread_join(thread, NULL);
    if (waiter.result != 1 || waiter.lease.frame_seq != 5U ||
        !lease_matches(&waiter.lease, 5))
        goto fail;
    frame_mailbox_release(&g_state, &waiter.lease);

    memset(&waiter, 0, sizeof(waiter));
    waiter.after_seq = 5U;
    if (pthread_create(&thread, NULL, waiter_thread, &waiter) != 0)
        goto fail;
    usleep(20000);
    frame_mailbox_wake_all(&g_state);
    pthread_join(thread, NULL);
    if (waiter.result != 0 || waiter.lease.valid)
        goto fail;

    frame_mailbox_destroy(&g_state);
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
