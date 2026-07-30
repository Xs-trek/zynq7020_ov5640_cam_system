#define _GNU_SOURCE
#include "cam_system/app_state.h"
#include "cam_system/frame_mailbox.h"

#include <pthread.h>
#include <signal.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>
#include <time.h>
#include <unistd.h>

#define PIPELINE_FRAMES 120U
#define PERIOD_US       20000L

app_state_t g_state;
uint8_t *g_frame_pool[FRAME_BUF_CNT];
uint8_t *g_cached_pool[FRAME_BUF_CNT];
volatile sig_atomic_t g_app_signal_exit_requested;

static uint8_t cached[FRAME_BUF_CNT][FRAME_SIZE];
static uint8_t source[FRAME_SIZE];

typedef struct {
    const char *name;
    long work_us;
    unsigned int last_seq;
    unsigned long processed;
    unsigned long skipped;
    int mismatch;
    double pickup_sum_ms;
    double pickup_max_ms;
} consumer_t;

static double monotonic_sec(void)
{
    struct timespec ts;
    clock_gettime(CLOCK_MONOTONIC, &ts);
    return (double)ts.tv_sec + (double)ts.tv_nsec / 1000000000.0;
}

static void sleep_us(long usec)
{
    struct timespec request = {
        .tv_sec = usec / 1000000L,
        .tv_nsec = (usec % 1000000L) * 1000L,
    };
    while (nanosleep(&request, &request) != 0)
        ;
}

static uint8_t value_for_sequence(unsigned int sequence)
{
    return (uint8_t)((sequence % 251U) + 1U);
}

static void *consumer_thread(void *opaque)
{
    consumer_t *consumer = (consumer_t *)opaque;

    while (!app_state_load_int(&g_state.quit)) {
        frame_lease_t lease;
        unsigned int last_seq =
            __atomic_load_n(&consumer->last_seq, __ATOMIC_ACQUIRE);
        int result = frame_mailbox_wait_latest(&g_state, g_cached_pool,
                                                last_seq, 100,
                                                &lease);
        if (result <= 0)
            continue;

        double pickup_ms =
            (monotonic_sec() - lease.publish_time_sec) * 1000.0;
        uint8_t expected = value_for_sequence(lease.frame_seq);
        if (last_seq != 0U) {
            unsigned int gap = lease.frame_seq - last_seq;
            if (gap > 1U && gap < 0x80000000U)
                consumer->skipped += (unsigned long)(gap - 1U);
        }
        if (lease.data[0] != expected ||
            lease.data[FRAME_SIZE / 2] != expected ||
            lease.data[FRAME_SIZE - 1] != expected)
            consumer->mismatch = 1;

        sleep_us(consumer->work_us);

        if (lease.data[0] != expected ||
            lease.data[FRAME_SIZE / 2] != expected ||
            lease.data[FRAME_SIZE - 1] != expected)
            consumer->mismatch = 1;
        __atomic_store_n(&consumer->last_seq, lease.frame_seq,
                         __ATOMIC_RELEASE);
        consumer->processed++;
        consumer->pickup_sum_ms += pickup_ms;
        if (pickup_ms > consumer->pickup_max_ms)
            consumer->pickup_max_ms = pickup_ms;
        frame_mailbox_release(&g_state, &lease);
    }
    return NULL;
}

int main(void)
{
    consumer_t detector = {.name = "detector", .work_us = 16000L};
    consumer_t jpeg = {.name = "jpeg", .work_us = 12000L};
    pthread_t detector_thread;
    pthread_t jpeg_thread;
    struct timespec next;
    unsigned long publish_failures = 0;

    memset(&g_state, 0, sizeof(g_state));
    frame_mailbox_init(&g_state);
    for (int index = 0; index < FRAME_BUF_CNT; ++index)
        g_cached_pool[index] = cached[index];

    if (pthread_create(&detector_thread, NULL, consumer_thread, &detector) != 0 ||
        pthread_create(&jpeg_thread, NULL, consumer_thread, &jpeg) != 0)
        return 1;

    clock_gettime(CLOCK_MONOTONIC, &next);
    for (unsigned int frame = 1U; frame <= PIPELINE_FRAMES; ++frame) {
        frame_view_t view;
        uint8_t value = value_for_sequence(frame);
        memset(&view, 0, sizeof(view));
        view.capture_time_sec = monotonic_sec();
        memset(source, value, sizeof(source));
        if (frame_mailbox_publish(&g_state, g_cached_pool, source, &view,
                                  NULL, NULL) != 0)
            publish_failures++;

        next.tv_nsec += PERIOD_US * 1000L;
        while (next.tv_nsec >= 1000000000L) {
            next.tv_sec++;
            next.tv_nsec -= 1000000000L;
        }
        clock_nanosleep(CLOCK_MONOTONIC, TIMER_ABSTIME, &next, NULL);
    }

    for (int wait = 0; wait < 1000; ++wait) {
        if (__atomic_load_n(&detector.last_seq, __ATOMIC_ACQUIRE) ==
                PIPELINE_FRAMES &&
            __atomic_load_n(&jpeg.last_seq, __ATOMIC_ACQUIRE) ==
                PIPELINE_FRAMES)
            break;
        usleep(1000);
    }

    app_state_store_int(&g_state.quit, 1);
    frame_mailbox_wake_all(&g_state);
    pthread_join(detector_thread, NULL);
    pthread_join(jpeg_thread, NULL);

    printf("frame_pipeline_timing_test: publish_fail=%lu mailbox_drop=%lu\n",
           publish_failures, g_state.frame_publish_drops);
    printf("  %s processed=%lu skipped=%lu pickup=%.3f/%.3fms last=%u\n",
           detector.name, detector.processed, detector.skipped,
           detector.processed ? detector.pickup_sum_ms / detector.processed : 0.0,
           detector.pickup_max_ms, detector.last_seq);
    printf("  %s processed=%lu skipped=%lu pickup=%.3f/%.3fms last=%u\n",
           jpeg.name, jpeg.processed, jpeg.skipped,
           jpeg.processed ? jpeg.pickup_sum_ms / jpeg.processed : 0.0,
           jpeg.pickup_max_ms, jpeg.last_seq);

    int failed = publish_failures != 0UL ||
                 g_state.frame_publish_drops != 0UL ||
                 detector.mismatch || jpeg.mismatch ||
                 detector.last_seq != PIPELINE_FRAMES ||
                 jpeg.last_seq != PIPELINE_FRAMES ||
                 detector.skipped > 1UL || jpeg.skipped > 1UL;
    frame_mailbox_destroy(&g_state);
    if (failed) {
        fprintf(stderr, "frame_pipeline_timing_test: FAILED\n");
        return 1;
    }

    puts("frame_pipeline_timing_test: OK");
    return 0;
}
