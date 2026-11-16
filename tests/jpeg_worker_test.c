#define _GNU_SOURCE
#include "cam_system/app_state.h"
#include "cam_system/frame_mailbox.h"
#include "media/jpeg_worker.h"
#include "common/time_util.h"

#include <pthread.h>
#include <signal.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>
#include <unistd.h>

app_state_t g_state;
uint8_t *g_frame_pool[CAPTURE_BUF_CNT];
uint8_t *g_cached_pool[FRAME_BUF_CNT];

static int encoded_count;
static unsigned int last_seq;
static int data_mismatch;

static void publish_at(uint8_t *source, uint8_t value, double capture_time_sec)
{
    frame_view_t view = {0};

    view.capture_time_sec = capture_time_sec;
    memset(source, value, FRAME_SIZE);
    view.frame_seq = frame_mailbox_begin_frame(&g_state);
    frame_mailbox_publish(&g_state, g_cached_pool, source, &view);
}

static void publish(uint8_t *source, uint8_t value)
{
    publish_at(source, value, 0.0);
}

static int encoded(void)
{
    return __atomic_load_n(&encoded_count, __ATOMIC_ACQUIRE);
}

int jpeg_encode_frame(const uint8_t *rgb_data, unsigned int frame_seq)
{
    usleep(50000);
    if (rgb_data[0] != (uint8_t)frame_seq)
        __atomic_store_n(&data_mismatch, 1, __ATOMIC_RELEASE);
    __atomic_store_n(&last_seq, frame_seq, __ATOMIC_RELEASE);
    __atomic_add_fetch(&encoded_count, 1, __ATOMIC_RELEASE);
    return 0;
}

int main(void)
{
    static uint8_t source[FRAME_SIZE];
    static uint8_t cached[FRAME_BUF_CNT][FRAME_SIZE];
    pthread_t worker;

    memset(&g_state, 0, sizeof(g_state));
    frame_mailbox_init(&g_state);
    for (int index = 0; index < FRAME_BUF_CNT; ++index)
        g_cached_pool[index] = cached[index];

    if (jpeg_worker_init() != 0 ||
        pthread_create(&worker, NULL, jpeg_worker_thread, NULL) != 0)
        return 1;

    /* No stream client: frames are published but nothing is encoded. */
    publish(source, 1);
    usleep(80000);
    if (encoded() != 0) {
        fprintf(stderr, "jpeg worker encoded without a client: %d\n", encoded());
        return 1;
    }

    /* First client: the newest frame is encoded at once. */
    jpeg_worker_set_demand(1);
    usleep(120000);
    if (encoded() != 1 || __atomic_load_n(&last_seq, __ATOMIC_ACQUIRE) != 1u) {
        fprintf(stderr, "jpeg worker did not encode for the first client: %d\n", encoded());
        return 1;
    }

    /* While streaming the newest frame replaces waiting ones. */
    publish(source, 2);
    usleep(5000);
    publish(source, 3);
    publish(source, 4);
    usleep(130000);
    int streamed = encoded();
    if (streamed < 2 || streamed > 3 || __atomic_load_n(&last_seq, __ATOMIC_ACQUIRE) != 4u) {
        fprintf(stderr, "jpeg worker failed count=%d last_seq=%u\n", streamed, last_seq);
        return 1;
    }

    /* Last client gone: encoding stops again.  One job may still have been in flight. */
    jpeg_worker_set_demand(0);
    usleep(80000);
    publish(source, 5);
    usleep(1000);
    publish(source, 6);
    usleep(120000);
    if (encoded() > streamed + 1) {
        fprintf(stderr, "jpeg worker kept encoding without clients: %d after %d\n", encoded(), streamed);
        return 1;
    }

    /* A worker sleeping for demand must still stop. */
    jpeg_worker_request_stop();
    pthread_join(worker, NULL);
    jpeg_worker_close();

    /* stream.max_fps = 15: of frames captured 1/30 s apart, every second one is encoded; the
     * first at once.  Each frame is published after the previous encode has finished. */
    {
        int before;
        double base;

        if (jpeg_worker_init() != 0)
            return 1;
        jpeg_worker_set_max_fps(15);
        jpeg_worker_set_demand(1);
        if (pthread_create(&worker, NULL, jpeg_worker_thread, NULL) != 0)
            return 1;
        /* The new worker first encodes frame 6, still the newest in the mailbox; it has no
         * capture time, so the worker uses the clock.  The frames below start a second later. */
        usleep(80000);
        before = encoded();
        base = monotonic_sec() + 1.0;
        for (int k = 0; k < 6; k++) {
            publish_at(source, (uint8_t)(7 + k), base + k / 30.0);
            usleep(70000);
        }
        usleep(50000);
        if (encoded() - before != 3 || __atomic_load_n(&last_seq, __ATOMIC_ACQUIRE) != 11u) {
            fprintf(stderr, "jpeg worker at max_fps 15 encoded %d of 6, last_seq=%u\n",
                    encoded() - before, last_seq);
            return 1;
        }
        jpeg_worker_request_stop();
        pthread_join(worker, NULL);
        jpeg_worker_close();
    }
    frame_mailbox_destroy(&g_state);

    if (__atomic_load_n(&data_mismatch, __ATOMIC_ACQUIRE)) {
        fputs("jpeg worker data mismatch\n", stderr);
        return 1;
    }

    puts("jpeg_worker_test: OK");
    return 0;
}
