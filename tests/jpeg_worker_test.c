#define _GNU_SOURCE
#include "cam_system/app_state.h"
#include "cam_system/frame_mailbox.h"
#include "media/jpeg_worker.h"

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

static int encoded_count;
static unsigned int last_seq;
static int data_mismatch;

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

    frame_view_t view = {0};
    memset(source, 1, sizeof(source));
    frame_mailbox_publish(&g_state, g_cached_pool, source, &view, NULL, NULL);
    usleep(5000);
    memset(source, 2, sizeof(source));
    frame_mailbox_publish(&g_state, g_cached_pool, source, &view, NULL, NULL);
    memset(source, 3, sizeof(source));
    frame_mailbox_publish(&g_state, g_cached_pool, source, &view, NULL, NULL);

    usleep(130000);
    jpeg_worker_request_stop();
    pthread_join(worker, NULL);
    jpeg_worker_close();
    frame_mailbox_destroy(&g_state);

    int count = __atomic_load_n(&encoded_count, __ATOMIC_ACQUIRE);
    unsigned int seq = __atomic_load_n(&last_seq, __ATOMIC_ACQUIRE);
    if (count < 1 || count > 2 || seq != 3 ||
        __atomic_load_n(&data_mismatch, __ATOMIC_ACQUIRE)) {
        fprintf(stderr, "jpeg worker failed count=%d last_seq=%u mismatch=%d\n",
                count, seq, data_mismatch);
        return 1;
    }

    puts("jpeg_worker_test: OK");
    return 0;
}
