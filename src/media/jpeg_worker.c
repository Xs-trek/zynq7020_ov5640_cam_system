#define _GNU_SOURCE
#include "media/jpeg_worker.h"

#include "cam_system/app_state.h"
#include "cam_system/config.h"
#include "cam_system/frame_mailbox.h"
#include "media/jpeg_enc.h"

#include <errno.h>
#include <pthread.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/resource.h>
#include <time.h>

typedef struct {
    int stop;
    unsigned long observed;
    unsigned long replaced;
    unsigned long encoded;
    int log_every;
    int log_count;
    double log_encode_ms;
    double log_pickup_ms;
    double log_capture_age_ms;
    double log_start_ms;
    int initialized;
} jpeg_worker_state_t;

static jpeg_worker_state_t g_jpeg_worker;

static double monotonic_ms(void)
{
    struct timespec ts;
    clock_gettime(CLOCK_MONOTONIC, &ts);
    return (double)ts.tv_sec * 1000.0 + (double)ts.tv_nsec / 1000000.0;
}

int jpeg_worker_init(void)
{
    const char *log_every;

    memset(&g_jpeg_worker, 0, sizeof(g_jpeg_worker));
    log_every = getenv("CAM_JPEG_LOG_EVERY");
    if (log_every) {
        g_jpeg_worker.log_every = atoi(log_every);
        if (g_jpeg_worker.log_every < 0)
            g_jpeg_worker.log_every = 0;
    }

    g_jpeg_worker.initialized = 1;
    printf("[jpeg-worker] ready: shared immutable latest-frame mailbox\n");
    return 0;
}

void jpeg_worker_request_stop(void)
{
    if (!g_jpeg_worker.initialized)
        return;
    __atomic_store_n(&g_jpeg_worker.stop, 1, __ATOMIC_RELEASE);
    frame_mailbox_wake_all(&g_state);
}

static void jpeg_worker_lower_priority(void)
{
    errno = 0;
    if (setpriority(PRIO_PROCESS, 0, 5) < 0) {
        fprintf(stderr, "[jpeg-worker] unable to set nice=5: %s\n",
                strerror(errno));
        return;
    }
    errno = 0;
    int nice_value = getpriority(PRIO_PROCESS, 0);
    if (errno == 0)
        printf("[jpeg-worker] scheduling nice=%d (detector priority preserved)\n",
               nice_value);
}

void *jpeg_worker_thread(void *arg)
{
    unsigned int last_seq = 0;

    (void)arg;
    pthread_setname_np(pthread_self(), "jpeg-worker");
    jpeg_worker_lower_priority();
    printf("[jpeg-worker] thread started on Core %d\n", CAM_CPU_JPEG_CORE);

    while (!app_state_load_int(&g_state.quit) &&
           !__atomic_load_n(&g_jpeg_worker.stop, __ATOMIC_ACQUIRE)) {
        frame_lease_t lease;
        int acquired = frame_mailbox_wait_latest(&g_state, g_cached_pool,
                                                  last_seq, 250, &lease);
        if (acquired < 0) {
            fprintf(stderr, "[jpeg-worker] mailbox wait failed: %d\n", acquired);
            break;
        }
        if (acquired == 0)
            continue;
        if (__atomic_load_n(&g_jpeg_worker.stop, __ATOMIC_ACQUIRE) ||
            app_state_load_int(&g_state.quit)) {
            frame_mailbox_release(&g_state, &lease);
            break;
        }

        double pickup_ms = monotonic_ms() - lease.publish_time_sec * 1000.0;
        double capture_age_ms = lease.view.capture_time_sec > 0.0 ?
            monotonic_ms() - lease.view.capture_time_sec * 1000.0 : 0.0;
        if (last_seq != 0U) {
            unsigned int gap = lease.frame_seq - last_seq;
            if (gap > 1U && gap < 0x80000000U)
                g_jpeg_worker.replaced += (unsigned long)(gap - 1U);
        }
        last_seq = lease.frame_seq;
        g_jpeg_worker.observed++;

        double encode_t0 = monotonic_ms();
        int encode_ret = jpeg_encode_frame(lease.data, lease.frame_seq);
        double encode_ms = monotonic_ms() - encode_t0;
        frame_mailbox_release(&g_state, &lease);
        if (encode_ret == 0)
            g_jpeg_worker.encoded++;

        if (g_jpeg_worker.log_every > 0) {
            if (g_jpeg_worker.log_count == 0)
                g_jpeg_worker.log_start_ms = encode_t0;
            g_jpeg_worker.log_encode_ms += encode_ms;
            g_jpeg_worker.log_pickup_ms += pickup_ms;
            g_jpeg_worker.log_capture_age_ms += capture_age_ms;
            g_jpeg_worker.log_count++;
            if (g_jpeg_worker.log_count >= g_jpeg_worker.log_every) {
                double elapsed_ms = monotonic_ms() - g_jpeg_worker.log_start_ms;
                printf("[jpeg-worker] avg%d encode=%.2fms pickup=%.2fms "
                       "capture_age=%.2fms output_fps=%.2f "
                       "observed=%lu encoded=%lu replaced=%lu\n",
                       g_jpeg_worker.log_count,
                       g_jpeg_worker.log_encode_ms /
                           (double)g_jpeg_worker.log_count,
                       g_jpeg_worker.log_pickup_ms /
                           (double)g_jpeg_worker.log_count,
                       g_jpeg_worker.log_capture_age_ms /
                           (double)g_jpeg_worker.log_count,
                       elapsed_ms > 0.0 ?
                           1000.0 * (double)g_jpeg_worker.log_count / elapsed_ms : 0.0,
                       g_jpeg_worker.observed, g_jpeg_worker.encoded,
                       g_jpeg_worker.replaced);
                g_jpeg_worker.log_count = 0;
                g_jpeg_worker.log_encode_ms = 0.0;
                g_jpeg_worker.log_pickup_ms = 0.0;
                g_jpeg_worker.log_capture_age_ms = 0.0;
            }
        }
    }

    printf("[jpeg-worker] thread exiting observed=%lu encoded=%lu replaced=%lu\n",
           g_jpeg_worker.observed, g_jpeg_worker.encoded,
           g_jpeg_worker.replaced);
    return NULL;
}

void jpeg_worker_close(void)
{
    memset(&g_jpeg_worker, 0, sizeof(g_jpeg_worker));
}
