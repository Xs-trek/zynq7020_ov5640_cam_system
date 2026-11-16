#define _GNU_SOURCE
#define LOG_TAG "jpeg-worker"

#include "media/jpeg_worker.h"

#include "cam_system/app_state.h"
#include "cam_system/config.h"
#include "cam_system/frame_mailbox.h"
#include "media/jpeg_enc.h"
#include "media/stream_rate.h"
#include "common/log.h"
#include "common/time_util.h"

#include <errno.h>
#include <pthread.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/resource.h>

typedef struct {
    int stop;
    unsigned long observed;
    unsigned long replaced;
    unsigned long encoded;
    unsigned long skipped;      /* left out by stream.max_fps */
    stream_rate_t rate;
    int log_every;
    int log_count;
    double log_encode_ms;
    double log_pickup_ms;
    double log_capture_age_ms;
    double log_start_ms;
    int initialized;
} jpeg_worker_state_t;

static jpeg_worker_state_t g_jpeg_worker;

static pthread_mutex_t g_demand_lock = PTHREAD_MUTEX_INITIALIZER;
static pthread_cond_t g_demand_cond = PTHREAD_COND_INITIALIZER;
static int g_demand_clients;

int jpeg_worker_init(void)
{
    memset(&g_jpeg_worker, 0, sizeof(g_jpeg_worker));

    g_jpeg_worker.initialized = 1;
    LOG_I("ready: shared immutable latest-frame mailbox");
    return 0;
}

void jpeg_worker_set_log_every(int frames)
{
    g_jpeg_worker.log_every = frames > 0 ? frames : 0;
}

void jpeg_worker_set_max_fps(int max_fps)
{
    stream_rate_init(&g_jpeg_worker.rate, max_fps);
    if (max_fps > 0)
        LOG_I("page stream limited to %d fps (stream.max_fps)", max_fps);
}

void jpeg_worker_set_demand(int clients)
{
    pthread_mutex_lock(&g_demand_lock);
    g_demand_clients = clients;
    pthread_cond_broadcast(&g_demand_cond);
    pthread_mutex_unlock(&g_demand_lock);
}

int jpeg_worker_has_demand(void)
{
    int clients;

    pthread_mutex_lock(&g_demand_lock);
    clients = g_demand_clients;
    pthread_mutex_unlock(&g_demand_lock);
    return clients > 0;
}

static int jpeg_worker_stopping(void)
{
    return app_state_load_int(&g_state.quit) ||
           __atomic_load_n(&g_jpeg_worker.stop, __ATOMIC_ACQUIRE);
}

/* Blocks until a stream client is connected or the worker stops.  Returns 1 if it had to wait. */
static int jpeg_worker_wait_for_demand(void)
{
    int waited = 0;

    pthread_mutex_lock(&g_demand_lock);
    while (g_demand_clients <= 0 && !jpeg_worker_stopping()) {
        waited = 1;
        pthread_cond_wait(&g_demand_cond, &g_demand_lock);
    }
    pthread_mutex_unlock(&g_demand_lock);
    return waited;
}

void jpeg_worker_request_stop(void)
{
    if (!g_jpeg_worker.initialized)
        return;
    __atomic_store_n(&g_jpeg_worker.stop, 1, __ATOMIC_RELEASE);
    pthread_mutex_lock(&g_demand_lock);
    pthread_cond_broadcast(&g_demand_cond);
    pthread_mutex_unlock(&g_demand_lock);
    frame_mailbox_wake_all(&g_state);
}

static void jpeg_worker_lower_priority(void)
{
    errno = 0;
    if (setpriority(PRIO_PROCESS, 0, 5) < 0) {
        LOG_W("unable to set nice=5: %s",
      strerror(errno));
        return;
    }
    errno = 0;
    int nice_value = getpriority(PRIO_PROCESS, 0);
    if (errno == 0)
        LOG_I("scheduling nice=%d (detector priority preserved)",
              nice_value);
}

void *jpeg_worker_thread(void *arg)
{
    unsigned int last_seq = 0;

    (void)arg;
    pthread_setname_np(pthread_self(), "jpeg-worker");
    jpeg_worker_lower_priority();
    LOG_I("thread started on Core %d", CAM_CPU_JPEG_CORE);

    while (!jpeg_worker_stopping()) {
        frame_lease_t lease;
        int acquired;

        /* Frames that passed while idle are not "replaced" in the statistics, and the first
         * frame for a new client is encoded at once. */
        if (jpeg_worker_wait_for_demand()) {
            last_seq = 0;
            stream_rate_reset(&g_jpeg_worker.rate);
        }
        if (jpeg_worker_stopping())
            break;

        acquired = frame_mailbox_wait_latest(&g_state, g_cached_pool, last_seq, 250, &lease);
        if (acquired < 0) {
            LOG_E("mailbox wait failed: %d", acquired);
            break;
        }
        if (acquired == 0)
            continue;
        if (jpeg_worker_stopping()) {
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
        if (!stream_rate_take(&g_jpeg_worker.rate, lease.view.capture_time_sec > 0.0 ?
                                                       lease.view.capture_time_sec :
                                                       monotonic_sec())) {
            frame_mailbox_release(&g_state, &lease);
            g_jpeg_worker.skipped++;
            continue;
        }

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
                LOG_I("avg%d encode=%.2fms pickup=%.2fms "
                      "capture_age=%.2fms output_fps=%.2f "
                      "observed=%lu encoded=%lu replaced=%lu skipped=%lu",
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
                      g_jpeg_worker.replaced, g_jpeg_worker.skipped);
                g_jpeg_worker.log_count = 0;
                g_jpeg_worker.log_encode_ms = 0.0;
                g_jpeg_worker.log_pickup_ms = 0.0;
                g_jpeg_worker.log_capture_age_ms = 0.0;
            }
        }
    }

    LOG_I("thread exiting observed=%lu encoded=%lu replaced=%lu skipped=%lu",
          g_jpeg_worker.observed, g_jpeg_worker.encoded,
          g_jpeg_worker.replaced, g_jpeg_worker.skipped);
    return NULL;
}

void jpeg_worker_close(void)
{
    jpeg_worker_set_demand(0);
    memset(&g_jpeg_worker, 0, sizeof(g_jpeg_worker));
}
