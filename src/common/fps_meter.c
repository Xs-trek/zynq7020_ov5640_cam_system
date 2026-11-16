/* fps_meter.c - see fps_meter.h */
#include "common/fps_meter.h"

#include <stdint.h>

#define WINDOW_SEC 1.0
#define STALE_MS 2500

static int to_ms(double seconds)
{
    return (int)(uint32_t)(uint64_t)(seconds * 1000.0);
}

void fps_meter_tick(fps_meter_t *meter, double now)
{
    double elapsed;

    if (meter->window_start <= 0.0) {
        meter->window_start = now;
        meter->count = 0u;
        return;
    }
    meter->count++;
    elapsed = now - meter->window_start;
    if (elapsed < WINDOW_SEC)
        return;
    __atomic_store_n(&meter->milli_fps, (int)(1000.0 * (double)meter->count / elapsed + 0.5), __ATOMIC_RELAXED);
    __atomic_store_n(&meter->updated_ms, to_ms(now), __ATOMIC_RELAXED);
    meter->window_start = now;
    meter->count = 0u;
}

float fps_meter_read(const fps_meter_t *meter, double now)
{
    const int milli_fps = __atomic_load_n(&meter->milli_fps, __ATOMIC_RELAXED);
    const int updated_ms = __atomic_load_n(&meter->updated_ms, __ATOMIC_RELAXED);

    if ((int)((uint32_t)to_ms(now) - (uint32_t)updated_ms) > STALE_MS)
        return 0.0f;
    return (float)milli_fps / 1000.0f;
}
