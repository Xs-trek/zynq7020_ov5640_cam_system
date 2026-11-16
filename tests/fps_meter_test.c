/* fps_meter_test.c - rate over windows, stale rate, clock wrap. */
#include "common/fps_meter.h"

#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

static int g_checks;

static void require(int cond, const char *what)
{
    g_checks++;
    if (!cond) {
        fprintf(stderr, "fps_meter_test: FAIL: %s\n", what);
        exit(1);
    }
}

static double run(fps_meter_t *meter, double start, double rate, int events)
{
    double now = start;

    for (int i = 0; i < events; i++) {
        now = start + (double)i / rate;
        fps_meter_tick(meter, now);
    }
    return now;
}

int main(void)
{
    fps_meter_t meter;
    double now;

    memset(&meter, 0, sizeof(meter));
    require(fps_meter_read(&meter, 100.0) == 0.0f, "no events: rate 0");

    now = run(&meter, 100.0, 30.0, 95);
    require(fabsf(fps_meter_read(&meter, now) - 30.0f) < 0.05f, "30 Hz measured");
    now = run(&meter, now + 1.0 / 15.0, 15.0, 60);
    require(fabsf(fps_meter_read(&meter, now) - 15.0f) < 0.1f, "rate follows a change");

    require(fps_meter_read(&meter, now + 1.0) > 0.0f, "rate kept across a short gap");
    require(fps_meter_read(&meter, now + 3.5) == 0.0f, "rate drops when events stop");

    /* The millisecond clock wraps at 2^32 ms; the age is taken by difference. */
    memset(&meter, 0, sizeof(meter));
    now = run(&meter, 4294967.0, 30.0, 95);
    require(fps_meter_read(&meter, now + 0.5) > 0.0f, "fresh across the millisecond wrap");
    require(fps_meter_read(&meter, now + 3.5) == 0.0f, "stale across the millisecond wrap");

    printf("fps_meter_test: OK (%d checks)\n", g_checks);
    return 0;
}
