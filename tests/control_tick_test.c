/* control_tick_test.c - control tick timing: detection on time, late, missing and disabled. */
#include "capture/control_tick.h"

#include <stdio.h>
#include <stdlib.h>

#define DEADLINE_MS 24.0
#define FRAME_SEC (1.0 / 30.0)

static int g_checks;

static void require(int cond, const char *what)
{
    g_checks++;
    if (!cond) {
        fprintf(stderr, "control_tick_test: FAIL: %s\n", what);
        exit(1);
    }
}

/* Detection of this frame arrives before the deadline: the tick runs on it and the write lands
 * before the next frame, so the view is registered under this frame. */
static void test_on_time(void)
{
    control_tick_t tick;
    uint32_t issued = 0;

    control_tick_init(&tick, DEADLINE_MS);
    require(control_tick_settle(&tick, 1.0, &issued) == 0, "nothing pending at start");
    control_tick_begin(&tick, 100u, 500u, 10.0, 1);
    require(control_tick_poll(&tick, 499u, 10.001) == CONTROL_TICK_WAIT, "older detection does not end the wait");
    require(control_tick_poll(&tick, 499u, 10.023) == CONTROL_TICK_WAIT, "still waiting before the deadline");
    require(control_tick_poll(&tick, 500u, 10.0235) == CONTROL_TICK_ON_DETECTION, "this frame's detection runs it");
    control_tick_end(&tick, CONTROL_TICK_ON_DETECTION, 1, 10.027);
    require(control_tick_settle(&tick, 10.0 + FRAME_SEC, &issued) == 1 && issued == 100u,
            "write before the frame end stays on this frame");
    require(control_tick_settle(&tick, 10.1, &issued) == 0, "settled only once");
    require(tick.stats.on_detection == 1u && tick.stats.isp_slips == 0u && tick.stats.writes == 1u,
            "statistics of an on-time tick");
    require(tick.stats.margin_min_ms > 6.2 && tick.stats.margin_min_ms < 6.4, "margin recorded");
}

/* Detection arrives after the deadline: the tick runs without it, the next tick consumes it. */
static void test_late(void)
{
    control_tick_t tick;

    control_tick_init(&tick, DEADLINE_MS);
    control_tick_begin(&tick, 100u, 500u, 10.0, 1);
    require(control_tick_poll(&tick, 499u, 10.024) == CONTROL_TICK_ON_DEADLINE, "deadline ends the wait");
    require(control_tick_deadline(&tick) > 10.0239 && control_tick_deadline(&tick) < 10.0241,
            "deadline is relative to the frame");
    control_tick_end(&tick, CONTROL_TICK_ON_DEADLINE, 0, 10.0241);
    require(tick.stats.on_deadline == 1u, "deadline tick counted");
}

/* The detector published nothing for this frame at all. */
static void test_missing(void)
{
    control_tick_t tick;
    uint32_t issued = 0;

    control_tick_init(&tick, DEADLINE_MS);
    control_tick_begin(&tick, 100u, 500u, 10.0, 1);
    require(control_tick_poll(&tick, 0u, 10.03) == CONTROL_TICK_ON_DEADLINE, "missing detection is bounded");
    control_tick_end(&tick, CONTROL_TICK_ON_DEADLINE, 1, 10.03);
    require(control_tick_settle(&tick, 10.0 + FRAME_SEC, &issued) == 1 && issued == 100u, "still before frame end");
}

/* Detection disabled or recovering, or tracking idle: same cadence as the baseline. */
static void test_not_expected(void)
{
    control_tick_t tick;

    control_tick_init(&tick, DEADLINE_MS);
    control_tick_begin(&tick, 100u, 500u, 10.0, 0);
    require(control_tick_poll(&tick, 0u, 10.0) == CONTROL_TICK_IMMEDIATE, "no detection expected: no wait");
    control_tick_end(&tick, CONTROL_TICK_IMMEDIATE, 0, 10.0);
    require(tick.stats.immediate == 1u, "immediate tick counted");

    /* Deadline 0 selects the baseline cadence for A/B runs. */
    control_tick_init(&tick, 0.0);
    control_tick_begin(&tick, 100u, 500u, 10.0, 1);
    require(control_tick_poll(&tick, 0u, 10.0) == CONTROL_TICK_IMMEDIATE, "deadline 0 never waits");
}

/* The write ends after the next frame: the command lands one frame later and the view must say so. */
static void test_slip(void)
{
    control_tick_t tick;
    uint32_t issued = 0;

    control_tick_init(&tick, DEADLINE_MS);
    control_tick_begin(&tick, 100u, 500u, 10.0, 1);
    control_tick_end(&tick, CONTROL_TICK_ON_DEADLINE, 1, 10.0 + FRAME_SEC + 0.001);
    require(control_tick_settle(&tick, 10.0 + FRAME_SEC, &issued) == 1 && issued == 101u,
            "late write is registered one frame later");
    require(tick.stats.isp_slips == 1u && tick.stats.margin_min_ms < 0.0, "slip counted with a negative margin");

    /* No write, no slip: the view did not change. */
    control_tick_begin(&tick, 101u, 501u, 10.0 + FRAME_SEC, 1);
    control_tick_end(&tick, CONTROL_TICK_ON_DEADLINE, 0, 10.0 + 3.0 * FRAME_SEC);
    require(control_tick_settle(&tick, 10.0 + 2.0 * FRAME_SEC, &issued) == 1 && issued == 101u,
            "a tick without a write cannot slip");
    require(tick.stats.isp_slips == 1u && tick.stats.writes == 1u, "statistics unchanged");

    control_tick_stats_reset(&tick);
    require(tick.stats.isp_slips == 0u && tick.deadline_sec > 0.0, "statistics reset keeps the deadline");
    require(tick.isp_slips_total == 1u, "the slip total survives a statistics reset");
}

static void test_sequence_wrap(void)
{
    control_tick_t tick;
    uint32_t issued = 0;

    control_tick_init(&tick, DEADLINE_MS);
    control_tick_begin(&tick, UINT32_MAX, 0xfffffff0u, 10.0, 1);
    require(control_tick_poll(&tick, 0xffffffefu, 10.0) == CONTROL_TICK_WAIT, "older before the wrap");
    require(control_tick_poll(&tick, 0x00000002u, 10.0) == CONTROL_TICK_ON_DETECTION, "newer after the wrap");
    control_tick_end(&tick, CONTROL_TICK_ON_DETECTION, 1, 10.1);
    require(control_tick_settle(&tick, 10.0, &issued) == 1 && issued == 0u, "issued sequence wraps");
}

static void test_reset(void)
{
    control_tick_t tick;
    uint32_t issued = 0;

    control_tick_init(&tick, DEADLINE_MS);
    control_tick_begin(&tick, 100u, 500u, 10.0, 1);
    control_tick_end(&tick, CONTROL_TICK_ON_DETECTION, 1, 10.02);
    control_tick_reset(&tick);
    require(control_tick_settle(&tick, 10.03, &issued) == 0 && tick.stats.on_detection == 0u,
            "reset drops the pending tick and the statistics");
    require(tick.deadline_sec > 0.0239 && tick.deadline_sec < 0.0241, "reset keeps the deadline");
}

/* The deadline is too short when most waiting ticks run on it (board measurement against a 22 ms default). */
static void test_watch(void)
{
    control_tick_t tick;
    unsigned int late = 0;

    control_tick_init(&tick, 24.0);
    for (unsigned int i = 0; i < CONTROL_TICK_WATCH_WAITS - 1u; i++) {
        control_tick_end(&tick, CONTROL_TICK_ON_DEADLINE, 0, 0.0);
        control_tick_end(&tick, CONTROL_TICK_IMMEDIATE, 0, 0.0);
    }
    require(control_tick_watch(&tick, &late) == 0, "no verdict before the window is full");
    require(tick.watch_waits == CONTROL_TICK_WATCH_WAITS - 1u, "immediate ticks do not count");
    control_tick_end(&tick, CONTROL_TICK_ON_DETECTION, 0, 0.0);
    require(control_tick_watch(&tick, &late) == 1 && late == CONTROL_TICK_WATCH_WAITS - 1u,
            "most ticks late: too short");
    require(control_tick_watch(&tick, &late) == 0 && tick.watch_waits == 0, "the window starts over");

    for (unsigned int i = 0; i < CONTROL_TICK_WATCH_WAITS; i++)
        control_tick_end(&tick, i % 2u ? CONTROL_TICK_ON_DEADLINE : CONTROL_TICK_ON_DETECTION, 0, 0.0);
    require(control_tick_watch(&tick, &late) == 0, "half late is not most");
    for (unsigned int i = 0; i < CONTROL_TICK_WATCH_WAITS; i++)
        control_tick_end(&tick, CONTROL_TICK_ON_DETECTION, 0, 0.0);
    control_tick_reset(&tick);
    require(tick.watch_waits == 0, "a reset clears the window");
}

int main(void)
{
    test_on_time();
    test_late();
    test_missing();
    test_not_expected();
    test_slip();
    test_sequence_wrap();
    test_reset();
    test_watch();
    printf("control_tick_test: OK (%d checks)\n", g_checks);
    return 0;
}
