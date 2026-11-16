/* dvp_counts_test.c - the bridge counters as the status interface reports them. */
#include "capture/dvp_counts.h"

#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>

static int g_checks;

static void require(int cond, const char *what)
{
    g_checks++;
    if (!cond) {
        fprintf(stderr, "dvp_counts_test: FAIL: %s\n", what);
        exit(1);
    }
}

/* Board measurement: the first start of the day added 47, a stop and start 63; a steady run adds none. */
static void test_board_sequence(void)
{
    dvp_counts_t c;

    dvp_counts_init(&c);
    require(dvp_counts_update(&c, 429, 47) == 1, "the first reading is the base");
    require(c.run_frames == 0 && c.run_bad == 0, "the start-up transition is not this run's");
    require(dvp_counts_update(&c, 1659, 47) == 0, "later readings are not the base");
    require(c.run_frames == 1230 && c.run_bad == 0, "a steady run counts frames, no errors");
    dvp_counts_update(&c, 22839, 47);
    require(c.run_frames == 22410 && c.run_bad == 0, "759 s, still no errors");

    /* the next process starts after a stop: the 63 belong to neither run */
    dvp_counts_init(&c);
    require(dvp_counts_update(&c, 23106, 110) == 1, "the next run takes its own base");
    dvp_counts_update(&c, 23706, 110);
    require(c.run_frames == 600 && c.run_bad == 0, "and reports from it");
}

static void test_errors_in_run(void)
{
    dvp_counts_t c;

    dvp_counts_init(&c);
    dvp_counts_update(&c, 1000, 50);
    dvp_counts_update(&c, 1030, 52);
    dvp_counts_update(&c, 1060, 52);
    require(c.run_bad == 2, "errors during the run are counted");
    dvp_counts_update(&c, 1090, 53);
    require(c.run_bad == 3 && c.run_frames == 90, "and accumulate");
}

/* The camera turned off and on: the stop and the start add bad frames that are neither stream's. */
static void test_camera_toggle(void)
{
    dvp_counts_t c;

    dvp_counts_init(&c);
    dvp_counts_update(&c, 1000, 47);
    dvp_counts_update(&c, 1300, 47);   /* last reading before the stop */
    dvp_counts_restart(&c);
    require(c.run_frames == 300 && c.run_bad == 0, "a restart keeps the totals");
    require(dvp_counts_update(&c, 1400, 110) == 1, "the new stream takes a new base");
    require(c.run_frames == 300 && c.run_bad == 0, "the 63 of the transition are not counted");
    dvp_counts_update(&c, 1430, 111);
    require(c.run_frames == 330 && c.run_bad == 1, "the new stream counts on");
    dvp_counts_restart(&c);
    dvp_counts_restart(&c);
    require(dvp_counts_update(&c, 2000, 200) == 1 && c.run_frames == 330,
            "two restarts without a reading are one");
}

static void test_odd_readings(void)
{
    dvp_counts_t c;

    dvp_counts_init(&c);
    require(dvp_counts_update(&c, -1, 0) == 0 && !c.have_base, "a negative reading is ignored");
    dvp_counts_update(&c, 5000, 10);
    dvp_counts_update(&c, 5030, 10);
    dvp_counts_update(&c, 20, 1);   /* the PL was reset: counting starts over */
    require(c.run_frames == 50 && c.run_bad == 1, "a reset counter counts on from 0");
    dvp_counts_update(&c, 50, 1);
    require(c.run_frames == 80 && c.run_bad == 1, "and on from there");

    dvp_counts_init(&c);
    dvp_counts_update(&c, INT32_MAX - 10, 0);
    dvp_counts_update(&c, INT32_MAX, 0);
    require(c.run_frames == 10, "counts up to the top of the range");
}

int main(void)
{
    test_board_sequence();
    test_errors_in_run();
    test_camera_toggle();
    test_odd_readings();
    printf("dvp_counts_test: OK (%d checks)\n", g_checks);
    return 0;
}
