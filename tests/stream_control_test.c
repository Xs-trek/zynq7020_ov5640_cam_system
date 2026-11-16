/* stream_control_test.c - when the capture stream, and the sensor with it, runs. */
#include "capture/stream_control.h"

#include <stdio.h>
#include <stdlib.h>

static int g_checks;

static void require(int cond, const char *what)
{
    g_checks++;
    if (!cond) {
        fprintf(stderr, "stream_control_test: FAIL: %s\n", what);
        exit(1);
    }
}

/* The application starts with the camera off: nothing streams until the user enables it. */
static void test_on_and_off(void)
{
    stream_control_t c;

    stream_control_init(&c, 20.0 * 60.0);
    require(stream_control_poll(&c, 0, 10.0) == STREAM_KEEP, "no stream at start-up");
    require(stream_control_run_sec(&c, 10.0) == 0.0, "nothing has run");
    require(stream_control_poll(&c, 1, 11.0) == STREAM_START, "enabling the camera starts it");
    stream_control_started(&c, 11.0);
    require(stream_control_poll(&c, 1, 12.0) == STREAM_KEEP, "it keeps running");
    require(stream_control_run_sec(&c, 71.0) == 60.0, "run time counts from the start");
    require(stream_control_poll(&c, 0, 72.0) == STREAM_STOP, "disabling the camera stops it");
    stream_control_stopped(&c, STREAM_STOP);
    require(!c.streaming && !c.stopped_by_limit, "a user stop is not the limit's");
    require(stream_control_run_sec(&c, 73.0) == 0.0, "no run time while stopped");
    require(stream_control_poll(&c, 0, 74.0) == STREAM_KEEP, "and it stays stopped");
}

/* The limit counts the time of one stream; a stop starts it over. */
static void test_limit(void)
{
    stream_control_t c;

    stream_control_init(&c, 1200.0);
    stream_control_started(&c, 100.0);
    require(stream_control_poll(&c, 1, 1299.9) == STREAM_KEEP, "just under the limit");
    require(stream_control_poll(&c, 1, 1300.0) == STREAM_STOP_LIMIT, "the limit stops it");
    stream_control_stopped(&c, STREAM_STOP_LIMIT);
    require(c.stopped_by_limit, "the reason is kept");
    /* the caller disabled the camera */
    require(stream_control_poll(&c, 0, 1301.0) == STREAM_KEEP, "it stays off");
    require(c.stopped_by_limit, "the reason stays until the next start");
    require(stream_control_poll(&c, 1, 1400.0) == STREAM_START, "the user enables it again");
    stream_control_started(&c, 1400.0);
    require(!c.stopped_by_limit, "a start clears the reason");
    require(stream_control_poll(&c, 1, 2599.0) == STREAM_KEEP, "a new stream has the whole limit");

    /* off and on again within the limit: the time starts over */
    require(stream_control_poll(&c, 0, 2000.0) == STREAM_STOP, "off");
    stream_control_stopped(&c, STREAM_STOP);
    stream_control_started(&c, 2001.0);
    require(stream_control_poll(&c, 1, 3200.0) == STREAM_KEEP, "the earlier stream does not count");
    require(stream_control_poll(&c, 1, 3201.0) == STREAM_STOP_LIMIT, "only this one does");
    require(stream_control_poll(&c, 0, 3201.0) == STREAM_STOP, "a user stop at the same time wins");
}

static void test_no_limit(void)
{
    stream_control_t c;

    stream_control_init(&c, 0.0);
    stream_control_started(&c, 0.0);
    require(stream_control_poll(&c, 1, 1e7) == STREAM_KEEP, "0 means no limit");
    stream_control_init(&c, -5.0);
    stream_control_started(&c, 0.0);
    require(stream_control_poll(&c, 1, 1e7) == STREAM_KEEP, "a negative limit is none");
    require(stream_control_run_sec(&c, -1.0) == 0.0, "time before the start is no run time");
}

int main(void)
{
    test_on_and_off();
    test_limit();
    test_no_limit();
    printf("stream_control_test: OK (%d checks)\n", g_checks);
    return 0;
}
