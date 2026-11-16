/* capture_recovery_test.c - the capture stall policy. */
#include "capture/capture_recovery.h"

#include <stdio.h>
#include <stdlib.h>

static int g_checks;

static void require(int cond, const char *what)
{
    g_checks++;
    if (!cond) {
        fprintf(stderr, "capture_recovery_test: FAIL: %s\n", what);
        exit(1);
    }
}

int main(void)
{
    caprec_t rec;
    double now = 100.0;

    caprec_init(&rec, now);
    require(caprec_on_no_frame(&rec, now + 2.0) == CAPREC_WAIT, "one poll timeout is not a stall");
    require(caprec_on_no_frame(&rec, now + 3.9) == CAPREC_WAIT, "just under the limit");

    /* a stream that never comes back: three restarts, then give up */
    for (int i = 1; i <= CAPREC_RESTART_LIMIT; i++) {
        now += CAPREC_STALL_SEC;
        require(caprec_on_no_frame(&rec, now) == CAPREC_RESTART, "restart after a stall");
        require(rec.restarts == i, "restart counted");
        require(caprec_on_no_frame(&rec, now + 1.0) == CAPREC_WAIT, "a restart gets its own time");
    }
    now += CAPREC_STALL_SEC;
    require(caprec_on_no_frame(&rec, now) == CAPREC_GIVE_UP, "give up after the limit");
    require(caprec_on_no_frame(&rec, now + 10.0) == CAPREC_GIVE_UP, "and stays that way");

    /* warm-up after a restart touches the clock but keeps the count */
    caprec_init(&rec, 0.0);
    require(caprec_on_no_frame(&rec, 4.0) == CAPREC_RESTART, "first restart");
    caprec_touch(&rec, 4.5);
    require(rec.restarts == 1, "touch keeps the count");
    require(caprec_on_no_frame(&rec, 8.0) == CAPREC_WAIT, "touch moved the clock");

    /* one good frame forgets the history */
    caprec_on_frame(&rec, 8.5);
    require(rec.restarts == 0, "a frame clears the count");
    require(caprec_on_no_frame(&rec, 12.0) == CAPREC_WAIT, "a frame moved the clock");
    require(caprec_on_no_frame(&rec, 12.5) == CAPREC_RESTART, "a later stall starts over");
    require(rec.restarts == 1, "counted from one again");

    /* error frames: attempts come fast, the clock decides */
    caprec_init(&rec, 0.0);
    for (double t = 0.0; t < 3.9; t += 0.033)
        require(caprec_on_no_frame(&rec, t) == CAPREC_WAIT, "error frames inside the limit");
    require(caprec_on_no_frame(&rec, 4.0) == CAPREC_RESTART, "error frames for too long");

    printf("capture_recovery_test: OK (%d checks)\n", g_checks);
    return 0;
}
