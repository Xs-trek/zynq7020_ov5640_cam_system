/* detector_recovery_test.c - fault injection against the detector recovery policy. */
#include "detect/detector_recovery.h"

#include <errno.h>
#include <stdio.h>
#include <stdlib.h>

static int g_checks;

static void require(int cond, const char *what)
{
    g_checks++;
    if (!cond) {
        fprintf(stderr, "detector_recovery_test: FAIL: %s\n", what);
        exit(1);
    }
}

static drec_t fresh(void)
{
    drec_config_t cfg = {3, 0.5, 4.0};
    drec_t r;

    drec_init(&r, &cfg);
    return r;
}

/* One transient failure: retry on the next frame, recover on success. */
static void test_transient(void)
{
    drec_t r = fresh();

    require(drec_next_action(&r, 0.0) == DREC_ACT_RUN && drec_health(&r) == DREC_HEALTH_RUNNING,
            "starts running");
    drec_on_job(&r, -ETIMEDOUT, 1.0);
    require(r.state == DREC_RETRY && drec_next_action(&r, 1.0) == DREC_ACT_RUN, "failure -> retry with a job");
    require(drec_health(&r) == DREC_HEALTH_RECOVERING && r.last_error == -ETIMEDOUT, "recovering, error kept");
    drec_on_job(&r, 0, 1.1);
    require(r.state == DREC_RUNNING && r.recoveries == 1 && r.total_failures == 1, "success -> running");
    require(r.consecutive_failures == 0, "failure streak cleared");
}

/* Persistent job failures escalate to a reset; a good job after the reset recovers. */
static void test_reset_recovers(void)
{
    drec_t r = fresh();

    drec_on_job(&r, -EIO, 1.0);
    drec_on_job(&r, -EIO, 1.1);
    require(r.state == DREC_RETRY, "two failures stay in retry");
    drec_on_job(&r, -EIO, 1.2);
    require(r.state == DREC_RESET && drec_next_action(&r, 1.2) == DREC_ACT_RESET,
            "retry_limit failures -> reset");
    drec_on_reset(&r, 0, 1.3);
    require(r.state == DREC_RESET_CHECK && drec_next_action(&r, 1.3) == DREC_ACT_RUN, "reset ok -> check job");
    drec_on_job(&r, 0, 1.4);
    require(r.state == DREC_RUNNING && r.recoveries == 1, "check job ok -> running");
}

/* Reset that does not help: reopen; reopen failures back off exponentially up to the cap. */
static void test_reopen_and_backoff(void)
{
    drec_t r = fresh();

    for (int i = 0; i < 3; i++)
        drec_on_job(&r, -EIO, 1.0);
    drec_on_reset(&r, -EIO, 1.1);
    require(r.state == DREC_REOPEN && drec_next_action(&r, 1.1) == DREC_ACT_REOPEN, "reset failed -> reopen");
    drec_on_reopen(&r, -ENODEV, 1.2);
    require(r.state == DREC_BACKOFF && r.backoff_sec == 0.5, "reopen failed -> backoff 0.5 s");
    require(drec_next_action(&r, 1.5) == DREC_ACT_WAIT, "waits before the deadline");
    require(drec_wait_remaining(&r, 1.5) > 0.19 && drec_wait_remaining(&r, 1.5) < 0.21, "remaining wait");
    require(drec_next_action(&r, 1.71) == DREC_ACT_REOPEN, "reopens after the deadline");
    drec_on_reopen(&r, -ENODEV, 1.7);
    require(r.backoff_sec == 1.0 && r.wait_until > 2.69 && r.wait_until < 2.71, "second failure doubles the wait");
    drec_on_reopen(&r, -ENODEV, 2.7);
    require(r.backoff_sec == 2.0, "third failure doubles again");
    require(drec_health(&r) == DREC_HEALTH_RECOVERING, "below the cap: recovering");
    drec_on_reopen(&r, -ENODEV, 4.7);
    require(r.backoff_sec == 4.0 && drec_health(&r) == DREC_HEALTH_FAILED, "cap reached: failed");
    drec_on_reopen(&r, -ENODEV, 8.7);
    require(r.backoff_sec == 4.0, "wait stays at the cap");
    require(r.last_error == -ENODEV, "last error is the reopen error");

    drec_on_reopen(&r, 0, 12.7);
    require(r.state == DREC_REOPEN_CHECK && drec_next_action(&r, 12.7) == DREC_ACT_RUN,
            "reopen ok -> check job");
    drec_on_job(&r, 0, 12.8);
    require(r.state == DREC_RUNNING && r.backoff_sec == 0.0, "check job ok -> running, backoff cleared");
    require(r.total_failures == 3 + 1 + 5, "every failed job, reset and reopen counted");
}

/* A device that reopens but keeps failing jobs must not cycle at frame rate. */
static void test_reopen_but_jobs_fail(void)
{
    drec_t r = fresh();

    for (int i = 0; i < 3; i++)
        drec_on_job(&r, -EIO, 1.0);
    drec_on_reset(&r, 0, 1.0);
    drec_on_job(&r, -EIO, 1.0);
    require(r.state == DREC_REOPEN, "reset check failed -> reopen");
    drec_on_reopen(&r, 0, 1.0);
    drec_on_job(&r, -EIO, 1.0);
    require(r.state == DREC_BACKOFF && r.backoff_sec == 0.5, "reopen check failed -> backoff");
    drec_on_reopen(&r, 0, 1.5);
    drec_on_job(&r, -EIO, 1.5);
    require(r.state == DREC_BACKOFF && r.backoff_sec == 1.0, "repeated check failures keep doubling");
}

/* Device missing at start-up: back off and keep trying. */
static void test_start_failed(void)
{
    drec_t r = fresh();

    drec_start_failed(&r, -ENOENT, 0.0);
    require(r.state == DREC_BACKOFF && drec_next_action(&r, 0.1) == DREC_ACT_WAIT, "start failure backs off");
    require(drec_next_action(&r, 0.5) == DREC_ACT_REOPEN, "then reopens");
    drec_on_reopen(&r, 0, 0.5);
    drec_on_job(&r, 0, 0.6);
    require(r.state == DREC_RUNNING && r.recoveries == 1, "late device recovers");
}

static void test_limits_and_names(void)
{
    drec_config_t cfg = {0, -1.0, 0.0};
    drec_t r;

    drec_init(&r, &cfg);
    require(r.cfg.retry_limit == 1 && r.cfg.backoff_initial_sec > 0.0 &&
                r.cfg.backoff_max_sec >= r.cfg.backoff_initial_sec,
            "invalid configuration is clamped");
    drec_on_job(&r, -EIO, 0.0);
    require(r.state == DREC_RESET, "retry_limit 1 resets on the first failure");
    drec_on_reopen(&r, -EIO, 0.0);
    require(r.state == DREC_RESET, "reopen report outside reopen is ignored");
    require(drec_state_name(DREC_REOPEN_CHECK)[0] == 'r' && drec_health_name(DREC_HEALTH_FAILED)[0] == 'f',
            "names");
}

int main(void)
{
    test_transient();
    test_reset_recovers();
    test_reopen_and_backoff();
    test_reopen_but_jobs_fail();
    test_start_failed();
    test_limits_and_names();
    printf("detector_recovery_test: OK (%d checks)\n", g_checks);
    return 0;
}
