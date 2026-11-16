/* detector_recovery.c - see detector_recovery.h */
#include "detect/detector_recovery.h"

#include <string.h>

void drec_init(drec_t *r, const drec_config_t *cfg)
{
    memset(r, 0, sizeof(*r));
    r->cfg = *cfg;
    if (r->cfg.retry_limit < 1)
        r->cfg.retry_limit = 1;
    if (r->cfg.backoff_initial_sec <= 0.0)
        r->cfg.backoff_initial_sec = 0.5;
    if (r->cfg.backoff_max_sec < r->cfg.backoff_initial_sec)
        r->cfg.backoff_max_sec = r->cfg.backoff_initial_sec;
    r->state = DREC_RUNNING;
}

static void record_failure(drec_t *r, int error)
{
    r->total_failures++;
    r->last_error = error < 0 ? error : -1;
}

static void enter_backoff(drec_t *r, double now)
{
    if (r->backoff_sec <= 0.0)
        r->backoff_sec = r->cfg.backoff_initial_sec;
    else if (r->backoff_sec * 2.0 < r->cfg.backoff_max_sec)
        r->backoff_sec *= 2.0;
    else
        r->backoff_sec = r->cfg.backoff_max_sec;
    r->wait_until = now + r->backoff_sec;
    r->state = DREC_BACKOFF;
}

static void enter_running(drec_t *r)
{
    if (r->state != DREC_RUNNING)
        r->recoveries++;
    r->state = DREC_RUNNING;
    r->consecutive_failures = 0;
    r->backoff_sec = 0.0;
}

void drec_start_failed(drec_t *r, int error, double now)
{
    record_failure(r, error);
    enter_backoff(r, now);
}

drec_action_t drec_next_action(const drec_t *r, double now)
{
    switch (r->state) {
    case DREC_RESET:
        return DREC_ACT_RESET;
    case DREC_REOPEN:
        return DREC_ACT_REOPEN;
    case DREC_BACKOFF:
        return now < r->wait_until ? DREC_ACT_WAIT : DREC_ACT_REOPEN;
    default:
        return DREC_ACT_RUN;
    }
}

double drec_wait_remaining(const drec_t *r, double now)
{
    if (r->state != DREC_BACKOFF || now >= r->wait_until)
        return 0.0;
    return r->wait_until - now;
}

void drec_on_job(drec_t *r, int result, double now)
{
    if (result >= 0) {
        enter_running(r);
        return;
    }
    record_failure(r, result);
    switch (r->state) {
    case DREC_RUNNING:
        r->consecutive_failures = 1;
        r->state = r->cfg.retry_limit > 1 ? DREC_RETRY : DREC_RESET;
        break;
    case DREC_RETRY:
        r->consecutive_failures++;
        if (r->consecutive_failures >= r->cfg.retry_limit)
            r->state = DREC_RESET;
        break;
    case DREC_RESET_CHECK:
        r->state = DREC_REOPEN;
        break;
    case DREC_REOPEN_CHECK:
        enter_backoff(r, now);
        break;
    default:
        /* No job runs in RESET, REOPEN or BACKOFF; treat a stray report as a reopen failure. */
        enter_backoff(r, now);
        break;
    }
}

void drec_on_reset(drec_t *r, int result, double now)
{
    (void)now;
    if (r->state != DREC_RESET)
        return;
    if (result >= 0) {
        r->state = DREC_RESET_CHECK;
        return;
    }
    record_failure(r, result);
    r->state = DREC_REOPEN;
}

void drec_on_reopen(drec_t *r, int result, double now)
{
    if (r->state != DREC_REOPEN && r->state != DREC_BACKOFF)
        return;
    if (result >= 0) {
        r->state = DREC_REOPEN_CHECK;
        return;
    }
    record_failure(r, result);
    enter_backoff(r, now);
}

drec_health_t drec_health(const drec_t *r)
{
    if (r->state == DREC_RUNNING)
        return DREC_HEALTH_RUNNING;
    if (r->state == DREC_BACKOFF && r->backoff_sec >= r->cfg.backoff_max_sec)
        return DREC_HEALTH_FAILED;
    return DREC_HEALTH_RECOVERING;
}

const char *drec_state_name(drec_state_t state)
{
    static const char *const names[] = {
        "running", "retry", "reset", "reset_check", "reopen", "reopen_check", "backoff"
    };
    return (unsigned)state < sizeof(names) / sizeof(names[0]) ? names[state] : "unknown";
}

const char *drec_health_name(drec_health_t health)
{
    static const char *const names[] = {"running", "recovering", "failed"};
    return (unsigned)health < sizeof(names) / sizeof(names[0]) ? names[health] : "unknown";
}
