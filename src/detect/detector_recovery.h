/* detector_recovery.h - recovery policy for a hardware detector backend (pure logic, no I/O).
 *
 * The detector thread asks drec_next_action() what to do before each job and reports every
 * outcome back.  States and transitions:
 *
 *   RUNNING       job ok -> RUNNING;  job failed -> RETRY
 *   RETRY         job ok -> RUNNING;  retry_limit consecutive failures -> RESET
 *   RESET         reset ok -> RESET_CHECK;  reset failed -> REOPEN
 *   RESET_CHECK   job ok -> RUNNING;  job failed -> REOPEN
 *   REOPEN        reopen ok -> REOPEN_CHECK;  reopen failed -> BACKOFF
 *   REOPEN_CHECK  job ok -> RUNNING;  job failed -> BACKOFF
 *   BACKOFF       wait (no frames taken), then REOPEN; the wait doubles up to backoff_max_sec
 *
 * The *_CHECK states make a device that opens or resets but still fails its jobs fall back to
 * BACKOFF instead of cycling through RETRY/RESET/REOPEN at frame rate.  The backoff interval
 * returns to its initial value only after a successful job.
 */
#ifndef CAM_DETECTOR_RECOVERY_H
#define CAM_DETECTOR_RECOVERY_H

#ifdef __cplusplus
extern "C" {
#endif

typedef enum {
    DREC_RUNNING = 0,
    DREC_RETRY,
    DREC_RESET,
    DREC_RESET_CHECK,
    DREC_REOPEN,
    DREC_REOPEN_CHECK,
    DREC_BACKOFF
} drec_state_t;

typedef enum {
    DREC_ACT_RUN = 0,   /* take a frame and run a job */
    DREC_ACT_RESET,     /* reset the device, then report drec_on_reset() */
    DREC_ACT_REOPEN,    /* close and reopen the device, then report drec_on_reopen() */
    DREC_ACT_WAIT       /* back off; drec_wait_remaining() says how long */
} drec_action_t;

typedef enum {
    DREC_HEALTH_RUNNING = 0,
    DREC_HEALTH_RECOVERING,
    DREC_HEALTH_FAILED      /* backing off at the maximum interval */
} drec_health_t;

typedef struct {
    int retry_limit;            /* consecutive job failures before a reset, >= 1 */
    double backoff_initial_sec; /* > 0 */
    double backoff_max_sec;     /* >= backoff_initial_sec */
} drec_config_t;

typedef struct {
    drec_config_t cfg;
    drec_state_t state;
    int consecutive_failures;
    double backoff_sec;         /* current interval, 0 when not backing off */
    double wait_until;
    unsigned long total_failures;   /* failed jobs, resets and reopens */
    unsigned long recoveries;       /* returns to RUNNING from any other state */
    int last_error;                 /* last negative errno reported, 0 if none */
} drec_t;

void drec_init(drec_t *r, const drec_config_t *cfg);

/* The device could not be opened at start-up: start in BACKOFF. */
void drec_start_failed(drec_t *r, int error, double now);

drec_action_t drec_next_action(const drec_t *r, double now);
double drec_wait_remaining(const drec_t *r, double now);

/* result: 0 on success, negative errno on failure. */
void drec_on_job(drec_t *r, int result, double now);
void drec_on_reset(drec_t *r, int result, double now);
void drec_on_reopen(drec_t *r, int result, double now);

drec_health_t drec_health(const drec_t *r);
const char *drec_state_name(drec_state_t state);
const char *drec_health_name(drec_health_t health);

#ifdef __cplusplus
}
#endif

#endif /* CAM_DETECTOR_RECOVERY_H */
