/* launcher_policy.h - when cam-launcher restarts cam_system.
 *
 * Exit status 0 is an orderly stop.  EX_USAGE and EX_CONFIG mean the same start would fail the same
 * way.  Anything else is a fault: restart, with a delay that doubles while the process keeps dying
 * and starts over once it has run for a while.
 */
#ifndef CAM_LAUNCHER_POLICY_H
#define CAM_LAUNCHER_POLICY_H

#include <sysexits.h>

#ifndef LAUNCHER_BACKOFF_INITIAL_SEC
#define LAUNCHER_BACKOFF_INITIAL_SEC 1.0
#endif
#ifndef LAUNCHER_BACKOFF_MAX_SEC
#define LAUNCHER_BACKOFF_MAX_SEC 30.0
#endif
#ifndef LAUNCHER_STABLE_RUN_SEC
#define LAUNCHER_STABLE_RUN_SEC 60.0
#endif
/* An orderly stop takes the capture thread's frame wait (2 s) plus teardown. */
#ifndef LAUNCHER_STOP_TIMEOUT_SEC
#define LAUNCHER_STOP_TIMEOUT_SEC 5.0
#endif

typedef struct {
    double backoff_sec;
} launcher_policy_t;

static inline void launcher_policy_init(launcher_policy_t *policy)
{
    policy->backoff_sec = LAUNCHER_BACKOFF_INITIAL_SEC;
}

/*
 * exited: the process called exit, with exit_code; otherwise a signal ended it.
 * Returns the delay before restarting, or a negative value for no restart.
 */
static inline double launcher_restart_delay(launcher_policy_t *policy, int exited, int exit_code,
                                            double ran_sec)
{
    double delay;

    if (exited && (exit_code == 0 || exit_code == EX_USAGE || exit_code == EX_CONFIG)) {
        launcher_policy_init(policy);
        return -1.0;
    }
    if (ran_sec >= LAUNCHER_STABLE_RUN_SEC)
        launcher_policy_init(policy);
    delay = policy->backoff_sec;
    policy->backoff_sec = delay * 2.0 < LAUNCHER_BACKOFF_MAX_SEC ? delay * 2.0
                                                                 : LAUNCHER_BACKOFF_MAX_SEC;
    return delay;
}

#endif /* CAM_LAUNCHER_POLICY_H */
