#include "capture/capture_recovery.h"

void caprec_init(caprec_t *rec, double now)
{
    rec->last_progress = now;
    rec->restarts = 0;
}

void caprec_touch(caprec_t *rec, double now)
{
    rec->last_progress = now;
}

void caprec_on_frame(caprec_t *rec, double now)
{
    rec->last_progress = now;
    rec->restarts = 0;
}

caprec_action_t caprec_on_no_frame(caprec_t *rec, double now)
{
    if (now - rec->last_progress < CAPREC_STALL_SEC)
        return CAPREC_WAIT;
    if (rec->restarts >= CAPREC_RESTART_LIMIT)
        return CAPREC_GIVE_UP;
    rec->restarts++;
    rec->last_progress = now;
    return CAPREC_RESTART;
}
