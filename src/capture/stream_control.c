#include "capture/stream_control.h"

#include <string.h>

void stream_control_init(stream_control_t *ctl, double limit_sec)
{
    memset(ctl, 0, sizeof(*ctl));
    ctl->limit_sec = limit_sec > 0.0 ? limit_sec : 0.0;
}

stream_action_t stream_control_poll(const stream_control_t *ctl, int camera_enabled, double now)
{
    if (!ctl->streaming)
        return camera_enabled ? STREAM_START : STREAM_KEEP;
    if (!camera_enabled)
        return STREAM_STOP;
    if (ctl->limit_sec > 0.0 && now - ctl->started_at >= ctl->limit_sec)
        return STREAM_STOP_LIMIT;
    return STREAM_KEEP;
}

void stream_control_started(stream_control_t *ctl, double now)
{
    ctl->streaming = 1;
    ctl->started_at = now;
    ctl->stopped_by_limit = 0;
}

void stream_control_stopped(stream_control_t *ctl, stream_action_t why)
{
    ctl->streaming = 0;
    ctl->stopped_by_limit = why == STREAM_STOP_LIMIT;
}

double stream_control_run_sec(const stream_control_t *ctl, double now)
{
    if (!ctl->streaming || now < ctl->started_at)
        return 0.0;
    return now - ctl->started_at;
}
