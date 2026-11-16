/* control_tick.c - see control_tick.h */
#include "capture/control_tick.h"

#include <string.h>

void control_tick_init(control_tick_t *tick, double deadline_ms)
{
    memset(tick, 0, sizeof(*tick));
    tick->deadline_sec = deadline_ms > 0.0 ? deadline_ms / 1000.0 : 0.0;
}

void control_tick_reset(control_tick_t *tick)
{
    control_tick_init(tick, tick->deadline_sec * 1000.0);
}

void control_tick_begin(control_tick_t *tick, uint32_t sequence, unsigned int frame_seq, double frame_time,
                        int expect_detection)
{
    tick->sequence = sequence;
    tick->frame_seq = frame_seq;
    tick->frame_time = frame_time;
    tick->expect_detection = expect_detection;
}

control_tick_action_t control_tick_poll(const control_tick_t *tick, unsigned int latest_det_frame_seq, double now)
{
    if (!tick->expect_detection || tick->deadline_sec <= 0.0)
        return CONTROL_TICK_IMMEDIATE;
    /* Sequence numbers wrap; a detection of an earlier frame does not end the wait. */
    if ((int32_t)(latest_det_frame_seq - tick->frame_seq) >= 0)
        return CONTROL_TICK_ON_DETECTION;
    if (now >= control_tick_deadline(tick))
        return CONTROL_TICK_ON_DEADLINE;
    return CONTROL_TICK_WAIT;
}

double control_tick_deadline(const control_tick_t *tick)
{
    return tick->frame_time + tick->deadline_sec;
}

void control_tick_end(control_tick_t *tick, control_tick_action_t action, int wrote_offset, double write_done)
{
    switch (action) {
    case CONTROL_TICK_ON_DETECTION:
        tick->stats.on_detection++;
        tick->watch_waits++;
        break;
    case CONTROL_TICK_ON_DEADLINE:
        tick->stats.on_deadline++;
        tick->watch_waits++;
        tick->watch_late++;
        break;
    default:
        tick->stats.immediate++;
        break;
    }
    tick->pending = 1;
    tick->pending_wrote = wrote_offset;
    tick->pending_sequence = tick->sequence;
    tick->pending_write_done = write_done;
}

int control_tick_settle(control_tick_t *tick, double next_frame_time, uint32_t *issued_sequence)
{
    if (!tick->pending)
        return 0;
    tick->pending = 0;
    *issued_sequence = tick->pending_sequence;
    if (tick->pending_wrote) {
        const double margin_ms = (next_frame_time - tick->pending_write_done) * 1000.0;

        if (tick->stats.writes == 0 || margin_ms < tick->stats.margin_min_ms)
            tick->stats.margin_min_ms = margin_ms;
        tick->stats.margin_sum_ms += margin_ms;
        tick->stats.writes++;
        if (margin_ms < 0.0) {
            tick->stats.isp_slips++;
            tick->isp_slips_total++;
            *issued_sequence += 1u;
        }
    }
    return 1;
}

void control_tick_stats_reset(control_tick_t *tick)
{
    memset(&tick->stats, 0, sizeof(tick->stats));
}

int control_tick_watch(control_tick_t *tick, unsigned int *late)
{
    int mostly_late;

    if (tick->watch_waits < CONTROL_TICK_WATCH_WAITS)
        return 0;
    mostly_late = 2u * tick->watch_late > tick->watch_waits;
    *late = tick->watch_late;
    tick->watch_waits = 0;
    tick->watch_late = 0;
    return mostly_late;
}
