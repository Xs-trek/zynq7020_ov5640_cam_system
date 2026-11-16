/* stream_rate.c - see stream_rate.h */
#include "media/stream_rate.h"

void stream_rate_init(stream_rate_t *rate, int max_fps)
{
    rate->interval_sec = max_fps > 0 ? 1.0 / (double)max_fps : 0.0;
    stream_rate_reset(rate);
}

void stream_rate_reset(stream_rate_t *rate)
{
    rate->last_sec = 0.0;
    rate->have_last = 0;
}

int stream_rate_take(stream_rate_t *rate, double time_sec)
{
    if (rate->interval_sec > 0.0 && rate->have_last && time_sec >= rate->last_sec &&
        time_sec - rate->last_sec < rate->interval_sec - STREAM_RATE_SLACK_SEC)
        return 0;
    rate->last_sec = time_sec;
    rate->have_last = 1;
    return 1;
}
