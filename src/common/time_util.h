/* time_util.h - CLOCK_MONOTONIC in the units the code uses. */
#ifndef TIME_UTIL_H
#define TIME_UTIL_H

#include <time.h>

static inline double monotonic_sec(void)
{
    struct timespec ts;

    clock_gettime(CLOCK_MONOTONIC, &ts);
    return (double)ts.tv_sec + (double)ts.tv_nsec / 1000000000.0;
}

static inline double monotonic_ms(void)
{
    return monotonic_sec() * 1000.0;
}

#endif /* TIME_UTIL_H */
