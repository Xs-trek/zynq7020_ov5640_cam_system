/* stream_rate_test.c - which frames the page stream encodes under stream.max_fps. */
#include "media/stream_rate.h"

#include <stdio.h>
#include <stdlib.h>

#define FRAME_SEC (1.0 / 30.0)

static int g_checks;

static void require(int cond, const char *what)
{
    g_checks++;
    if (!cond) {
        fprintf(stderr, "stream_rate_test: FAIL: %s\n", what);
        exit(1);
    }
}

/* Frames of a 30 fps camera for `seconds`, with an alternating timestamp jitter of +-jitter;
 * returns how many were encoded. */
static int run(stream_rate_t *rate, double start, double seconds, double jitter)
{
    const int frames = (int)(seconds * 30.0 + 0.5);
    int taken = 0;

    for (int i = 0; i < frames; i++)
        taken += stream_rate_take(rate, start + i * FRAME_SEC + ((i & 1) ? jitter : -jitter));
    return taken;
}

static void test_no_limit(void)
{
    stream_rate_t rate;

    stream_rate_init(&rate, 0);
    require(run(&rate, 10.0, 10.0, 0.002) == 300, "no limit: every frame");
    require(stream_rate_take(&rate, 20.0) && stream_rate_take(&rate, 20.0),
            "no limit: even frames with the same time");
}

/* The camera runs at 30 fps, so the encoded rate is 30/n. */
static void test_limits(void)
{
    static const struct {
        int max_fps;
        int expected;   /* encoded frames out of 300 (10 s) */
    } cases[] = {
        {30, 300}, {20, 150}, {16, 150}, {15, 150}, {14, 100}, {10, 100},
        {8, 75}, {5, 50}, {1, 10},
    };

    for (size_t c = 0; c < sizeof(cases) / sizeof(cases[0]); c++) {
        for (int j = 0; j < 2; j++) {
            stream_rate_t rate;
            char what[96];
            int taken;

            stream_rate_init(&rate, cases[c].max_fps);
            taken = run(&rate, 100.0, 10.0, j ? 0.001 : 0.0);
            snprintf(what, sizeof(what), "max_fps %d%s: %d frames of 300, expected %d",
                     cases[c].max_fps, j ? " with 1 ms jitter" : "", taken, cases[c].expected);
            require(taken == cases[c].expected, what);
        }
    }
}

/* Never two encoded frames closer than the interval less the slack. */
static void test_spacing(void)
{
    stream_rate_t rate;
    double last = -1.0;
    double min_gap = 1.0;

    stream_rate_init(&rate, 15);
    for (int i = 0; i < 300; i++) {
        double t = 50.0 + i * FRAME_SEC + ((i % 3) - 1) * 0.003;

        if (stream_rate_take(&rate, t)) {
            if (last >= 0.0 && t - last < min_gap)
                min_gap = t - last;
            last = t;
        }
    }
    require(min_gap >= 1.0 / 15.0 - STREAM_RATE_SLACK_SEC, "encoded frames keep the interval");
}

/* The first frame is encoded at once; after a reset (a new client) again; a time going back (a
 * new stream) is encoded. */
static void test_first_reset_and_clock(void)
{
    stream_rate_t rate;

    stream_rate_init(&rate, 10);
    require(stream_rate_take(&rate, 5.0), "first frame encoded");
    require(!stream_rate_take(&rate, 5.0 + FRAME_SEC), "next frame within the interval skipped");
    stream_rate_reset(&rate);
    require(stream_rate_take(&rate, 5.0 + 2 * FRAME_SEC), "first frame after a reset encoded");
    require(!stream_rate_take(&rate, 5.0 + 3 * FRAME_SEC), "and the limit applies after it");
    require(stream_rate_take(&rate, 1.0), "a time before the last one is encoded");
    require(!stream_rate_take(&rate, 1.0 + FRAME_SEC), "and becomes the new reference");
    require(stream_rate_take(&rate, 1.0 + 3 * FRAME_SEC), "one interval later: encoded");
}

int main(void)
{
    test_no_limit();
    test_limits();
    test_spacing();
    test_first_reset_and_clock();
    printf("stream_rate_test: OK (%d checks)\n", g_checks);
    return 0;
}
