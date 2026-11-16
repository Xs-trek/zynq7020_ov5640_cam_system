#include "capture/dvp_counts.h"

#include <string.h>

void dvp_counts_init(dvp_counts_t *counts)
{
    memset(counts, 0, sizeof(*counts));
}

static long dvp_delta(int32_t now, int32_t last)
{
    return now >= last ? (long)now - (long)last : (long)now;
}

int dvp_counts_update(dvp_counts_t *counts, int32_t frames, int32_t bad_frames)
{
    if (frames < 0 || bad_frames < 0)
        return 0;
    if (!counts->have_base) {
        counts->have_base = 1;
        counts->last_frames = frames;
        counts->last_bad = bad_frames;
        return 1;
    }
    counts->run_frames += dvp_delta(frames, counts->last_frames);
    counts->run_bad += dvp_delta(bad_frames, counts->last_bad);
    counts->last_frames = frames;
    counts->last_bad = bad_frames;
    return 0;
}

void dvp_counts_restart(dvp_counts_t *counts)
{
    counts->have_base = 0;
}
