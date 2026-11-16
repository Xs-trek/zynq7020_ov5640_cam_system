/* dvp_counts.h - the PL bridge's frame counters, counted for this run.
 *
 * The bridge counts frames and wrong-sized frames from power-on (FRAME_COUNT 0x14,
 * SYNC_ERROR_COUNT 0x24), and every start and stop of the stream adds a few dozen wrong-sized
 * frames.  The status interface reports this run: it adds up the streams of this process, each
 * from its first reading, taken once the stream has delivered frames, to its last before it
 * stops, so the transitions at start and stop are left out.
 *
 * Pure logic: the caller reads the counters and passes them in.
 */
#ifndef CAM_DVP_COUNTS_H
#define CAM_DVP_COUNTS_H

#include <stdint.h>

typedef struct {
    int have_base;
    int32_t last_frames;        /* raw values of the previous reading */
    int32_t last_bad;
    long run_frames;            /* over the streams of this run */
    long run_bad;
} dvp_counts_t;

void dvp_counts_init(dvp_counts_t *counts);

/* Takes one raw reading (0..INT32_MAX).  Returns 1 if it became the base, 0 otherwise.  A counter
 * that went backwards was reset with the PL; it counts on from 0. */
int dvp_counts_update(dvp_counts_t *counts, int32_t frames, int32_t bad_frames);

/* A new stream starts: its first reading is a new base.  The totals so far stay. */
void dvp_counts_restart(dvp_counts_t *counts);

#endif /* CAM_DVP_COUNTS_H */
