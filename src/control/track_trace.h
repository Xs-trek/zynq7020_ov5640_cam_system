/* track_trace.h - numeric trace of the control loop (tracker.trace), for offline analysis.
 *
 * One CSV line per control tick: the frame, how the tick ran, every detection the tick took in
 * (control point and size in the absolute 1312x972 plane), the observation the tracker adopted,
 * the Kalman estimate, the path reference, the crop path, the ISP offset written and the tracker
 * state.  Numbers only: no image data.  Detections are written while nothing is tracked too, to
 * show how the person moved.
 *
 * Off by default.  The file must be under /tmp (RAM on the board) and is created 0600.  Writes go
 * through a 64 KiB stdio buffer, so the control tick only formats a line in memory; the buffer is
 * flushed when full and when the trace is closed.  A failing write is logged once and stops the
 * trace; the application runs on.  The trace stops at TRACK_TRACE_MAX_BYTES.
 *
 * Called from the capture thread only.
 */
#ifndef CAM_TRACK_TRACE_H
#define CAM_TRACK_TRACE_H

#include <stddef.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

#define TRACK_TRACE_MAX_DETS 20
#define TRACK_TRACE_MAX_BYTES (16L * 1024L * 1024L)  /* about 30 min at 30 fps */
#define TRACK_TRACE_LINE_MAX 2048

typedef enum {
    TRACK_TRACE_TICK_DETECTION = 0,   /* this frame's detection arrived */
    TRACK_TRACE_TICK_DEADLINE,        /* the deadline came first */
    TRACK_TRACE_TICK_IMMEDIATE,       /* nothing to wait for */
} track_trace_tick_t;

typedef struct {
    int id;             /* detector track id, -1 if none */
    float x, y;         /* control point (face anchor), absolute plane px */
    float w, h;         /* box size, absolute plane px */
    float score;
    int publish;        /* 1: published to the page, 0: tracker-only candidate */
    int target;         /* the tracker adopted it as its measurement */
} track_trace_det_t;

typedef struct {
    unsigned int frame_seq;     /* mailbox frame sequence */
    uint32_t v4l2_seq;          /* V4L2 buffer sequence */
    double capture_time;        /* V4L2 capture time, monotonic s */
    track_trace_tick_t tick;
    const char *state;          /* tracker state name */
    int active;                 /* tracking: the estimate and path fields below are valid */
    float kf_x, kf_y, kf_vx, kf_vy, kf_log_h;
    float ref_x, ref_y, ref_vx, ref_vy;
    float path_x, path_y, path_vx, path_vy, path_ax, path_ay;
    int adopted;                /* an observation was adopted this tick */
    float obs_x, obs_y, obs_h;
    double obs_age_ms;          /* tick time minus the observation's capture time */
    int isp_x, isp_y;           /* the ISP offset after this tick */
    int prev_slip;              /* the previous tick's write finished after this frame began */
    int n_det;
    track_trace_det_t det[TRACK_TRACE_MAX_DETS];
} track_trace_row_t;

/* The CSV header line, without the newline. */
const char *track_trace_header(void);

/* Formats row as one CSV line with a newline.  Returns its length, or -1 if it does not fit. */
int track_trace_format(char *buf, size_t len, const track_trace_row_t *row);

/* Opens the trace at path; an empty path leaves it off.  Returns 0 when off or open, -1 when the
 * path was refused or could not be created (logged; the trace stays off). */
int track_trace_open(const char *path);

/* Whether the trace is on.  The capture thread fills no row while it is off. */
int track_trace_on(void);

/* Appends one row.  Does nothing while the trace is off. */
void track_trace_write(const track_trace_row_t *row);

/* Rows written since open. */
unsigned long track_trace_rows(void);

/* Flushes and closes; logs the row count.  Safe to call when off. */
void track_trace_close(void);

#ifdef __cplusplus
}
#endif

#endif /* CAM_TRACK_TRACE_H */
