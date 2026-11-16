/* camera_switch.h - what turning the camera on and off does.
 *
 * Off: the user (POST /api/camera) and the stream limit (capture.max_stream_min) end in the same
 * state: the camera and detection disabled, no tracking, no detection or tracking output, no
 * track command left queued, and the stream stopped with the sensor in standby.
 *
 * A tracker that still runs is stopped first, while the sensor still streams: its restoring
 * writes (scaler on, default offset) are then made on a running sensor.  The offset goes through
 * a group write launched at a frame boundary, and a sensor in standby has none.
 *
 * On: the sensor is initialized again (hardware reset and register table) instead of being woken
 * from software standby.  On the board, writing 0x3008=0x02 after the standby write 0x42 did not
 * bring the DVP output back; every stream that had worked on the board had started right after a
 * full initialization.  The table leaves the scaler on and the offset at
 * its default, and the tracker's copies are set to match; the colour bars, when configured, are
 * written again after the table, which clears them.  The stream starts last.
 *
 * Both run on the capture thread, which owns the tracker and the stream.  The sensor and stream
 * steps come from the caller, so the sequences can be tested without hardware.
 */
#ifndef CAM_CAMERA_SWITCH_H
#define CAM_CAMERA_SWITCH_H

#include "cam_system/tracker.h"

typedef struct {
    void (*last_reading)(void);     /* the stream's last health reading, before the stop */
    void (*stop_stream)(void);      /* STREAMOFF */
    void (*standby)(void);          /* sensor into software standby */
} camera_off_stream_ops_t;

typedef struct {
    int (*initialize)(void);        /* hardware reset and register table; the sensor runs after it */
    int (*test_pattern)(void);      /* colour bars on */
    int (*start_stream)(void);      /* STREAMON */
} camera_on_stream_ops_t;

void camera_off(tracker_t *tracker, const camera_off_stream_ops_t *ops);

/* Returns 0 when the stream runs, -1 when a step failed; the failing step logs why.  With
 * test_pattern set, the stream does not start without the colour bars. */
int camera_on(tracker_t *tracker, int test_pattern, const camera_on_stream_ops_t *ops);

#endif /* CAM_CAMERA_SWITCH_H */
