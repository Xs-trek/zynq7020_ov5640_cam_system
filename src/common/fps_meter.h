/* fps_meter.h - event rate over one-second windows, for the status interface.
 *
 * One thread calls fps_meter_tick; any thread may call fps_meter_read.  The rate drops to 0 when
 * the events stop.  Times are CLOCK_MONOTONIC seconds, passed in by the caller.
 */
#ifndef CAM_FPS_METER_H
#define CAM_FPS_METER_H

#ifdef __cplusplus
extern "C" {
#endif

typedef struct {
    double window_start;
    unsigned int count;
    int milli_fps;      /* atomic */
    int updated_ms;     /* atomic; wraps, compared by difference */
} fps_meter_t;

void fps_meter_tick(fps_meter_t *meter, double now);
float fps_meter_read(const fps_meter_t *meter, double now);

#ifdef __cplusplus
}
#endif

#endif /* CAM_FPS_METER_H */
