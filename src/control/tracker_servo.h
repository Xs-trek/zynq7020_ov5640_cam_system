#ifndef TRACKER_SERVO_H
#define TRACKER_SERVO_H

#include "cam_system/tracker.h"

#ifdef __cplusplus
extern "C" {
#endif

/* Schmitt noise gate used by the screen composer.  direction is -1, 0 or +1
 * and is retained by the axis so a noisy sample cannot chatter at the entry
 * boundary. */
float tracker_servo_band_error(float error, float enter_radius,
                               float exit_radius, int *direction);

void tracker_servo_start(tracker_view_controller_t *controller,
                         float x, float y);
void tracker_servo_step(tracker_view_controller_t *controller,
                        float target_x, float target_y,
                        float noise_enter_x, float noise_enter_y,
                        float dt);
void tracker_servo_brake(tracker_view_controller_t *controller, float dt);

#ifdef __cplusplus
}
#endif

#endif
