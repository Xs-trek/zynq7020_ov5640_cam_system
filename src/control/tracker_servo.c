#include "control/tracker_servo.h"

#include "cam_system/config.h"

#include <math.h>
#include <string.h>

#define SERVO_X_MIN 0.0f
#define SERVO_X_MAX ((float)(SUBSAMPLED_1X_W - IMG_W))
#define SERVO_Y_MIN 0.0f
#define SERVO_Y_MAX ((float)(SUBSAMPLED_1X_H - IMG_H))

static float clampf_servo(float value, float minimum, float maximum)
{
    return value < minimum ? minimum :
           (value > maximum ? maximum : value);
}

static float stop_distance_from_rest_accel(float speed,
                                           float max_acceleration,
                                           float max_jerk)
{
    float triangular_limit = max_acceleration * max_acceleration / max_jerk;

    speed = fmaxf(speed, 0.0f);
    if (speed <= triangular_limit)
        return speed * sqrtf(speed / max_jerk);
    return 0.5f * speed *
        (max_acceleration / max_jerk + speed / max_acceleration);
}

static float conservative_stop_distance(float speed, float acceleration,
                                        float max_acceleration,
                                        float max_jerk)
{
    float distance = 0.0f;

    speed = fmaxf(speed, 0.0f);
    if (acceleration > 0.0f) {
        float time_to_zero_accel = acceleration / max_jerk;
        distance += speed * time_to_zero_accel +
                    acceleration * acceleration * acceleration /
                        (3.0f * max_jerk * max_jerk);
        speed += acceleration * acceleration / (2.0f * max_jerk);
    }
    return distance + stop_distance_from_rest_accel(
        speed, max_acceleration, max_jerk);
}

float tracker_servo_band_error(float error, float enter_radius,
                               float exit_radius, int *direction)
{
    int next_direction = direction ? *direction : 0;

    enter_radius = fmaxf(enter_radius, 0.0f);
    exit_radius = clampf_servo(exit_radius, 0.0f, enter_radius);
    if (next_direction == 0) {
        if (error > enter_radius)
            next_direction = 1;
        else if (error < -enter_radius)
            next_direction = -1;
    } else if (next_direction > 0) {
        if (error < -enter_radius)
            next_direction = -1;
        else if (error <= exit_radius)
            next_direction = 0;
    } else {
        if (error > enter_radius)
            next_direction = 1;
        else if (error >= -exit_radius)
            next_direction = 0;
    }
    if (direction)
        *direction = next_direction;
    if (next_direction == 0)
        return 0.0f;
    return error - next_direction * exit_radius;
}

void tracker_servo_start(tracker_view_controller_t *controller,
                         float x, float y)
{
    memset(controller, 0, sizeof(*controller));
    controller->x.pos = clampf_servo(x, SERVO_X_MIN, SERVO_X_MAX);
    controller->y.pos = clampf_servo(y, SERVO_Y_MIN, SERVO_Y_MAX);
    controller->reference_x.pos = controller->x.pos;
    controller->reference_y.pos = controller->y.pos;
    controller->initialized = 1;
}

static void reference_step(tracker_reference_axis_t *reference,
                           float target_position, float dt)
{
    float predicted_position = reference->pos + reference->vel * dt;
    float residual = target_position - predicted_position;

    /* Fixed-gain fading local-linear fit.  For the three-sample window,
     * g=2(2N-1)/(N(N+1))=5/6 and h=6/(N(N+1))=1/2.  Position and
     * velocity therefore come from the same causal trajectory, unlike the
     * rejected path that filtered KF velocity independently. */
    reference->pos = predicted_position + TRACK_REFERENCE_ALPHA * residual;
    reference->vel += TRACK_REFERENCE_BETA * residual / dt;
}

static void step_axis(tracker_view_axis_t *axis,
                      float target_position, float target_velocity, float dt,
                      float minimum, float maximum,
                      float noise_enter, float omega,
                      float max_velocity, float max_acceleration,
                      float max_jerk)
{
    float position_error;
    float controlled_error;
    float requested_jerk;
    float exit_radius;
    float unclamped_target_position = target_position;

    target_position = clampf_servo(target_position, minimum, maximum);
    /* Keep the stable reference outside the actuator range.  Its derivative
     * then reveals that a subject at the pan limit has turned back before the
     * ideal crop itself re-enters the range.  The final command remains
     * clamped, while the position term opposes unsafe leading when the target
     * is still far beyond the boundary. */
    position_error = unclamped_target_position - axis->pos;
    exit_radius = noise_enter * TRACK_VIEW_NOISE_EXIT_RATIO;
    controlled_error = tracker_servo_band_error(
        position_error, noise_enter, exit_radius,
        &axis->response_direction);

    /* A triple real pole follows the stable reference and constrains the
     * actual viewport path.  Reference velocity is enabled only while the
     * composition error is outside the Schmitt band; stationary bbox noise
     * therefore cannot move the crop through a velocity-only path. */
    target_velocity = axis->response_direction == 0 ? 0.0f :
        clampf_servo(target_velocity, -max_velocity, max_velocity);
    if ((target_position <= minimum && target_velocity < 0.0f) ||
        (target_position >= maximum && target_velocity > 0.0f))
        target_velocity = 0.0f;
    requested_jerk = omega * omega * omega * controlled_error +
                     3.0f * omega * omega *
                         (target_velocity - axis->vel) -
                     3.0f * omega * axis->accel;
    /* Boundary brake (docs/V020_RELEASE_NOTES.md).  Look one frame ahead with the jerk the
     * law asks for: when the S-curve stopping distance from that state, plus
     * one frame, no longer fits in the remaining range, brake towards the
     * constant deceleration that ends at the pan limit, within the normal
     * acceleration and jerk limits.  The path then arrives at the limit at
     * rest instead of being stopped there by the projection below. */
    {
        float j_try = clampf_servo(requested_jerk, -max_jerk, max_jerk);
        float a_next = clampf_servo(axis->accel + j_try * dt,
                                    -max_acceleration, max_acceleration);
        float v_next = clampf_servo(axis->vel + a_next * dt,
                                    -max_velocity, max_velocity);
        float p_next = axis->pos + v_next * dt;

        if (v_next > 0.0f &&
            conservative_stop_distance(v_next, a_next, max_acceleration,
                                       max_jerk) + v_next * dt >=
                maximum - p_next) {
            float room = fmaxf(maximum - axis->pos - axis->vel * dt, 0.5f);
            float a_stop = fmaxf(-axis->vel * axis->vel / (2.0f * room),
                                 -max_acceleration);

            requested_jerk = fminf(requested_jerk,
                                   (a_stop - axis->accel) / dt);
        } else if (v_next < 0.0f &&
                   conservative_stop_distance(-v_next, -a_next,
                                              max_acceleration, max_jerk) -
                       v_next * dt >= p_next - minimum) {
            float room = fmaxf(axis->pos - minimum + axis->vel * dt, 0.5f);
            float a_stop = fminf(axis->vel * axis->vel / (2.0f * room),
                                 max_acceleration);

            requested_jerk = fmaxf(requested_jerk,
                                   (a_stop - axis->accel) / dt);
        }
    }
    requested_jerk = clampf_servo(requested_jerk, -max_jerk, max_jerk);
    axis->accel = clampf_servo(axis->accel + requested_jerk * dt,
                               -max_acceleration, max_acceleration);
    axis->vel = clampf_servo(axis->vel + axis->accel * dt,
                             -max_velocity, max_velocity);
    axis->pos += axis->vel * dt;

    /* Project the final path, not an intermediate detector state.  This is
     * also anti-windup at the rectangular hardware pan limits. */
    if (axis->pos <= minimum) {
        axis->pos = minimum;
        if (axis->vel < 0.0f) {
            axis->vel = 0.0f;
            axis->accel = 0.0f;
        }
    } else if (axis->pos >= maximum) {
        axis->pos = maximum;
        if (axis->vel > 0.0f) {
            axis->vel = 0.0f;
            axis->accel = 0.0f;
        }
    }

    /* Quantized output cannot benefit from sub-pixel residual momentum. */
    if (axis->response_direction == 0 &&
        fabsf(axis->vel) < 0.25f &&
        fabsf(axis->accel) <= max_jerk * dt) {
        axis->vel = 0.0f;
        axis->accel = 0.0f;
    }
}

static void brake_axis(tracker_view_axis_t *axis, float dt,
                       float minimum, float maximum,
                       float omega, float max_velocity,
                       float max_acceleration,
                       float max_jerk)
{
    float previous_velocity = axis->vel;
    float requested_jerk = -3.0f * omega * omega * axis->vel -
                           3.0f * omega * axis->accel;

    requested_jerk = clampf_servo(requested_jerk, -max_jerk, max_jerk);
    axis->accel = clampf_servo(axis->accel + requested_jerk * dt,
                               -max_acceleration, max_acceleration);
    axis->vel = clampf_servo(axis->vel + axis->accel * dt,
                             -max_velocity, max_velocity);
    if (previous_velocity != 0.0f &&
        previous_velocity * axis->vel <= 0.0f) {
        axis->vel = 0.0f;
        axis->accel = 0.0f;
    }
    axis->pos += axis->vel * dt;
    if (axis->pos <= minimum) {
        axis->pos = minimum;
        if (axis->vel < 0.0f) {
            axis->vel = 0.0f;
            axis->accel = 0.0f;
        }
    } else if (axis->pos >= maximum) {
        axis->pos = maximum;
        if (axis->vel > 0.0f) {
            axis->vel = 0.0f;
            axis->accel = 0.0f;
        }
    }
    axis->response_direction = 0;
}

void tracker_servo_step(tracker_view_controller_t *controller,
                        float target_x, float target_y,
                        float noise_enter_x, float noise_enter_y,
                        float dt)
{
    if (!controller->initialized) {
        tracker_servo_start(controller, target_x, target_y);
        return;
    }
    /* One call represents one delivered frame.  Scheduling delay must not be
     * converted to several frames of crop movement in one ISP write. */
    dt = clampf_servo(dt, 0.001f, 1.0f / TRACK_CONTROL_FPS);
    noise_enter_x = clampf_servo(
        noise_enter_x, TRACK_VIEW_NOISE_ENTER_X_MIN_PX,
        TRACK_VIEW_NOISE_ENTER_X_MAX_PX);
    noise_enter_y = clampf_servo(
        noise_enter_y, TRACK_VIEW_NOISE_ENTER_Y_MIN_PX,
        TRACK_VIEW_NOISE_ENTER_Y_MAX_PX);
    reference_step(&controller->reference_x, target_x, dt);
    reference_step(&controller->reference_y, target_y, dt);
    step_axis(&controller->x,
              controller->reference_x.pos,
              controller->reference_x.vel, dt,
              SERVO_X_MIN, SERVO_X_MAX,
              noise_enter_x, TRACK_VIEW_PATH_OMEGA_X,
              TRACK_VIEW_MAX_VEL_X, TRACK_VIEW_MAX_ACCEL_X,
              TRACK_VIEW_MAX_JERK_X);
    step_axis(&controller->y,
              controller->reference_y.pos,
              controller->reference_y.vel, dt,
              SERVO_Y_MIN, SERVO_Y_MAX,
              noise_enter_y, TRACK_VIEW_PATH_OMEGA_Y,
              TRACK_VIEW_MAX_VEL_Y, TRACK_VIEW_MAX_ACCEL_Y,
              TRACK_VIEW_MAX_JERK_Y);
}

void tracker_servo_brake(tracker_view_controller_t *controller, float dt)
{
    if (!controller->initialized)
        return;
    dt = clampf_servo(dt, 0.001f, 1.0f / TRACK_CONTROL_FPS);
    brake_axis(&controller->x, dt, SERVO_X_MIN, SERVO_X_MAX,
               TRACK_VIEW_PATH_OMEGA_X, TRACK_VIEW_MAX_VEL_X,
               TRACK_VIEW_MAX_ACCEL_X, TRACK_VIEW_MAX_JERK_X);
    brake_axis(&controller->y, dt, SERVO_Y_MIN, SERVO_Y_MAX,
               TRACK_VIEW_PATH_OMEGA_Y, TRACK_VIEW_MAX_VEL_Y,
               TRACK_VIEW_MAX_ACCEL_Y, TRACK_VIEW_MAX_JERK_Y);
    controller->reference_x.pos = controller->x.pos;
    controller->reference_x.vel = 0.0f;
    controller->reference_y.pos = controller->y.pos;
    controller->reference_y.vel = 0.0f;
}
