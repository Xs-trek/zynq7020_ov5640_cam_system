#include "cam_system/config.h"
#include "control/tracker_servo.h"

#include <math.h>
#include <stdio.h>
#include <stdlib.h>

static int check_axis_step(const char *name, int frame,
                           const tracker_view_axis_t *axis,
                           float previous_position,
                           float previous_velocity,
                           float previous_acceleration,
                           float max_velocity,
                           float max_acceleration,
                           float max_jerk)
{
    const float dt = 1.0f / TRACK_CONTROL_FPS;

    if (fabsf(axis->pos - previous_position) > max_velocity * dt + 1e-3f ||
        fabsf(axis->vel - previous_velocity) >
            max_acceleration * dt + 1e-3f ||
        fabsf(axis->accel - previous_acceleration) >
            max_jerk * dt + 1e-3f) {
        fprintf(stderr,
                "%s frame=%d violated final path bounds pos=%.3f vel=%.3f accel=%.3f\n",
                name, frame, axis->pos, axis->vel, axis->accel);
        return 1;
    }
    return 0;
}

static int test_noise_gate(void)
{
    int direction = 0;
    float value;

    value = tracker_servo_band_error(6.0f, 6.0f, 3.0f, &direction);
    if (value != 0.0f || direction != 0)
        return 1;
    value = tracker_servo_band_error(7.0f, 6.0f, 3.0f, &direction);
    if (fabsf(value - 4.0f) > 1e-6f || direction != 1)
        return 1;
    value = tracker_servo_band_error(5.0f, 6.0f, 3.0f, &direction);
    if (fabsf(value - 2.0f) > 1e-6f || direction != 1)
        return 1;
    value = tracker_servo_band_error(3.0f, 6.0f, 3.0f, &direction);
    if (value != 0.0f || direction != 0)
        return 1;
    value = tracker_servo_band_error(-7.0f, 6.0f, 3.0f, &direction);
    if (fabsf(value + 4.0f) > 1e-6f || direction != -1)
        return 1;
    value = tracker_servo_band_error(-3.0f, 6.0f, 3.0f, &direction);
    if (value != 0.0f || direction != 0)
        return 1;
    return 0;
}

static int test_static_noise(float *maximum_jitter_x,
                             float *maximum_jitter_y)
{
    const float dt = 1.0f / TRACK_CONTROL_FPS;
    tracker_view_controller_t composer;

    tracker_servo_start(&composer, 300.0f, 200.0f);
    for (int frame = 0; frame < 300; frame++) {
        float target_x = 300.0f + (frame & 1 ? 5.5f : -5.5f);
        float target_y = 200.0f + (frame & 1 ? -7.5f : 7.5f);

        tracker_servo_step(&composer, target_x, target_y, 6.0f, 8.0f, dt);
        *maximum_jitter_x = fmaxf(*maximum_jitter_x,
                                  fabsf(composer.x.pos - 300.0f));
        *maximum_jitter_y = fmaxf(*maximum_jitter_y,
                                  fabsf(composer.y.pos - 200.0f));
    }
    if (*maximum_jitter_x > 1e-6f || *maximum_jitter_y > 1e-6f) {
        fprintf(stderr, "noise gate leaked to viewport max=(%.3f,%.3f)\n",
                *maximum_jitter_x, *maximum_jitter_y);
        return 1;
    }
    return 0;
}

static int test_constant_motion(float *lag_x, float *lag_y)
{
    const float dt = 1.0f / TRACK_CONTROL_FPS;
    tracker_view_controller_t composer;

    tracker_servo_start(&composer, 100.0f, 100.0f);
    for (int frame = 1; frame <= 60; frame++) {
        float target_x = 100.0f + 240.0f * frame * dt;
        float target_y = 100.0f + 150.0f * frame * dt;
        float previous_x = composer.x.pos;
        float previous_y = composer.y.pos;
        float previous_vx = composer.x.vel;
        float previous_vy = composer.y.vel;
        float previous_ax = composer.x.accel;
        float previous_ay = composer.y.accel;

        tracker_servo_step(&composer, target_x, target_y, 6.0f, 8.0f, dt);
        if (check_axis_step("ramp-x", frame, &composer.x,
                            previous_x, previous_vx, previous_ax,
                            TRACK_VIEW_MAX_VEL_X,
                            TRACK_VIEW_MAX_ACCEL_X,
                            TRACK_VIEW_MAX_JERK_X) ||
            check_axis_step("ramp-y", frame, &composer.y,
                            previous_y, previous_vy, previous_ay,
                            TRACK_VIEW_MAX_VEL_Y,
                            TRACK_VIEW_MAX_ACCEL_Y,
                            TRACK_VIEW_MAX_JERK_Y))
            return 1;
    }
    *lag_x = 580.0f - composer.x.pos;
    *lag_y = 400.0f - composer.y.pos;
    if (fabsf(*lag_x) > 12.0f || fabsf(*lag_y) > 12.0f) {
        fprintf(stderr, "stable-reference ramp response lag=(%.2f,%.2f)\n",
                *lag_x, *lag_y);
        return 1;
    }
    return 0;
}

static int test_step_response(float *overshoot_x, float *overshoot_y)
{
    const float dt = 1.0f / TRACK_CONTROL_FPS;
    tracker_view_controller_t composer;

    *overshoot_x = 0.0f;
    *overshoot_y = 0.0f;
    tracker_servo_start(&composer, 100.0f, 100.0f);
    for (int frame = 0; frame < 240; frame++) {
        float previous_x = composer.x.pos;
        float previous_y = composer.y.pos;
        float previous_vx = composer.x.vel;
        float previous_vy = composer.y.vel;
        float previous_ax = composer.x.accel;
        float previous_ay = composer.y.accel;

        tracker_servo_step(&composer, 500.0f, 400.0f, 6.0f, 8.0f, dt);
        *overshoot_x = fmaxf(*overshoot_x, composer.x.pos - 500.0f);
        *overshoot_y = fmaxf(*overshoot_y, composer.y.pos - 400.0f);
        if (check_axis_step("step-x", frame, &composer.x,
                            previous_x, previous_vx, previous_ax,
                            TRACK_VIEW_MAX_VEL_X,
                            TRACK_VIEW_MAX_ACCEL_X,
                            TRACK_VIEW_MAX_JERK_X) ||
            check_axis_step("step-y", frame, &composer.y,
                            previous_y, previous_vy, previous_ay,
                            TRACK_VIEW_MAX_VEL_Y,
                            TRACK_VIEW_MAX_ACCEL_Y,
                            TRACK_VIEW_MAX_JERK_Y))
            return 1;
    }
    if (*overshoot_x > 2.0f || *overshoot_y > 2.0f ||
        fabsf(composer.x.pos - 500.0f) > 6.0f ||
        fabsf(composer.y.pos - 400.0f) > 8.0f ||
        fabsf(composer.x.vel) > 0.5f || fabsf(composer.y.vel) > 0.5f) {
        fprintf(stderr,
                "step failed pos=(%.2f,%.2f) vel=(%.2f,%.2f) overshoot=(%.2f,%.2f)\n",
                composer.x.pos, composer.y.pos,
                composer.x.vel, composer.y.vel,
                *overshoot_x, *overshoot_y);
        return 1;
    }
    return 0;
}

static int test_discrete_command(float *maximum_step_x,
                                 float *maximum_step_y,
                                 int *maximum_delta_step_x,
                                 int *maximum_delta_step_y,
                                 int *maximum_jerk_step_x,
                                 int *maximum_jerk_step_y)
{
    const float dt = 1.0f / TRACK_CONTROL_FPS;
    tracker_view_controller_t composer;
    int previous_x = 180;
    int previous_y = 140;
    int previous_dx = 0;
    int previous_dy = 0;
    int previous_ddx = 0;
    int previous_ddy = 0;

    tracker_servo_start(&composer, (float)previous_x, (float)previous_y);
    for (int frame = 0; frame < 180; frame++) {
        float target_x;
        float target_y;
        int output_x;
        int output_y;
        int dx;
        int dy;
        int ddx;
        int ddy;
        int dddx;
        int dddy;

        if (frame < 45) {
            target_x = 180.0f + 240.0f * frame * dt;
            target_y = 140.0f + 150.0f * frame * dt;
        } else if (frame < 75) {
            target_x = 540.0f;
            target_y = 365.0f;
        } else if (frame < 120) {
            target_x = 540.0f - 240.0f * (frame - 75) * dt;
            target_y = 365.0f - 150.0f * (frame - 75) * dt;
        } else {
            target_x = 180.0f;
            target_y = 140.0f;
        }
        tracker_servo_step(&composer, target_x, target_y, 6.0f, 8.0f, dt);
        output_x = (int)lroundf(composer.x.pos);
        output_y = (int)lroundf(composer.y.pos);
        dx = output_x - previous_x;
        dy = output_y - previous_y;
        ddx = dx - previous_dx;
        ddy = dy - previous_dy;
        dddx = ddx - previous_ddx;
        dddy = ddy - previous_ddy;
        *maximum_step_x = fmaxf(*maximum_step_x, (float)abs(dx));
        *maximum_step_y = fmaxf(*maximum_step_y, (float)abs(dy));
        *maximum_delta_step_x = abs(ddx) > *maximum_delta_step_x ?
                                abs(ddx) : *maximum_delta_step_x;
        *maximum_delta_step_y = abs(ddy) > *maximum_delta_step_y ?
                                abs(ddy) : *maximum_delta_step_y;
        *maximum_jerk_step_x = abs(dddx) > *maximum_jerk_step_x ?
                               abs(dddx) : *maximum_jerk_step_x;
        *maximum_jerk_step_y = abs(dddy) > *maximum_jerk_step_y ?
                               abs(dddy) : *maximum_jerk_step_y;
        previous_x = output_x;
        previous_y = output_y;
        previous_dx = dx;
        previous_dy = dy;
        previous_ddx = ddx;
        previous_ddy = ddy;
    }
    if (*maximum_step_x > ceilf(TRACK_VIEW_MAX_STEP_X_PX) ||
        *maximum_step_y > ceilf(TRACK_VIEW_MAX_STEP_Y_PX) ||
        *maximum_delta_step_x > 3 || *maximum_delta_step_y > 3 ||
        *maximum_jerk_step_x > 3 || *maximum_jerk_step_y > 3) {
        fprintf(stderr,
                "integer path discontinuity step=(%.0f,%.0f) delta=(%d,%d) jerk=(%d,%d)\n",
                *maximum_step_x, *maximum_step_y,
                *maximum_delta_step_x, *maximum_delta_step_y,
                *maximum_jerk_step_x, *maximum_jerk_step_y);
        return 1;
    }

    /* A late callback still represents one delivered frame. */
    {
        float previous_position_x = composer.x.pos;
        float previous_position_y = composer.y.pos;
        tracker_servo_step(&composer, 650.0f, 480.0f, 6.0f, 8.0f, 0.1f);
        if (fabsf(composer.x.pos - previous_position_x) >
                TRACK_VIEW_MAX_STEP_X_PX + 1e-3f ||
            fabsf(composer.y.pos - previous_position_y) >
                TRACK_VIEW_MAX_STEP_Y_PX + 1e-3f) {
            fprintf(stderr, "late callback spent multiple frame moves\n");
            return 1;
        }
    }
    return 0;
}

static int test_tracking_to_brake_transition(void)
{
    const float dt = 1.0f / TRACK_CONTROL_FPS;
    tracker_view_controller_t composer;
    int previous_x = 80;
    int previous_y = 80;
    int previous_dx = 0;
    int previous_dy = 0;
    int previous_ddx = 0;
    int previous_ddy = 0;
    int max_step_x = 0;
    int max_step_y = 0;
    int max_delta_x = 0;
    int max_delta_y = 0;
    int max_jerk_x = 0;
    int max_jerk_y = 0;

    tracker_servo_start(&composer, (float)previous_x, (float)previous_y);
    for (int frame = 0; frame < 90; frame++) {
        float previous_position_x = composer.x.pos;
        float previous_position_y = composer.y.pos;
        float previous_velocity_x = composer.x.vel;
        float previous_velocity_y = composer.y.vel;
        float previous_accel_x = composer.x.accel;
        float previous_accel_y = composer.y.accel;
        int output_x;
        int output_y;
        int dx;
        int dy;
        int ddx;
        int ddy;
        int dddx;
        int dddy;

        if (frame < 30)
            tracker_servo_step(&composer, 900.0f, 700.0f,
                               6.0f, 8.0f, dt);
        else
            tracker_servo_brake(&composer, dt);

        if (check_axis_step("brake-x", frame, &composer.x,
                            previous_position_x, previous_velocity_x,
                            previous_accel_x, TRACK_VIEW_MAX_VEL_X,
                            TRACK_VIEW_MAX_ACCEL_X, TRACK_VIEW_MAX_JERK_X) ||
            check_axis_step("brake-y", frame, &composer.y,
                            previous_position_y, previous_velocity_y,
                            previous_accel_y, TRACK_VIEW_MAX_VEL_Y,
                            TRACK_VIEW_MAX_ACCEL_Y, TRACK_VIEW_MAX_JERK_Y))
            return 1;

        output_x = (int)lroundf(composer.x.pos);
        output_y = (int)lroundf(composer.y.pos);
        dx = output_x - previous_x;
        dy = output_y - previous_y;
        ddx = dx - previous_dx;
        ddy = dy - previous_dy;
        dddx = ddx - previous_ddx;
        dddy = ddy - previous_ddy;
        if (abs(dx) > max_step_x) max_step_x = abs(dx);
        if (abs(dy) > max_step_y) max_step_y = abs(dy);
        if (abs(ddx) > max_delta_x) max_delta_x = abs(ddx);
        if (abs(ddy) > max_delta_y) max_delta_y = abs(ddy);
        if (abs(dddx) > max_jerk_x) max_jerk_x = abs(dddx);
        if (abs(dddy) > max_jerk_y) max_jerk_y = abs(dddy);
        previous_x = output_x;
        previous_y = output_y;
        previous_dx = dx;
        previous_dy = dy;
        previous_ddx = ddx;
        previous_ddy = ddy;
    }

    if (max_step_x > (int)ceilf(TRACK_VIEW_MAX_STEP_X_PX) ||
        max_step_y > (int)ceilf(TRACK_VIEW_MAX_STEP_Y_PX) ||
        max_delta_x > 3 || max_delta_y > 3 ||
        max_jerk_x > 4 || max_jerk_y > 4 ||
        fabsf(composer.x.vel) > 0.25f ||
        fabsf(composer.y.vel) > 0.25f) {
        fprintf(stderr,
                "tracking/brake discontinuity step=(%d,%d) delta=(%d,%d) jerk=(%d,%d) vel=(%.2f,%.2f)\n",
                max_step_x, max_step_y, max_delta_x, max_delta_y,
                max_jerk_x, max_jerk_y,
                composer.x.vel, composer.y.vel);
        return 1;
    }
    return 0;
}

int main(void)
{
    float maximum_jitter_x = 0.0f;
    float maximum_jitter_y = 0.0f;
    float lag_x = 0.0f;
    float lag_y = 0.0f;
    float overshoot_x = 0.0f;
    float overshoot_y = 0.0f;
    float maximum_step_x = 0.0f;
    float maximum_step_y = 0.0f;
    int maximum_delta_step_x = 0;
    int maximum_delta_step_y = 0;
    int maximum_jerk_step_x = 0;
    int maximum_jerk_step_y = 0;

    if (test_noise_gate()) {
        fprintf(stderr, "Schmitt noise-gate contract failed\n");
        return 1;
    }
    if (test_static_noise(&maximum_jitter_x, &maximum_jitter_y) ||
        test_constant_motion(&lag_x, &lag_y) ||
        test_step_response(&overshoot_x, &overshoot_y) ||
        test_discrete_command(&maximum_step_x, &maximum_step_y,
                              &maximum_delta_step_x,
                              &maximum_delta_step_y,
                              &maximum_jerk_step_x,
                              &maximum_jerk_step_y) ||
        test_tracking_to_brake_transition())
        return 1;

    printf("tracker_servo_test: OK jitter=(%.3f,%.3f) lag=(%.1f,%.1f) "
           "overshoot=(%.2f,%.2f) integer_step=(%.0f,%.0f) "
           "integer_delta=(%d,%d) integer_jerk=(%d,%d)\n",
           maximum_jitter_x, maximum_jitter_y, lag_x, lag_y,
           overshoot_x, overshoot_y, maximum_step_x, maximum_step_y,
           maximum_delta_step_x, maximum_delta_step_y,
           maximum_jerk_step_x, maximum_jerk_step_y);
    return 0;
}
