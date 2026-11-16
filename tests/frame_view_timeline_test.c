#include "cam_system/frame_view_timeline.h"

#include <stdio.h>
#include <stdlib.h>

static frame_view_t make_view(float x, float y, int scaler_enabled)
{
    frame_view_t view = {0};
    view.roi_x = x;
    view.roi_y = y;
    view.roi_w = scaler_enabled ? 1312.0f : 640.0f;
    view.roi_h = scaler_enabled ? 972.0f : 480.0f;
    view.output_w = 640;
    view.output_h = 480;
    view.scaler_enabled = scaler_enabled;
    return view;
}

static void require_view(const frame_view_t *view, float x, float y,
                         int scaler_enabled, const char *message)
{
    if (view->roi_x != x || view->roi_y != y ||
        view->scaler_enabled != scaler_enabled) {
        fprintf(stderr,
                "%s: got=(%.0f,%.0f,%d) expected=(%.0f,%.0f,%d)\n",
                message, view->roi_x, view->roi_y, view->scaler_enabled,
                x, y, scaler_enabled);
        exit(1);
    }
}

int main(void)
{
    frame_view_timeline_t timeline;
    frame_view_t full = make_view(0.0f, 0.0f, 1);
    frame_view_t crop_a = make_view(100.0f, 80.0f, 0);
    frame_view_t crop_b = make_view(140.0f, 110.0f, 0);
    frame_view_t resolved;

    frame_view_timeline_init(&timeline);
    frame_view_timeline_resolve(&timeline, 10u, &full, &resolved);
    require_view(&resolved, 0.0f, 0.0f, 1, "empty timeline fallback");

    /* A command issued after frame 10 becomes the frame-12 view. */
    frame_view_timeline_schedule(&timeline, 8u, &full);
    frame_view_timeline_schedule(&timeline, 10u, &crop_a);
    frame_view_timeline_resolve(&timeline, 11u, &crop_b, &resolved);
    require_view(&resolved, 0.0f, 0.0f, 1, "pre-apply frame mismatch");
    frame_view_timeline_resolve(&timeline, 12u, &full, &resolved);
    require_view(&resolved, 100.0f, 80.0f, 0, "apply frame mismatch");

    frame_view_timeline_schedule(&timeline, 11u, &crop_b);
    frame_view_timeline_resolve(&timeline, 12u, &full, &resolved);
    require_view(&resolved, 100.0f, 80.0f, 0,
                 "future command leaked into frame");
    frame_view_timeline_resolve(&timeline, 13u, &full, &resolved);
    require_view(&resolved, 140.0f, 110.0f, 0,
                 "successor command mismatch");

    /* Sequence arithmetic must remain correct across uint32 wrap. */
    frame_view_timeline_init(&timeline);
    frame_view_timeline_schedule(&timeline, 0xfffffffdu, &crop_a);
    frame_view_timeline_schedule(&timeline, 0xffffffffu, &crop_b);
    frame_view_timeline_resolve(&timeline, 0u, &full, &resolved);
    require_view(&resolved, 100.0f, 80.0f, 0, "wrap predecessor mismatch");
    frame_view_timeline_resolve(&timeline, 1u, &full, &resolved);
    require_view(&resolved, 140.0f, 110.0f, 0, "wrap successor mismatch");

    /* Overwriting the ring must retain the newest effective command. */
    frame_view_timeline_init(&timeline);
    for (uint32_t sequence = 20u; sequence < 50u; sequence++) {
        frame_view_t view = make_view((float)sequence, 0.0f, 0);
        frame_view_timeline_schedule(&timeline, sequence - 2u, &view);
    }
    frame_view_timeline_resolve(&timeline, 49u, &full, &resolved);
    require_view(&resolved, 49.0f, 0.0f, 0, "ring newest mismatch");

    /* A slipped command and its successor share a sequence: the later write is the real one. */
    frame_view_timeline_init(&timeline);
    frame_view_timeline_schedule(&timeline, 60u, &crop_a);
    frame_view_timeline_schedule(&timeline, 60u, &crop_b);
    frame_view_timeline_resolve(&timeline, 62u, &full, &resolved);
    require_view(&resolved, 140.0f, 110.0f, 0, "same-sequence successor mismatch");

    puts("frame_view_timeline_test: OK");
    return 0;
}
