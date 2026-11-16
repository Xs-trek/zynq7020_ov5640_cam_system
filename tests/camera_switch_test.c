/* camera_switch_test.c - turning the camera on, and off by the user or by the stream limit.
 *
 * The real tracker is compiled in; the sensor's control calls and the stream steps record into
 * one log, so the test sees what reaches the sensor and in which order.  The sensor model follows
 * the register table: an initialization turns the scaler on, sets the default offset and clears
 * the colour bars.
 */
#include "capture/camera_switch.h"

#include "cam_system/app_state.h"
#include "cam_system/config.h"
#include "cam_system/tracker.h"

#include <stdio.h>
#include <stdlib.h>
#include <string.h>

app_state_t g_state;
uint8_t *g_frame_pool[CAPTURE_BUF_CNT];
uint8_t *g_cached_pool[FRAME_BUF_CNT];

static int g_checks;
static char g_log[48][24];
static int g_log_len;
static int g_streaming;
static int g_scaler = 1;
static int g_xoff = ISP_DEFAULT_XOFF;
static int g_yoff = ISP_DEFAULT_YOFF;
static int g_bars;
static int g_fail_crop_writes;      /* the scaler and offset writes fail */
static int g_fail_init;
static int g_fail_bars;

static void require(int cond, const char *what)
{
    g_checks++;
    if (!cond) {
        fprintf(stderr, "camera_switch_test: FAIL: %s\n", what);
        exit(1);
    }
}

static void record(const char *event)
{
    require(g_log_len < 48, "log room");
    snprintf(g_log[g_log_len++], sizeof(g_log[0]), "%s%s", event, g_streaming ? "" : " (stopped)");
}

/* Position of the first entry that starts with prefix, or -1. */
static int logged(const char *prefix)
{
    for (int i = 0; i < g_log_len; i++)
        if (strncmp(g_log[i], prefix, strlen(prefix)) == 0)
            return i;
    return -1;
}

int ov5640_apply_isp_offset(int x, int y)
{
    if (g_fail_crop_writes)
        return -1;
    g_xoff = x;
    g_yoff = y;
    record("offset");
    return 0;
}

int ov5640_set_scaler_enabled(int enabled)
{
    if (g_fail_crop_writes)
        return -1;
    g_scaler = enabled;
    record(enabled ? "scaler on" : "scaler off");
    return 0;
}

static void last_reading(void) { record("reading"); }
static void stop_stream(void) { record("stream off"); g_streaming = 0; }
static void standby(void) { record("standby"); }

static const camera_off_stream_ops_t ops = { last_reading, stop_stream, standby };

static int initialize(void)
{
    if (g_fail_init)
        return -1;
    g_scaler = 1;
    g_xoff = ISP_DEFAULT_XOFF;
    g_yoff = ISP_DEFAULT_YOFF;
    g_bars = 0;
    record("init");
    return 0;
}

static int test_pattern(void)
{
    if (g_fail_bars)
        return -1;
    g_bars = 1;
    record("bars");
    return 0;
}

static int start_stream(void)
{
    g_streaming = 1;
    record(g_bars ? "stream on, bars" : "stream on, no bars");
    return 0;
}

static const camera_on_stream_ops_t on_ops = { initialize, test_pattern, start_stream };

static void reset_state(void)
{
    pthread_mutex_destroy(&g_state.det_mutex);
    pthread_mutex_destroy(&g_state.pred_mutex);
    pthread_mutex_destroy(&g_state.cmd_mutex);
    pthread_mutex_destroy(&g_state.frame_mutex);
    pthread_cond_destroy(&g_state.frame_cond);
    memset(&g_state, 0, sizeof(g_state));
    pthread_mutex_init(&g_state.det_mutex, NULL);
    pthread_mutex_init(&g_state.pred_mutex, NULL);
    pthread_mutex_init(&g_state.cmd_mutex, NULL);
    pthread_mutex_init(&g_state.frame_mutex, NULL);
    pthread_cond_init(&g_state.frame_cond, NULL);
    g_state.cam_enabled = 1;
    g_state.detector_enabled = 1;
    g_state.tracking_target_id = -1;
    g_log_len = 0;
    g_streaming = 1;
    g_bars = 0;
    g_fail_crop_writes = 0;
    g_fail_init = 0;
    g_fail_bars = 0;
}

/* One published face, selected and tracked: the scaler goes off and the crop moves. */
static void start_tracking(tracker_t *tracker)
{
    detection_t *det = &g_state.detections[0];
    frame_view_t *view = &g_state.det_frame_view;

    view->roi_w = SUBSAMPLED_1X_W;
    view->roi_h = SUBSAMPLED_1X_H;
    view->output_w = IMG_W;
    view->output_h = IMG_H;
    view->flip_x = view->flip_y = 1;
    view->scaler_enabled = 1;
    view->capture_time_sec = 100.0;
    det->bbox[0] = 0.10f;
    det->bbox[1] = 0.10f;
    det->bbox[2] = 0.10f;
    det->bbox[3] = 0.13f;
    det->class_id = DETECTION_CLASS_FACE;
    det->confidence = 0.95f;
    det->track_id = 7;
    det->publish = 1;
    g_state.det_count = 1;
    tracker_apply_cmd(tracker, TRACK_CMD_SELECT, 7);
    tracker_apply_cmd(tracker, TRACK_CMD_START, -1);
    require(tracker_is_active(tracker) && g_scaler == 0, "tracking: crop, scaler off");
    require(g_xoff != ISP_DEFAULT_XOFF || g_yoff != ISP_DEFAULT_YOFF, "the crop left the default");
    app_state_store_int(&g_state.tracking_active, 1);
    app_state_store_int(&g_state.tracking_target_id, 7);
    g_state.pred_box.valid = 1;
    g_log_len = 0;
}

static void require_off_state(const tracker_t *tracker)
{
    require(!app_state_load_int(&g_state.cam_enabled), "the camera is off");
    require(!app_state_load_int(&g_state.detector_enabled), "detection is off");
    require(!app_state_load_int(&g_state.tracking_active), "tracking is off");
    require(app_state_load_int(&g_state.tracking_target_id) == -1, "no target");
    require(tracker_get_state(tracker) == TRACK_STATE_IDLE, "the tracker is idle");
    require(g_state.det_count == 0 && !g_state.pred_box.valid, "no detection or prediction output");
    require(g_state.cmd_count == 0 && g_state.track_cmd == TRACK_CMD_NONE, "no track command left");
    require(g_scaler == 1 && g_xoff == ISP_DEFAULT_XOFF && g_yoff == ISP_DEFAULT_YOFF,
            "full view restored");
    require(!g_streaming, "the stream is stopped");
    require(logged("reading") >= 0 && logged("reading") < logged("stream off") &&
            logged("stream off") < logged("standby"),
            "last reading, stream off, standby, in that order");
}

/* The user turns the camera off while tracking: the full view is restored on a running sensor. */
static void test_user_off_while_tracking(void)
{
    tracker_t tracker;
    unsigned int status_before;

    reset_state();
    tracker_init(&tracker);
    start_tracking(&tracker);
    app_state_store_int(&g_state.cam_enabled, 0);     /* what POST /api/camera does */
    g_state.cmd_queue[0] = TRACK_CMD_START;            /* a command still in the queue */
    g_state.cmd_count = 1;
    g_state.cmd_tail = 1;
    status_before = app_state_load_u32(&g_state.status_seq);
    camera_off(&tracker, &ops);
    require_off_state(&tracker);
    require(logged("scaler on") >= 0 && logged("offset") >= 0, "scaler and offset restored");
    require(logged("scaler on") < logged("stream off") && logged("offset") < logged("stream off"),
            "the restoring writes come before the stream stops");
    require(logged("scaler on (stopped)") < 0 && logged("offset (stopped)") < 0,
            "nothing is written to a stopped sensor");
    require(app_state_load_u32(&g_state.status_seq) != status_before, "the status changed");
}

/* The limit turns the camera off while tracking: the same state as the user's off. */
static void test_limit_off_while_tracking(void)
{
    tracker_t tracker;

    reset_state();
    tracker_init(&tracker);
    start_tracking(&tracker);
    /* cam_enabled is still 1: the limit, not the user, ends this stream */
    camera_off(&tracker, &ops);
    require_off_state(&tracker);
    require(logged("offset") < logged("stream off"), "restored before the stop here too");
}

/* Nothing tracked: only the stream steps reach the hardware. */
static void test_off_without_tracking(void)
{
    tracker_t tracker;

    reset_state();
    tracker_init(&tracker);
    camera_off(&tracker, &ops);
    require_off_state(&tracker);
    require(g_log_len == 3, "no sensor write besides the stream steps");
}

static void require_tracker_full_view(const tracker_t *tracker)
{
    frame_view_t view;
    int xoff;
    int yoff;

    tracker_get_isp_offset(&xoff, &yoff);
    tracker_snapshot_view(&view, xoff, yoff, 1u, 0.0);
    require(tracker_get_state(tracker) == TRACK_STATE_IDLE, "the tracker is idle");
    require(xoff == ISP_DEFAULT_XOFF && yoff == ISP_DEFAULT_YOFF,
            "the tracker's offset is the table's default");
    require(view.scaler_enabled && view.roi_w == (float)SUBSAMPLED_1X_W,
            "the tracker's view is the scaled full view");
}

/* Every camera-on initializes the sensor before STREAMON; the colour bars, which the table
 * clears, are written between the two.  The same on each of several off/on cycles. */
static void test_on_off_cycles(int bars)
{
    tracker_t tracker;

    reset_state();
    g_streaming = 0;
    tracker_init(&tracker);
    for (int cycle = 0; cycle < 3; cycle++) {
        g_log_len = 0;
        require(camera_on(&tracker, bars, &on_ops) == 0, "the camera turns on");
        require(logged("init") >= 0 && logged("init") < logged("stream on"),
                "initialization before STREAMON");
        if (bars) {
            require(logged("bars") > logged("init") && logged("bars") < logged("stream on"),
                    "colour bars after the initialization, before STREAMON");
            require(logged("stream on, bars") >= 0, "the stream starts with the colour bars");
        } else {
            require(logged("bars") < 0 && logged("stream on, no bars") >= 0,
                    "no colour bars unless configured");
        }
        require(logged("standby") < 0, "no standby write on the way on");
        require(g_scaler == 1 && g_xoff == ISP_DEFAULT_XOFF && g_yoff == ISP_DEFAULT_YOFF,
                "the sensor shows the full view");
        require_tracker_full_view(&tracker);
        camera_off(&tracker, &ops);
        require_off_state(&tracker);
    }
}

/* The full view could not be restored at camera-off (the writes failed): the tracker still holds
 * a crop the sensor no longer has after the initialization.  Camera-on sets its copies to the
 * table's values, and the next tracking start reaches the sensor. */
static void test_on_after_failed_restore(void)
{
    tracker_t tracker;
    int xoff;
    int yoff;

    reset_state();
    tracker_init(&tracker);
    start_tracking(&tracker);
    g_fail_crop_writes = 1;
    camera_off(&tracker, &ops);
    g_fail_crop_writes = 0;
    tracker_get_isp_offset(&xoff, &yoff);
    require(xoff != ISP_DEFAULT_XOFF || yoff != ISP_DEFAULT_YOFF,
            "set-up: the tracker kept its crop offset");
    require(camera_on(&tracker, 1, &on_ops) == 0, "the camera turns on");
    require_tracker_full_view(&tracker);
    start_tracking(&tracker);   /* requires the scaler-off and offset writes to reach the sensor */
}

/* A tracker that still runs at camera-on is stopped on the initialized sensor. */
static void test_on_while_tracking(void)
{
    tracker_t tracker;

    reset_state();
    tracker_init(&tracker);
    start_tracking(&tracker);
    require(camera_on(&tracker, 0, &on_ops) == 0, "the camera turns on");
    require(logged("init") >= 0 && logged("scaler on") > logged("init") &&
            logged("offset") > logged("init"),
            "the tracker's restoring writes follow the initialization");
    require(g_scaler == 1 && g_xoff == ISP_DEFAULT_XOFF && g_yoff == ISP_DEFAULT_YOFF,
            "the sensor shows the full view");
    require_tracker_full_view(&tracker);
}

/* A failed initialization or colour-bar write leaves the stream off. */
static void test_on_failures(void)
{
    tracker_t tracker;

    reset_state();
    g_streaming = 0;
    tracker_init(&tracker);
    g_fail_init = 1;
    require(camera_on(&tracker, 1, &on_ops) < 0, "a failed initialization fails the camera-on");
    require(logged("stream on") < 0 && !g_streaming, "no stream without an initialization");

    g_fail_init = 0;
    g_fail_bars = 1;
    g_log_len = 0;
    require(camera_on(&tracker, 1, &on_ops) < 0, "failed colour bars fail the camera-on");
    require(logged("init") >= 0 && logged("stream on") < 0 && !g_streaming,
            "no stream without the configured colour bars");

    g_log_len = 0;
    require(camera_on(&tracker, 0, &on_ops) == 0, "without colour bars their write is not made");
    require(logged("stream on, no bars") >= 0, "the stream starts");
}

int main(void)
{
    test_user_off_while_tracking();
    test_limit_off_while_tracking();
    test_off_without_tracking();
    test_on_off_cycles(1);
    test_on_off_cycles(0);
    test_on_after_failed_restore();
    test_on_while_tracking();
    test_on_failures();
    printf("camera_switch_test: OK (%d checks)\n", g_checks);
    return 0;
}
