#define LOG_TAG "capture"

#include "capture/camera_switch.h"

#include "cam_system/app_state.h"
#include "cam_system/frame_mailbox.h"
#include "common/log.h"

#include <string.h>

static void drop_track_commands(void)
{
    pthread_mutex_lock(&g_state.cmd_mutex);
    g_state.cmd_head = 0;
    g_state.cmd_tail = 0;
    g_state.cmd_count = 0;
    g_state.track_cmd = TRACK_CMD_NONE;
    g_state.track_target_id = -1;
    pthread_mutex_unlock(&g_state.cmd_mutex);
}

static void clear_outputs(void)
{
    pthread_mutex_lock(&g_state.det_mutex);
    g_state.det_count = 0;
    g_state.det_frame_seq = 0;
    g_state.det_publish_time_sec = 0.0;
    memset(&g_state.det_frame_view, 0, sizeof(g_state.det_frame_view));
    app_state_inc_u32(&g_state.det_seq);
    pthread_mutex_unlock(&g_state.det_mutex);

    pthread_mutex_lock(&g_state.pred_mutex);
    g_state.pred_box.valid = 0;
    pthread_mutex_unlock(&g_state.pred_mutex);
    app_state_store_int(&g_state.tracking_active, 0);
    app_state_store_int(&g_state.tracking_target_id, -1);
}

void camera_off(tracker_t *tracker, const camera_off_stream_ops_t *ops)
{
    app_state_store_int(&g_state.cam_enabled, 0);
    app_state_store_int(&g_state.detector_enabled, 0);
    drop_track_commands();
    if (tracker_get_state(tracker) != TRACK_STATE_IDLE)
        tracker_stop(tracker);
    clear_outputs();
    frame_mailbox_wake_all(&g_state);

    ops->last_reading();
    ops->stop_stream();
    ops->standby();
    app_state_inc_u32(&g_state.status_seq);
}

int camera_on(tracker_t *tracker, int test_pattern, const camera_on_stream_ops_t *ops)
{
    if (ops->initialize() < 0) {
        LOG_E("the sensor did not initialize");
        return -1;
    }
    /* camera_off leaves the tracker idle; a tracker that still runs would hold a crop the table
     * has just reset, so it is stopped here, on the running sensor. */
    if (tracker_get_state(tracker) != TRACK_STATE_IDLE)
        tracker_stop(tracker);
    tracker_sensor_initialized();
    if (test_pattern && ops->test_pattern() < 0) {
        LOG_E("the colour bars were not set again after the initialization");
        return -1;
    }
    return ops->start_stream();
}
