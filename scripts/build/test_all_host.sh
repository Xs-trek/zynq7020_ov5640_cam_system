#!/usr/bin/env bash
set -euo pipefail

ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"

tests=(
    test_app_config.sh
    test_log.sh
    test_frmbuf_capture.sh
    test_detector_recovery.sh
    test_face_decode_fast.sh
    test_control_tick.sh
    test_fps_meter.sh
    test_face_accel_driver.sh
    test_ov5640_driver.sh
    test_face_accel_client.sh
    test_capture_recovery.sh
    test_dvp_counts.sh
    test_stream_control.sh
    test_stream_rate.sh
    test_track_trace.sh
    test_camera_switch.sh
    test_cam_launcher.sh
    validate_ssh_access_dev.sh
    validate_sudo_policy.sh
    test_wifi_init.sh
    check_project_static.sh
    test_cmd_handler_route.sh
    test_face_accel_diagnostics.sh
    test_face_temporal_tracker.sh
    test_frame_mailbox.sh
    test_frame_pipeline_timing.sh
    test_frame_view_timeline.sh
    test_jpeg_worker.sh
    test_ps_pipeline_log_analyzer.sh
    test_tracker_servo.sh
    test_tracker_detection_update.sh
    test_tracker_pipeline_replay.sh
    test_tracker_closed_loop.sh
    test_ws_protocol.sh
    test_mjpeg_framing.sh
    test_http_parse.sh
    test_status_json.sh
    test_net_server.sh
)

for test_script in "${tests[@]}"; do
    echo "[host-tests] ${test_script}"
    "${ROOT_DIR}/scripts/build/${test_script}"
done

echo "host_test_suite: OK (${#tests[@]} checks)"
