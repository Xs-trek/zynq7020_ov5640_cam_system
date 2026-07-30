#!/usr/bin/env bash
set -euo pipefail

ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"

tests=(
    check_project_static.sh
    test_cmd_handler_route.sh
    test_face_temporal_tracker.sh
    test_frame_mailbox.sh
    test_frame_pipeline_timing.sh
    test_frame_view_timeline.sh
    test_jpeg_worker.sh
    validate_ssh_access_dev.sh
    test_tracker_servo.sh
    test_tracker_detection_update.sh
    test_tracker_pipeline_replay.sh
    test_tracker_closed_loop.sh
    test_ws_protocol.sh
)

for test_script in "${tests[@]}"; do
    echo "[host-tests] ${test_script}"
    "${ROOT_DIR}/scripts/build/${test_script}"
done

echo "host_test_suite: OK (${#tests[@]} checks)"
