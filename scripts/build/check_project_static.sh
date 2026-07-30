#!/usr/bin/env bash
set -euo pipefail

ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"
TMP_REPORT="$(mktemp)"
trap 'rm -f "${TMP_REPORT}"' EXIT

SCAN_DIRS=(
    "${ROOT_DIR}/src"
    "${ROOT_DIR}/include"
    "${ROOT_DIR}/packaging/petalinux/recipes-apps/cam-system"
    "${ROOT_DIR}/packaging/petalinux/recipes-kernel/cam-ov5640-control"
    "${ROOT_DIR}/packaging/petalinux/recipes-kernel/cam-dvp-bridge"
    "${ROOT_DIR}/packaging/petalinux/recipes-kernel/cam-face-accel"
)

failures=0

check_absent() {
    local pattern="$1"
    local description="$2"

    : > "${TMP_REPORT}"
    for dir in "${SCAN_DIRS[@]}"; do
        if [[ -d "${dir}" ]]; then
            grep -RInE -- "${pattern}" "${dir}" >> "${TMP_REPORT}" || true
        fi
    done

    if [[ -s "${TMP_REPORT}" ]]; then
        echo "FAIL: ${description}" >&2
        cat "${TMP_REPORT}" >&2
        failures=$((failures + 1))
    fi
}

check_present() {
    local pattern="$1"
    local file="$2"
    local description="$3"

    if ! grep -Eq -- "${pattern}" "${ROOT_DIR}/${file}"; then
        echo "FAIL: ${description}" >&2
        failures=$((failures + 1))
    fi
}

check_absent '/dev/mem' \
    'runtime source must not use raw /dev/mem access'
check_absent '/sys/class/gpio' \
    'runtime source must not use legacy sysfs GPIO control'
check_absent 'strstr[[:space:]]*\([[:space:]]*request[[:space:]]*,[[:space:]]*"POST /api' \
    'REST API routing must not use substring matching'
check_absent 'TRACK_CMD_TEST|POST /api/test|"test"[[:space:]]*:' \
    'development-only tracking test command must not be present in runtime source'
check_absent 'jpeg_worker_submit|frame_snapshot' \
    'runtime must not restore detector/JPEG full-frame snapshot copies'

check_present 'frame_mailbox_wait_latest' 'src/vision/detector.cpp' \
    'detector must consume the event-driven frame mailbox'
check_present 'frame_mailbox_wait_latest' 'src/media/jpeg_worker.c' \
    'JPEG worker must consume the event-driven frame mailbox'
check_present '\\"frame_seq\\"' 'src/network/ws_protocol.c' \
    'WebSocket detections must expose their source frame sequence'
check_present 'cam_ws_format_detection_frame_seq' 'src/network/net_server.c' \
    'WebSocket detection broadcast must serialize source frame sequence'

if [[ "${failures}" -ne 0 ]]; then
    exit 1
fi

"${ROOT_DIR}/scripts/build/verify_face_accel_contract.py"

echo "project_static_check: OK"
