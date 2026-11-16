#!/usr/bin/env bash
set -euo pipefail

ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"
BUILD_DIR="${ROOT_DIR}/build/host-tests"
BIN="${BUILD_DIR}/camera_switch_test"

mkdir -p "${BUILD_DIR}"

gcc -Wall -Wextra -Wformat=2 -Werror -std=c11 \
    -fsanitize=address,undefined -fno-omit-frame-pointer \
    -I"${ROOT_DIR}/include" \
    -I"${ROOT_DIR}/src" \
    "${ROOT_DIR}/tests/camera_switch_test.c" \
    "${ROOT_DIR}/src/capture/camera_switch.c" \
    "${ROOT_DIR}/src/capture/frame_mailbox.c" \
    "${ROOT_DIR}/src/control/tracker.c" \
    "${ROOT_DIR}/src/control/tracker_servo.c" \
    "${ROOT_DIR}/src/common/log.c" \
    -lpthread -lm \
    -o "${BIN}"

ASAN_OPTIONS=detect_leaks=0 "${BIN}"
