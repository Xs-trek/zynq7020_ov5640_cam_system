#!/usr/bin/env bash
set -euo pipefail

ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"
BUILD_DIR="${ROOT_DIR}/build/host-tests"
BIN="${BUILD_DIR}/tracker_closed_loop_test"

mkdir -p "${BUILD_DIR}"

gcc -Wall -Wextra -Werror -std=c11 \
    -I"${ROOT_DIR}/include" \
    -I"${ROOT_DIR}/src" \
    "${ROOT_DIR}/tests/tracker_closed_loop_test.c" \
    "${ROOT_DIR}/src/control/tracker_servo.c" \
    "${ROOT_DIR}/src/control/tracker.c" \
    "${ROOT_DIR}/src/common/log.c" \
    -lpthread -lm \
    -o "${BIN}"

"${BIN}"
