#!/usr/bin/env bash
set -euo pipefail

ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"
BUILD_DIR="${ROOT_DIR}/build/host-tests"
BIN="${BUILD_DIR}/frmbuf_capture_test"

mkdir -p "${BUILD_DIR}"

gcc -Wall -Wextra -Werror -std=c11 \
    -fsanitize=address,undefined -fno-omit-frame-pointer \
    -I"${ROOT_DIR}/include" \
    -I"${ROOT_DIR}/src" \
    -I"${ROOT_DIR}/third_party/face_accel_ref" \
    "${ROOT_DIR}/tests/frmbuf_capture_test.c" \
    "${ROOT_DIR}/src/common/app_config.c" \
    "${ROOT_DIR}/src/common/log.c" \
    -lm -lpthread \
    -o "${BIN}"

ASAN_OPTIONS=detect_leaks=0 "${BIN}"
