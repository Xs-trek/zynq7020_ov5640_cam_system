#!/usr/bin/env bash
set -euo pipefail

ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"
BUILD_DIR="${ROOT_DIR}/build/host-tests"
BIN="${BUILD_DIR}/face_accel_client_test"

mkdir -p "${BUILD_DIR}"

gcc -Wall -Wextra -Wformat=2 -Werror -std=gnu11 \
    -fsanitize=address,undefined -fno-omit-frame-pointer \
    -I"${ROOT_DIR}/src" \
    -I"${ROOT_DIR}/include" \
    -I"${ROOT_DIR}/third_party/face_accel_ref" \
    "${ROOT_DIR}/tests/face_accel_client_test.c" \
    "${ROOT_DIR}/src/detect/face_decode_fast.c" \
    "${ROOT_DIR}/third_party/face_accel_ref/ultraface_decode.c" \
    "${ROOT_DIR}/src/common/log.c" \
    -lm -lpthread \
    -o "${BIN}"

ASAN_OPTIONS=detect_leaks=1 "${BIN}"
