#!/usr/bin/env bash
set -euo pipefail

ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"
BUILD_DIR="${ROOT_DIR}/build/host-tests"
BIN="${BUILD_DIR}/face_decode_fast_test"

mkdir -p "${BUILD_DIR}"

gcc -O2 -Wall -Wextra -Wformat=2 -Werror -std=c11 \
    -fsanitize=address,undefined -fno-omit-frame-pointer \
    -I"${ROOT_DIR}/src" \
    -I"${ROOT_DIR}/third_party/face_accel_ref" \
    "${ROOT_DIR}/tests/face_decode_fast_test.c" \
    "${ROOT_DIR}/src/detect/face_decode_fast.c" \
    "${ROOT_DIR}/third_party/face_accel_ref/ultraface_decode.c" \
    -lm \
    -o "${BIN}"

ASAN_OPTIONS=detect_leaks=0 "${BIN}" "${ROOT_DIR}/tests/data/face_accel_vectors"
