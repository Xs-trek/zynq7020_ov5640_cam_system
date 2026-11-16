#!/usr/bin/env bash
set -euo pipefail

ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"
BUILD_DIR="${ROOT_DIR}/build/host-tests"
BIN="${BUILD_DIR}/face_accel_diagnostics_test"

mkdir -p "${BUILD_DIR}"

gcc -Wall -Wextra -Werror -std=c11 \
    -I"${ROOT_DIR}/third_party/face_accel_ref" \
    "${ROOT_DIR}/tests/face_accel_diagnostics_test.c" \
    "${ROOT_DIR}/third_party/face_accel_ref/ultraface_decode.c" \
    -lm \
    -o "${BIN}"

"${BIN}"
