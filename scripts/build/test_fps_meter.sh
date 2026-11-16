#!/usr/bin/env bash
set -euo pipefail

ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"
BUILD_DIR="${ROOT_DIR}/build/host-tests"
BIN="${BUILD_DIR}/fps_meter_test"

mkdir -p "${BUILD_DIR}"

gcc -Wall -Wextra -Wformat=2 -Werror -std=c11 \
    -fsanitize=address,undefined -fno-omit-frame-pointer \
    -I"${ROOT_DIR}/src" \
    "${ROOT_DIR}/tests/fps_meter_test.c" \
    "${ROOT_DIR}/src/common/fps_meter.c" \
    -lm \
    -o "${BIN}"

ASAN_OPTIONS=detect_leaks=0 "${BIN}"
