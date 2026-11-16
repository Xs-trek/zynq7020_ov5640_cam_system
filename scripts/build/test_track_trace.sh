#!/usr/bin/env bash
set -euo pipefail

ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"
BUILD_DIR="${ROOT_DIR}/build/host-tests"
BIN="${BUILD_DIR}/track_trace_test"

mkdir -p "${BUILD_DIR}"

gcc -Wall -Wextra -Wformat=2 -Werror -std=c11 \
    -fsanitize=address,undefined -fno-omit-frame-pointer \
    -I"${ROOT_DIR}/include" \
    -I"${ROOT_DIR}/src" \
    "${ROOT_DIR}/tests/track_trace_test.c" \
    "${ROOT_DIR}/src/control/track_trace.c" \
    "${ROOT_DIR}/src/common/log.c" \
    -lpthread \
    -o "${BIN}"

ASAN_OPTIONS=detect_leaks=0 "${BIN}"
