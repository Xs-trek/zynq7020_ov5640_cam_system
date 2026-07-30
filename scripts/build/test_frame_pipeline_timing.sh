#!/usr/bin/env bash
set -euo pipefail

ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"
BUILD_DIR="${ROOT_DIR}/build/host-tests"
BIN="${BUILD_DIR}/frame_pipeline_timing_test"

mkdir -p "${BUILD_DIR}"

gcc -Wall -Wextra -Werror -std=c11 \
    -fsanitize=address,undefined -fno-omit-frame-pointer \
    -I"${ROOT_DIR}/include" \
    -I"${ROOT_DIR}/src" \
    "${ROOT_DIR}/tests/frame_pipeline_timing_test.c" \
    "${ROOT_DIR}/src/capture/frame_mailbox.c" \
    -lpthread \
    -o "${BIN}"

ASAN_OPTIONS=detect_leaks=0 "${BIN}"
