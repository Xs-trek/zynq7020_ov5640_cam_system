#!/usr/bin/env bash
set -euo pipefail

ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"
BUILD_DIR="${ROOT_DIR}/build/host-tests"
BIN="${BUILD_DIR}/jpeg_worker_test"

mkdir -p "${BUILD_DIR}"

gcc -Wall -Wextra -Werror -std=c11 \
    -I"${ROOT_DIR}/include" \
    -I"${ROOT_DIR}/src" \
    "${ROOT_DIR}/tests/jpeg_worker_test.c" \
    "${ROOT_DIR}/src/capture/frame_mailbox.c" \
    "${ROOT_DIR}/src/media/jpeg_worker.c" \
    "${ROOT_DIR}/src/media/stream_rate.c" \
    "${ROOT_DIR}/src/common/log.c" \
    -lpthread \
    -o "${BIN}"

"${BIN}"
