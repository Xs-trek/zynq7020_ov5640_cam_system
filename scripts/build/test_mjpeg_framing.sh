#!/usr/bin/env bash
set -euo pipefail

ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"
BUILD_DIR="${ROOT_DIR}/build/host-tests"
BIN="${BUILD_DIR}/mjpeg_framing_test"

mkdir -p "${BUILD_DIR}"

gcc -Wall -Wextra -Werror -std=c11 \
    -I"${ROOT_DIR}/src" -I"${ROOT_DIR}/include" \
    "${ROOT_DIR}/tests/mjpeg_framing_test.c" \
    "${ROOT_DIR}/src/network/mjpeg_framing.c" \
    -o "${BIN}"

"${BIN}"
