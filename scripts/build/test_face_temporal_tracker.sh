#!/usr/bin/env bash
set -euo pipefail

ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"
BUILD_DIR="${ROOT_DIR}/build/host-tests"
BIN="${BUILD_DIR}/face_temporal_tracker_test"

mkdir -p "${BUILD_DIR}"

g++ -Wall -Wextra -Werror -std=c++11 \
    -I"${ROOT_DIR}/include" \
    -I"${ROOT_DIR}/src" \
    "${ROOT_DIR}/tests/face_temporal_tracker_test.cpp" \
    "${ROOT_DIR}/src/vision/face_temporal_tracker.cpp" \
    -o "${BIN}"

"${BIN}"
