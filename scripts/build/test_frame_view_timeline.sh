#!/bin/sh
set -eu

ROOT_DIR=$(CDPATH= cd -- "$(dirname -- "$0")/../.." && pwd)
BUILD_DIR=${BUILD_DIR:-/tmp/cam_tracker_v2_frame_view_test}
BIN="${BUILD_DIR}/frame_view_timeline_test"

mkdir -p "${BUILD_DIR}"
${CC:-cc} -std=c11 -Wall -Wextra -Werror \
    -I"${ROOT_DIR}/include" -I"${ROOT_DIR}/src" \
    "${ROOT_DIR}/tests/frame_view_timeline_test.c" \
    "${ROOT_DIR}/src/capture/frame_view_timeline.c" \
    -o "${BIN}"
"${BIN}"
