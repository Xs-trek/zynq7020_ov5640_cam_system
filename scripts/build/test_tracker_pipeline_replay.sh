#!/usr/bin/env bash
set -euo pipefail

ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"
BUILD_DIR="${ROOT_DIR}/build/host-tests/pipeline-replay"
BIN="${BUILD_DIR}/tracker_pipeline_replay_test"

mkdir -p "${BUILD_DIR}"

gcc -Wall -Wextra -Werror -std=c11 \
    -I"${ROOT_DIR}/include" \
    -I"${ROOT_DIR}/src" \
    -c "${ROOT_DIR}/src/control/tracker.c" \
    -o "${BUILD_DIR}/tracker.o"
gcc -Wall -Wextra -Werror -std=c11 \
    -I"${ROOT_DIR}/include" \
    -I"${ROOT_DIR}/src" \
    -c "${ROOT_DIR}/src/control/tracker_servo.c" \
    -o "${BUILD_DIR}/tracker_servo.o"
gcc -Wall -Wextra -Werror -std=c11 \
    -I"${ROOT_DIR}/src" \
    -c "${ROOT_DIR}/src/common/log.c" \
    -o "${BUILD_DIR}/log.o"
g++ -Wall -Wextra -Werror -std=c++11 \
    -I"${ROOT_DIR}/include" \
    -I"${ROOT_DIR}/src" \
    "${ROOT_DIR}/tests/tracker_pipeline_replay_test.cpp" \
    "${ROOT_DIR}/src/vision/face_temporal_tracker.cpp" \
    "${BUILD_DIR}/tracker.o" \
    "${BUILD_DIR}/tracker_servo.o" \
    "${BUILD_DIR}/log.o" \
    -lpthread -lm \
    -o "${BIN}"

"${BIN}"
