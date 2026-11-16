#!/usr/bin/env bash
set -euo pipefail

ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"
BUILD_DIR="${ROOT_DIR}/build/host-tests"
BIN="${BUILD_DIR}/net_server_test"

mkdir -p "${BUILD_DIR}/generated/cam_system"
sed -e 's/@PROJECT_VERSION@/0.0.0/; s/@CAM_SYSTEM_GIT_COMMIT@/test/' \
    "${ROOT_DIR}/include/cam_system/version.h.in" > "${BUILD_DIR}/generated/cam_system/version.h"

gcc -Wall -Wextra -Wformat=2 -Werror -std=gnu11 -Wno-unused-parameter -pthread \
    -DHTTP_REQUEST_TIMEOUT_SEC=0.2 \
    -fsanitize=address,undefined -fno-omit-frame-pointer \
    -I"${BUILD_DIR}/generated" -I"${ROOT_DIR}/include" -I"${ROOT_DIR}/src" -I"${ROOT_DIR}/third_party" \
    "${ROOT_DIR}/tests/net_server_test.c" \
    "${ROOT_DIR}/src/network/net_server.c" \
    "${ROOT_DIR}/src/network/http_parse.c" \
    "${ROOT_DIR}/src/network/status_json.c" \
    "${ROOT_DIR}/src/network/mjpeg_framing.c" \
    "${ROOT_DIR}/src/network/ws_protocol.c" \
    "${ROOT_DIR}/src/common/fps_meter.c" \
    "${ROOT_DIR}/src/common/log.c" \
    -lcrypto -o "${BIN}"

"${BIN}"
