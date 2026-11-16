#!/usr/bin/env bash
set -euo pipefail

ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"
BUILD_DIR="${ROOT_DIR}/build/host-tests"

mkdir -p "${BUILD_DIR}"

# Product build (PL backend only) and CPU-detector build accept different backend names.
for variant in product cpu_detectors; do
    BIN="${BUILD_DIR}/app_config_test_${variant}"
    extra=()
    if [[ "${variant}" == "cpu_detectors" ]]; then
        extra=(-DCAM_SYSTEM_ENABLE_CPU_DETECTORS)
    fi
    gcc -Wall -Wextra -Wformat=2 -Werror -std=c11 \
        -fsanitize=address,undefined -fno-omit-frame-pointer \
        "${extra[@]}" \
        -I"${ROOT_DIR}/include" \
        -I"${ROOT_DIR}/src" \
        -I"${ROOT_DIR}/third_party/face_accel_ref" \
        "${ROOT_DIR}/tests/app_config_test.c" \
        "${ROOT_DIR}/src/common/app_config.c" \
        -lm \
        -o "${BIN}"
    ASAN_OPTIONS=detect_leaks=0 "${BIN}" "${ROOT_DIR}/config/cam_system.conf"
done
