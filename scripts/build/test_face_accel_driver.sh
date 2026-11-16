#!/usr/bin/env bash
set -euo pipefail

ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"
BUILD_DIR="${ROOT_DIR}/build/host-tests"
SHIM_DIR="${BUILD_DIR}/kshim"
DRIVER_DIR="${ROOT_DIR}/packaging/petalinux/recipes-kernel/cam-face-accel/files"
BIN="${BUILD_DIR}/face_accel_driver_test"

mkdir -p "${SHIM_DIR}/linux" "${SHIM_DIR}/crypto" "${SHIM_DIR}/asm"

# The driver's kernel includes are empty here: tests/kshim/kshim.h is all of the kernel it sees.
# The UAPI header includes the host's <linux/ioctl.h> and <linux/types.h>, which stay as they are.
sed -n 's/^#include <\(linux\|crypto\|asm\)\/\(.*\)>$/\1\/\2/p' "${DRIVER_DIR}/cam-face-accel.c" |
    while read -r header; do
        : > "${SHIM_DIR}/${header}"
    done
ln -sf "${ROOT_DIR}/include/cam_system/face_accel_uapi.h" "${SHIM_DIR}/cam_face_accel_uapi.h"

gcc -Wall -Wextra -Wformat=2 -Werror -std=gnu11 \
    -Wno-unused-parameter -Wno-unused-function -Wno-sign-compare \
    -fsanitize=address,undefined -fno-omit-frame-pointer \
    -include "${ROOT_DIR}/tests/kshim/kshim.h" \
    -I"${SHIM_DIR}" \
    -I"${DRIVER_DIR}" \
    -I"${ROOT_DIR}/third_party/face_accel_ref" \
    "${ROOT_DIR}/tests/face_accel_driver_test.c" \
    -o "${BIN}"

ASAN_OPTIONS=detect_leaks=1 "${BIN}"
