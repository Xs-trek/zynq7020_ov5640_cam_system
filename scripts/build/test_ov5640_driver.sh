#!/usr/bin/env bash
set -euo pipefail

ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"
BUILD_DIR="${ROOT_DIR}/build/host-tests"
SHIM_DIR="${BUILD_DIR}/kshim_v4l2"
DRIVER_DIR="${ROOT_DIR}/packaging/petalinux/recipes-kernel/cam-ov5640-control/files"
BIN="${BUILD_DIR}/ov5640_driver_test"

mkdir -p "${SHIM_DIR}/linux/gpio" "${SHIM_DIR}/media"

# The driver's kernel includes are empty here: tests/kshim/kshim_v4l2.h is all of the kernel it
# sees.  The kernel builds with -Wno-pointer-sign, and i2c_master_send takes char.
sed -n 's/^#include <\(linux\|media\)\/\(.*\)>$/\1\/\2/p' "${DRIVER_DIR}/cam-ov5640-control.c" |
    while read -r header; do
        : > "${SHIM_DIR}/${header}"
    done

gcc -Wall -Wextra -Wformat=2 -Werror -std=gnu11 \
    -Wno-unused-parameter -Wno-sign-compare -Wno-pointer-sign \
    -fsanitize=address,undefined -fno-omit-frame-pointer \
    -include "${ROOT_DIR}/tests/kshim/kshim_v4l2.h" \
    -I"${SHIM_DIR}" \
    -I"${DRIVER_DIR}" \
    "${ROOT_DIR}/tests/ov5640_driver_test.c" \
    -o "${BIN}"

ASAN_OPTIONS=detect_leaks=1 "${BIN}"
