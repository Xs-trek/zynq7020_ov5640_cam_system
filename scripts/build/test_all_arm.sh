#!/usr/bin/env bash
# Runs the tests that depend on the target's arithmetic and data layout on the target's instruction
# set, under user-mode qemu.  The rest of the host suite says nothing more when run here.
#
# Needs the PetaLinux SDK environment: CC, SDKTARGETSYSROOT, OECORE_NATIVE_SYSROOT.
set -euo pipefail

ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"
BUILD_DIR="${ROOT_DIR}/build/arm-tests"
QEMU="${OECORE_NATIVE_SYSROOT}/usr/bin/qemu-arm"
REF_DIR="${ROOT_DIR}/third_party/face_accel_ref"

mkdir -p "${BUILD_DIR}"

checks=0

build()
{
    local name="$1"
    shift
    # CC carries the target flags, so it is split on purpose.
    # shellcheck disable=SC2086
    ${CC} -O2 -Wall -Wextra -Werror -std=gnu11 \
        -I"${ROOT_DIR}/src" -I"${ROOT_DIR}/include" -I"${REF_DIR}" \
        "$@" -lm -lpthread -o "${BUILD_DIR}/${name}"
}

run()
{
    local name="$1"
    shift
    echo "[arm-tests] ${name}"
    "${QEMU}" -L "${SDKTARGETSYSROOT}" "${BUILD_DIR}/${name}" "$@"
    checks=$((checks + 1))
}

# Floating point: the prefilter must not change a single face.
build face_decode_fast_test "${ROOT_DIR}/tests/face_decode_fast_test.c" \
    "${ROOT_DIR}/src/detect/face_decode_fast.c" "${REF_DIR}/ultraface_decode.c"
run face_decode_fast_test "${ROOT_DIR}/tests/data/face_accel_vectors"

# Data layout: the driver interface as a 32-bit client sees it.
build face_accel_client_test "${ROOT_DIR}/tests/face_accel_client_test.c" \
    "${ROOT_DIR}/src/detect/face_decode_fast.c" "${REF_DIR}/ultraface_decode.c" \
    "${ROOT_DIR}/src/common/log.c"
run face_accel_client_test

# Sequence arithmetic, double-precision time and 32-bit atomics.
build control_tick_test "${ROOT_DIR}/tests/control_tick_test.c" "${ROOT_DIR}/src/capture/control_tick.c"
run control_tick_test
build frame_mailbox_test "${ROOT_DIR}/tests/frame_mailbox_test.c" "${ROOT_DIR}/src/capture/frame_mailbox.c"
run frame_mailbox_test
build detector_recovery_test "${ROOT_DIR}/tests/detector_recovery_test.c" \
    "${ROOT_DIR}/src/detect/detector_recovery.c"
run detector_recovery_test
build capture_recovery_test "${ROOT_DIR}/tests/capture_recovery_test.c" \
    "${ROOT_DIR}/src/capture/capture_recovery.c"
run capture_recovery_test
# 32-bit long: the run totals of the bridge counters.
build dvp_counts_test "${ROOT_DIR}/tests/dvp_counts_test.c" "${ROOT_DIR}/src/capture/dvp_counts.c"
run dvp_counts_test

echo "arm_test_suite: OK (${checks} checks)"
