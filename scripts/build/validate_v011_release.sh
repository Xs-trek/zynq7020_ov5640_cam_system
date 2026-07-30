#!/usr/bin/env bash
set -euo pipefail

usage()
{
    cat <<EOF
Usage: $0 IMAGE_DIR [ROOTFS_DIR]

IMAGE_DIR must contain BOOT.BIN, image.ub, system.dtb and rootfs.manifest.
ROOTFS_DIR is optional; when supplied, installed app/module/firmware contents are
also checked.  A release-manifest.env in IMAGE_DIR is verified when present.
EOF
}

[[ $# -ge 1 && $# -le 2 ]] || { usage >&2; exit 2; }
IMAGE_DIR=$(realpath "$1")
ROOTFS_DIR=""
if [[ $# -eq 2 ]]; then
    ROOTFS_DIR=$(realpath "$2")
fi

fail()
{
    echo "validate_v011_release: ERROR: $*" >&2
    exit 1
}

for name in BOOT.BIN image.ub system.dtb rootfs.manifest; do
    [[ -s "${IMAGE_DIR}/${name}" ]] || fail "missing ${IMAGE_DIR}/${name}"
done

grep -Eq '^cam-system([[:space:]]|$)' "${IMAGE_DIR}/rootfs.manifest" ||
    fail "rootfs.manifest does not contain cam-system"
grep -Eq '^cam-face-accel([[:space:]]|$)' "${IMAGE_DIR}/rootfs.manifest" ||
    fail "rootfs.manifest does not contain cam-face-accel"
grep -Eq '^cam-launcher([[:space:]]|$)' "${IMAGE_DIR}/rootfs.manifest" ||
    fail "rootfs.manifest does not contain cam-launcher"

TMP_DIR=$(mktemp -d)
trap 'rm -rf "${TMP_DIR}"' EXIT

if [[ -z "${ROOTFS_DIR}" ]]; then
    [[ -s "${IMAGE_DIR}/rootfs.cpio" ]] ||
        fail "pass ROOTFS_DIR or provide ${IMAGE_DIR}/rootfs.cpio for content validation"
    command -v cpio >/dev/null 2>&1 || fail "cpio is required"
    ROOTFS_DIR="${TMP_DIR}/rootfs"
    mkdir -p "${ROOTFS_DIR}"
    (cd "${ROOTFS_DIR}" && cpio -idm --quiet) < "${IMAGE_DIR}/rootfs.cpio"
fi

command -v dtc >/dev/null 2>&1 || fail "dtc is required"
DT_DUMP="${TMP_DIR}/system.dts"
dtc -q -I dtb -O dts -o "${DT_DUMP}" "${IMAGE_DIR}/system.dtb"
grep -q 'face_accel@43c20000' "${DT_DUMP}" ||
    fail "system.dtb has no face_accel@43c20000 node"
grep -q 'xlnx,face-accel-1.0' "${DT_DUMP}" ||
    fail "system.dtb has no xlnx,face-accel-1.0 compatible"

APP="${ROOTFS_DIR}/usr/bin/cam_system"
[[ -x "${APP}" ]] || fail "rootfs has no executable /usr/bin/cam_system"
APP_STRINGS="${TMP_DIR}/cam_system.strings"
strings "${APP}" > "${APP_STRINGS}"
grep -q 'face_accel' "${APP_STRINGS}" ||
    fail "rootfs cam_system lacks the PL face detector backend"
grep -q 'CAM_FACE_CONFIRM_FRAMES' "${APP_STRINGS}" ||
    fail "rootfs cam_system lacks temporal confirmation support"
grep -q 'CAM_FACE_TRACK_SCORE_THRESH' "${APP_STRINGS}" ||
    fail "rootfs cam_system lacks tracker-only continuation threshold"
grep -q 'Init timestamped-kf' "${APP_STRINGS}" ||
    fail "rootfs cam_system lacks timestamped Kalman tracker"
if grep -q 'CAM_OPTICAL_FLOW_MODE' "${APP_STRINGS}"; then
    fail "rootfs cam_system still contains retired optical-flow control"
fi
find "${ROOTFS_DIR}/lib/modules" -type f -name 'cam-face-accel.ko' \
    -print -quit | grep -q . || fail "rootfs has no cam-face-accel.ko"
[[ -s "${ROOTFS_DIR}/lib/firmware/cam-system/face-accel/unified128_dw32_weights_v1.bin" ]] ||
    fail "rootfs has no face accelerator weight firmware"

MANIFEST="${IMAGE_DIR}/release-manifest.env"
verify_manifest_hash()
{
    local key="$1"
    local file="$2"
    local expected
    expected=$(sed -n "s/^${key}=//p" "${MANIFEST}" | tail -n 1)
    [[ -n "${expected}" ]] || fail "${MANIFEST} lacks ${key}"
    local actual
    actual=$(sha256sum "${file}" | awk '{print $1}')
    [[ "${actual}" == "${expected}" ]] ||
        fail "${key} mismatch: manifest=${expected} actual=${actual}"
}

if [[ -f "${MANIFEST}" ]]; then
    verify_manifest_hash IMAGE_UB_SHA256 "${IMAGE_DIR}/image.ub"
    verify_manifest_hash SYSTEM_DTB_SHA256 "${IMAGE_DIR}/system.dtb"
    verify_manifest_hash BOOT_BIN_SHA256 "${IMAGE_DIR}/BOOT.BIN"
    grep -q '^FACE_ACCEL_IP_ID=0xFACE0004$' "${MANIFEST}" ||
        fail "release manifest has the wrong accelerator IP_ID"
fi

echo "validate_v011_release: OK"
sha256sum "${IMAGE_DIR}/BOOT.BIN" "${IMAGE_DIR}/image.ub" \
          "${IMAGE_DIR}/system.dtb"
