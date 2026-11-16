#!/usr/bin/env bash
# Builds a complete boot image from this commit, in a PetaLinux project of its own.
#
#   scripts/petalinux/build_image.sh XSA [PROJECT]
#
# The baseline project is only read: its configuration and recipes are copied, its build tree is
# not, and downloads and shared state come through its mirrors.  PROJECT defaults to
# ~/workdir/plnx_prj/cam_system_image; a second run reuses it, and a build tree it already has.
set -euo pipefail

ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"
XSA="${1:?usage: build_image.sh XSA [PROJECT]}"
PROJECT="${2:-${HOME}/workdir/plnx_prj/cam_system_image}"
BASE_PROJECT="${PLNX_BASE_PROJECT:-${HOME}/workdir/plnx_prj/zynq_plnx}"
PETALINUX_SETTINGS="${PETALINUX_SETTINGS:-/opt/pkg/petalinux/settings.sh}"

if [[ ! -f "${XSA}" ]]; then
    echo "XSA not found: ${XSA}" >&2
    exit 1
fi
if [[ "$(realpath -m "${PROJECT}")" == "$(realpath -m "${BASE_PROJECT}")" ]]; then
    echo "refusing to build in the baseline project: ${BASE_PROJECT}" >&2
    exit 1
fi

if [[ ! -d "${PROJECT}/project-spec" ]]; then
    install -d "${PROJECT}"
    rsync -a --exclude=/build --exclude=/images --exclude=/components \
        --exclude='/*.xsa' "${BASE_PROJECT}/" "${PROJECT}/"
    # The toolchain extracted under components/ belongs to its project; the tools extract their
    # own when the metadata does not name one.
    sed -i '/^YOCTO_SDK=/d' "${PROJECT}/.petalinux/metadata"
    rm -rf "${PROJECT}/project-spec/meta-user/recipes-apps/cam-system/files"
fi

export PLNX_PROJECT="${PROJECT}"
"${ROOT_DIR}/scripts/petalinux/stage_cam_system.sh"
"${ROOT_DIR}/scripts/petalinux/stage_face_accel.sh"
"${ROOT_DIR}/scripts/petalinux/stage_ov5640_control.sh"
"${ROOT_DIR}/scripts/petalinux/stage_cam_launcher.sh"
"${ROOT_DIR}/scripts/petalinux/stage_wifi.sh"
"${ROOT_DIR}/scripts/petalinux/stage_ota.sh"
"${ROOT_DIR}/scripts/petalinux/stage_sudo_policy.sh"

HW_DIR="${PROJECT}/hardware"
rm -rf "${HW_DIR}"
install -d "${HW_DIR}"
install -m 0644 "${XSA}" "${HW_DIR}/system.xsa"

# settings.sh takes the installation path from its arguments, and a sourced script sees ours.
set --
set +u
# shellcheck disable=SC1090
source "${PETALINUX_SETTINGS}" > /dev/null
set -u

petalinux-config -p "${PROJECT}" --get-hw-description="${HW_DIR}" --silentconfig
petalinux-build -p "${PROJECT}"
(cd "${PROJECT}" && petalinux-package --boot --force --fsbl --u-boot --fpga images/linux/system.bit)

echo "Image built in ${PROJECT}/images/linux"
sha256sum "${PROJECT}/images/linux/BOOT.BIN" "${PROJECT}/images/linux/image.ub"
