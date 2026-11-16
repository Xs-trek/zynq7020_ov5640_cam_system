#!/usr/bin/env bash
set -euo pipefail

ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"
PLNX_PROJECT="${PLNX_PROJECT:-${HOME}/workdir/plnx_prj/zynq_plnx}"
RECIPE_DIR="${PLNX_PROJECT}/project-spec/meta-user/recipes-apps/cam-system"
IMAGE_APPEND="${PLNX_PROJECT}/project-spec/meta-user/recipes-core/images/petalinux-image-minimal.bbappend"
TEMPLATE_RECIPE="${ROOT_DIR}/packaging/petalinux/recipes-apps/cam-system/cam-system.bb"
MODEL_DIR="${ROOT_DIR}/assets/models"
SOURCE_ARCHIVE="${RECIPE_DIR}/files/cam_system-src.tar.gz"

if [[ ! -d "${PLNX_PROJECT}/project-spec/meta-user" ]]; then
    echo "PetaLinux project not found or missing meta-user: ${PLNX_PROJECT}" >&2
    exit 1
fi

if [[ -n "$(git -C "${ROOT_DIR}" status --porcelain)" ]]; then
    echo "cam_system git worktree is dirty; commit or stash changes before staging a source recipe." >&2
    exit 1
fi

git_commit="$(git -C "${ROOT_DIR}" rev-parse HEAD)"

install -d "${RECIPE_DIR}/files"
install -m 0644 "${TEMPLATE_RECIPE}" "${RECIPE_DIR}/cam-system.bb"
install -m 0755 "${ROOT_DIR}/scripts/build/verify_elf.sh" "${RECIPE_DIR}/files/verify_elf.sh"

rm -f "${RECIPE_DIR}/files/cam_system" \
    "${RECIPE_DIR}/files/yolov6lite_s.ncnn.param" \
    "${RECIPE_DIR}/files/yolov6lite_s.ncnn.bin"
rm -f "${RECIPE_DIR}/files/libturbojpeg.so.0.4.0" \
    "${RECIPE_DIR}/files/libjpeg.so.62.4.0"

git -C "${ROOT_DIR}" archive --format=tar --prefix=cam_system-src/ HEAD -- \
    CMakeLists.txt \
    assets \
    cmake \
    config \
    include \
    src \
    third_party/cjson \
    third_party/libfacedetection \
    third_party/face_accel_ref \
    third_party/ultraface | gzip -n > "${SOURCE_ARCHIVE}"

{
    printf 'git_commit %s\n' "${git_commit}"
    printf 'source_archive_sha256 %s\n' "$(sha256sum "${SOURCE_ARCHIVE}" | awk '{print $1}')"
    find "${MODEL_DIR}" -type f -print0 |
        sort -z |
        xargs -0 sha256sum |
        sed "s#${ROOT_DIR}/##"
} > "${RECIPE_DIR}/files/cam-system-artifacts.sha256"

if [[ -f "${IMAGE_APPEND}" ]] && ! grep -Eq '(^|[[:space:]])cam-system([[:space:]]|")' "${IMAGE_APPEND}"; then
    printf '\nIMAGE_INSTALL:append = " cam-system"\n' >> "${IMAGE_APPEND}"
fi

echo "Staged cam-system source recipe artifacts in ${RECIPE_DIR}"
sha256sum "${SOURCE_ARCHIVE}"
