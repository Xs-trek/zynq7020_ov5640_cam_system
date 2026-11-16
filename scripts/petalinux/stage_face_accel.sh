#!/usr/bin/env bash
set -euo pipefail

ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"
PLNX_PROJECT="${PLNX_PROJECT:-${HOME}/workdir/plnx_prj/zynq_plnx}"
META_USER="${PLNX_PROJECT}/project-spec/meta-user"
RECIPE_SRC="${ROOT_DIR}/packaging/petalinux/recipes-kernel/cam-face-accel"
RECIPE_DST="${META_USER}/recipes-kernel/cam-face-accel"
IMAGE_APPEND="${META_USER}/recipes-core/images/petalinux-image-minimal.bbappend"
UAPI_SRC="${ROOT_DIR}/include/cam_system/face_accel_uapi.h"
REGS_SRC="${ROOT_DIR}/third_party/face_accel_ref/face_accel_regs.h"
WEIGHT_SRC="${ROOT_DIR}/assets/models/face_accel/unified128_dw32_weights_v1.bin"
EXPECTED_WEIGHT_SHA256="93ae933897bb510a2a47a34a7ee27249a66904cc09964a1938187891239cbfab"

if [[ ! -d "${META_USER}" ]]; then
    echo "PetaLinux project not found or missing meta-user: ${PLNX_PROJECT}" >&2
    exit 1
fi

"${ROOT_DIR}/scripts/build/verify_face_accel_contract.py"

actual_weight_sha256="$(sha256sum "${WEIGHT_SRC}" | awk '{print $1}')"
if [[ "${actual_weight_sha256}" != "${EXPECTED_WEIGHT_SHA256}" ]]; then
    echo "R9 weight SHA-256 mismatch: ${actual_weight_sha256}" >&2
    exit 1
fi
if [[ "$(stat -c %s "${WEIGHT_SRC}")" != "358400" ]]; then
    echo "R9 weight size is not 358400 bytes" >&2
    exit 1
fi

install -d "${RECIPE_DST}/files"
rm -f "${RECIPE_DST}"/cam-face-accel_*.bb
install -m 0644 "${RECIPE_SRC}/cam-face-accel_0.2.0.bb" \
    "${RECIPE_DST}/cam-face-accel_0.2.0.bb"
install -m 0644 "${RECIPE_SRC}/files/Makefile" "${RECIPE_DST}/files/Makefile"
install -m 0644 "${RECIPE_SRC}/files/cam-face-accel.c" \
    "${RECIPE_DST}/files/cam-face-accel.c"
install -m 0644 "${UAPI_SRC}" "${RECIPE_DST}/files/cam_face_accel_uapi.h"
install -m 0644 "${REGS_SRC}" "${RECIPE_DST}/files/face_accel_regs.h"
install -m 0644 "${WEIGHT_SRC}" \
    "${RECIPE_DST}/files/unified128_dw32_weights_v1.bin"

install -d "$(dirname "${IMAGE_APPEND}")"
touch "${IMAGE_APPEND}"
if ! grep -Eq '(^|[[:space:]])cam-face-accel([[:space:]]|\")' "${IMAGE_APPEND}"; then
    printf '\nIMAGE_INSTALL:append = " cam-face-accel"\n' >> "${IMAGE_APPEND}"
fi

echo "Staged face accelerator recipe in ${RECIPE_DST}"
sha256sum "${RECIPE_DST}/files/cam-face-accel.c" \
          "${RECIPE_DST}/files/cam_face_accel_uapi.h" \
          "${RECIPE_DST}/files/unified128_dw32_weights_v1.bin"
