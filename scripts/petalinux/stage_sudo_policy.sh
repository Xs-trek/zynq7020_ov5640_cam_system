#!/usr/bin/env bash
set -euo pipefail

ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"
PLNX_PROJECT="${PLNX_PROJECT:-${HOME}/workdir/plnx_prj/zynq_plnx}"
META_USER="${PLNX_PROJECT}/project-spec/meta-user"
SRC_DIR="${ROOT_DIR}/packaging/petalinux/recipes-apps/sudo-policy"
RECIPE_DIR="${META_USER}/recipes-apps/sudo-policy"

if [[ ! -d "${META_USER}" ]]; then
    echo "PetaLinux meta-user layer not found: ${META_USER}" >&2
    exit 1
fi

"${ROOT_DIR}/scripts/build/validate_sudo_policy.sh"

install -d "${RECIPE_DIR}/files"
install -m 0644 "${SRC_DIR}/sudo-policy.bb" "${RECIPE_DIR}/sudo-policy.bb"
install -m 0644 "${SRC_DIR}/files/00-secure-path" "${RECIPE_DIR}/files/00-secure-path"

image_append="${META_USER}/recipes-core/images/petalinux-image-minimal.bbappend"
if [[ -f "${image_append}" ]] && ! grep -Eq '(^|[[:space:]])sudo-policy([[:space:]]|")' "${image_append}"; then
    printf '\nIMAGE_INSTALL:append = " sudo-policy"\n' >> "${image_append}"
fi

echo "Staged sudo-policy in ${RECIPE_DIR}"
