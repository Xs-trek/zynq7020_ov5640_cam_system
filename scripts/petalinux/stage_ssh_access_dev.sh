#!/usr/bin/env bash
set -euo pipefail

ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"
PLNX_PROJECT="${PLNX_PROJECT:-${HOME}/workdir/plnx_prj/zynq_plnx}"
META_USER="${PLNX_PROJECT}/project-spec/meta-user"
RECIPE_DIR="${META_USER}/recipes-apps/ssh-access-dev"
IMAGE_APPEND="${META_USER}/recipes-core/images/petalinux-image-minimal.bbappend"
AUTHORIZED_KEYS_FILE="${CAM_SSH_AUTHORIZED_KEYS_FILE:-}"

if [[ ! -d "${META_USER}" ]]; then
    echo "PetaLinux meta-user layer not found: ${META_USER}" >&2
    exit 1
fi
if [[ -z "${AUTHORIZED_KEYS_FILE}" ]]; then
    echo "CAM_SSH_AUTHORIZED_KEYS_FILE must name an external public-key file" >&2
    exit 1
fi
if [[ ! -s "${AUTHORIZED_KEYS_FILE}" ]]; then
    echo "SSH authorized-keys input is missing or empty: ${AUTHORIZED_KEYS_FILE}" >&2
    exit 1
fi
if grep -q 'PRIVATE KEY' "${AUTHORIZED_KEYS_FILE}"; then
    echo "refusing private-key material: ${AUTHORIZED_KEYS_FILE}" >&2
    exit 1
fi
if ! ssh-keygen -lf "${AUTHORIZED_KEYS_FILE}" >/dev/null 2>&1; then
    echo "invalid OpenSSH public-key file: ${AUTHORIZED_KEYS_FILE}" >&2
    exit 1
fi

install -d "${RECIPE_DIR}/files"
install -m 0644 \
    "${ROOT_DIR}/packaging/petalinux/recipes-apps/ssh-access-dev/ssh-access-dev.bb" \
    "${RECIPE_DIR}/ssh-access-dev.bb"
install -m 0644 \
    "${AUTHORIZED_KEYS_FILE}" \
    "${RECIPE_DIR}/files/authorized_keys"

if [[ ! -f "${IMAGE_APPEND}" ]]; then
    install -d "$(dirname "${IMAGE_APPEND}")"
    : > "${IMAGE_APPEND}"
fi

if ! grep -Eq '(^|[[:space:]])ssh-access-dev([[:space:]]|")' "${IMAGE_APPEND}"; then
    printf '\nIMAGE_INSTALL:append = " ssh-access-dev"\n' >> "${IMAGE_APPEND}"
fi

echo "Staged ssh-access-dev in ${RECIPE_DIR}"
ssh-keygen -lf "${RECIPE_DIR}/files/authorized_keys"
