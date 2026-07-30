#!/usr/bin/env bash
set -euo pipefail

ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"
STAGE_SCRIPT="${ROOT_DIR}/scripts/petalinux/stage_ssh_access_dev.sh"
RECIPE_KEY="${ROOT_DIR}/packaging/petalinux/recipes-apps/ssh-access-dev/files/authorized_keys"
TMP_DIR="$(mktemp -d)"
trap 'rm -rf "${TMP_DIR}"' EXIT

fail()
{
    echo "validate_ssh_access_dev: ERROR: $*" >&2
    exit 1
}

[[ ! -e "${RECIPE_KEY}" ]] || fail "repository contains authorized_keys"
if git -C "${ROOT_DIR}" grep -nE \
    'ssh-(rsa|ed25519)[[:space:]]+[A-Za-z0-9+/]{40,}|BEGIN (OPENSSH |RSA |EC )?PRIVATE KEY|\$6\$[A-Za-z0-9./]{16,}\$' \
    -- ':!logs/**' ':!packaging/petalinux/recipes-apps/ssh-access-dev/files/README.md'; then
    fail "repository contains embedded SSH identity or password-hash policy"
fi

PROJECT_DIR="${TMP_DIR}/project"
META_USER="${PROJECT_DIR}/project-spec/meta-user"
install -d "${META_USER}/recipes-core/images"
: > "${META_USER}/recipes-core/images/petalinux-image-minimal.bbappend"

if PLNX_PROJECT="${PROJECT_DIR}" "${STAGE_SCRIPT}" >/dev/null 2>&1; then
    fail "staging succeeded without explicit key input"
fi

ssh-keygen -q -t ed25519 -N '' -f "${TMP_DIR}/temporary_test_key"
CAM_SSH_AUTHORIZED_KEYS_FILE="${TMP_DIR}/temporary_test_key.pub" \
PLNX_PROJECT="${PROJECT_DIR}" "${STAGE_SCRIPT}" >/dev/null

INJECTED_KEY="${META_USER}/recipes-apps/ssh-access-dev/files/authorized_keys"
IMAGE_APPEND="${META_USER}/recipes-core/images/petalinux-image-minimal.bbappend"
cmp -s "${TMP_DIR}/temporary_test_key.pub" "${INJECTED_KEY}" ||
    fail "staged key differs from explicit input"
grep -Eq '(^|[[:space:]])ssh-access-dev([[:space:]]|")' "${IMAGE_APPEND}" ||
    fail "ssh-access-dev is absent from IMAGE_INSTALL"
grep -q '^EXTRA_USERS_SUDOERS:append = " debug ALL=(ALL) NOPASSWD: ALL;"$' \
    "${IMAGE_APPEND}" || fail "debug sudo policy is absent"
if grep -Eq 'DEBUG_USER_PASSWORD_HASH|set_debug_user_password_hash' "${IMAGE_APPEND}"; then
    fail "generated image policy contains a fixed password hash"
fi

echo "validate_ssh_access_dev: OK (external key injection, password locked)"
