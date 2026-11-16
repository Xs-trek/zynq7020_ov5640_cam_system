#!/usr/bin/env bash
# Checks the sudoers fragment before it goes into an image: a syntax error in /etc/sudoers.d
# disables sudo altogether, and the board has no other way to become root.
set -euo pipefail

ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"
RECIPE_DIR="${ROOT_DIR}/packaging/petalinux/recipes-apps/sudo-policy"
FRAGMENT="${RECIPE_DIR}/files/00-secure-path"
RECIPE="${RECIPE_DIR}/sudo-policy.bb"

fail()
{
    echo "validate_sudo_policy: $*" >&2
    exit 1
}

[[ -f "${FRAGMENT}" ]] || fail "missing ${FRAGMENT}"
[[ -f "${RECIPE}" ]] || fail "missing ${RECIPE}"

# Only the one setting: user rules do not belong here.
rules="$(grep -Ev '^[[:space:]]*(#|$)' "${FRAGMENT}")"
[[ "${rules}" == 'Defaults secure_path="/usr/sbin:/usr/bin:/sbin:/bin"' ]] ||
    fail "the fragment must hold only the secure_path default, found: ${rules}"

# sudo skips files in sudoers.d whose names contain a dot or end in a tilde.
case "$(basename "${FRAGMENT}")" in
    *.*|*~) fail "sudo would ignore $(basename "${FRAGMENT}")" ;;
esac

grep -Fq 'install -m 0440 ${WORKDIR}/00-secure-path ${D}${sysconfdir}/sudoers.d/00-secure-path' \
    "${RECIPE}" || fail "the fragment must be installed with mode 0440"
grep -Fq 'RDEPENDS:${PN} += "sudo"' "${RECIPE}" || fail "missing the sudo runtime dependency"

visudo_bin="$(PATH="${PATH}:/usr/sbin:/sbin" command -v visudo || true)"
if [[ -n "${visudo_bin}" ]]; then
    tmp="$(mktemp -d)"
    trap 'rm -rf "${tmp}"' EXIT
    install -m 0440 "${FRAGMENT}" "${tmp}/00-secure-path"
    "${visudo_bin}" -c -q -f "${tmp}/00-secure-path" > /dev/null ||
        fail "visudo rejects ${FRAGMENT}"
else
    echo "validate_sudo_policy: visudo not found; syntax not checked" >&2
fi

echo "validate_sudo_policy: OK"
