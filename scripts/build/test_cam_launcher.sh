#!/usr/bin/env bash
set -euo pipefail

ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"
BUILD_DIR="${ROOT_DIR}/build/host-tests"
BIN="${BUILD_DIR}/cam_launcher_test"
FILES_DIR="${ROOT_DIR}/packaging/petalinux/recipes-apps/cam-launcher/files"

mkdir -p "${BUILD_DIR}"

gcc -Wall -Wextra -Wformat=2 -Werror -std=c11 -D_GNU_SOURCE \
    -fsanitize=address,undefined -fno-omit-frame-pointer \
    -I"${FILES_DIR}" \
    "${ROOT_DIR}/tests/cam_launcher_test.c" \
    -o "${BIN}"

"${BIN}"

# The init script hands /etc/default/cam-launcher to the daemon.  The daemon here records what it
# was given and exits.
tmp="$(mktemp -d)"
trap 'rm -rf "${tmp}"' EXIT

cat > "${tmp}/daemon" <<SCRIPT
#!/bin/sh
echo "autostart=\${CAM_LAUNCHER_AUTOSTART-unset}" > "${tmp}/seen"
SCRIPT
chmod +x "${tmp}/daemon"

run_init()
{
    rm -f "${tmp}/seen"
    DAEMON="${tmp}/daemon" PID_DIR="${tmp}/run" LOG_FILE="${tmp}/log" DEFAULTS_FILE="$1" \
        sh "${FILES_DIR}/cam-launcher" start > /dev/null
    for _ in $(seq 100); do
        [[ -s "${tmp}/seen" ]] && break
        sleep 0.02
    done
    cat "${tmp}/seen"
}

checks=0
expect()
{
    checks=$((checks + 1))
    if [[ "$1" != "$2" ]]; then
        echo "test_cam_launcher: FAIL: $3: got '$1', expected '$2'" >&2
        exit 1
    fi
}

expect "$(run_init "${FILES_DIR}/cam-launcher.default")" "autostart=0" "the shipped defaults keep autostart off"
echo "CAM_LAUNCHER_AUTOSTART=1" > "${tmp}/on"
expect "$(run_init "${tmp}/on")" "autostart=1" "the defaults file turns autostart on"
expect "$(run_init "${tmp}/missing")" "autostart=0" "no defaults file means autostart off"

echo "test_cam_launcher: init script OK (${checks} checks)"
