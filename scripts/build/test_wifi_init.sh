#!/usr/bin/env bash
set -euo pipefail

ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"
WIFI="${ROOT_DIR}/packaging/petalinux/recipes-apps/wifi-init/files/S50wifi"
PROVISION="${ROOT_DIR}/scripts/board/provision_wifi.sh"
SECRET="not-a-real-key-7361"

WORK="$(mktemp -d)"
trap 'rm -rf "${WORK}"' EXIT
BIN="${WORK}/bin"
BOOT="${WORK}/boot"
RUN="${WORK}/run"
mkdir -p "${BIN}" "${BOOT}" "${RUN}"

fail()
{
    echo "test_wifi_init: FAIL: $*" >&2
    exit 1
}

cat >"${BIN}/wpa_passphrase" <<EOF
#!/bin/sh
echo "\$*" >"${WORK}/passphrase.args"
cat >"${WORK}/passphrase.stdin"
echo "network={"
echo "	ssid=\"\$1\""
echo "}"
EOF
cat >"${BIN}/wpa_supplicant" <<EOF
#!/bin/sh
for a; do c=\$a; done
cp "\$c" "${WORK}/seen.conf"
python3 -c 'import socket,sys; socket.socket(socket.AF_UNIX).bind(sys.argv[1])' "${RUN}/lo"
EOF
cat >"${BIN}/wpa_cli" <<EOF
#!/bin/sh
[ -f "${WORK}/associated" ] && echo wpa_state=COMPLETED
exit 0
EOF
cat >"${BIN}/udhcpc" <<EOF
#!/bin/sh
touch "${WORK}/udhcpc.ran"
EOF
for tool in killall ifconfig ip modprobe; do
    printf '#!/bin/sh\nexit 0\n' >"${BIN}/${tool}"
done
chmod +x "${BIN}"/*

MODULE="$(head -1 /proc/modules | cut -d' ' -f1)"
[[ -n "${MODULE}" ]] || fail "this host has no kernel module to stand in for the driver"

wifi()
{
    PATH="${BIN}:${PATH}" IFACE=lo MODULE_NAME="${MODULE}" IFACE_WAIT_SECS=1 WPA_WAIT_SECS=2 \
        WPA_CONF="${WORK}/wpa.conf" WPA_RUN_DIR="${RUN}" BOOT_DEV=/nonexistent BOOT_MOUNT="${BOOT}" \
        sh "${WIFI}" "$@"
}

reset_run()
{
    rm -f "${RUN}/lo" "${WORK}/seen.conf" "${WORK}/udhcpc.ran" "${WORK}/associated"
}

# A file from a Windows editor: BOM and CRLF.
printf '\357\273\277SSID=lab net\r\n# a comment\r\nPASSPHRASE=%s\r\n' "${SECRET}" >"${WORK}/windows.txt"
wifi check "${WORK}/windows.txt" >"${WORK}/out.txt" 2>&1 || fail "BOM and CRLF rejected"
if grep -q "${SECRET}" "${WORK}/out.txt"; then
    fail "check printed the key"
fi

printf 'SSID=x\nPASSPHRASE=short\n' >"${WORK}/short.txt"
printf 'PASSPHRASE=%s\n' "${SECRET}" >"${WORK}/nossid.txt"
printf 'SSID=x\nPSK=abc\n' >"${WORK}/badpsk.txt"
for bad in short nossid badpsk; do
    if wifi check "${WORK}/${bad}.txt" >/dev/null 2>&1; then
        fail "${bad}.txt accepted"
    fi
done

if grep -q '/etc/wifi' "${WIFI}"; then
    fail "S50wifi searches /etc, which can only come from the image"
fi

if wifi start >/dev/null 2>&1; then
    fail "started without credentials"
fi

# Boot does not wait for the network, and the key stays off the command line and out of the log.
reset_run
cp "${WORK}/windows.txt" "${BOOT}/wifi.txt"
begin=$(date +%s)
wifi start >"${WORK}/out.txt" 2>&1 || fail "start failed"
[[ $(($(date +%s) - begin)) -le 1 ]] || fail "start waited for the network"
[[ -f "${WORK}/seen.conf" ]] || fail "wpa_supplicant did not start"
grep -q 'ssid="lab net"' "${WORK}/seen.conf" || fail "SSID lost"
grep -q "${SECRET}" "${WORK}/passphrase.stdin" || fail "the key did not reach wpa_passphrase"
if grep -q "${SECRET}" "${WORK}/passphrase.args"; then
    fail "the key is on a command line"
fi
if grep -q "${SECRET}" "${WORK}/out.txt"; then
    fail "start printed the key"
fi
[[ ! -e "${WORK}/wpa.conf" ]] || fail "the generated configuration was left behind"

# DHCP still follows, once the wait is over.
for _ in 1 2 3 4 5 6; do
    [[ -f "${WORK}/udhcpc.ran" ]] && break
    sleep 1
done
[[ -f "${WORK}/udhcpc.ran" ]] || fail "udhcpc did not run"

# Provisioning.
mkdir -p "${WORK}/card"
if sh "${PROVISION}" "${WORK}/windows.txt" "${WORK}/card" >/dev/null 2>&1; then
    fail "wrote to a directory without BOOT.BIN"
fi
touch "${WORK}/card/BOOT.BIN"
if sh "${PROVISION}" "${WORK}/short.txt" "${WORK}/card" >/dev/null 2>&1; then
    fail "wrote an invalid file"
fi
[[ ! -e "${WORK}/card/wifi.txt" ]] || fail "an invalid file reached the card"
sh "${PROVISION}" "${WORK}/windows.txt" "${WORK}/card" >"${WORK}/out.txt" 2>&1 || fail "provisioning failed"
cmp -s "${WORK}/windows.txt" "${WORK}/card/wifi.txt" || fail "provisioned file differs"
if grep -q "${SECRET}" "${WORK}/out.txt"; then
    fail "provisioning printed the key"
fi

echo "test_wifi_init: OK"
