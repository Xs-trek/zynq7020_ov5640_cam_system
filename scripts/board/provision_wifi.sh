#!/bin/sh
# Copies a Wi-Fi credentials file to the SD boot partition. The file stays out of the image and the
# repository, and nothing from it is printed.
set -eu

ROOT_DIR=$(CDPATH= cd -- "$(dirname -- "$0")/../.." && pwd)
S50WIFI="${ROOT_DIR}/packaging/petalinux/recipes-apps/wifi-init/files/S50wifi"

[ "$#" -eq 2 ] || { echo "Usage: $0 <credentials.txt> <mounted boot partition>" >&2; exit 2; }
SRC=$1
BOOT=$2

[ -f "${BOOT}/BOOT.BIN" ] || { echo "no BOOT.BIN in ${BOOT}: not a boot partition" >&2; exit 1; }
sh "${S50WIFI}" check "${SRC}"

cp "${SRC}" "${BOOT}/wifi.txt.new"
if ! cmp -s "${SRC}" "${BOOT}/wifi.txt.new"; then
    rm -f "${BOOT}/wifi.txt.new"
    echo "readback differs" >&2
    exit 1
fi
mv -f "${BOOT}/wifi.txt.new" "${BOOT}/wifi.txt"
sync
echo "wifi.txt written to ${BOOT}"
