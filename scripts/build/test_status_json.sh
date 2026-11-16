#!/usr/bin/env bash
set -euo pipefail

ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"
BUILD_DIR="${ROOT_DIR}/build/host-tests"
BIN="${BUILD_DIR}/status_json_test"

mkdir -p "${BUILD_DIR}"

gcc -Wall -Wextra -Wformat=2 -Werror -std=c11 -D_DEFAULT_SOURCE     -fsanitize=address,undefined -fno-omit-frame-pointer     -I"${ROOT_DIR}/src"     "${ROOT_DIR}/tests/status_json_test.c"     "${ROOT_DIR}/src/network/status_json.c"     -o "${BIN}"

"${BIN}"
