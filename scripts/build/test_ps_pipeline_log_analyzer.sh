#!/usr/bin/env bash
set -euo pipefail

ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"
PYTHONDONTWRITEBYTECODE=1 \
    python3 "${ROOT_DIR}/tests/ps_pipeline_log_analyzer_test.py"
