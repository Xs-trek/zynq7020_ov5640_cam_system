#!/usr/bin/env python3
from __future__ import annotations

import argparse
import importlib.util
import sys
from pathlib import Path


ROOT = Path(__file__).resolve().parents[1]
MODULE_PATH = ROOT / "scripts" / "board" / "analyze_ps_pipeline_log.py"
SPEC = importlib.util.spec_from_file_location("ps_pipeline_analyzer", MODULE_PATH)
assert SPEC and SPEC.loader
MODULE = importlib.util.module_from_spec(SPEC)
sys.modules[SPEC.name] = MODULE
SPEC.loader.exec_module(MODULE)


GOOD_WINDOW = """
[capture] avg30 wait=28.90ms publish=4.00ms track=0.20ms total=33.10ms fps=30.21 capture_age=0.10/0.20ms ts_mono=30/30 seq_skip=0 mailbox_drop=0 det_on_time=30 det_late=0 det_age=37.0/38.0ms det_dispatch=3.8/4.2ms
[detector] backend=face_accel_r9 avg30 pre=0.0ms infer=29.1ms post=0.0ms total=29.2ms hw=26.1ms ps=3.0ms pickup=0.15ms source_age=4.2ms result_age=33.3/34.0ms publish=0.02ms hold=29.2ms skipped=0 raw=1.00 confirmed=1.00 positive=100.0% max_conf=0.900
[jpeg-worker] avg30 encode=17.50ms pickup=0.20ms capture_age=4.3ms output_fps=30.05 observed=90 encoded=90 replaced=0
"""


def thresholds() -> argparse.Namespace:
    return argparse.Namespace(
        min_windows=3,
        min_capture_fps=29.5,
        min_jpeg_fps=29.5,
        max_pickup_ms=3.0,
        max_source_age_ms=10.0,
        max_result_avg_ms=40.0,
        max_result_max_ms=45.0,
        max_consume_age_ms=45.0,
        max_dispatch_ms=10.0,
    )


def main() -> int:
    good = MODULE.parse_log(GOOD_WINDOW * 3)
    assert not MODULE.evaluate(good, thresholds())

    bad_text = (GOOD_WINDOW * 3).replace("mailbox_drop=0", "mailbox_drop=1", 1)
    bad_text = bad_text.replace("result_age=33.3/34.0", "result_age=44.0/52.0", 1)
    failures = MODULE.evaluate(MODULE.parse_log(bad_text), thresholds())
    assert any("mailbox" in failure for failure in failures)
    assert any("result age" in failure for failure in failures)
    print("ps_pipeline_log_analyzer_test: OK")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
