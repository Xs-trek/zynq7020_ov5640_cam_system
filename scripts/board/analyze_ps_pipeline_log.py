#!/usr/bin/env python3
"""Validate the log-only PS capture/detector/JPEG latency contract."""

from __future__ import annotations

import argparse
import re
import sys
from dataclasses import dataclass, field
from pathlib import Path


CAPTURE_RE = re.compile(
    r"^\[capture\] avg(?P<window>\d+).*?fps=(?P<fps>[0-9.]+).*?"
    r"seq_skip=(?P<seq_skip>\d+).*?mailbox_drop=(?P<mailbox_drop>\d+).*?"
    r"det_on_time=(?P<det_on_time>\d+).*?det_late=(?P<det_late>\d+).*?"
    r"det_age=(?P<det_age_avg>[0-9.]+)/(?P<det_age_max>[0-9.]+)ms.*?"
    r"det_dispatch=(?P<dispatch_avg>[0-9.]+)/(?P<dispatch_max>[0-9.]+)ms"
)
DETECTOR_RE = re.compile(
    r"^\[detector\].*?avg(?P<window>\d+).*?"
    r"pickup=(?P<pickup>[0-9.]+)ms.*?"
    r"source_age=(?P<source_age>[0-9.]+)ms.*?"
    r"result_age=(?P<result_age_avg>[0-9.]+)/"
    r"(?P<result_age_max>[0-9.]+)ms.*?skipped=(?P<skipped>\d+)"
)
JPEG_RE = re.compile(
    r"^\[jpeg-worker\] avg(?P<window>\d+).*?"
    r"pickup=(?P<pickup>[0-9.]+)ms.*?"
    r"output_fps=(?P<fps>[0-9.]+).*?replaced=(?P<replaced>\d+)"
)


@dataclass
class Metrics:
    capture: list[dict[str, float | int]] = field(default_factory=list)
    detector: list[dict[str, float | int]] = field(default_factory=list)
    jpeg: list[dict[str, float | int]] = field(default_factory=list)


def _typed_groups(match: re.Match[str]) -> dict[str, float | int]:
    values: dict[str, float | int] = {}
    for name, value in match.groupdict().items():
        values[name] = float(value) if "." in value else int(value)
    return values


def parse_log(text: str) -> Metrics:
    metrics = Metrics()
    for line in text.splitlines():
        match = CAPTURE_RE.search(line)
        if match:
            metrics.capture.append(_typed_groups(match))
            continue
        match = DETECTOR_RE.search(line)
        if match:
            metrics.detector.append(_typed_groups(match))
            continue
        match = JPEG_RE.search(line)
        if match:
            metrics.jpeg.append(_typed_groups(match))
    return metrics


def evaluate(metrics: Metrics, args: argparse.Namespace) -> list[str]:
    failures: list[str] = []
    for role in ("capture", "detector", "jpeg"):
        count = len(getattr(metrics, role))
        if count < args.min_windows:
            failures.append(
                f"{role}: only {count} timing windows, need {args.min_windows}"
            )
    if failures:
        return failures

    if min(float(row["fps"]) for row in metrics.capture) < args.min_capture_fps:
        failures.append(f"capture: fps below {args.min_capture_fps:.2f}")
    if sum(int(row["seq_skip"]) for row in metrics.capture) != 0:
        failures.append("capture: V4L2 sequence skips observed")
    if sum(int(row["mailbox_drop"]) for row in metrics.capture) != 0:
        failures.append("capture: cached-frame mailbox publication dropped")
    if sum(int(row["det_on_time"]) for row in metrics.capture) == 0:
        failures.append("capture: no detector result was consumed")
    if sum(int(row["det_late"]) for row in metrics.capture) != 0:
        failures.append("capture: detector result missed its one-frame deadline")
    if max(float(row["det_age_max"]) for row in metrics.capture) > args.max_consume_age_ms:
        failures.append(
            f"capture: detection consume age above {args.max_consume_age_ms:.1f}ms"
        )
    if max(float(row["dispatch_max"]) for row in metrics.capture) > args.max_dispatch_ms:
        failures.append(
            f"capture: completed-result dispatch above {args.max_dispatch_ms:.1f}ms"
        )

    if max(float(row["pickup"]) for row in metrics.detector) > args.max_pickup_ms:
        failures.append(f"detector: mailbox pickup above {args.max_pickup_ms:.1f}ms")
    if max(float(row["source_age"]) for row in metrics.detector) > args.max_source_age_ms:
        failures.append(
            f"detector: source age at inference start above {args.max_source_age_ms:.1f}ms"
        )
    if max(float(row["result_age_avg"]) for row in metrics.detector) > args.max_result_avg_ms:
        failures.append(
            f"detector: average result age above {args.max_result_avg_ms:.1f}ms"
        )
    if max(float(row["result_age_max"]) for row in metrics.detector) > args.max_result_max_ms:
        failures.append(
            f"detector: result age above {args.max_result_max_ms:.1f}ms"
        )
    if sum(int(row["skipped"]) for row in metrics.detector) != 0:
        failures.append("detector: input frame skips observed")

    if min(float(row["fps"]) for row in metrics.jpeg) < args.min_jpeg_fps:
        failures.append(f"jpeg: output fps below {args.min_jpeg_fps:.2f}")
    if sum(int(row["replaced"]) for row in metrics.jpeg) != 0:
        failures.append("jpeg: latest-frame replacements observed")
    return failures


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser()
    parser.add_argument("log", type=Path)
    parser.add_argument("--min-windows", type=int, default=3)
    parser.add_argument("--min-capture-fps", type=float, default=29.5)
    parser.add_argument("--min-jpeg-fps", type=float, default=29.5)
    parser.add_argument("--max-pickup-ms", type=float, default=3.0)
    parser.add_argument("--max-source-age-ms", type=float, default=10.0)
    parser.add_argument("--max-result-avg-ms", type=float, default=40.0)
    parser.add_argument("--max-result-max-ms", type=float, default=45.0)
    parser.add_argument("--max-consume-age-ms", type=float, default=45.0)
    parser.add_argument("--max-dispatch-ms", type=float, default=10.0)
    return parser


def main() -> int:
    args = build_parser().parse_args()
    metrics = parse_log(args.log.read_text(encoding="utf-8", errors="replace"))
    failures = evaluate(metrics, args)
    print(
        "ps_pipeline_log: "
        f"capture_windows={len(metrics.capture)} "
        f"detector_windows={len(metrics.detector)} "
        f"jpeg_windows={len(metrics.jpeg)}"
    )
    if failures:
        for failure in failures:
            print(f"FAIL: {failure}", file=sys.stderr)
        return 1
    print("ps_pipeline_log: PASS")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
