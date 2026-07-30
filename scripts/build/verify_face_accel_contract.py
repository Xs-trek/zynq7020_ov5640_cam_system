#!/usr/bin/env python3
"""Fail fast when the packaged accelerator ABI, diagnostics, and weight diverge."""

from __future__ import annotations

import hashlib
import re
from pathlib import Path


ROOT = Path(__file__).resolve().parents[2]
WEIGHT = ROOT / "assets/models/face_accel/unified128_dw32_weights_v1.bin"
DRIVER = (
    ROOT
    / "packaging/petalinux/recipes-kernel/cam-face-accel/files/cam-face-accel.c"
)
REGS = ROOT / "third_party/face_accel_ref/face_accel_regs.h"
UAPI = ROOT / "include/cam_system/face_accel_uapi.h"
DETECTOR = ROOT / "src/vision/detector.cpp"
EXPECTED = "93ae933897bb510a2a47a34a7ee27249a66904cc09964a1938187891239cbfab"


def main() -> None:
    weight = WEIGHT.read_bytes()
    digest = hashlib.sha256(weight).hexdigest()
    if len(weight) != 358400:
        raise SystemExit(f"weight size mismatch: {len(weight)}")
    if digest != EXPECTED:
        raise SystemExit(f"weight SHA-256 mismatch: {digest}")

    source = DRIVER.read_text(encoding="utf-8")
    match = re.search(
        r"cam_fa_weight_sha256\[32\]\s*=\s*\{(?P<body>.*?)\};",
        source,
        re.DOTALL,
    )
    if not match:
        raise SystemExit("driver SHA-256 initializer not found")
    compiled_digest = bytes(
        int(value, 16)
        for value in re.findall(r"0x([0-9a-fA-F]{2})", match.group("body"))
    )
    if len(compiled_digest) != 32:
        raise SystemExit(f"driver SHA-256 initializer has {len(compiled_digest)} bytes")
    if compiled_digest.hex() != digest:
        raise SystemExit(
            "driver SHA-256 constant mismatch: "
            f"{compiled_digest.hex()} != {digest}"
        )

    regs = REGS.read_text(encoding="utf-8")
    uapi = UAPI.read_text(encoding="utf-8")
    detector = DETECTOR.read_text(encoding="utf-8")
    required_regs = (
        "#define FACE_ACCEL_IP_ID_MAGIC   0xFACE0004u",
        "#define FA_DEBUG_TRANSPORT       0x74",
        "#define FA_DEBUG_DMA_READ        0x78",
        "#define FA_DEBUG_DMA_WRITE       0x7C",
    )
    for marker in required_regs:
        if marker not in regs:
            raise SystemExit(f"missing accelerator register contract: {marker}")
    if "#define CAM_FACE_ACCEL_IP_ID              0xFACE0004u" not in uapi:
        raise SystemExit("accelerator IP ID missing from UAPI")
    for field in ("debug_dma", "debug_transport", "debug_dma_read", "debug_dma_write"):
        if f"run->{field} = cam_fa_read" not in source:
            raise SystemExit(f"driver does not snapshot {field} before reset")
    if "face accelerator warmup failed; disabling backend" not in detector:
        raise SystemExit("warmup failure does not disable the face accelerator backend")
    print(f"face_accel_contract: OK weight_sha256={digest}")


if __name__ == "__main__":
    main()
