# Hardware release inputs

This directory contains version-matched Vivado hardware handoffs needed to
reconstruct the PetaLinux platform. These files contain no device credentials.

## v0.1.0

| File | Purpose | SHA-256 |
| --- | --- | --- |
| `v0.1.0/zynq7020_ov5640_dvp_driver_ready_20260617.xsa` | Vivado 2022.2 OV5640/V4L2 hardware export with embedded bitstream | `cc4f2132e8fa0a5cf74f5cd2bd00c69025b75a7ce5b716d3d2eaeaa9a3dcec47` |

Import it into a new or existing PetaLinux 2022.2 project:

```sh
petalinux-config \
  --get-hw-description=/absolute/path/to/hardware/v0.1.0/zynq7020_ov5640_dvp_driver_ready_20260617.xsa \
  --silentconfig
```

An XSA is sufficient for PetaLinux hardware metadata, device-tree generation,
FSBL and boot-image integration. It is not a replacement for editable Vivado
block-design, constraints, RTL/HLS or custom-IP source files.
