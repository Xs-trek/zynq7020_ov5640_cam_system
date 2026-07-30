# Hardware inputs

仓库只发布两个版本对应的 Vivado 2022.2 XSA。XSA 内含 bitstream，可用于
PetaLinux hardware import、device tree、FSBL 和 BOOT.BIN 重建。

| 版本 | XSA | SHA-256 |
| --- | --- | --- |
| v0.1.0 | `v0.1.0/zynq7020_ov5640_dvp_driver_ready_20260617.xsa` | `cc4f2132e8fa0a5cf74f5cd2bd00c69025b75a7ce5b716d3d2eaeaa9a3dcec47` |
| v0.1.1 | `v0.1.1/zynq7020_face_accel_revision9_20260719.xsa` | `2e7c10c56ec3e6864f2bab207cea9493c9874a20a33f0be1de2e0ae12017cf0b` |

v0.1.1 文件名中的 `revision9` 是最终采用的 PL 导出迭代标识，仅用于硬件溯源，
不是运行时后端、模型或系统架构名称。

导入示例：

```sh
petalinux-config \
  --get-hw-description=/absolute/path/to/hardware/v0.1.1/zynq7020_face_accel_revision9_20260719.xsa \
  --silentconfig
```

仓库不包含 editable Vivado block design、约束文件、RTL/HLS 或自定义 IP 源码，
因此不能只依靠本仓库重新综合完整 PL。
