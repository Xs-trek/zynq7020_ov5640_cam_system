# 第三方组件与许可证

根目录 MIT 许可证只覆盖项目原创内容，不重新许可第三方代码、模型或 Xilinx 生成内容。具体边界同时见 `NOTICE.md`。

## 源码与构建依赖

| 组件 | 用途 | 来源/固定版本 | 许可证 |
| --- | --- | --- | --- |
| cJSON | REST/配置 JSON | `third_party/cjson/`，本地内嵌 | MIT |
| libjpeg-turbo | MJPEG 编码 | Yocto recipe `3.1.4` | BSD-3-Clause、IJG、Zlib |
| OpenSSL libcrypto | WebSocket SHA-1/Base64 | PetaLinux 系统包 | Apache-2.0 |
| 88x2bu | USB WiFi 内核模块 | morrownr/88x2bu `fecac340fb117eb979f4bb6d28e29730384c382b` | GPL-2.0-only |
| Linux/Xilinx drivers | OV5640、V4L2、PL 加速器 | PetaLinux 2022.2 / Linux 5.15 | GPL-2.0-only 及对应上游条款 |

只在 `-DCAM_SYSTEM_ENABLE_CPU_DETECTORS=ON` 时编译的 CPU 检测候选。默认构建和 PetaLinux 配方都不含这些组件：

| 组件 | 用途 | 来源/固定版本 | 许可证 |
| --- | --- | --- | --- |
| ncnn | YOLOv6Lite-S、UltraFace、SCRFD 推理 | Tencent/ncnn `e54f7b1f88434e1d844ea0551b880a1cfb079ce1` | BSD-3-Clause、BSD-2-Clause、Zlib |
| libfacedetection | YuNet CPU 检测 | `third_party/libfacedetection/`，ShiqiYu/libfacedetection `3023e1289e3b85311632bcfd45c9895b4292778b`，只含 `src/facedetectcnn*`（含编译进源码的模型数据） | BSD-3-Clause |
| UltraFace NCNN 封装 | UltraFace RFB-320 CPU 检测 | `third_party/ultraface/`，Linzaer/Ultra-Light-Fast-Generic-Face-Detector-1MB `dffdddda9794a50607cba8f318507a28c1c27cab`，只含 C++ 封装 | MIT |

PetaLinux recipe 固定的 `SRCREV` 和 `LIC_FILES_CHKSUM` 是构建时的权威版本约束。

## 模型与硬件资产

| 资产 | 说明 |
| --- | --- |
| PL 人脸检测权重 | 运行所需二进制；训练/量化派生链和独立再分发声明不在仓库中。 |
| YOLOv6Lite-S NCNN | v0.1.0 兼容模型，v0.2.0 中属于 CPU 候选；候选上游为 Meituan YOLOv6 GPL-3.0，完整转换链未保留。 |
| UltraFace RFB-320、SCRFD NCNN | CPU 候选模型，再分发条件不清楚，不随仓库发布；获取方式见 `MODEL_ASSETS.md`。 |
| XSA | 包含项目设计和 Xilinx 工具/IP 生成内容；Xilinx 内容仍受其适用条款约束。 |

模型文件哈希和运行契约见 `MODEL_ASSETS.md`。

## 新增依赖要求

- 新增或升级第三方组件必须固定来源版本，记录许可证，优先通过 Yocto recipe 构建。
- 必须完成主机测试、ARM 构建及受影响功能验证。
- 不得提交来源不明的二进制、凭据或无法说明授权边界的代码。
