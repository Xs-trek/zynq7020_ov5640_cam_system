# 模型资产

## PL 人脸检测权重

| 项目 | 值 |
| --- | --- |
| 文件 | `assets/models/face_accel/unified128_dw32_weights_v1.bin` |
| 大小 | 358400 bytes |
| SHA-256 | `93ae933897bb510a2a47a34a7ee27249a66904cc09964a1938187891239cbfab` |
| 部署路径 | `/lib/firmware/cam-system/face-accel/` |
| 输入 | 640×480 packed RGB24 |
| 默认阈值 | public 0.80、tracking 0.60、NMS 0.30 |

这是 PL 加速器的运行依赖，v0.1.1（R9）和 v0.2.0（R10）用同一份权重。

- 驱动通过 `GET_CAPS` 报告权重 SHA-256，`/api/status` 的 `weights_sha256` 显示运行中的值。
- 当前仓库没有提供训练、量化和打包工具链，所以只能复现部署，不能从训练模型重新生成相同权重。
- 来源和再分发许可还没有形成独立的上游声明，使用者必须自行确认适用权利。根目录 MIT 许可证不覆盖其中的第三方模型内容。

## CPU 检测候选

产品构建只用 PL 检测。下面的模型只在 `-DCAM_SYSTEM_ENABLE_CPU_DETECTORS=ON` 的构建中使用，由 `detector.backend` 选择。打开这个选项时，`assets/models/` 下的内容按目录安装到 `/usr/share/cam-system/models/`。

### YOLOv6Lite-S（随仓库发布）

| 文件 | SHA-256 |
| --- | --- |
| `assets/models/yolov6lite_s/yolov6lite_s.ncnn.param` | `05af6a2006eed92fc0996f72bfd42d82c903513e88530fd697580e840306c026` |
| `assets/models/yolov6lite_s/yolov6lite_s.ncnn.bin` | `8f825ded3d1e7d06f70734a3e2f91df039f60a2f81538f4b76818d765f9548aa` |

**运行契约：**

- 输入层 `in0`，输出层 `out0`，输入尺寸 `256x192`；
- RGB 转 BGR 后按 `1/255` 归一化；
- 输出按 `[cx, cy, w, h, obj_conf, cls0..cls79]` 解码。

**来源：** 该模型用于 v0.1.0 兼容后端，候选来源为 Meituan YOLOv6 release `0.4.0`。仓库没有保存从 `.pt` 到 NCNN 文件的完整转换记录，不能证明本地文件与上游权重的完整派生链。YOLOv6 上游使用 GPL-3.0，模型资产的适用条款需由使用者确认。

### YuNet（libfacedetection）

模型数据编译在 `third_party/libfacedetection/src/facedetectcnn-data.cpp` 里，随上游 BSD-3-Clause 许可证发布，不需要另外的文件。

### UltraFace RFB-320 和 SCRFD（不随仓库发布）

这些模型的再分发条件不清楚，默认配置也不用，所以仓库里没有。需要时由使用者从上游获取 NCNN 格式文件，自行确认许可证。文件放到下表的位置，可以在构建前放进 `assets/models/`，也可以直接放到板上：

| `detector.backend` | 板上路径（`/usr/share/cam-system/models/` 下） | 上游 |
| --- | --- | --- |
| `ultraface`、`ultraface_rfb_320` | `ultraface_rfb_320/RFB-320.param`、`RFB-320.bin` | Linzaer/Ultra-Light-Fast-Generic-Face-Detector-1MB 提交 `dffdddda9794a50607cba8f318507a28c1c27cab` 的 NCNN `version-RFB/RFB-320` 文件，MIT |
| `scrfd_500m`、`scrfd_500m_640` | `scrfd_500m_640/scrfd_500m_shape640x640.param`、`.bin` | InsightFace SCRFD 的 NCNN 转换，许可证以模型发布方为准 |
| `scrfd_1g`、`scrfd_1g_640` | `scrfd_1g_640/scrfd_1g_shape640x640.param`、`.bin` | 同上 |
| `scrfd_2.5g`、`scrfd_2_5g`、`scrfd_2_5g_kps` | `scrfd_2_5g_kps_640/scrfd_2.5g_bnkps_shape640x640.param`、`.bin` | 同上 |

- 文件名和路径由 `include/cam_system/config.h` 固定。
- 文件缺失时选中的 CPU 后端打开失败，日志报错；PL 后端不受影响。

## 变更要求

- 替换模型时必须同步更新文件哈希、输入/输出契约、许可证说明、解码测试和 ARM 构建验证。
- PL 权重还必须与驱动 UAPI、XSA 和 firmware 安装路径保持一致。
