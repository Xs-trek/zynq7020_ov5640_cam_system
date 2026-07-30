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

该二进制是 v0.1.1 PL 加速器的运行依赖。当前仓库没有提供其训练、量化和打包
工具链，因此只能复现部署，不能从训练模型重新生成相同权重。来源和再分发许可
尚未形成独立上游声明，使用者必须自行确认适用权利；根目录 MIT 许可证不覆盖
其中的第三方模型内容。

## YOLOv6Lite-S 兼容模型

| 文件 | SHA-256 |
| --- | --- |
| `assets/models/yolov6lite_s/yolov6lite_s.ncnn.param` | `05af6a2006eed92fc0996f72bfd42d82c903513e88530fd697580e840306c026` |
| `assets/models/yolov6lite_s/yolov6lite_s.ncnn.bin` | `8f825ded3d1e7d06f70734a3e2f91df039f60a2f81538f4b76818d765f9548aa` |

运行契约：输入层 `in0`，输出层 `out0`，输入尺寸 `256x192`，RGB 转 BGR 后按
`1/255` 归一化，输出按 `[cx, cy, w, h, obj_conf, cls0..cls79]` 解码。

该模型用于 v0.1.0 兼容后端。候选来源为 Meituan YOLOv6 release `0.4.0`，但仓库
没有保存从 `.pt` 到 NCNN 文件的完整转换记录，不能证明本地文件与上游权重的完整
派生链。YOLOv6 上游使用 GPL-3.0，模型资产的适用条款需由使用者确认。

## 变更要求

替换模型时必须同步更新文件哈希、输入/输出契约、许可证说明、解码测试和 ARM
构建验证；PL 权重还必须与驱动 UAPI、XSA 和 firmware 安装路径保持一致。
