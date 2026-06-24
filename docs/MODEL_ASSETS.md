# YOLOv6Lite-S 模型资产

| 文件 | SHA-256 |
| --- | --- |
| `assets/models/yolov6lite_s/yolov6lite_s.ncnn.param` | `05af6a2006eed92fc0996f72bfd42d82c903513e88530fd697580e840306c026` |
| `assets/models/yolov6lite_s/yolov6lite_s.ncnn.bin` | `8f825ded3d1e7d06f70734a3e2f91df039f60a2f81538f4b76818d765f9548aa` |

运行契约：输入层 `in0`，输出层 `out0`，输入尺寸 `256x192`，RGB 转 BGR 后按
`1/255` 归一化，输出按 `[cx, cy, w, h, obj_conf, cls0..cls79]` 解码。

候选来源为 Meituan YOLOv6 release `0.4.0`。仓库没有保存从 `.pt` 到 NCNN 文件
的完整转换记录，不能证明本地文件与上游权重的完整派生链。YOLOv6 上游使用
GPL-3.0，模型资产的适用条款需由使用者确认；根目录 MIT 许可证不覆盖第三方模型。
