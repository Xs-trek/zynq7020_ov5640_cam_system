# 解码测试向量

三组 PL 交付向量的期望输出缓冲（26704 字节，`output_abi_v1`），由 PL 参考实现生成，原件不在本仓库。
这三个文件只能整体随 PL 交付更新，不要在这里单独修改。

| 文件 | 场景 | 人脸数（score 0.70、NMS 0.30，向量 `expected.json` 的约定） |
|---|---|---|
| `vector_00_output.bin` | 多个小人脸 | 3 |
| `vector_01_output.bin` | 单个大人脸 | 1 |
| `vector_02_output.bin` | 单个遮挡人脸 | 1 |
