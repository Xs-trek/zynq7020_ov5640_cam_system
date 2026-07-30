# cam_system 软件架构与接口

## 1. 范围

`cam_system` 运行于 Zynq-7020 PetaLinux，负责 OV5640 视频采集、PL 人脸检测
后处理、数字云台控制、MJPEG/REST/WebSocket 服务。`cam-launcher` 和 `cam-ota`
是独立系统组件，不属于应用业务状态机。

## 2. 数据链路

```text
OV5640 -> PL video -> DDR -> V4L2 capture (640x480 RGB24, 30 fps)
                              |-> latest-frame mailbox -> JPEG -> MJPEG
                              |-> face_accel -> decode/NMS/association
                                                    |
frontend select/start -> command queue -> tracker -> V4L2 ISP/scaler controls
```

- 采集固定为完整 `640x480` RGB24 帧。
- 默认 detector 为 PL `face_accel`；YOLOv6Lite-S 仅作为兼容后端。
- PS 负责输出校验、解码、NMS、检测框时序确认和稳定 ID 关联。
- tracker 使用时间戳 Kalman 状态、短时丢检保持、目标重关联和受 jerk 约束的
  ISP/scaler 控制输出。
- JPEG 与 detector 读取三槽 latest-only mailbox；慢消费者不反压采集。

## 3. 线程与所有权

| 线程 | 主要职责 | 默认 CPU |
| --- | --- | --- |
| main/network | HTTP、REST、WebSocket 和生命周期协调 | Core 0 |
| capture/control | V4L2 采集、帧发布、tracker 控制提交 | Core 0 |
| detector | PL/NCNN 检测、后处理和时序关联 | Core 1 |
| JPEG worker | MJPEG 编码 | Core 1，较低调度优先级 |

跨线程标志和序列通过 atomic accessor 发布；复合状态由 mutex 或 mailbox lease
保护。图像数据只能通过 mailbox 的只读租约跨线程访问。

## 4. 跟踪生命周期

```text
IDLE -> SELECTED -> ENGAGING -> TRACKING
                              -> COASTING -> HOLDING -> LOST
                                                \-> RECOVERING -> TRACKING
```

- 前端先选择当前检测框，再发送跟踪启动命令。
- 启动后固定放大比例；目标运动由 ISP 窗口偏移补偿。
- 短时漏检使用预测保持，超过门限后进入 LOST；重新关联需满足空间、尺度和
  innovation gate。
- 停止跟踪时恢复全视野并清理 tracker 私有状态。

## 5. 对外接口

服务默认监听 `8080`：

```text
GET  /
GET  /stream
GET  /ws
GET  /api/status
POST /api/camera
POST /api/yolo
POST /api/settings
POST /api/track
```

`/api/yolo` 是前端兼容名称，表示检测开关，不限定 detector 实现。WebSocket
消息和限制见 `WEBSOCKET_PROTOCOL.md`。

## 6. 运行配置

主要环境变量：

| 变量 | 默认值 | 作用 |
| --- | --- | --- |
| `CAM_DETECTOR_BACKEND` | `face_accel` | `face_accel` 或 `yolov6lite_s` |
| `CAM_DETECTOR_FPS` | `30` | 检测请求频率 |
| `CAM_FACE_SCORE_THRESH` | `0.80` | 前端发布阈值 |
| `CAM_FACE_TRACK_SCORE_THRESH` | `0.60` | 已选目标连续关联阈值 |
| `CAM_FACE_NMS_THRESH` | `0.30` | NMS 阈值 |
| `CAM_FACE_CONFIRM_FRAMES` | `3` | 新目标确认帧数 |
| `CAM_FACE_MAX_MISSES` | `2` | 检测框短缺失容忍数 |

限频统计日志默认关闭，可分别通过 `CAM_CAPTURE_LOG_EVERY`、
`CAM_DETECTOR_LOG_EVERY`、`CAM_JPEG_LOG_EVERY` 启用。tracker 仅保留
`CAM_TRACKER_VERBOSE` 和 `CAM_TRACKER_DIAG_EVERY` 作为运行诊断开关。

## 7. 安装路径

| 路径 | 内容 |
| --- | --- |
| `/usr/bin/cam_system` | 主应用 |
| `/usr/sbin/cam-launcher` | 物理按键启动器 |
| `/usr/sbin/cam-ota` | A/B OTA 管理器 |
| `/usr/share/cam-system/models/` | NCNN 兼容模型 |
| `/lib/firmware/cam-system/face-accel/` | PL 检测权重 |

## 8. 发布边界

- HTTP/WebSocket 不含认证和 TLS，只能用于可信隔离网络。
- v0.1.1 XSA 可重建 PetaLinux 硬件平台、FSBL、device tree 和 BOOT.BIN；仓库
  不包含自定义 PL IP 的 RTL/HLS 源，因此不能独立重新综合完整 PL。
- 仓库不保存摄像头原始帧、设备凭据、个人公钥、启动镜像或 rootfs。
