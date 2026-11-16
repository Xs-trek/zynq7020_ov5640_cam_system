# cam_system 软件架构与接口

## 1. 范围

`cam_system` 运行于 Zynq-7020 PetaLinux，负责以下功能：

- OV5640 视频采集；
- PL 人脸检测和 PS 侧后处理；
- 数字云台控制；
- MJPEG/REST/WebSocket 服务。

`cam-launcher` 和 `cam-ota` 是独立系统组件，不属于应用的业务状态机。

## 2. 数据链路

```text
OV5640 -> PL video -> DDR (reserved) -> V4L2 capture (640x480 RGB24, 30 fps)
                                          |-> dma-buf -> face_accel -> decode/NMS/association
                                          |-> copy (CPU consumers) -> JPEG -> MJPEG
                                                                         |
frontend select/start -> command queue -> control tick -> tracker -> V4L2 ISP/scaler controls
```

- **采集：** 固定为完整的 `640x480` RGB24 帧，缓冲 6 个，位于设备树保留区。
- **检测：** 默认 detector 是 PL `face_accel`。R10 上加速器经 dma-buf 直接读采集缓冲；驱动没有这个能力、配置关闭或运行失败时，退回拷贝路径。CPU 检测候选只在打开构建选项时编译。
- **PS 后处理：** 输出校验、解码、NMS、检测框时序确认和稳定 ID 关联。
- **控制节拍：** 每帧一次，在本帧检测发布或截止时刻（采集时间加 `control.deadline_ms`）中较早者运行。ISP 命令在第 N 帧发出，第 N+2 帧生效；写入晚于下一帧时按 N+3 登记。
- **跟踪：** 用时间戳 Kalman 状态、短时丢检保持和目标重关联。ISP/scaler 输出受速度、加速度和 jerk 约束，平移极限前有边界刹车。
- **缓存帧：** JPEG 读取三槽 latest-only mailbox 的缓存帧，只在有 CPU 消费者时复制；慢消费者不反压采集。

## 3. 线程与所有权

| 线程 | 主要职责 | 默认 CPU |
| --- | --- | --- |
| main/network | HTTP、REST、WebSocket、看门狗和生命周期协调 | Core 0 |
| capture/control | V4L2 采集、相机起停、帧发布、控制节拍和 tracker 控制提交 | Core 0 |
| detector | PL 检测（或 CPU 候选）、后处理、时序关联和恢复 | Core 1 |
| JPEG worker | MJPEG 编码，可限帧 | Core 1，较低调度优先级 |

- 跨线程标志和序列通过 atomic accessor 发布；复合状态由 mutex 或 mailbox lease 保护。
- 图像数据只能通过 mailbox 的只读租约跨线程访问。
- 应用以 `camsys` 用户运行，不直接访问寄存器、I2C 或 GPIO，只用 V4L2、V4L2 subdev 控制和加速器字符设备。

## 4. 跟踪生命周期

```text
IDLE -> SELECTED -> ENGAGING -> TRACKING
                              -> COASTING -> HOLDING -> LOST
                                                \-> RECOVERING -> TRACKING
```

- 前端先选择当前检测框，再发送跟踪启动命令。
- 启动后固定放大比例，目标运动由 ISP 窗口偏移补偿。
- 短时漏检用预测保持，超过门限后进入 LOST。重新关联要满足空间、尺度和 innovation gate。
- 停止跟踪或关相机时，恢复全视野并清理 tracker 私有状态。

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
POST /api/zoom
```

- **`/api/yolo`：** 前端兼容名称，表示检测开关，不限定 detector 实现。
- **`/api/zoom`：** 只为旧前端兼容保留，不改变任何状态；`status` 不含 `zoom`。
- **`/stream`：** 分隔符和下一分片的头紧跟 JPEG 发送。
- **WebSocket：** 消息和限制见 `WEBSOCKET_PROTOCOL.md`。

`/api/status` 的 `status` 对象包含以下字段：

| 类别 | 字段 |
| --- | --- |
| 基本状态 | `cam_enabled`、`yolo_enabled`、`tracking`、`target_id`、`fps_capture`、`fps_inference`、`fps_stream`、`uptime_sec`、`status_seq` |
| 检测器 | `detector_health`、`detector_failures`、`detector_last_error`、`detector_input`（`direct` 或 `copy`） |
| 相机 | `cam_off_reason`（`none` 或 `stream_limit`）、`cam_stream_sec` |
| 计数 | `isp_slips`、`capture_restarts`、`dvp_frames`、`dvp_bad_frames`（本次运行）、`i2c_write_failures`、`v4l2_error_frames` |
| 身份 | `version`、`git_commit`、`driver_uapi`、`hw_revision`、`weights_sha256` |

## 6. 运行配置

运行配置在 `/etc/cam_system.conf`，格式是每行一个 `key = value`。全部键、取值范围和默认值见 `config/cam_system.conf`。

- 同名环境变量（`CAM_*`）优先于文件，只用于调试。
- 未知键和越界值在启动时报错退出。

常用项：

| 键 | 默认值 | 作用 |
| --- | --- | --- |
| `detector.backend` | `face_accel` | CPU 候选只在 CPU 检测构建中可选 |
| `detector.fps` | `30` | 检测请求频率 |
| `face.score_thresh` | `0.8` | 前端发布阈值 |
| `face.track_score_thresh` | `0.6` | 已选目标的连续关联阈值 |
| `face.nms_thresh` | `0.3` | NMS 阈值 |
| `face.zero_copy` | `1` | 允许 dma-buf 直通 |
| `control.deadline_ms` | `24` | 控制节拍等本帧检测的最长时间，0 为不等 |
| `capture.max_stream_min` | `20` | 每段出流的最长分钟数，0 为不限 |
| `jpeg.quality` | `70` | MJPEG 质量 |
| `stream.max_fps` | `0` | 页面推流帧率上限，0 为不限 |
| `tracker.trace` | 空 | `/tmp/` 下的数字轨迹文件，空为关闭 |

限频统计日志默认关闭，可以分别用 `capture.log_every`、`detector.log_every`、`jpeg.log_every` 打开。`tracker.verbose`、`tracker.diag_every_sec` 和 `face.diag_*` 是运行诊断开关，会输出人脸坐标，默认关闭。

## 7. 安装路径

| 路径 | 内容 |
| --- | --- |
| `/usr/bin/cam_system` | 主应用 |
| `/etc/cam_system.conf` | 运行配置 |
| `/usr/sbin/cam-launcher` | 物理按键启动器和守护 |
| `/etc/default/cam-launcher` | launcher 选项 |
| `/usr/sbin/cam-ota` | A/B OTA 管理器 |
| `/usr/share/cam-system/models/` | CPU 候选模型（只在 CPU 检测构建中安装） |
| `/lib/firmware/cam-system/face-accel/` | PL 检测权重 |

## 8. 发布边界

- HTTP/WebSocket 不含认证和 TLS，只能用于可信隔离网络。
- v0.2.0 XSA 可以重建 PetaLinux 硬件平台、FSBL、device tree 和 BOOT.BIN。仓库不包含自定义 PL IP 的 RTL/HLS 源，所以不能独立重新综合 PL。
- 仓库不保存摄像头原始帧、设备凭据、个人公钥、启动镜像或 rootfs。
