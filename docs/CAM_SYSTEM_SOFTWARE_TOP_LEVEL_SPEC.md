# cam_system v0.1.0 软件架构

## 数据链路

```text
OV5640 -> PL video -> DDR -> V4L2 capture (640x480 RGB24)
                              |-> JPEG -> MJPEG
                              |-> NCNN YOLOv6Lite-S -> detections
                                                       |
frontend select/start -> tracker + optical flow -> V4L2 ISP controls
```

- YOLOv6Lite-S 输入固定为 `256x192`，NCNN 单线程运行于 Core 1。
- capture/control 和 network 运行于 Core 0。
- tracker 使用 alpha-beta 估计、二阶阻尼和状态机；光流用于检测间隔补偿。
- network 通过有界命令队列提交跟踪操作，不直接修改 tracker 状态。

## 对外接口

```text
GET  /api/status
POST /api/camera
POST /api/yolo
POST /api/settings
POST /api/track
GET  /stream
GET  /ws
```

`classes=[]` 表示不做类别过滤。关闭检测后必须发布空检测状态。WebSocket 只实现
`WEBSOCKET_PROTOCOL.md` 定义的受限子集。

## 安装路径

| 路径 | 内容 |
| --- | --- |
| `/usr/bin/cam_system` | 主应用 |
| `/usr/sbin/cam-launcher` | PS_KEY1 启动器 |
| `/usr/sbin/cam-ota` | A/B OTA 管理器 |
| `/usr/share/cam-system/models/yolov6lite_s/` | NCNN 模型 |

## 约束

- 图像、检测、JPEG、tracker 和配置状态必须在明确的 mutex/队列边界内发布。
- 热路径日志默认关闭或限频，网络客户端不得无限阻塞事件循环。
- HTTP/WebSocket 不含认证和 TLS，只能用于可信隔离网络。
- 仓库不保存摄像头帧、设备凭据或生成的启动镜像。
