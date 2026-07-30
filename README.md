# cam_system

License: MIT for project-authored material; third-party and vendor exceptions
are listed in [NOTICE.md](NOTICE.md).

`cam_system` 是面向 Zynq-7020 + OV5640 的嵌入式视觉与数字云台系统，提供
视频采集、目标检测与选择、数字跟踪、MJPEG/REST/WebSocket 服务，以及 A/B
`image.ub` OTA。`v0.1.0` 和 `v0.1.1` 面向相同的系统目标，采用不同的检测与
跟踪实现；当前 `main` 对应 `v0.1.1`。

## 版本实现

| 版本 | 检测链路 | 跟踪控制 |
| --- | --- | --- |
| `v0.1.0` | PL 视频采集；PS 运行 NCNN YOLOv6Lite-S | alpha-beta 状态估计、检测间隔光流补偿和二阶阻尼 |
| `v0.1.1` | PL 视频采集与人脸检测加速；PS 完成输出校验、解码、NMS 和时序关联 | 时间戳 Kalman 状态估计、短时丢检保持/重关联和 jerk-limited 控制；不再依赖光流或 OpenCV |

各标签中的 README 记录对应版本的完整构建与运行边界。以下内容描述当前
`v0.1.1`。

## v0.1.1 架构

```text
OV5640 -> PL video -> DDR -> V4L2 capture (640x480 RGB24, 30 fps)
                              |-> latest-frame mailbox -> JPEG -> MJPEG
                              |-> PL face accelerator -> decode/NMS/association
                                                              |
frontend select/start -> command queue -> tracker -> V4L2 ISP/scaler controls
```

- capture/control 与 main/network 运行在 Core 0；detector 与低优先级 JPEG worker
  运行在 Core 1。
- detector 和 JPEG 读取三槽 latest-only mailbox 的不可变租约；慢消费者不反压采集。
- 数字云台使用稳定参考框、Kalman 状态估计和 jerk-limited 控制输出。光流和 OpenCV
  运行依赖已退役。
- `cam-launcher` 负责物理按键启动；`cam-ota` 负责双槽 OTA 和 bootguard；它们不是
  `cam_system` 业务状态机的一部分。

## 构建与验证

```sh
scripts/build/test_all_host.sh
BUILD_DIR=/tmp/cam_system_arm scripts/build/build_arm.sh
```

## 硬件复现输入

仓库包含两个版本对应的 Vivado 2022.2 XSA：

- `hardware/v0.1.0/`：OV5640/V4L2 基线；
- `hardware/v0.1.1/`：PL 人脸检测加速器 XSA。

XSA 可用于 PetaLinux hardware import、device tree、FSBL 和 BOOT.BIN 重建。
仓库不包含可编辑 Vivado 工程、block-design Tcl、约束文件或自定义 IP 源码，
因此不能只依靠当前仓库重新综合完整 PL。具体哈希和导入命令见
`hardware/README.md`。

PetaLinux 集成使用 `scripts/petalinux/stage_*.sh`。开发镜像若需要 SSH，必须显式
提供仓库外公钥：

```sh
CAM_SSH_AUTHORIZED_KEYS_FILE=/absolute/path/to/authorized_keys \
PLNX_PROJECT=/absolute/path/to/petalinux-project \
scripts/petalinux/stage_ssh_access_dev.sh
```

Git 不保存个人公钥、`authorized_keys` 或固定密码哈希。`ssh-access-dev` 只适用于
可信实验室网络，账号密码保持锁定；量产/公开镜像默认不集成该 recipe。

## 发布边界

公开发布只包含源码、必要模型、PetaLinux 配置/脚本和版本匹配的 XSA，不包含
可启动系统镜像、rootfs、设备凭据、内部调试记录或摄像头真实图像。

当前状态和证据见 [v0.1.1 发布说明](docs/V011_RELEASE_NOTES.md)。顶层契约见
[软件顶层规格](docs/CAM_SYSTEM_SOFTWARE_TOP_LEVEL_SPEC.md)，PetaLinux/OTA 流程见
[PetaLinux package 工作流](docs/PETALINUX_PACKAGE_WORKFLOW.md) 和
[bootloader OTA 工作流](docs/BOOTLOADER_OTA_WORKFLOW.md)。
完整文档导航见 [docs/README.md](docs/README.md)。
