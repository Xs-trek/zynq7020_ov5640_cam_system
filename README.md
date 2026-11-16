# cam_system

License: MIT for project-authored material; third-party and vendor exceptions
are listed in [NOTICE.md](NOTICE.md).

`cam_system` 是面向 Zynq-7020 + OV5640 的嵌入式视觉与数字云台系统，提供视频采集、PL 人脸检测、目标选择、数字跟踪、MJPEG/REST/WebSocket 服务，以及 A/B `image.ub` OTA。当前 `main` 对应 `v0.2.0`。

## 版本实现

| 版本 | 检测链路 | 跟踪控制 |
| --- | --- | --- |
| `v0.1.0` | PL 视频采集；PS 运行 NCNN YOLOv6Lite-S | alpha-beta 状态估计、检测间隔光流补偿和二阶阻尼 |
| `v0.1.1` | PL 视频采集与人脸检测加速（R9）；PS 完成输出校验、解码、NMS 和时序关联 | 时间戳 Kalman 状态估计、短时丢检保持/重关联和 jerk-limited 控制 |
| `v0.2.0` | R10 加速器直接读采集缓冲（零拷贝），驱动 UAPI v2；PS 只做解码、NMS 和关联 | 控制节拍等本帧检测结果；跟踪参数按人体运动范围设定，加边界刹车 |

各标签中的 README 记录对应版本的构建与运行边界。以下内容描述 `v0.2.0`，变化和板测数值见 [v0.2.0 发布说明](docs/V020_RELEASE_NOTES.md)。

## v0.2.0 架构

```text
OV5640 -> PL video -> DDR (reserved) -> V4L2 capture (640x480 RGB24, 30 fps)
                                          |-> dma-buf -> PL face accelerator -> decode/NMS/association
                                          |-> copy (only for CPU consumers) -> JPEG -> MJPEG
                                                                                  |
frontend select/start -> command queue -> control tick (this frame's detection or deadline)
                                       -> tracker -> V4L2 ISP/scaler controls (effective at N+2)
```

- **线程分布：** capture/control 和 main/network 运行在 Core 0；detector 和低优先级 JPEG worker 运行在 Core 1。
- **零拷贝：** 加速器经 dma-buf 读采集缓冲；拷贝路径只在有 CPU 消费者时启用，同时作为回退。
- **控制节拍：** 每帧一次，在本帧检测发布或截止时刻（默认采集后 24 ms）中较早者运行。
- **权限：** `cam-launcher` 负责物理按键启动，并以专用用户 `camsys` 运行和守护 `cam_system`。`cam-ota` 负责双槽 OTA 和 bootguard。

## 构建与验证

主机测试（不需要板子和 SDK，需要 gcc、g++、cmake、python3、ssh-keygen）：

```sh
scripts/build/test_all_host.sh
```

ARM 交叉构建，需要 PetaLinux 2022.2 SDK：

```sh
PETALINUX_SDK_ENV=/absolute/path/to/environment-setup-cortexa9t2hf-neon-xilinx-linux-gnueabi \
BUILD_DIR=/tmp/cam_system_arm scripts/build/build_arm.sh
```

默认构建只含 PL 检测后端。要构建 CPU 检测候选，加 `-DCAM_SYSTEM_ENABLE_CPU_DETECTORS=ON`，这时需要 ncnn，模型见 [模型资产](docs/MODEL_ASSETS.md)。

## 部署

1. **导入硬件。** 用 v0.2.0 的 XSA 导入 PetaLinux 工程，命令见 [hardware/README.md](hardware/README.md)。
2. **同步配方并构建镜像。** `scripts/petalinux/build_image.sh` 会复制一个基础工程，执行全部 staging 脚本，再构建并打包 `BOOT.BIN`：

   ```sh
   PLNX_BASE_PROJECT=/absolute/path/to/base-petalinux-project \
   scripts/petalinux/build_image.sh hardware/v0.2.0/zynq7020_face_accel_r10_rc2_20261105.xsa \
       /absolute/path/to/new-project
   ```

   也可以逐个执行 `scripts/petalinux/stage_*.sh`，见 [PetaLinux 集成](docs/PETALINUX_PACKAGE_WORKFLOW.md)。
3. **写卡。** 把 `BOOT.BIN`、`image.ub` 写入 SD 卡启动分区。Wi-Fi 凭据单独放到卡上，不进镜像，见 [WiFi 配置](docs/WIFI_DRIVER_WORKFLOW.md)。
4. **启动应用。** 上电后按 PS_KEY1 启动应用，手机浏览器访问 `http://<板子地址>:8080/`。

运行配置在 `/etc/cam_system.conf`，各项和默认值见 `config/cam_system.conf`。

开发镜像如果需要 SSH，必须显式提供仓库外的公钥：

```sh
CAM_SSH_AUTHORIZED_KEYS_FILE=/absolute/path/to/authorized_keys \
PLNX_PROJECT=/absolute/path/to/petalinux-project \
scripts/petalinux/stage_ssh_access_dev.sh
```

Git 不保存个人公钥、`authorized_keys` 或固定密码哈希。`ssh-access-dev` 只适用于可信实验室网络，账号密码保持锁定。公开镜像默认不集成该配方，`build_image.sh` 也不调用它。

## 硬件复现输入

仓库包含三个版本对应的 Vivado 2022.2 XSA：

| 目录 | 内容 |
| --- | --- |
| `hardware/v0.1.0/` | OV5640/V4L2 基线 |
| `hardware/v0.1.1/` | R9 PL 人脸检测加速器 |
| `hardware/v0.2.0/` | R10 加速器，`zynq7020_face_accel_r10_rc2_20261105.xsa` |

v0.2.0 XSA 的 SHA-256 是 `47bc304995d645b9662990e01925430b2e3cf86797136bb1a9a65a87958668f7`。

XSA 可用于 PetaLinux hardware import、device tree、FSBL 和 BOOT.BIN 重建。仓库不包含可编辑的 Vivado 工程、block design、约束文件或自定义 IP 源码，因此不能只依靠本仓库重新综合 PL。

## 发布边界

- **包含：** 公开发布只包含源码、必要模型、PetaLinux 配置和脚本、版本匹配的 XSA。
- **不包含：** 可启动的系统镜像、rootfs、设备凭据、内部调试记录或摄像头真实图像。

更多文档：

- 顶层契约见 [软件顶层规格](docs/CAM_SYSTEM_SOFTWARE_TOP_LEVEL_SPEC.md)；
- 完整文档导航见 [docs/README.md](docs/README.md)。
