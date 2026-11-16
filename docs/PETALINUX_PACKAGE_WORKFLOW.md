# PetaLinux 集成

## 1. 一次构建完整镜像

`scripts/petalinux/build_image.sh` 从一个已配置好的基础 PetaLinux 工程复制出新工程。它只复制配置和配方，不复制构建树。然后执行下面第 3 节的全部 staging 脚本（SSH 除外），导入 XSA，构建，并打包 `BOOT.BIN`：

```sh
PLNX_BASE_PROJECT=/absolute/path/to/base-petalinux-project \
PETALINUX_SETTINGS=/absolute/path/to/petalinux/settings.sh \
scripts/petalinux/build_image.sh \
    hardware/v0.2.0/zynq7020_face_accel_r10_rc2_20261105.xsa \
    /absolute/path/to/new-project
```

- 基础工程只读。
- 新工程第二次运行时会复用已有的构建树。
- 脚本结束时打印 `BOOT.BIN` 和 `image.ub` 的 SHA-256。

## 2. 手动导入硬件

```sh
petalinux-config \
  --get-hw-description=/absolute/path/to/hardware/v0.2.0/zynq7020_face_accel_r10_rc2_20261105.xsa \
  --silentconfig
```

## 3. 同步 meta-user 内容

设置 PetaLinux 工程路径后，执行需要的 staging 脚本：

```sh
export PLNX_PROJECT=/absolute/path/to/petalinux-project

scripts/petalinux/stage_cam_system.sh
scripts/petalinux/stage_face_accel.sh
scripts/petalinux/stage_ov5640_control.sh
scripts/petalinux/stage_cam_launcher.sh
scripts/petalinux/stage_wifi.sh
scripts/petalinux/stage_ota.sh
scripts/petalinux/stage_sudo_policy.sh
```

- `stage_cam_system.sh` 要求 Git 工作树干净，并用 `git archive` 生成当前提交的源码包，提交号写进程序（`/api/status` 的 `git_commit`）。
- `stage_face_accel.sh` 校验驱动、UAPI、权重契约和权重 SHA-256。
- `stage_sudo_policy.sh` 安装 `/etc/sudoers.d/00-secure-path`。
- 产品构建只用 PL 检测，不需要 ncnn。只有构建 CPU 检测候选时才需要 `stage_ncnn.sh`。
- `stage_libjpeg_turbo.sh` 用于基础工程里还没有 libjpeg-turbo 3.1.4 配方的情况。

## 4. 可选 SSH 开发入口

仓库不包含 `authorized_keys`。需要 SSH 时必须显式提供仓库外的公钥：

```sh
CAM_SSH_AUTHORIZED_KEYS_FILE=/absolute/path/to/authorized_keys \
PLNX_PROJECT=/absolute/path/to/petalinux-project \
scripts/petalinux/stage_ssh_access_dev.sh
```

- 该配方建一个只能用密钥登录的 `debug` 用户，并锁定密码。
- 公开镜像或不需要 SSH 的镜像不要执行这个 staging 脚本。
- `scripts/build/validate_ssh_access_dev.sh` 检查仓库里没有密钥或密码哈希，并检查脚本只接受外部公钥。

## 5. 手动构建

```sh
cd /absolute/path/to/petalinux-project
petalinux-build
petalinux-package --boot --force \
  --fsbl images/linux/zynq_fsbl.elf \
  --fpga images/linux/system.bit \
  --u-boot
```

生成物通常在 `images/linux/`。GitHub 发布不包含这些镜像。
