# PetaLinux 集成

## 1. 导入硬件

```sh
petalinux-config \
  --get-hw-description=/absolute/path/to/hardware/v0.1.1/zynq7020_face_accel_revision9_20260719.xsa \
  --silentconfig
```

## 2. 同步 meta-user 内容

设置 PetaLinux 工程路径后执行需要的 staging 脚本：

```sh
export PLNX_PROJECT=/absolute/path/to/petalinux-project

scripts/petalinux/stage_libjpeg_turbo.sh
scripts/petalinux/stage_ncnn.sh
scripts/petalinux/stage_cam_system.sh
scripts/petalinux/stage_ov5640_control.sh
scripts/petalinux/stage_face_accel.sh
scripts/petalinux/stage_cam_launcher.sh
scripts/petalinux/stage_wifi.sh
scripts/petalinux/stage_ota.sh
```

`stage_cam_system.sh` 要求 Git 工作树干净，并通过 `git archive` 生成当前提交的
源码包。`stage_face_accel.sh` 会校验驱动/UAPI/权重契约和权重 SHA-256。

## 3. 可选 SSH 开发入口

仓库不包含 `authorized_keys`。需要 SSH 时必须显式提供仓库外公钥：

```sh
CAM_SSH_AUTHORIZED_KEYS_FILE=/absolute/path/to/authorized_keys \
PLNX_PROJECT=/absolute/path/to/petalinux-project \
scripts/petalinux/stage_ssh_access_dev.sh
```

该 recipe 使用仅密钥登录的 `debug` 用户并锁定密码；公开或无需 SSH 的镜像不要
执行此 staging 脚本。

## 4. 构建

```sh
cd /absolute/path/to/petalinux-project
petalinux-build
petalinux-package --boot --force \
  --fsbl images/linux/zynq_fsbl.elf \
  --fpga images/linux/system.bit \
  --u-boot
```

生成物通常位于 `images/linux/`。发布镜像内容可用以下命令检查：

```sh
/absolute/path/to/cam_system/scripts/build/validate_v011_release.sh images/linux
```

该验证器检查 boot 文件、rootfs manifest、device tree、应用后端、tracker、驱动和
firmware 权重。GitHub 发布不包含上述生成镜像。
