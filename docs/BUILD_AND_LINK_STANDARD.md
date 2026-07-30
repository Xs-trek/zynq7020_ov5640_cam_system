# 构建与链接

## 依赖

- CMake 3.10+
- PetaLinux 2022.2 Cortex-A9 SDK
- ncnn 静态库
- libjpeg-turbo
- OpenSSL libcrypto
- pthread、math、dl

## 主机测试

```sh
scripts/build/test_all_host.sh
```

测试使用主机编译器构建独立单元，不生成板端程序。

## ARM 构建

默认 SDK 路径可用 `PETALINUX_SDK_ENV` 覆盖：

```sh
PETALINUX_SDK_ENV=/absolute/path/to/environment-setup-cortexa9t2hf-neon-xilinx-linux-gnueabi \
BUILD_DIR=/tmp/cam_system_arm \
scripts/build/build_arm.sh
```

脚本使用 `cmake/toolchain.cmake`，完成编译后调用 `verify_elf.sh` 检查 ARM ELF
架构、动态解释器和禁止项。

## 依赖模式

`CAM_SYSTEM_DEPENDENCY_MODE` 支持：

- `EXTERNAL`：从 `CAM_SYSTEM_NCNN_ROOT` 和 `CAM_SYSTEM_JPEGTURBO_ROOT` 读取 SDK
  外部安装目录，用于直接交叉编译。
- `YOCTO`：从 recipe sysroot 查找 ncnn 和 libjpeg-turbo，用于 PetaLinux 构建。

项目启用 `-Wall -Wextra -Wformat=2`，不写入 RPATH。安装目标为
`/usr/bin/cam_system`；NCNN 兼容模型安装到 `/usr/share/cam-system/models/`，
PL 权重由 `cam-face-accel` recipe 安装到 firmware 目录。
