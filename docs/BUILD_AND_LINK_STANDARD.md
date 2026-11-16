# 构建与链接

## 依赖

- CMake 3.10+
- PetaLinux 2022.2 Cortex-A9 SDK
- libjpeg-turbo
- OpenSSL libcrypto
- pthread、math、dl
- ncnn 静态库：只在 `-DCAM_SYSTEM_ENABLE_CPU_DETECTORS=ON` 时需要

## 主机测试

```sh
scripts/build/test_all_host.sh
```

测试用主机编译器构建独立单元，不生成板端程序。

- 内核驱动经 `tests/kshim/` 编译成主机程序测试，覆盖逻辑和资源释放，不覆盖真实时序。
- `scripts/build/test_all_arm.sh` 在 SDK 环境下用 user-mode qemu 运行依赖目标算术和数据布局的测试。

## ARM 构建

默认 SDK 路径可用 `PETALINUX_SDK_ENV` 覆盖：

```sh
PETALINUX_SDK_ENV=/absolute/path/to/environment-setup-cortexa9t2hf-neon-xilinx-linux-gnueabi \
BUILD_DIR=/tmp/cam_system_arm \
scripts/build/build_arm.sh
```

脚本使用 `cmake/toolchain.cmake`。编译完成后调用 `verify_elf.sh`，检查 ARM ELF 架构、动态解释器和禁止项。

## 构建选项

| 选项 | 默认 | 作用 |
| --- | --- | --- |
| `CAM_SYSTEM_ENABLE_CPU_DETECTORS` | `OFF` | 编译 ncnn、UltraFace、libfacedetection 检测候选并安装 `assets/models/` |
| `CAM_SYSTEM_DEPENDENCY_MODE` | `EXTERNAL` | `EXTERNAL` 从 `CAM_SYSTEM_NCNN_ROOT`、`CAM_SYSTEM_JPEGTURBO_ROOT` 读取 SDK 外部安装目录；`YOCTO` 从 recipe sysroot 查找 |
| `CAM_SYSTEM_WERROR` | `ON` | 项目自己的代码警告即错误 |

**构建身份：** 用 `git archive` 导出的源码树会把提交号写进 `cmake/git_commit.txt`（`.gitattributes` 的 `export-subst`）；Git 工作树则直接问 Git。程序在 `/api/status` 的 `git_commit` 里报告这个值。

项目启用 `-Wall -Wextra -Wformat=2`，不写入 RPATH。安装目标：

- `/usr/bin/cam_system`；
- `/etc/cam_system.conf`；
- 打开 CPU 候选时，模型安装到 `/usr/share/cam-system/models/`。

PL 权重由 `cam-face-accel` 配方安装到 firmware 目录。
