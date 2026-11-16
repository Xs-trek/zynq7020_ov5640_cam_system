# PS_KEY1 启动器

`cam-launcher` 通过 Linux `gpio-keys`/input 子系统监听开发板 PS_KEY1，按键按下时启动 `/usr/bin/cam_system`，并守护这个进程。

| 项目 | 值 |
| --- | --- |
| 当前按键引脚 | Zynq PS MIO10，低有效 |
| input code | `KEY_PROG1`（148） |
| 用户态程序 | `/usr/sbin/cam-launcher` |
| init 入口 | `/etc/init.d/cam-launcher` |
| 选项 | `/etc/default/cam-launcher` |
| 运行用户 | `camsys`（`video` 组） |

## 行为

- **启动与权限：** launcher 以 root 启动。启动应用时 fork 后切到 `camsys` 用户再 exec，用户不存在时拒绝启动。设备访问经 `video` 组，`/dev/face_accel` 由 udev 规则 `60-cam-system.rules` 授予。
- **重复按键：** `cam_system` 已运行时，重复按键不会创建第二个进程。按键不提供停止功能。
- **守护：** 应用异常退出时退避重启，从 1 s 起翻倍到 30 s，运行满 60 s 后归零。退出码 0 和配置错误（64、78）不重启。
- **停止：** launcher 收到 SIGTERM 时先停 `cam_system`，5 s 不退出就 SIGKILL，所以 `/etc/init.d/cam-launcher stop` 和 `restart` 对应用同样生效。
- **开机自启：** 可选，默认关。`CAM_LAUNCHER_AUTOSTART=1` 时，launcher 启动后立即像按一次键那样启动应用，按键仍然有效。只接受 0 和 1，其他值按 0 处理并记日志。
  - 传感器只在相机开着时出流，每段出流最长 `capture.max_stream_min` 分钟。
  - rootfs 在内存里，板上改这个文件重启后失效，长期打开要改配方。
- **配置：** 应用配置以 `/etc/cam_system.conf` 为准，launcher 不导出 `CAM_*` 环境变量。
- **日志：** 应用日志在 `/tmp/cam_system.log`，每次启动时超过 1 MiB 就改名为 `.old`。

## 同步与构建

```sh
scripts/build/validate_cam_launcher.sh
PLNX_PROJECT=/absolute/path/to/petalinux-project \
scripts/petalinux/stage_cam_launcher.sh

cd /absolute/path/to/petalinux-project
petalinux-build -c device-tree
petalinux-build -c cam-launcher
petalinux-build
```

守护和权限切换的逻辑由主机测试 `scripts/build/test_cam_launcher.sh` 覆盖。

更换底板、XSA 或 device tree 后，应重新确认 input 设备、按键中断计数和实际的 MIO 映射。
