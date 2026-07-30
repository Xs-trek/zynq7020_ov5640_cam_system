# PS_KEY1 启动器

`cam-launcher` 通过 Linux `gpio-keys`/input 子系统监听开发板 PS_KEY1，并在按键
按下时启动 `/usr/bin/cam_system`。

| 项目 | 值 |
| --- | --- |
| 当前按键引脚 | Zynq PS MIO10，低有效 |
| input code | `KEY_PROG1`（148） |
| 用户态程序 | `/usr/sbin/cam-launcher` |
| init 入口 | `/etc/init.d/cam-launcher` |

若 `cam_system` 已运行，重复按键不会创建第二个进程。启动器不提供停止、崩溃重启
或业务控制功能。

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

更换底板、XSA 或 device tree 后，应重新确认 input 设备、按键中断计数和实际
MIO 映射。
