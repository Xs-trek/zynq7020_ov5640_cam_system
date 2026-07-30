# WiFi 集成

项目使用 `morrownr/88x2bu-20210702` 的源码 recipe，固定提交
`fecac340fb117eb979f4bb6d28e29730384c382b`，随 PetaLinux 内核 ABI 构建
`8812bu.ko`。驱动许可证为 GPL-2.0-only。

## 同步与构建

```sh
PLNX_PROJECT=/absolute/path/to/petalinux-project \
scripts/petalinux/stage_wifi.sh

cd /absolute/path/to/petalinux-project
petalinux-build -c 8812bu
petalinux-build
```

recipe 关闭 Bluetooth coexistence 和高频驱动日志，并设置模块自动加载。

## 运行配置

`wifi-init` 按以下顺序查找配置：

1. `/etc/wifi-init.conf`
2. `/etc/wifi.conf`
3. SD 启动分区中的配置文件

配置示例：

```sh
WIFI_SSID='example'
WIFI_PSK='replace-me'
```

不得把实际 WiFi 凭据提交到 Git。缺少配置时启动脚本不会阻塞系统启动；需要串口
交互输入时可执行：

```sh
WIFI_INTERACTIVE=1 /etc/init.d/S50wifi restart
```

板端应确认模块加载、`wpa_state=COMPLETED`、DHCP 地址和目标前端连通性。
