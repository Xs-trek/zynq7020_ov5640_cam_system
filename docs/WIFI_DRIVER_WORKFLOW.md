# WiFi 集成

项目使用 `morrownr/88x2bu-20210702` 的源码 recipe，固定提交 `fecac340fb117eb979f4bb6d28e29730384c382b`，随 PetaLinux 内核 ABI 构建 `8812bu.ko`。驱动许可证为 GPL-2.0-only。

## 同步与构建

```sh
PLNX_PROJECT=/absolute/path/to/petalinux-project \
scripts/petalinux/stage_wifi.sh

cd /absolute/path/to/petalinux-project
petalinux-build -c 8812bu
petalinux-build
```

recipe 关闭 Bluetooth coexistence 和高频驱动日志，并设置模块自动加载。

## 凭据

凭据与程序分开：

| 部分 | 内容 | 位置 |
|---|---|---|
| 程序 | `S50wifi`（`wifi-init` 配方），不含任何网络信息 | 镜像 |
| 凭据文件 | SSID 和密钥 | 本机保管，写卡时复制到 SD 启动分区根目录的 `wifi.txt` |
| 写入工具 | `scripts/board/provision_wifi.sh`，校验格式，不输出文件内容 | 仓库 |

凭据不进镜像，也不进仓库。RAM rootfs 下 `/etc` 的文件只能来自构建，所以 `S50wifi` 只在 SD 启动分区查找 `wifi.txt`、`wifi-init.conf`、`wifi.conf`，不搜索 `/etc`。换网络时只换卡上的文件，不用重建镜像。

文件是 `KEY=VALUE` 文本，UTF-8 编码，Windows 换行和 BOM 都可以：

```text
SSID=your-ssid
PASSPHRASE=your-password
```

也可以用预计算的 WPA PSK：

```text
SSID=your-ssid
PSK=64_hex_wpa_psk
```

约束：

- 键名大写，`=` 两边不加空格，值不加引号。`#` 开头的行是注释。
- `PASSPHRASE` 是 8 到 63 个字符。`PSK` 是 64 位十六进制，它和密码一样是入网凭据，不能公开。
- FAT 分区没有文件权限，拿到卡就能读到文件，所以这个方式只用于调试。
- `/etc/init.d/S50wifi check <file>` 只校验格式，不输出内容。

写卡（启动分区已挂载，目录里有 `BOOT.BIN`）：

```sh
scripts/board/provision_wifi.sh /path/to/wifi.txt /path/to/mounted/boot-partition
```

## 运行

- 板子是 Wi-Fi 客户端，连路由器或手机热点，没有热点模式。
- `S50wifi` 启动 `wpa_supplicant` 后，最多等 25 s 关联完成，然后启动 `udhcpc`。
- 超时后 `wpa_supplicant` 和 `udhcpc` 继续在后台尝试，所以路由器晚于板子启动时仍能连上。系统启动不等网络。
- 没有凭据时启动脚本不会阻塞系统启动。需要串口交互输入时执行：

```sh
WIFI_INTERACTIVE=1 /etc/init.d/S50wifi restart
```

板端应确认模块已加载、`wpa_state=COMPLETED`、拿到 DHCP 地址、前端可以连通。

## 已知限制

USB 网卡在部分环境下会间歇丢包。页面画面卡顿时，可以用 `jpeg.quality`、`stream.max_fps` 降低推流码率，见 `config/cam_system.conf`。
