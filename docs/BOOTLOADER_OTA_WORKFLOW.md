# A/B image.ub OTA

OTA 更新范围限定为 `image.ub`。`BOOT.BIN`、FSBL、U-Boot 和 bitstream 不在
无人值守 OTA 范围内。

## 存储布局

```text
/image.ub                 baseline fallback
/ota/slot_a/image.ub      slot A
/ota/slot_b/image.ub      slot B
/ota/state/               manifest and rollback hooks
/uEnv.txt                 active U-Boot hook
```

## 状态机

- `good`：当前槽已确认。
- `pending`：新槽首次启动；U-Boot 在启动前预置 rollback hook。
- `rollback`：pending 未确认或失败后切回上一槽。
- `disabled`：移除 OTA hook，使用根目录 `/image.ub`。

安装时先写非活动槽临时文件，校验 SHA-256 后原子改名，再更新 manifest 和
`uEnv.txt`。所有槽位 FIT 在启动前必须通过 U-Boot `iminfo`；槽加载失败时回退到
baseline `image.ub`。pending 状态由 watchdog 和 `cam-ota-bootguard` 共同保护，
未执行 `mark-good` 的启动会在下一次引导回滚。

## 同步与验证

```sh
scripts/build/validate_ota_env.sh
scripts/build/validate_boot_scr.sh
PLNX_PROJECT=/absolute/path/to/petalinux-project \
scripts/petalinux/stage_ota.sh
```

构建 U-Boot/OTA 组件：

```sh
cd /absolute/path/to/petalinux-project
petalinux-build -c u-boot-xlnx
petalinux-build -c cam-ota
petalinux-build
```

## 板端命令

```sh
cam-ota init
cam-ota status
cam-ota install /tmp/image.ub <version>
cam-ota mark-good <version>
cam-ota rollback
cam-ota disable
```

首次部署应分别验证正常安装/确认、未确认自动回滚、槽文件损坏回退和 disable 后
baseline 启动。OTA 操作前必须确保具有可恢复的启动介质。
