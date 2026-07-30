# v0.1.1 发布说明

日期：2026-07-30

## 主要变化

- 默认检测后端改为 PL `face_accel`，PS 完成输出校验、解码、NMS 和时序关联。
- 使用三槽 latest-only mailbox 解耦采集、检测和 JPEG 编码。
- 数字云台采用稳定参考框、时间戳 Kalman 估计、短时丢检保持/重关联和
  jerk-limited ISP 控制。
- 删除运行时光流和 OpenCV 依赖。
- 集成物理按键 launcher 和 A/B `image.ub` OTA/bootguard。
- SSH 开发入口只允许显式注入仓库外公钥，账号密码保持锁定。

## 验证摘要

- 主机自动化测试和 ARM 交叉构建通过。
- 板端采集、检测和推流稳态约 30 fps。
- 前端检测、目标选择、跟踪启停和真实人脸数字云台效果已完成验收。
- 测试过程未保存摄像头原始帧或 JPEG。

## 发布内容与限制

- GitHub 仅发布源码、必要模型、PetaLinux 配置/脚本和两个版本对应的 XSA。
- 不发布 `image.ub`、`BOOT.BIN`、rootfs、设备凭据或内部调试记录。
- XSA 可用于 PetaLinux 平台重建，但仓库不包含自定义 PL IP 源码，不能独立重新
  综合完整 PL。
- HTTP/WebSocket 没有认证和 TLS，仅适用于可信隔离局域网。
