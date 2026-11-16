# 前端

`gimbal/index.html` 是手机 Gimbal 应用的页面。Android 工程只是 WebView 外壳，不在本仓库。

更新应用：把这个文件复制到 Gimbal 工程的 `app/src/main/assets/index.html`，再重新打包 APK。

前端只用于手机查看和基础控制，接口约定见 `docs/CAM_SYSTEM_SOFTWARE_TOP_LEVEL_SPEC.md` 和 `docs/WEBSOCKET_PROTOCOL.md`。
