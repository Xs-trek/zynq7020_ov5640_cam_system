# cam_system 测试目录

本目录预留给主机侧自动化测试。

只有当测试可以在不产生板端硬件副作用的前提下执行时，才在本目录新增可执行测试。

全部主机测试由 `scripts/build/test_all_host.sh` 依次运行。其中 `face_accel_driver_test` 把驱动源码接到 `tests/kshim/` 和一个按 R10 契约检查参数的加速器模型上，验证逻辑和资源释放，不验证真实时序。`ov5640_driver_test` 同样把传感器驱动接到 `tests/kshim/kshim_v4l2.h` 和一个只在上电时应答 I2C 的传感器模型上，验证什么时候有电。

当前主机侧测试入口：

- `scripts/build/test_ws_protocol.sh`
- `scripts/build/test_mjpeg_framing.sh`
- `scripts/build/test_http_parse.sh`
- `scripts/build/test_status_json.sh`
- `scripts/build/test_net_server.sh`
- `scripts/build/test_cmd_handler_route.sh`
- `scripts/build/test_frame_view_timeline.sh`
- `scripts/build/test_face_temporal_tracker.sh`
- `scripts/build/test_face_accel_diagnostics.sh`
- `scripts/build/test_tracker_servo.sh`
- `scripts/build/test_tracker_detection_update.sh`
- `scripts/build/test_tracker_closed_loop.sh`
- `scripts/build/test_tracker_pipeline_replay.sh`
- `scripts/build/test_jpeg_worker.sh`
- `scripts/build/check_project_static.sh`
