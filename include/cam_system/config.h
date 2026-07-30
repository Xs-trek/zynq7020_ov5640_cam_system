/* config.h — 全局硬件常量与配置 */
#ifndef CONFIG_H
#define CONFIG_H

/* ── 帧参数 ── */
#define IMG_W           640
#define IMG_H           480
#define IMG_BPP         3
#define FRAME_SIZE      (IMG_W * IMG_H * IMG_BPP)   /* 921600 */
#define FRAME_BUF_CNT   3

/* Crop/scaler commands are issued after V4L2 frame N is dequeued.  Board
 * calibration for this hardware generation maps that command to frame N+2.
 * The release acceptance trace must verify this single value; consumers must
 * never average multiple possible viewport transforms. */
#define ISP_APPLY_DELAY_FRAMES 2

/* ── PS CPU 亲和性 ── */
#define CAM_CPU_CONTROL_CORE   0   /* capture/control + HTTP/WebSocket */
#define CAM_CPU_CAPTURE_CORE   CAM_CPU_CONTROL_CORE
#define CAM_CPU_NETWORK_CORE   CAM_CPU_CONTROL_CORE
#define CAM_CPU_DETECTOR_CORE  1   /* 检测线程；PL 加速路径等待 IRQ，YOLO 回退使用 NCNN */
#define CAM_CPU_JPEG_CORE      1   /* JPEG 与等待 PL 完成的检测线程共享 Core 1 */

/* ── OV5640 Sensor 物理参数 ── */
#define SENSOR_FULL_W         2624      /* 物理窗口最大宽 (X: 0~2623) */
#define SENSOR_FULL_H         1944      /* 物理窗口最大高 (Y: 4~1947) */
#define SENSOR_Y_START_MIN    4         /* Y 起始最小值 */
#define SENSOR_X_START_MIN    0         /* X 起始最小值 */

/* ── ISP 窗口 / 数字云台参数 ── */
#define SUBSAMPLE_FACTOR      2         /* 当前 2x subsample 固定 */
#define ZOOM_MIN              100       /* 1.0x */
#define ZOOM_MAX              200       /* 2.0x (P3: subsample=2x 下极限) */
#define ZOOM_DEFAULT          100       /* 默认 1.0x */

/* zoom=1x 时的 sensor 窗口（与 init 寄存器表一致） */
#define ISP_WIN_1X_W          2624      /* 0x0000 ~ 0x0A3F */
#define ISP_WIN_1X_H          1944      /* 0x0004 ~ 0x079B */
#define ISP_WIN_1X_CX         1312      /* 中心 X */
#define ISP_WIN_1X_CY         976       /* 中心 Y = (4+1947)/2 ≈ 976 */

/* subsampled 后中心（zoom=1x, offset=(16,4) → 居中值） */
#define SUBSAMPLED_1X_W       1312      /* 2624/2 */
#define SUBSAMPLED_1X_H       972       /* 1944/2 */

/* PAN 极限：offset 最大范围（subsampled - output） */
#define PAN_MAX_X             672       /* 1312 - 640 */
#define PAN_MAX_Y             492       /* 972 - 480 */

/* ── 网络 ── */
#define HTTP_PORT             8080
#define MAX_HTTP_CLIENTS      4
#define MAX_WS_CLIENTS        4
#define MJPEG_BOUNDARY        "frameboundary"
#define CONTROL_CMD_QUEUE_LEN 8
#define MJPEG_SOCKET_SNDBUF_BYTES       (128 * 1024)
#define MJPEG_OUTQ_DROP_THRESHOLD_BYTES (96 * 1024)
#define MJPEG_SEND_WAIT_BUDGET_MS       12
#define HTTP_SEND_WAIT_BUDGET_MS        50

/* ── 检测器上限与 YOLOv6Lite-S 兼容回退 ── */
#define YOLO_INPUT_W          256
#define YOLO_INPUT_H          192
#define YOLO_CONF_THRESH      0.5f
#define YOLO_NMS_THRESH       0.5f
#define DETECTOR_MAX_DETECTIONS 20
#define YOLO_NUM_CLASSES      80
#define DETECTION_CLASS_FACE  80

/* ── JPEG ── */
#define JPEG_QUALITY          70

/* ── 模型路径（板上部署路径） ── */
#define MODEL_PARAM_PATH      "/usr/share/cam-system/models/yolov6lite_s/yolov6lite_s.ncnn.param"
#define MODEL_BIN_PATH        "/usr/share/cam-system/models/yolov6lite_s/yolov6lite_s.ncnn.bin"

/* ── 跟踪参数：时间戳 Kalman + 独立 viewport 控制器 ── */
#define TRACK_HISTORY_SIZE          8
#define TRACK_KF_ACCEL_VAR_XY       100000.0f
#define TRACK_KF_ACCEL_VAR_SCALE    1.5f
#define TRACK_ASSOC_NIS_GATE        9.21f
#define TRACK_ASSOC_EXACT_NIS_GATE  25.0f
#define TRACK_REBUILD_IOU_GATE      0.10f
#define TRACK_REBIND_CONFIRM_FRAMES 2
#define TRACK_ASSOC_PRIVATE_VAR_SCALE 4.0f
#define TRACK_UNASSOCIATED_VAR_SCALE 9.0f
#define TRACK_MEAS_SIGMA_BASE_PX    3.0f
#define TRACK_MEAS_Y_VAR_SCALE      1.6f
#define TRACK_FACE_ANCHOR_Y_RATIO   0.42f
#define TRACK_VIEW_ANCHOR_X_RATIO   0.50f
#define TRACK_VIEW_ANCHOR_Y_RATIO   0.42f
#define TRACK_EXACT_GATE_BASE_PX    32.0f
#define TRACK_EXACT_GATE_SPEED_PX_S 900.0f
#define TRACK_EXACT_GATE_MAX_PX     150.0f

#define TRACK_MISS_GRACE_SEC        0.12f
#define TRACK_COAST_SEC             0.50f
#define TRACK_HOLD_SEC              0.75f
#define TRACK_LOST_SEC              1.50f
#define TRACK_RECOVER_CONFIRM_FRAMES 2

/* Target-state retention and viewport motion are deliberately separate.
 * The association estimator may coast for TRACK_COAST_SEC, but the camera
 * follows an unobserved constant-velocity target for only this bounded time
 * before entering the same jerk-limited brake used by HOLD/LOST/RECOVER. */
#define TRACK_COMMAND_COAST_SEC     0.15f

/* Frontend identity tracking uses a bounded centre-velocity prediction.
 * Retention is not tied to camera HOLD state, and bbox edges are never
 * extrapolated independently because near-face scale changes make that model
 * geometrically unstable. */
#define TRACK_TEMPORAL_RETAIN_SEC       0.75f
#define TRACK_TEMPORAL_PREDICT_MAX_SEC  0.35f
#define TRACK_TEMPORAL_MAX_SPEED_PX_S   900.0f
#define TRACK_TEMPORAL_VELOCITY_BLEND   0.35f

/* Digital-gimbal screen composer.  The detector/Kalman layer supplies one
 * target position at the N+2 command-effective time.  The composer acts only
 * on that position: detector velocity is deliberately not a camera command.
 *
 * Small errors are ignored with Schmitt hysteresis.  The entry floors cover
 * the measured bbox noise scale; live KF covariance may widen them up to the
 * caps below.  A three-frame fading local-linear fit then supplies a mutually
 * consistent reference position and velocity to the final path generator. */
#define TRACK_CONTROL_FPS           30.0f
#define TRACK_COMMAND_LEAD_SEC      \
    ((float)ISP_APPLY_DELAY_FRAMES / TRACK_CONTROL_FPS)
#define TRACK_VIEW_NOISE_SIGMA_MULT 1.5f
#define TRACK_VIEW_NOISE_ENTER_X_MIN_PX 6.0f
#define TRACK_VIEW_NOISE_ENTER_Y_MIN_PX 8.0f
#define TRACK_VIEW_NOISE_ENTER_X_MAX_PX 12.0f
#define TRACK_VIEW_NOISE_ENTER_Y_MAX_PX 16.0f
#define TRACK_VIEW_NOISE_EXIT_RATIO 0.50f
#define TRACK_REFERENCE_WINDOW_FRAMES 3.0f
#define TRACK_REFERENCE_ALPHA       \
    (2.0f * (2.0f * TRACK_REFERENCE_WINDOW_FRAMES - 1.0f) / \
     (TRACK_REFERENCE_WINDOW_FRAMES * \
      (TRACK_REFERENCE_WINDOW_FRAMES + 1.0f)))
#define TRACK_REFERENCE_BETA        \
    (6.0f / (TRACK_REFERENCE_WINDOW_FRAMES * \
             (TRACK_REFERENCE_WINDOW_FRAMES + 1.0f)))
#define TRACK_VIEW_PATH_OMEGA_X     6.0f
#define TRACK_VIEW_MAX_VEL_X        450.0f
#define TRACK_VIEW_MAX_ACCEL_X      1800.0f
#define TRACK_VIEW_MAX_JERK_X       7200.0f
#define TRACK_VIEW_PATH_OMEGA_Y     6.0f
#define TRACK_VIEW_MAX_VEL_Y        300.0f
#define TRACK_VIEW_MAX_ACCEL_Y      1800.0f
#define TRACK_VIEW_MAX_JERK_Y       7200.0f
#define TRACK_VIEW_MAX_STEP_X_PX    \
    (TRACK_VIEW_MAX_VEL_X / TRACK_CONTROL_FPS)
#define TRACK_VIEW_MAX_STEP_Y_PX    \
    (TRACK_VIEW_MAX_VEL_Y / TRACK_CONTROL_FPS)

/* The same moving crop is also the detector input.  These are acceptance
 * margins for the control anchor, not a demand that the whole bbox be visible.
 * The vertical value protects the 42% control anchor, not the full near-face
 * bbox, which may be physically impossible to contain. */
#define TRACK_VIEW_CORE_MARGIN_X_PX 96.0f
#define TRACK_VIEW_CORE_MARGIN_Y_PX 24.0f

/* ISP offset 默认值 */
#define ISP_DEFAULT_XOFF    16
#define ISP_DEFAULT_YOFF    4

#endif /* CONFIG_H */
