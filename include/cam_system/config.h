/* config.h — 全局硬件常量与配置 */
#ifndef CONFIG_H
#define CONFIG_H

/* ── 帧参数 ── */
#define IMG_W           640
#define IMG_H           480
#define IMG_BPP         3
#define FRAME_SIZE      (IMG_W * IMG_H * IMG_BPP)   /* 921600 */
#define FRAME_BUF_CNT   3   /* cached copies, for consumers that read frames on the CPU */
/* V4L2 buffers.  The mailbox can hold two for direct readers (the newest frame and one being
 * read); the rest stay with the hardware. */
#define CAPTURE_BUF_CNT 6

/* Crop/scaler commands are issued after V4L2 frame N is dequeued.  Board
 * calibration for this hardware generation maps that command to frame N+2.
 * The release acceptance trace must verify this single value; consumers must
 * never average multiple possible viewport transforms. */
#define ISP_APPLY_DELAY_FRAMES 2

/* ── PS CPU 亲和性 ── */
#define CAM_CPU_CONTROL_CORE   0   /* capture/control + HTTP/WebSocket */
#define CAM_CPU_CAPTURE_CORE   CAM_CPU_CONTROL_CORE
#define CAM_CPU_NETWORK_CORE   CAM_CPU_CONTROL_CORE
#define CAM_CPU_DETECTOR_CORE  1   /* 检测线程，大部分时间在等 PL 完成 */
#define CAM_CPU_JPEG_CORE      1   /* JPEG 与等待 PL 完成的检测线程共享 Core 1 */

/* ── ISP 窗口 / 数字云台参数 ── */
/* 2x subsample 后的窗口（offset=(16,4) 时居中） */
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
#define WS_STATUS_PERIOD_SEC            1.0
/* A client that connects and says nothing must not keep a slot. */
#ifndef HTTP_REQUEST_TIMEOUT_SEC
#define HTTP_REQUEST_TIMEOUT_SEC        10.0
#endif
/* A connection that goes silent is given up after about ten seconds; the page reconnects. */
#define CLIENT_KEEPALIVE_IDLE_SEC       3
#define CLIENT_KEEPALIVE_INTERVAL_SEC   2
#define CLIENT_KEEPALIVE_PROBES         3
#define CLIENT_TCP_USER_TIMEOUT_MS      10000u

/* ── 检测 ── */
#define DETECTOR_CONF_THRESH  0.5f
#define DETECTOR_MAX_DET      20
#define DETECTOR_MAX_CLASSES  80    /* 只有 CPU 候选模型有类别；人脸模型只有一类 */
/* 只用于 CPU 候选模型 YOLOv6 */
#define YOLO_INPUT_W          256
#define YOLO_INPUT_H          192
#define YOLO_NMS_THRESH       0.5f
#define DETECTION_CLASS_FACE  80

/* ── JPEG ── */
#define JPEG_QUALITY          70        /* default of jpeg.quality (app_config) */

/* ── 模型路径（板上部署路径） ── */
#define MODEL_PARAM_PATH      "/usr/share/cam-system/models/yolov6lite_s/yolov6lite_s.ncnn.param"
#define MODEL_BIN_PATH        "/usr/share/cam-system/models/yolov6lite_s/yolov6lite_s.ncnn.bin"
#define ULTRAFACE_RFB320_PARAM_PATH \
    "/usr/share/cam-system/models/ultraface_rfb_320/RFB-320.param"
#define ULTRAFACE_RFB320_BIN_PATH \
    "/usr/share/cam-system/models/ultraface_rfb_320/RFB-320.bin"
#define SCRFD_500M_640_PARAM_PATH \
    "/usr/share/cam-system/models/scrfd_500m_640/scrfd_500m_shape640x640.param"
#define SCRFD_500M_640_BIN_PATH \
    "/usr/share/cam-system/models/scrfd_500m_640/scrfd_500m_shape640x640.bin"
#define SCRFD_1G_640_PARAM_PATH \
    "/usr/share/cam-system/models/scrfd_1g_640/scrfd_1g_shape640x640.param"
#define SCRFD_1G_640_BIN_PATH \
    "/usr/share/cam-system/models/scrfd_1g_640/scrfd_1g_shape640x640.bin"
#define SCRFD_2_5G_KPS_640_PARAM_PATH \
    "/usr/share/cam-system/models/scrfd_2_5g_kps_640/scrfd_2.5g_bnkps_shape640x640.param"
#define SCRFD_2_5G_KPS_640_BIN_PATH \
    "/usr/share/cam-system/models/scrfd_2_5g_kps_640/scrfd_2.5g_bnkps_shape640x640.bin"

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
/* Path bandwidth for v0.2.0 timing: the tick gets this frame's detection (0 frames) with the
 * 24 ms deadline.  brake_axis uses the same omega and jerk (docs/V020_RELEASE_NOTES.md).
 * Speed limits: the P99 of a face's image speed over the human motion envelope
 * (docs/V020_RELEASE_NOTES.md).  Acceleration stays at 1800 px/s^2, 2 px/frame^2: the integer
 * delta <= 3 line bounds it.
 * The single-frame step line of the tests is the speed limit times the frame interval. */
#define TRACK_VIEW_PATH_OMEGA_X     8.0f
#define TRACK_VIEW_MAX_VEL_X        780.0f
#define TRACK_VIEW_MAX_ACCEL_X      1800.0f
#define TRACK_VIEW_MAX_JERK_X       18000.0f
#define TRACK_VIEW_PATH_OMEGA_Y     8.0f
#define TRACK_VIEW_MAX_VEL_Y        540.0f
#define TRACK_VIEW_MAX_ACCEL_Y      1800.0f
#define TRACK_VIEW_MAX_JERK_Y       18000.0f
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
