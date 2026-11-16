/* main.cpp — cam_system 入口 */
#define LOG_TAG "main"

#include <errno.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <signal.h>
#include <sys/signalfd.h>
#include <sysexits.h>
#include <time.h>
#include <unistd.h>
#include <pthread.h>
#include <sched.h>

extern "C" {
#include "cam_system/version.h"
#include "common/app_config.h"
#include "common/log.h"
#include "cam_system/config.h"
#include "cam_system/app_state.h"
#include "cam_system/frame_mailbox.h"
#include "platform/hal_ov5640.h"
#include "platform/hal_frmbuf.h"
#include "capture/capture.h"
#include "control/track_trace.h"
#include "media/jpeg_enc.h"
#include "media/jpeg_worker.h"
#include "network/net_server.h"
#include "cam_system/tracker.h"
}
#include "detect/detector.h"

/* ── 全局状态 ── */
app_state_t g_state;
uint8_t *g_frame_pool[CAPTURE_BUF_CNT];
uint8_t *g_cached_pool[FRAME_BUF_CNT];

/* 全局 tracker 实例 */
tracker_t g_tracker;

void app_state_init(app_state_t *s)
{
    struct timespec start;

    clock_gettime(CLOCK_MONOTONIC, &start);
    s->start_time_sec = (double)start.tv_sec + (double)start.tv_nsec / 1000000000.0;
    s->cam_enabled = 0;
    s->detector_enabled = 0;
    s->quit = 0;
    s->exit_code = 0;
    s->tracking_active = 0;
    s->tracking_target_id = -1;
    s->latest_frame_idx = -1;
    s->frame_seq = 0;
    s->status_seq = 0;

    for (int i = 0; i < FRAME_BUF_CNT; i++) {
        s->frame_views[i] = frame_view_t{};
        s->frame_views[i].roi_w = SUBSAMPLED_1X_W;
        s->frame_views[i].roi_h = SUBSAMPLED_1X_H;
        s->frame_views[i].output_w = IMG_W;
        s->frame_views[i].output_h = IMG_H;
        s->frame_views[i].flip_x = 1;
        s->frame_views[i].flip_y = 1;
        s->frame_views[i].scaler_enabled = 1;
    }
    s->pred_box = predict_box_t{};
    for (int i = 0; i < DETECTOR_MAX_DET; i++)
        s->detections[i] = detection_t{};

    pthread_mutex_init(&s->jpeg_mutex, NULL);
    pthread_mutex_init(&s->det_mutex, NULL);
    pthread_condattr_t det_cond_attr;
    pthread_condattr_init(&det_cond_attr);
    pthread_condattr_setclock(&det_cond_attr, CLOCK_MONOTONIC);
    pthread_cond_init(&s->det_cond, &det_cond_attr);
    pthread_condattr_destroy(&det_cond_attr);
    pthread_mutex_init(&s->cmd_mutex, NULL);
    pthread_mutex_init(&s->settings_mutex, NULL);
    pthread_mutex_init(&s->pred_mutex, NULL);
    frame_mailbox_init(s);

    s->jpeg_buf = NULL;
    s->jpeg_size = 0;
    s->jpeg_seq = 0;
    s->det_count = 0;
    s->det_seq = 0;
    s->det_frame_seq = 0;
    s->det_frame_view = s->frame_views[0];
    s->det_publish_time_sec = 0.0;
    s->track_cmd = TRACK_CMD_NONE;
    s->track_target_id = -1;
    s->cmd_head = 0;
    s->cmd_tail = 0;
    s->cmd_count = 0;
    for (int i = 0; i < CONTROL_CMD_QUEUE_LEN; i++) {
        s->cmd_queue[i] = TRACK_CMD_NONE;
        s->cmd_target_queue[i] = -1;
    }
    s->max_det = 10;

    memset(s->enabled_classes, 0, sizeof(s->enabled_classes));
    s->enabled_classes[0] = 0;
    s->enabled_class_count = 1;
}

/* ── 信号处理 ──
 * SIGINT/SIGTERM/SIGHUP are blocked before any thread is created, so every thread inherits the
 * mask and none of them is interrupted; the network event loop reads them from a signalfd and
 * requests an orderly shutdown.  SIGPIPE is ignored (sockets report EPIPE instead).
 * Returns the signalfd, or -1. */
static int setup_signals(void)
{
    sigset_t mask;
    int fd;
    int rc;

    if (signal(SIGPIPE, SIG_IGN) == SIG_ERR) {
        LOG_E("signal SIGPIPE: %s", strerror(errno));
        return -1;
    }
    sigemptyset(&mask);
    sigaddset(&mask, SIGINT);
    sigaddset(&mask, SIGTERM);
    sigaddset(&mask, SIGHUP);
    rc = pthread_sigmask(SIG_BLOCK, &mask, NULL);
    if (rc != 0) {
        LOG_E("pthread_sigmask: %s", strerror(rc));
        return -1;
    }
    fd = signalfd(-1, &mask, SFD_NONBLOCK | SFD_CLOEXEC);
    if (fd < 0) {
        LOG_E("signalfd: %s", strerror(errno));
        return -1;
    }
    return fd;
}

/* ── CPU 亲和性 ── */
static int bind_current_to_core(const char *role, int core)
{
    cpu_set_t cpuset;
    CPU_ZERO(&cpuset);
    CPU_SET(core, &cpuset);
    int rc = pthread_setaffinity_np(pthread_self(), sizeof(cpuset), &cpuset);
    if (rc != 0) {
        LOG_E("Failed to bind %s to Core %d: %s", role, core, strerror(rc));
        return -1;
    }
    LOG_I("%s bound to Core %d", role, core);
    return 0;
}

static int create_thread_on_core(pthread_t *thread, const char *role, int core,
                                 void *(*start_routine)(void *), void *arg)
{
    pthread_attr_t attr;
    int rc = pthread_attr_init(&attr);
    if (rc != 0) {
        LOG_E("Failed to init %s attr: %s", role, strerror(rc));
        return -1;
    }

    cpu_set_t cpuset;
    CPU_ZERO(&cpuset);
    CPU_SET(core, &cpuset);
    rc = pthread_attr_setaffinity_np(&attr, sizeof(cpuset), &cpuset);
    if (rc != 0) {
        LOG_E("Failed to set %s attr Core %d: %s", role, core, strerror(rc));
        pthread_attr_destroy(&attr);
        return -1;
    }

    rc = pthread_create(thread, &attr, start_routine, arg);
    pthread_attr_destroy(&attr);
    if (rc != 0) {
        LOG_E("Failed to create %s: %s", role, strerror(rc));
        return -1;
    }

    LOG_I("%s created on Core %d", role, core);
    return 0;
}

/* ── 启动参数与配置 ── */
static void print_usage(FILE *out, const char *prog)
{
    fprintf(out,
            "Usage: %s [-c FILE] [-V] [-h]\n"
            "  -c FILE  configuration file (default %s, optional)\n"
            "  -V       print version and exit\n"
            "  -h       print this help and exit\n",
            prog, APP_CONFIG_DEFAULT_PATH);
}

static void log_config_line(const char *line, void *ctx)
{
    (void)ctx;
    LOG_I("config %s", line);
}

/* Parses arguments and loads the configuration.  Returns -1 to continue, otherwise an exit code. */
static int load_startup_config(int argc, char *argv[])
{
    const char *path = APP_CONFIG_DEFAULT_PATH;
    int required = 0;
    int opt;
    char err[256];
    app_config_t cfg;
    cam_log_level_t level = CAM_LOG_INFO;

    while ((opt = getopt(argc, argv, "c:Vh")) != -1) {
        switch (opt) {
        case 'c':
            path = optarg;
            required = 1;
            break;
        case 'V':
            printf("cam_system %s (%s)\n", CAM_SYSTEM_VERSION, CAM_SYSTEM_GIT_COMMIT);
            return EXIT_SUCCESS;
        case 'h':
            print_usage(stdout, argv[0]);
            return EXIT_SUCCESS;
        default:
            print_usage(stderr, argv[0]);
            return EX_USAGE;
        }
    }
    if (optind != argc) {
        print_usage(stderr, argv[0]);
        return EX_USAGE;
    }

    if (app_config_load(&cfg, path, required, err, sizeof(err)) != 0) {
        fprintf(stderr, "[main] configuration error: %s\n", err);
        return EX_CONFIG;
    }
    (void)cam_log_level_from_name(cfg.log_level, &level);  /* validated by app_config */
    cam_log_init(level,
                 strcmp(cfg.log_target, "syslog") == 0 ? CAM_LOG_TARGET_SYSLOG : CAM_LOG_TARGET_STDERR,
                 "cam_system");
    app_config_set_global(&cfg);
    return -1;
}

/* ── 主函数 ── */
int main(int argc, char *argv[])
{
    /* stdout and stderr share the launcher's log file; line buffering keeps their lines in order. */
    setvbuf(stdout, NULL, _IOLBF, 0);

    int rc = load_startup_config(argc, argv);
    if (rc >= 0)
        return rc;

    LOG_I("cam_system %s (%s)", CAM_SYSTEM_VERSION, CAM_SYSTEM_GIT_COMMIT);
    app_config_describe(app_config(), log_config_line, NULL);

    int signal_fd = setup_signals();
    if (signal_fd < 0)
        return EXIT_FAILURE;

    if (bind_current_to_core("main/network thread", CAM_CPU_NETWORK_CORE) < 0)
        return EXIT_FAILURE;

    /* 初始化全局状态 */
    app_state_init(&g_state);

    /* The bridge counts from power-on.  With the reading after each stream start (capture thread),
     * this tells the transitions of a start from what earlier runs left behind. */
    {
        int32_t dvp_frames;
        int32_t dvp_bad_frames;

        if (frmbuf_get_bridge_counts(&dvp_frames, &dvp_bad_frames) == 0)
            LOG_I("DVP bridge before start-up: %d frame(s), %d of the wrong size, since boot",
                  dvp_frames, dvp_bad_frames);
    }

    /* 初始化 OV5640 */
    if (ov5640_init() < 0) {
        LOG_E("OV5640 init failed");
        return EXIT_FAILURE;
    }
    /* capture.test_pattern: every camera-on initializes the sensor, which clears the colour
     * bars, and writes them again before STREAMON (capture/camera_switch.c). */

    /* 初始化 frmbuf_wr */
    if (frmbuf_init() < 0) {
        LOG_E("frmbuf init failed");
        return EXIT_FAILURE;
    }

    for (int i = 0; i < CAPTURE_BUF_CNT; i++) {
        g_frame_pool[i] = frmbuf_get_buffer(i);
        if (!g_frame_pool[i]) {
            LOG_E("Failed to get frame buffer %d", i);
            return EXIT_FAILURE;
        }
    }
    for (int i = 0; i < FRAME_BUF_CNT; i++) {
        g_cached_pool[i] = (uint8_t *)malloc(FRAME_SIZE);
        if (!g_cached_pool[i]) {
            LOG_E("Failed to alloc cached pool %d", i);
            return EXIT_FAILURE;
        }
        LOG_I("Frame pool[%d] ready", i);
    }

    /* 初始化 JPEG 编码器 */
    if (jpeg_init() < 0) {
        LOG_E("JPEG init failed");
        return EXIT_FAILURE;
    }
    if (jpeg_worker_init() < 0) {
        LOG_E("JPEG worker init failed");
        return EXIT_FAILURE;
    }
    jpeg_worker_set_log_every(app_config()->jpeg_log_every);
    jpeg_worker_set_max_fps(app_config()->stream_max_fps);

    pthread_t jpeg_tid;
    if (create_thread_on_core(&jpeg_tid, "JPEG worker",
                              CAM_CPU_JPEG_CORE,
                              jpeg_worker_thread, NULL) < 0)
        return EXIT_FAILURE;

    /* 初始化 tracker */
    tracker_init(&g_tracker);
    tracker_set_diagnostics(app_config()->tracker_verbose, app_config()->tracker_diag_every_sec);
    /* Numbers only; a refused or failing trace leaves the application running without it. */
    track_trace_open(app_config()->tracker_trace);

    /* 启动采集/控制线程 */
    pthread_t cap_tid;
    if (create_thread_on_core(&cap_tid, "capture/control thread",
                              CAM_CPU_CAPTURE_CORE,
                              capture_thread, NULL) < 0)
        return EXIT_FAILURE;

    /* 初始化并启动检测线程 */
    if (detector_init() < 0) {
        LOG_W("Detector init failed, continuing without detection");
    }
    pthread_t detector_tid;
    if (create_thread_on_core(&detector_tid, "detector thread", CAM_CPU_DETECTOR_CORE,
                              detector_thread, NULL) < 0)
        return EXIT_FAILURE;

    LOG_I("Starting HTTP server on port %d...", HTTP_PORT);
    int net_rc = net_server_run(signal_fd);  /* 阻塞直到 quit 或收到信号 */
    app_state_store_int(&g_state.quit, 1);
    frame_mailbox_wake_all(&g_state);

    /* ── 清理 ── */
    LOG_I("Shutting down...");

    jpeg_worker_request_stop();
    pthread_join(cap_tid, NULL);
    pthread_join(detector_tid, NULL);
    pthread_join(jpeg_tid, NULL);
    track_trace_close();

    /* The stream still runs if the camera is on.  STREAMOFF stops the pipeline, the sensor
     * first (its s_stream(0) writes standby), then terminates the DMA, which ends with a reset of
     * v_frmbuf_wr; our own standby write follows and covers a sensor the pipeline missed. */
    frmbuf_stop();
    ov5640_standby(1);
    {
        int32_t dvp_frames;
        int32_t dvp_bad_frames;

        if (frmbuf_get_bridge_counts(&dvp_frames, &dvp_bad_frames) == 0)
            LOG_I("DVP bridge at exit: %d frame(s), %d of the wrong size, since boot",
                  dvp_frames, dvp_bad_frames);
    }

    uint32_t i2c_write_failures = 0;
    uint32_t isp_group_aborts = 0;
    if (ov5640_get_error_counts(&i2c_write_failures, &isp_group_aborts) == 0)
        LOG_I("OV5640 i2c_write_failures=%u isp_group_aborts=%u",
              i2c_write_failures, isp_group_aborts);
    LOG_I("V4L2 error frames dropped=%lu", frmbuf_error_frame_count());

    jpeg_close();
    jpeg_worker_close();
    detector_close();
    frmbuf_close();
    ov5640_close();
    close(signal_fd);

    for (int i = 0; i < FRAME_BUF_CNT; i++) {
        if (g_cached_pool[i]) 
            free(g_cached_pool[i]);
    }

    pthread_mutex_destroy(&g_state.jpeg_mutex);
    pthread_cond_destroy(&g_state.det_cond);
    pthread_mutex_destroy(&g_state.det_mutex);
    pthread_mutex_destroy(&g_state.cmd_mutex);
    pthread_mutex_destroy(&g_state.settings_mutex);
    pthread_mutex_destroy(&g_state.pred_mutex);
    frame_mailbox_destroy(&g_state);
    if (g_state.jpeg_buf) free(g_state.jpeg_buf);

    if (net_rc < 0) {
        LOG_E("network server failed");
        app_state_fail(&g_state, EX_SOFTWARE);
    }
    const int exit_code = app_state_load_int(&g_state.exit_code);
    if (exit_code != 0)
        LOG_E("Exiting with status %d", exit_code);
    else
        LOG_I("Clean exit");
    cam_log_close();
    return exit_code;
}
