#define _POSIX_C_SOURCE 200809L

#include "common/app_config.h"

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>

static int g_checks;

static void require(int cond, const char *what)
{
    g_checks++;
    if (!cond) {
        fprintf(stderr, "app_config_test: FAIL: %s\n", what);
        exit(1);
    }
}

static void clear_env(void)
{
    static const char *const names[] = {
        "CAM_CAPTURE_DEV", "CAM_CAPTURE_SKIP_MEDIA_SETUP", "CAM_CAPTURE_LOG_EVERY", "CAM_OV5640_SUBDEV",
        "CAM_DETECTOR_BACKEND", "CAM_DETECTOR_FPS", "CAM_DETECTOR_LOG_EVERY", "CAM_YOLO_LOG_EVERY",
        "CAM_DETECTOR_RETRY_LIMIT", "CAM_DETECTOR_BACKOFF_MAX_SEC", "CAM_CONTROL_DEADLINE_MS",
        "CAM_FACE_SCORE_THRESH", "CAM_FACE_TRACK_SCORE_THRESH", "CAM_FACE_NMS_THRESH", "CAM_FACE_TIMEOUT_MS",
        "CAM_FACE_CONFIRM_FRAMES", "CAM_FACE_MAX_MISSES", "CAM_FACE_TEMPORAL_IOU", "CAM_FACE_DIAG_LOG_EVERY",
        "CAM_FACE_DIAG_REPEAT", "CAM_FACE_DIAG_RESET_REPEAT", "CAM_JPEG_LOG_EVERY", "CAM_TRACKER_DIAG_EVERY",
        "CAM_TRACKER_VERBOSE", "CAM_LOG_LEVEL", "CAM_LOG_TARGET", "CAM_JPEG_QUALITY", "CAM_STREAM_MAX_FPS",
        "CAM_TRACKER_TRACE",
    };
    for (size_t i = 0; i < sizeof(names) / sizeof(names[0]); i++)
        unsetenv(names[i]);
}

/* Writes text to a fresh temporary file; returns its path in a static buffer. */
static const char *temp_conf(const char *text)
{
    static char path[64];
    FILE *f;
    int fd;

    snprintf(path, sizeof(path), "/tmp/app_config_test_XXXXXX");
    fd = mkstemp(path);
    require(fd >= 0, "mkstemp");
    f = fdopen(fd, "w");
    require(f != NULL, "fdopen");
    fputs(text, f);
    fclose(f);
    return path;
}

static int load_text(app_config_t *cfg, const char *text, char *err, size_t len)
{
    const char *path = temp_conf(text);
    int rc = app_config_load(cfg, path, 1, err, len);

    unlink(path);
    return rc;
}

static void expect_error(const char *text, const char *needle, const char *what)
{
    app_config_t cfg;
    char err[256] = "";

    require(load_text(&cfg, text, err, sizeof(err)) == -1, what);
    if (!strstr(err, needle))
        fprintf(stderr, "app_config_test: message for \"%s\" was \"%s\"\n", what, err);
    require(strstr(err, needle) != NULL, what);
}

typedef struct {
    char keys[APP_CONFIG_MAX_KEYS][48];
    char lines[APP_CONFIG_MAX_KEYS][192];
    int n;
} described_t;

static void collect(const char *line, void *ctx)
{
    described_t *d = (described_t *)ctx;
    const char *sp = strchr(line, ' ');

    require(d->n < APP_CONFIG_MAX_KEYS && sp != NULL, "describe line count and shape");
    snprintf(d->keys[d->n], sizeof(d->keys[0]), "%.*s", (int)(sp - line), line);
    snprintf(d->lines[d->n], sizeof(d->lines[0]), "%s", line);
    d->n++;
}

static int has_line(const described_t *d, const char *line)
{
    for (int i = 0; i < d->n; i++)
        if (strcmp(d->lines[i], line) == 0)
            return 1;
    return 0;
}

static void test_defaults(void)
{
    app_config_t cfg;
    char err[256];

    require(app_config_load(&cfg, NULL, 0, err, sizeof(err)) == 0, "defaults load");
    require(strcmp(cfg.detector_backend, "face_accel") == 0, "default backend is face_accel");
    require(cfg.detector_fps == 30.0f, "default detector fps");
    require(cfg.face_score_thresh == 0.80f, "default score threshold");
    require(cfg.face_track_score_thresh == 0.60f, "default tracker score threshold");
    require(cfg.face_nms_thresh == 0.30f, "default nms threshold");
    require(cfg.face_timeout_ms == 250, "default timeout");
    require(cfg.face_zero_copy == 1, "default zero copy");
    require(cfg.face_confirm_frames == 3 && cfg.face_max_misses == 2, "default temporal filter");
    require(cfg.face_temporal_iou == 0.15f, "default temporal iou");
    require(cfg.capture_device[0] == '\0' && cfg.ov5640_subdev[0] == '\0', "default device discovery");
    require(cfg.capture_skip_media_setup == 0 && cfg.tracker_verbose == 0, "default flags off");
    require(cfg.capture_log_every == 0 && cfg.detector_log_every == 0 && cfg.jpeg_log_every == 0,
            "periodic logs off by default");
    require(strcmp(cfg.log_level, "info") == 0 && strcmp(cfg.log_target, "stderr") == 0, "default logging");
    require(cfg.detector_retry_limit == 3 && cfg.detector_backoff_max_sec == 8.0f, "default recovery policy");
    require(cfg.control_deadline_ms == 24.0f, "default control deadline");
    require(cfg.capture_max_stream_min == 20, "default stream limit");
    require(cfg.jpeg_quality == 70 && cfg.stream_max_fps == 0,
            "default page stream: quality 70, no frame limit");
    require(app_config_load(&cfg, "/nonexistent/cam_system.conf", 0, err, sizeof(err)) == 0,
            "optional missing file is not an error");
    require(app_config_load(&cfg, "/nonexistent/cam_system.conf", 1, err, sizeof(err)) == -1,
            "required missing file is an error");
}

static void test_file(void)
{
    app_config_t cfg;
    char err[256];
    described_t d;

    require(load_text(&cfg,
                      "# comment\n"
                      "\n"
                      "  detector.fps = 15   # trailing comment\n"
                      "face.score_thresh=0.75\n"
                      "capture.device = /dev/video3\n"
                      "tracker.verbose = yes\n"
                      "capture.skip_media_setup = off\n"
                      "log.level = debug\n",
                      err, sizeof(err)) == 0,
            "valid file loads");
    require(cfg.detector_fps == 15.0f, "file sets fps");
    require(cfg.face_score_thresh == 0.75f, "file sets threshold without spaces");
    require(strcmp(cfg.capture_device, "/dev/video3") == 0, "file sets string");
    require(cfg.tracker_verbose == 1 && cfg.capture_skip_media_setup == 0, "file sets booleans");
    require(strcmp(cfg.log_level, "debug") == 0, "file sets enum");

    memset(&d, 0, sizeof(d));
    app_config_describe(&cfg, collect, &d);
    require(has_line(&d, "detector.fps = 15 (file)"), "describe marks file source");
    require(has_line(&d, "detector.backend = face_accel (default)"), "describe marks default source");
}

static void test_errors(void)
{
    expect_error("detector.speed = 3\n", "unknown key", "unknown key");
    expect_error("detector.fps = 10\ndetector.fps = 12\n", "duplicate key", "duplicate key");
    expect_error("detector.fps = 31\n", "detector.fps", "float above range");
    expect_error("face.confirm_frames = 0\n", "face.confirm_frames", "int below range");
    expect_error("face.timeout_ms = 12ms\n", "face.timeout_ms", "trailing garbage");
    expect_error("face.timeout_ms =\n", "face.timeout_ms", "empty value");
    expect_error("detector.fps = nan\n", "detector.fps", "nan rejected");
    expect_error("tracker.verbose = 2\n", "tracker.verbose", "bad boolean");
    expect_error("detector.backend = gpu\n", "unknown value", "unknown enum value");
    expect_error("detector.retry_limit = 0\n", "detector.retry_limit", "retry limit below range");
    expect_error("control.deadline_ms = 31\n", "control.deadline_ms", "deadline past the frame period");
    expect_error("capture.max_stream_min = -1\n", "capture.max_stream_min", "negative stream limit");
    expect_error("capture.max_stream_min = 601\n", "capture.max_stream_min", "stream limit above range");
    expect_error("jpeg.quality = 19\n", "jpeg.quality", "jpeg quality below range");
    expect_error("jpeg.quality = 96\n", "jpeg.quality", "jpeg quality above range");
    expect_error("stream.max_fps = -1\n", "stream.max_fps", "negative frame limit");
    expect_error("stream.max_fps = 31\n", "stream.max_fps", "frame limit above the camera rate");
    expect_error("stream.max_fps = 12.5\n", "stream.max_fps", "frame limit is an integer");
#ifdef CAM_SYSTEM_ENABLE_CPU_DETECTORS
    {
        app_config_t cfg;
        char err[256];
        require(load_text(&cfg, "detector.backend = scrfd_1g\n", err, sizeof(err)) == 0 &&
                    strcmp(cfg.detector_backend, "scrfd_1g") == 0,
                "CPU backend accepted in a CPU-detector build");
    }
#else
    expect_error("detector.backend = yolo\n", "unknown value", "CPU backend rejected in the product build");
#endif
    expect_error("log.target = file\n", "unknown value", "unknown log target");
    expect_error("detector.fps\n", "expected", "missing equals sign");
    expect_error("capture.device = /dev/"
                 "aaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaa\n",
                 "longer than", "overlong string");
    expect_error("face.nms_thresh = 0.4\n\nface.nms_thresh = -1\n", ":3:", "error names the line");
}

static void test_env(void)
{
    app_config_t cfg;
    char err[256];
    described_t d;

    setenv("CAM_FACE_SCORE_THRESH", "0.7", 1);
    require(load_text(&cfg, "face.score_thresh = 0.9\n", err, sizeof(err)) == 0, "env load");
    require(cfg.face_score_thresh == 0.70f, "env overrides file");
    setenv("CAM_DETECTOR_LOG_EVERY", "10", 1);
    require(app_config_load(&cfg, NULL, 0, err, sizeof(err)) == 0, "env load of a second variable");
    require(cfg.detector_log_every == 10, "second variable applied");
    memset(&d, 0, sizeof(d));
    app_config_describe(&cfg, collect, &d);
    require(has_line(&d, "detector.log_every = 10 (env)"), "describe marks env source");

    setenv("CAM_JPEG_QUALITY", "50", 1);
    setenv("CAM_STREAM_MAX_FPS", "15", 1);
    require(load_text(&cfg, "jpeg.quality = 60\nstream.max_fps = 10\n", err, sizeof(err)) == 0 &&
                cfg.jpeg_quality == 50 && cfg.stream_max_fps == 15,
            "page stream knobs from the environment override the file");
    unsetenv("CAM_JPEG_QUALITY");
    unsetenv("CAM_STREAM_MAX_FPS");
    require(load_text(&cfg, "jpeg.quality = 60\nstream.max_fps = 10\n", err, sizeof(err)) == 0 &&
                cfg.jpeg_quality == 60 && cfg.stream_max_fps == 10,
            "page stream knobs from the file");
    require(app_config_load(&cfg, NULL, 0, err, sizeof(err)) == 0 && cfg.tracker_trace[0] == '\0',
            "the control trace is off by default");
    setenv("CAM_TRACKER_TRACE", "/tmp/cam_trace.csv", 1);
    require(app_config_load(&cfg, NULL, 0, err, sizeof(err)) == 0 &&
                strcmp(cfg.tracker_trace, "/tmp/cam_trace.csv") == 0,
            "the control trace path from the environment");
    unsetenv("CAM_TRACKER_TRACE");

    setenv("CAM_FACE_SCORE_THRESH", "", 1);
    require(app_config_load(&cfg, NULL, 0, err, sizeof(err)) == 0 && cfg.face_score_thresh == 0.80f,
            "empty env value is ignored");
    setenv("CAM_DETECTOR_FPS", "0", 1);
    require(app_config_load(&cfg, NULL, 0, err, sizeof(err)) == -1 && strstr(err, "CAM_DETECTOR_FPS"),
            "invalid env value is an error naming the variable");
    clear_env();
}

/* Every key the shipped example documents must exist, and every key must be documented. */
static void test_example(const char *path)
{
    app_config_t cfg;
    char err[256];
    char line[256];
    described_t d;
    int documented = 0;
    FILE *f;

    require(app_config_load(&cfg, path, 1, err, sizeof(err)) == 0, "shipped example loads");
    memset(&d, 0, sizeof(d));
    app_config_describe(&cfg, collect, &d);
    for (int i = 0; i < d.n; i++)
        require(strstr(d.lines[i], "(default)") != NULL, "shipped example changes no default");

    f = fopen(path, "r");
    require(f != NULL, "open shipped example");
    while (fgets(line, sizeof(line), f)) {
        char key[48];
        int found = 0;

        if (sscanf(line, "# %47[a-z0-9_.] =", key) != 1 || !strchr(key, '.'))
            continue;
        documented++;
        for (int i = 0; i < d.n; i++)
            if (strcmp(d.keys[i], key) == 0)
                found = 1;
        if (!found)
            fprintf(stderr, "app_config_test: example documents unknown key %s\n", key);
        require(found, "example key exists");
    }
    fclose(f);
    require(documented == d.n, "example documents every key");
}

static void test_global(void)
{
    app_config_t cfg;
    char err[256];

    require(strcmp(app_config()->detector_backend, "face_accel") == 0, "global defaults before set");
    require(load_text(&cfg, "detector.fps = 20\n", err, sizeof(err)) == 0, "load for global");
    app_config_set_global(&cfg);
    require(app_config()->detector_fps == 20.0f, "global reflects set value");
}

int main(int argc, char **argv)
{
    if (argc != 2) {
        fprintf(stderr, "usage: %s path/to/cam_system.conf\n", argv[0]);
        return 2;
    }
    clear_env();
    test_defaults();
    test_file();
    test_errors();
    test_env();
    test_example(argv[1]);
    test_global();
    printf("app_config_test: OK (%d checks)\n", g_checks);
    return 0;
}
