/* app_config.c - see app_config.h */
#define _POSIX_C_SOURCE 200809L
#include "common/app_config.h"

#include "cam_system/config.h"
#include "ultraface_decode.h"

#include <ctype.h>
#include <errno.h>
#include <math.h>
#include <stdarg.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <strings.h>

typedef enum { K_INT, K_FLOAT, K_BOOL, K_STR, K_ENUM } kind_t;

typedef struct {
    const char *key;
    const char *env;
    kind_t kind;
    size_t offset;
    double def;              /* K_INT, K_FLOAT, K_BOOL */
    const char *def_str;     /* K_STR, K_ENUM */
    double min;
    double max;
    const char *const *choices;
} entry_t;

#define OFF(field) offsetof(app_config_t, field)

/* The CPU model candidates exist only in builds with CAM_SYSTEM_ENABLE_CPU_DETECTORS. */
static const char *const k_backends[] = {
    "face_accel",
#ifdef CAM_SYSTEM_ENABLE_CPU_DETECTORS
    "yolo", "yolov6", "ultraface", "ultraface_rfb_320",
    "yunet", "yunet_320", "yunet_320x240", "yunet_384", "yunet_384x288",
    "scrfd_500m", "scrfd_500m_640", "scrfd_1g", "scrfd_1g_640",
    "scrfd_2.5g", "scrfd_2_5g", "scrfd_2_5g_kps",
#endif
    NULL
};
static const char *const k_levels[] = {"error", "warn", "info", "debug", NULL};
static const char *const k_targets[] = {"stderr", "syslog", NULL};

/* Defaults and ranges are the ones the v0.1.1 code applied to its environment variables; the
 * product launcher selects face_accel, which is therefore the default backend. */
static const entry_t k_entries[] = {
    {"capture.device", "CAM_CAPTURE_DEV", K_STR, OFF(capture_device), 0, "", 0, 0, NULL},
    {"capture.skip_media_setup", "CAM_CAPTURE_SKIP_MEDIA_SETUP", K_BOOL,
     OFF(capture_skip_media_setup), 0, NULL, 0, 1, NULL},
    {"capture.test_pattern", "CAM_CAPTURE_TEST_PATTERN", K_BOOL, OFF(capture_test_pattern), 0, NULL, 0, 1,
     NULL},
    {"capture.max_stream_min", "CAM_CAPTURE_MAX_STREAM_MIN", K_INT, OFF(capture_max_stream_min), 20, NULL,
     0, 600, NULL},
    {"capture.log_every", "CAM_CAPTURE_LOG_EVERY", K_INT, OFF(capture_log_every), 0, NULL, 0, 100000,
     NULL},
    {"control.deadline_ms", "CAM_CONTROL_DEADLINE_MS", K_FLOAT, OFF(control_deadline_ms), 24.0, NULL, 0.0, 30.0,
     NULL},
    {"ov5640.subdev", "CAM_OV5640_SUBDEV", K_STR, OFF(ov5640_subdev), 0, "", 0, 0, NULL},
    {"detector.backend", "CAM_DETECTOR_BACKEND", K_ENUM, OFF(detector_backend), 0, "face_accel", 0, 0,
     k_backends},
    {"detector.fps", "CAM_DETECTOR_FPS", K_FLOAT, OFF(detector_fps), 30.0, NULL, 1.0, 30.0, NULL},
    {"detector.log_every", "CAM_DETECTOR_LOG_EVERY", K_INT, OFF(detector_log_every), 0,
     NULL, 0, 100000, NULL},
    {"detector.retry_limit", "CAM_DETECTOR_RETRY_LIMIT", K_INT, OFF(detector_retry_limit), 3, NULL, 1, 100,
     NULL},
    {"detector.backoff_max_sec", "CAM_DETECTOR_BACKOFF_MAX_SEC", K_FLOAT, OFF(detector_backoff_max_sec), 8.0,
     NULL, 0.5, 300.0, NULL},
    {"face.score_thresh", "CAM_FACE_SCORE_THRESH", K_FLOAT, OFF(face_score_thresh),
     UF_DEFAULT_SCORE_THRESHOLD, NULL, 0.0, 1.0, NULL},
    {"face.track_score_thresh", "CAM_FACE_TRACK_SCORE_THRESH", K_FLOAT, OFF(face_track_score_thresh),
     0.60, NULL, 0.0, 1.0, NULL},
    {"face.nms_thresh", "CAM_FACE_NMS_THRESH", K_FLOAT, OFF(face_nms_thresh), UF_DEFAULT_NMS_THRESHOLD,
     NULL, 0.0, 1.0, NULL},
    {"face.timeout_ms", "CAM_FACE_TIMEOUT_MS", K_INT, OFF(face_timeout_ms), 250, NULL, 1, 2000, NULL},
    {"face.zero_copy", "CAM_FACE_ZERO_COPY", K_BOOL, OFF(face_zero_copy), 1, NULL, 0, 1, NULL},
    {"face.confirm_frames", "CAM_FACE_CONFIRM_FRAMES", K_INT, OFF(face_confirm_frames), 3, NULL, 1, 10,
     NULL},
    {"face.max_misses", "CAM_FACE_MAX_MISSES", K_INT, OFF(face_max_misses), 2, NULL, 0, 10, NULL},
    {"face.temporal_iou", "CAM_FACE_TEMPORAL_IOU", K_FLOAT, OFF(face_temporal_iou), 0.15, NULL, 0.0, 1.0,
     NULL},
    {"face.diag_log_every", "CAM_FACE_DIAG_LOG_EVERY", K_INT, OFF(face_diag_log_every), 0, NULL, 0, 10000,
     NULL},
    {"face.diag_repeat", "CAM_FACE_DIAG_REPEAT", K_INT, OFF(face_diag_repeat), 0, NULL, 0, 500, NULL},
    {"face.diag_reset_repeat", "CAM_FACE_DIAG_RESET_REPEAT", K_INT, OFF(face_diag_reset_repeat), 0, NULL,
     0, 100, NULL},
    {"jpeg.quality", "CAM_JPEG_QUALITY", K_INT, OFF(jpeg_quality), JPEG_QUALITY, NULL, 20, 95, NULL},
    {"jpeg.log_every", "CAM_JPEG_LOG_EVERY", K_INT, OFF(jpeg_log_every), 0, NULL, 0, 100000, NULL},
    {"stream.max_fps", "CAM_STREAM_MAX_FPS", K_INT, OFF(stream_max_fps), 0, NULL, 0, 30, NULL},
    {"tracker.diag_every_sec", "CAM_TRACKER_DIAG_EVERY", K_FLOAT, OFF(tracker_diag_every_sec), 0.0, NULL,
     0.0, 3600.0, NULL},
    {"tracker.verbose", "CAM_TRACKER_VERBOSE", K_BOOL, OFF(tracker_verbose), 0, NULL, 0, 1, NULL},
    {"tracker.trace", "CAM_TRACKER_TRACE", K_STR, OFF(tracker_trace), 0, "", 0, 0, NULL},
    {"log.level", "CAM_LOG_LEVEL", K_ENUM, OFF(log_level), 0, "info", 0, 0, k_levels},
    {"log.target", "CAM_LOG_TARGET", K_ENUM, OFF(log_target), 0, "stderr", 0, 0, k_targets},
};

#define N_ENTRIES (sizeof(k_entries) / sizeof(k_entries[0]))

_Static_assert(sizeof(k_entries) / sizeof(k_entries[0]) <= APP_CONFIG_MAX_KEYS,
               "APP_CONFIG_MAX_KEYS too small for the parameter table");

static void set_err(char *err, size_t len, const char *fmt, ...) __attribute__((format(printf, 3, 4)));

static void set_err(char *err, size_t len, const char *fmt, ...)
{
    va_list ap;

    if (!err || len == 0)
        return;
    va_start(ap, fmt);
    vsnprintf(err, len, fmt, ap);
    va_end(ap);
}

static void *field(app_config_t *cfg, const entry_t *e)
{
    return (char *)cfg + e->offset;
}

static const void *cfield(const app_config_t *cfg, const entry_t *e)
{
    return (const char *)cfg + e->offset;
}

static int parse_bool(const char *text, int *out)
{
    static const char *const yes[] = {"1", "true", "yes", "on"};
    static const char *const no[] = {"0", "false", "no", "off"};

    for (size_t i = 0; i < sizeof(yes) / sizeof(yes[0]); i++) {
        if (strcasecmp(text, yes[i]) == 0) {
            *out = 1;
            return 0;
        }
        if (strcasecmp(text, no[i]) == 0) {
            *out = 0;
            return 0;
        }
    }
    return -1;
}

/* Parses text into the field of entry e.  origin names the source for error messages. */
static int parse_value(app_config_t *cfg, const entry_t *e, const char *text, const char *origin,
                       char *err, size_t err_len)
{
    char *end = NULL;

    switch (e->kind) {
    case K_INT: {
        errno = 0;
        long v = strtol(text, &end, 10);
        if (errno != 0 || end == text || *end != '\0' || v < (long)e->min || v > (long)e->max) {
            set_err(err, err_len, "%s: %s must be an integer in [%ld, %ld], got \"%s\"", origin, e->key,
                    (long)e->min, (long)e->max, text);
            return -1;
        }
        *(int *)field(cfg, e) = (int)v;
        return 0;
    }
    case K_FLOAT: {
        errno = 0;
        float v = strtof(text, &end);
        if (errno != 0 || end == text || *end != '\0' || !isfinite(v) || (double)v < e->min ||
            (double)v > e->max) {
            set_err(err, err_len, "%s: %s must be a number in [%g, %g], got \"%s\"", origin, e->key, e->min,
                    e->max, text);
            return -1;
        }
        *(float *)field(cfg, e) = v;
        return 0;
    }
    case K_BOOL: {
        int v = 0;
        if (parse_bool(text, &v) != 0) {
            set_err(err, err_len, "%s: %s must be 0/1, true/false, yes/no or on/off, got \"%s\"", origin,
                    e->key, text);
            return -1;
        }
        *(int *)field(cfg, e) = v;
        return 0;
    }
    case K_ENUM: {
        int found = 0;
        for (const char *const *c = e->choices; *c; c++)
            if (strcmp(*c, text) == 0)
                found = 1;
        if (!found) {
            set_err(err, err_len, "%s: %s has unknown value \"%s\"", origin, e->key, text);
            return -1;
        }
    }
        /* fall through */
    case K_STR:
        if (strlen(text) >= APP_CONFIG_STR_MAX) {
            set_err(err, err_len, "%s: %s is longer than %d characters", origin, e->key, APP_CONFIG_STR_MAX - 1);
            return -1;
        }
        snprintf((char *)field(cfg, e), APP_CONFIG_STR_MAX, "%s", text);
        return 0;
    }
    return -1;
}

static void set_defaults(app_config_t *cfg)
{
    memset(cfg, 0, sizeof(*cfg));
    for (size_t i = 0; i < N_ENTRIES; i++) {
        const entry_t *e = &k_entries[i];
        switch (e->kind) {
        case K_INT:
        case K_BOOL:
            *(int *)field(cfg, e) = (int)e->def;
            break;
        case K_FLOAT:
            *(float *)field(cfg, e) = (float)e->def;
            break;
        case K_STR:
        case K_ENUM:
            snprintf((char *)field(cfg, e), APP_CONFIG_STR_MAX, "%s", e->def_str);
            break;
        }
        cfg->source[i] = APP_CONFIG_SRC_DEFAULT;
    }
}

static char *trim(char *s)
{
    char *end;

    while (isspace((unsigned char)*s))
        s++;
    end = s + strlen(s);
    while (end > s && isspace((unsigned char)end[-1]))
        *--end = '\0';
    return s;
}

static int find_entry(const char *key)
{
    for (size_t i = 0; i < N_ENTRIES; i++)
        if (strcmp(k_entries[i].key, key) == 0)
            return (int)i;
    return -1;
}

static int load_file(app_config_t *cfg, const char *path, int required, char *err, size_t err_len)
{
    char line[256];
    char origin[96];
    int seen[N_ENTRIES];
    int lineno = 0;
    FILE *f = fopen(path, "r");

    if (!f) {
        if (!required && errno == ENOENT)
            return 0;
        set_err(err, err_len, "%s: cannot open: %s", path, strerror(errno));
        return -1;
    }
    memset(seen, 0, sizeof(seen));
    while (fgets(line, sizeof(line), f)) {
        char *hash;
        char *eq;
        char *key;
        char *value;
        int idx;

        lineno++;
        if (!strchr(line, '\n') && !feof(f)) {
            set_err(err, err_len, "%s:%d: line longer than %zu characters", path, lineno, sizeof(line) - 2);
            fclose(f);
            return -1;
        }
        hash = strchr(line, '#');
        if (hash)
            *hash = '\0';
        key = trim(line);
        if (*key == '\0')
            continue;
        eq = strchr(key, '=');
        if (!eq) {
            set_err(err, err_len, "%s:%d: expected \"key = value\"", path, lineno);
            fclose(f);
            return -1;
        }
        *eq = '\0';
        value = trim(eq + 1);
        key = trim(key);
        idx = find_entry(key);
        if (idx < 0) {
            set_err(err, err_len, "%s:%d: unknown key \"%s\"", path, lineno, key);
            fclose(f);
            return -1;
        }
        if (seen[idx]) {
            set_err(err, err_len, "%s:%d: duplicate key \"%s\"", path, lineno, key);
            fclose(f);
            return -1;
        }
        seen[idx] = 1;
        snprintf(origin, sizeof(origin), "%s:%d", path, lineno);
        if (parse_value(cfg, &k_entries[idx], value, origin, err, err_len) != 0) {
            fclose(f);
            return -1;
        }
        cfg->source[idx] = APP_CONFIG_SRC_FILE;
    }
    fclose(f);
    return 0;
}

static int load_env(app_config_t *cfg, char *err, size_t err_len)
{
    for (size_t i = 0; i < N_ENTRIES; i++) {
        const entry_t *e = &k_entries[i];
        const char *value = getenv(e->env);

        if (!value || *value == '\0')
            continue;
        if (parse_value(cfg, e, value, e->env, err, err_len) != 0)
            return -1;
        cfg->source[i] = APP_CONFIG_SRC_ENV;
    }
    return 0;
}

int app_config_load(app_config_t *cfg, const char *path, int required, char *err, size_t err_len)
{
    if (!cfg) {
        set_err(err, err_len, "no configuration object");
        return -1;
    }
    set_defaults(cfg);
    if (path && load_file(cfg, path, required, err, err_len) != 0)
        return -1;
    return load_env(cfg, err, err_len);
}

static app_config_t g_config;
static int g_config_set;

void app_config_set_global(const app_config_t *cfg)
{
    g_config = *cfg;
    g_config_set = 1;
}

const app_config_t *app_config(void)
{
    if (!g_config_set) {
        set_defaults(&g_config);
        g_config_set = 1;
    }
    return &g_config;
}

void app_config_describe(const app_config_t *cfg, void (*emit)(const char *line, void *ctx), void *ctx)
{
    static const char *const src_name[] = {"default", "file", "env"};
    char line[192];

    for (size_t i = 0; i < N_ENTRIES; i++) {
        const entry_t *e = &k_entries[i];
        const void *p = cfield(cfg, e);
        char value[APP_CONFIG_STR_MAX + 2];

        switch (e->kind) {
        case K_INT:
        case K_BOOL:
            snprintf(value, sizeof(value), "%d", *(const int *)p);
            break;
        case K_FLOAT:
            snprintf(value, sizeof(value), "%g", (double)*(const float *)p);
            break;
        case K_STR:
        case K_ENUM:
            snprintf(value, sizeof(value), "%s", (const char *)p);
            break;
        }
        snprintf(line, sizeof(line), "%s = %s (%s)", e->key, value,
                 src_name[cfg->source[i] <= APP_CONFIG_SRC_ENV ? cfg->source[i] : 0]);
        emit(line, ctx);
    }
}
