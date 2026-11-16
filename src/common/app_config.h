/* app_config.h - runtime configuration: defaults, configuration file, environment overrides.
 *
 * One table in app_config.c defines every parameter (key, environment variable, type, default, range).
 * Loading order: defaults, then the file, then environment variables (kept as debugging overrides).
 * Unknown keys, malformed or out-of-range values and duplicate keys are errors: a bad configuration
 * stops the program at start-up instead of being silently ignored.
 */
#ifndef CAM_APP_CONFIG_H
#define CAM_APP_CONFIG_H

#include <stddef.h>

#ifdef __cplusplus
extern "C" {
#endif

#define APP_CONFIG_DEFAULT_PATH "/etc/cam_system.conf"
#define APP_CONFIG_STR_MAX      64
#define APP_CONFIG_MAX_KEYS     32

typedef enum {
    APP_CONFIG_SRC_DEFAULT = 0,
    APP_CONFIG_SRC_FILE,
    APP_CONFIG_SRC_ENV
} app_config_source_t;

typedef struct {
    char  capture_device[APP_CONFIG_STR_MAX];   /* "" = scan /dev/video* */
    int   capture_skip_media_setup;
    int   capture_test_pattern;
    int   capture_log_every;
    int   capture_max_stream_min;               /* 0 = no limit */
    float control_deadline_ms;                  /* 0 = tick right after dequeue */
    char  ov5640_subdev[APP_CONFIG_STR_MAX];    /* "" = scan /sys/class/video4linux */
    char  detector_backend[APP_CONFIG_STR_MAX];
    float detector_fps;
    int   detector_log_every;
    int   detector_retry_limit;
    float detector_backoff_max_sec;
    float face_score_thresh;
    float face_track_score_thresh;
    float face_nms_thresh;
    int   face_timeout_ms;
    int   face_zero_copy;
    int   face_confirm_frames;
    int   face_max_misses;
    float face_temporal_iou;
    int   face_diag_log_every;
    int   face_diag_repeat;
    int   face_diag_reset_repeat;
    int   jpeg_quality;                         /* page stream only */
    int   jpeg_log_every;
    int   stream_max_fps;                       /* page stream only; 0 = no limit */
    float tracker_diag_every_sec;
    int   tracker_verbose;
    char  tracker_trace[APP_CONFIG_STR_MAX];    /* "" = off; a file under /tmp: numeric control trace */
    char  log_level[APP_CONFIG_STR_MAX];        /* error | warn | info | debug */
    char  log_target[APP_CONFIG_STR_MAX];       /* stderr | syslog */
    unsigned char source[APP_CONFIG_MAX_KEYS];  /* app_config_source_t per table entry */
} app_config_t;

/* Defaults, then the file at path (NULL: no file; a missing file is an error only if required),
 * then environment overrides.  Returns 0, or -1 with a message in err. */
int app_config_load(app_config_t *cfg, const char *path, int required, char *err, size_t err_len);

/* Process-wide configuration.  main sets it once before any thread starts; readers never write it.
 * Before it is set, app_config() returns the defaults. */
void app_config_set_global(const app_config_t *cfg);
const app_config_t *app_config(void);

/* Calls emit once per parameter with "key = value (source)". */
void app_config_describe(const app_config_t *cfg, void (*emit)(const char *line, void *ctx), void *ctx);

#ifdef __cplusplus
}
#endif

#endif /* CAM_APP_CONFIG_H */
