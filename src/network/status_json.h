/* status_json.h - the device status as JSON, for GET /api/status and the WebSocket.
 *
 * STATUS_BODY_MAX holds the body with every field at its widest; tests/status_json_test.c checks it.
 */
#ifndef STATUS_JSON_H
#define STATUS_JSON_H

#include <stddef.h>

#ifdef __cplusplus
extern "C" {
#endif

#define STATUS_BODY_MAX 2048

typedef struct {
    int cam_enabled;
    int detector_enabled;
    int tracking;
    int target_id;              /* negative: none */
    float fps_capture;
    float fps_inference;
    float fps_stream;
    unsigned int uptime_sec;
    unsigned int status_seq;
    const char *detector_health;
    unsigned long detector_failures;
    int detector_last_error;
    unsigned long isp_slips;
    unsigned long capture_restarts;
    long dvp_frames;            /* negative: not read yet */
    long dvp_bad_frames;
    long i2c_write_failures;
    unsigned long v4l2_error_frames;
    const char *version;
    const char *git_commit;
    unsigned int driver_uapi;
    unsigned int hw_revision;
    const char *weights_sha256;
    const char *detector_input;
    const char *cam_off_reason; /* "none" or "stream_limit" */
    unsigned int cam_stream_sec;  /* how long the current stream has run; 0 while stopped */
} status_fields_t;

/* Appends to json at pos, keeping it NUL-terminated.  Returns the new position, or -1 when the
 * text does not fit or pos is already -1, so calls can be chained and checked once. */
int json_append(char *json, size_t sz, int pos, const char *fmt, ...)
    __attribute__((format(printf, 4, 5)));

int status_json_append_fields(char *json, size_t sz, int pos, const status_fields_t *status);

/* The body of GET /api/status.  Returns its length, or -1. */
int status_json_body(char *body, size_t sz, const status_fields_t *status);

#ifdef __cplusplus
}
#endif

#endif /* STATUS_JSON_H */
