/* status_json.c - see status_json.h */
#include "network/status_json.h"

#include <stdarg.h>
#include <stdio.h>

int json_append(char *json, size_t sz, int pos, const char *fmt, ...)
{
    va_list ap;
    int n;

    if (pos < 0 || (size_t)pos >= sz)
        return -1;

    va_start(ap, fmt);
    n = vsnprintf(json + pos, sz - (size_t)pos, fmt, ap);
    va_end(ap);

    if (n < 0 || (size_t)n >= sz - (size_t)pos)
        return -1;
    return pos + n;
}

int status_json_append_fields(char *json, size_t sz, int pos, const status_fields_t *s)
{
    char target_id[16];

    if (s->target_id >= 0)
        snprintf(target_id, sizeof(target_id), "%d", s->target_id);
    else
        snprintf(target_id, sizeof(target_id), "null");

    return json_append(json, sz, pos,
        "\"cam_enabled\":%s,"
        "\"yolo_enabled\":%s,"
        "\"tracking\":%s,"
        "\"target_id\":%s,"
        "\"fps_capture\":%.1f,"
        "\"fps_inference\":%.1f,"
        "\"fps_stream\":%.1f,"
        "\"uptime_sec\":%u,"
        "\"status_seq\":%u,"
        "\"detector_health\":\"%s\","
        "\"detector_failures\":%lu,"
        "\"detector_last_error\":%d,"
        "\"isp_slips\":%lu,"
        "\"capture_restarts\":%lu,"
        "\"dvp_frames\":%ld,"
        "\"dvp_bad_frames\":%ld,"
        "\"i2c_write_failures\":%ld,"
        "\"v4l2_error_frames\":%lu,"
        "\"version\":\"%s\","
        "\"git_commit\":\"%s\","
        "\"driver_uapi\":%u,"
        "\"hw_revision\":%u,"
        "\"weights_sha256\":\"%s\","
        "\"detector_input\":\"%s\","
        "\"cam_off_reason\":\"%s\","
        "\"cam_stream_sec\":%u",
        s->cam_enabled ? "true" : "false",
        s->detector_enabled ? "true" : "false",
        s->tracking ? "true" : "false",
        target_id,
        (double)s->fps_capture,
        (double)s->fps_inference,
        (double)s->fps_stream,
        s->uptime_sec,
        s->status_seq,
        s->detector_health,
        s->detector_failures,
        s->detector_last_error,
        s->isp_slips,
        s->capture_restarts,
        s->dvp_frames,
        s->dvp_bad_frames,
        s->i2c_write_failures,
        s->v4l2_error_frames,
        s->version,
        s->git_commit,
        s->driver_uapi,
        s->hw_revision,
        s->weights_sha256,
        s->detector_input,
        s->cam_off_reason,
        s->cam_stream_sec);
}

int status_json_body(char *body, size_t sz, const status_fields_t *status)
{
    int pos = 0;

    pos = json_append(body, sz, pos, "{\"status\":{");
    pos = status_json_append_fields(body, sz, pos, status);
    pos = json_append(body, sz, pos, "},");
    pos = status_json_append_fields(body, sz, pos, status);
    pos = json_append(body, sz, pos, "}");
    return pos;
}
