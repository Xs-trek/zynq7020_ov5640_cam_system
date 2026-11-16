/* status_json_test.c - the status fits its buffer with every field at its widest. */
#include "network/status_json.h"

#include <float.h>
#include <limits.h>
#include <stdio.h>
#include <string.h>

static int failures;

static void expect_true(int cond, const char *name, int line)
{
    if (!cond) {
        fprintf(stderr, "FAIL:%d: %s\n", line, name);
        failures++;
    }
}

#define EXPECT_TRUE(cond) expect_true((cond), #cond, __LINE__)

static const char SHA[] = "0123456789abcdef0123456789abcdef0123456789abcdef0123456789abcdef";

static status_fields_t widest(void)
{
    status_fields_t s;

    memset(&s, 0, sizeof(s));
    s.cam_enabled = s.detector_enabled = s.tracking = 0;
    s.target_id = INT_MAX;
    s.fps_capture = s.fps_inference = s.fps_stream = 1000.0f;
    s.uptime_sec = UINT_MAX;
    s.status_seq = UINT_MAX;
    s.detector_health = "unavailable";
    s.detector_failures = ULONG_MAX;
    s.detector_last_error = INT_MIN;
    s.isp_slips = ULONG_MAX;
    s.capture_restarts = ULONG_MAX;
    s.dvp_frames = LONG_MIN;
    s.dvp_bad_frames = LONG_MIN;
    s.i2c_write_failures = LONG_MIN;
    s.v4l2_error_frames = ULONG_MAX;
    s.version = "999.999.999";
    s.git_commit = "0123456789abcdef0123456789abcdef01234567-dirty";
    s.driver_uapi = UINT_MAX;
    s.hw_revision = UINT_MAX;
    s.weights_sha256 = SHA;
    s.detector_input = "direct";
    s.cam_off_reason = "stream_limit";
    s.cam_stream_sec = UINT_MAX;
    return s;
}

static void test_widest_status_fits(void)
{
    status_fields_t s = widest();
    char body[STATUS_BODY_MAX];

    EXPECT_TRUE(status_json_body(body, sizeof(body), &s) > 0);
    EXPECT_TRUE(strstr(body, "{\"status\":{\"cam_enabled\":false") == body);
    EXPECT_TRUE(body[strlen(body) - 1] == '}');
}

static void test_no_target_is_null(void)
{
    status_fields_t s = widest();
    char body[STATUS_BODY_MAX];

    s.target_id = -1;
    EXPECT_TRUE(status_json_body(body, sizeof(body), &s) > 0);
    EXPECT_TRUE(strstr(body, "\"target_id\":null,") != NULL);
}

/* The camera that the stream limit turned off says so. */
static void test_camera_off_reason(void)
{
    status_fields_t s = widest();
    char body[STATUS_BODY_MAX];

    s.cam_off_reason = "none";
    s.cam_stream_sec = 75;
    EXPECT_TRUE(status_json_body(body, sizeof(body), &s) > 0);
    EXPECT_TRUE(strstr(body, "\"cam_off_reason\":\"none\",\"cam_stream_sec\":75}") != NULL);
    s.cam_off_reason = "stream_limit";
    s.cam_stream_sec = 0;
    EXPECT_TRUE(status_json_body(body, sizeof(body), &s) > 0);
    EXPECT_TRUE(strstr(body, "\"cam_off_reason\":\"stream_limit\",\"cam_stream_sec\":0") != NULL);
}

static void test_short_buffer_reports_failure(void)
{
    status_fields_t s = widest();
    char body[64];

    EXPECT_TRUE(status_json_body(body, sizeof(body), &s) == -1);
}

static void test_append_chains_a_failure(void)
{
    char json[8];
    int pos = json_append(json, sizeof(json), 0, "%s", "1234567");

    EXPECT_TRUE(pos == 7);
    pos = json_append(json, sizeof(json), pos, "x");
    EXPECT_TRUE(pos == -1);
    EXPECT_TRUE(json_append(json, sizeof(json), pos, "x") == -1);
}

int main(void)
{
    test_widest_status_fits();
    test_no_target_is_null();
    test_camera_off_reason();
    test_short_buffer_reports_failure();
    test_append_chains_a_failure();
    if (failures) {
        fprintf(stderr, "status_json_test: %d failure(s)\n", failures);
        return 1;
    }
    puts("status_json_test: OK");
    return 0;
}
