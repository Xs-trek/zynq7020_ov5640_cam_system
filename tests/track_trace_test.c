/* track_trace_test.c - the tracker.trace CSV: format, off by default, path rules, size limit. */
#define _GNU_SOURCE
#include "control/track_trace.h"

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/stat.h>
#include <unistd.h>

static int g_checks;

static void require(int cond, const char *what)
{
    g_checks++;
    if (!cond) {
        fprintf(stderr, "track_trace_test: FAIL: %s\n", what);
        exit(1);
    }
}

/* Top-level CSV fields: the detections column uses ';' and ':' only. */
static int fields(const char *line)
{
    int n = 1;

    for (const char *p = line; *p && *p != '\n'; p++)
        n += *p == ',';
    return n;
}

static const char *field(const char *line, int index, char *out, size_t len)
{
    const char *p = line;
    size_t n = 0;

    for (int i = 0; i < index; i++) {
        p = strchr(p, ',');
        if (!p)
            return NULL;
        p++;
    }
    while (p[n] && p[n] != ',' && p[n] != '\n' && n + 1 < len) {
        out[n] = p[n];
        n++;
    }
    out[n] = '\0';
    return out;
}

static track_trace_row_t idle_row(void)
{
    track_trace_row_t row;

    memset(&row, 0, sizeof(row));
    row.frame_seq = 1201;
    row.v4l2_seq = 1188;
    row.capture_time = 4321.0123456;
    row.tick = TRACK_TRACE_TICK_IMMEDIATE;
    row.state = "IDLE";
    row.isp_x = 16;
    row.isp_y = 4;
    row.n_det = 2;
    row.det[0] = (track_trace_det_t){ 7, 640.25f, 480.5f, 85.8f, 110.0f, 0.934f, 1, 0 };
    row.det[1] = (track_trace_det_t){ -1, 300.0f, 200.0f, 40.0f, 51.3f, 0.655f, 0, 0 };
    return row;
}

static void test_format(void)
{
    char line[TRACK_TRACE_LINE_MAX];
    char f[256];
    track_trace_row_t row = idle_row();
    const int columns = fields(track_trace_header());

    require(columns == 29, "the header has 29 columns");
    require(track_trace_format(line, sizeof(line), &row) > 0, "an idle row formats");
    require(fields(line) == columns, "an idle row has every column");
    require(strcmp(field(line, 0, f, sizeof(f)), "1201") == 0, "frame_seq");
    require(strcmp(field(line, 2, f, sizeof(f)), "4321.012346") == 0, "capture time in microseconds");
    require(strcmp(field(line, 3, f, sizeof(f)), "immediate") == 0, "tick name");
    require(strcmp(field(line, 4, f, sizeof(f)), "IDLE") == 0, "state name");
    require(strcmp(field(line, 8, f, sizeof(f)), "") == 0 && strcmp(field(line, 22, f, sizeof(f)), "") == 0,
            "no estimate or path while idle");
    require(strcmp(field(line, 23, f, sizeof(f)), "0") == 0 && strcmp(field(line, 24, f, sizeof(f)), "") == 0,
            "no adopted observation");
    require(strcmp(field(line, 28, f, sizeof(f)),
                   "7:640.25:480.50:85.8:110.0:0.934:1:0;-1:300.00:200.00:40.0:51.3:0.655:0:0") == 0,
            "detections in the absolute plane, ';'-separated");
    require(line[strlen(line) - 1] == '\n', "the line ends with a newline");

    row.state = "TRACKING";
    row.tick = TRACK_TRACE_TICK_DETECTION;
    row.active = 1;
    row.kf_x = 650.5f; row.kf_y = 470.25f; row.kf_vx = 120.0f; row.kf_vy = -30.0f; row.kf_log_h = 4.7f;
    row.ref_x = 331.0f; row.ref_y = 192.0f; row.ref_vx = 118.0f; row.ref_vy = -29.0f;
    row.path_x = 329.5f; row.path_y = 190.0f; row.path_vx = 110.0f; row.path_vy = -25.0f;
    row.path_ax = 900.0f; row.path_ay = -300.0f;
    row.adopted = 1; row.obs_x = 640.25f; row.obs_y = 480.5f; row.obs_h = 110.0f; row.obs_age_ms = 21.4;
    row.det[0].target = 1;
    row.prev_slip = 1;
    require(track_trace_format(line, sizeof(line), &row) > 0, "a tracking row formats");
    require(fields(line) == columns, "a tracking row has every column");
    require(strcmp(field(line, 3, f, sizeof(f)), "det") == 0, "tick on the detection");
    require(strcmp(field(line, 7, f, sizeof(f)), "1") == 0, "the previous write slipped");
    require(strcmp(field(line, 8, f, sizeof(f)), "650.50") == 0, "KF x");
    require(strcmp(field(line, 12, f, sizeof(f)), "4.7000") == 0, "KF log h");
    require(strcmp(field(line, 21, f, sizeof(f)), "900") == 0, "path accel x");
    require(strcmp(field(line, 23, f, sizeof(f)), "1") == 0 && strcmp(field(line, 27, f, sizeof(f)), "21.4") == 0,
            "adopted observation and its age");

    row.n_det = TRACK_TRACE_MAX_DETS;
    for (int i = 0; i < TRACK_TRACE_MAX_DETS; i++)
        row.det[i] = (track_trace_det_t){ 100000 + i, 1311.99f, 971.99f, 1312.0f, 972.0f, 1.0f, 1, 1 };
    require(track_trace_format(line, sizeof(line), &row) > 0, "the widest row fits in a line");
    require(track_trace_format(line, 40, &row) == -1, "a row that does not fit is refused");
}

static void test_off(void)
{
    track_trace_row_t row = idle_row();

    require(!track_trace_on(), "off before open");
    track_trace_write(&row);
    require(track_trace_rows() == 0, "nothing is written while off");
    require(track_trace_open("") == 0 && !track_trace_on(), "an empty path leaves it off");
    require(track_trace_open(NULL) == 0 && !track_trace_on(), "no path leaves it off");
    track_trace_close();
    require(track_trace_open("/var/tmp/trace.csv") == -1 && !track_trace_on(), "outside /tmp is refused");
    require(track_trace_open("/tmp/../etc/trace.csv") == -1 && !track_trace_on(), "'..' is refused");
}

static void test_file(const char *dir)
{
    char path[256];
    char link_path[256];
    char line[TRACK_TRACE_LINE_MAX];
    track_trace_row_t row = idle_row();
    struct stat st;
    FILE *f;
    int lines = 0;

    snprintf(path, sizeof(path), "%s/trace.csv", dir);
    require(track_trace_open(path) == 0 && track_trace_on(), "a file under /tmp opens");
    for (int i = 0; i < 3; i++) {
        row.frame_seq = 10u + (unsigned int)i;
        track_trace_write(&row);
    }
    require(track_trace_rows() == 3, "three rows counted");
    track_trace_close();
    require(!track_trace_on(), "off after close");
    require(stat(path, &st) == 0 && (st.st_mode & 0777) == 0600, "the trace is created 0600");
    f = fopen(path, "r");
    if (!f) {
        require(0, "the trace can be read back");
        return;
    }
    while (fgets(line, sizeof(line), f)) {
        if (lines == 0)
            require(strncmp(line, track_trace_header(), strlen(track_trace_header())) == 0, "header first");
        else
            require(strtoul(line, NULL, 10) == 9u + (unsigned int)lines, "rows in order");
        lines++;
    }
    fclose(f);
    require(lines == 4, "header and three rows");

    snprintf(link_path, sizeof(link_path), "%s/link.csv", dir);
    require(symlink(path, link_path) == 0, "set-up: a symlink");
    require(track_trace_open(link_path) == -1 && !track_trace_on(), "a symlink is not followed");
    unlink(link_path);

    /* the size limit stops the trace and leaves the file within it */
    require(track_trace_open(path) == 0, "reopen");
    for (long i = 0; i < 200000 && track_trace_on(); i++)
        track_trace_write(&row);
    require(!track_trace_on(), "the trace stopped at the size limit");
    require(stat(path, &st) == 0 && st.st_size <= TRACK_TRACE_MAX_BYTES + 1024, "the file is within the limit");
    track_trace_close();
    unlink(path);
}

int main(void)
{
    char dir[] = "/tmp/track_trace_test_XXXXXX";

    test_format();
    test_off();
    require(mkdtemp(dir) != NULL, "set-up: a directory under /tmp");
    test_file(dir);
    rmdir(dir);
    printf("track_trace_test: OK (%d checks)\n", g_checks);
    return 0;
}
