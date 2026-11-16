/* track_trace.c - see track_trace.h */
#define _GNU_SOURCE
#define LOG_TAG "trace"

#include "control/track_trace.h"

#include "common/log.h"

#include <errno.h>
#include <fcntl.h>
#include <stdarg.h>
#include <stdio.h>
#include <string.h>
#include <sys/stat.h>
#include <unistd.h>

#define TRACK_TRACE_DIR "/tmp/"
#define TRACK_TRACE_BUFFER (64 * 1024)

static FILE *g_file;
static int g_on;
static unsigned long g_rows;
static long g_bytes;
static char g_path[256];
static char g_buffer[TRACK_TRACE_BUFFER];

static const char *const k_tick_names[] = { "det", "deadline", "immediate" };

const char *track_trace_header(void)
{
    return "frame_seq,v4l2_seq,t,tick,state,isp_x,isp_y,prev_slip,"
           "kf_x,kf_y,kf_vx,kf_vy,kf_log_h,ref_x,ref_y,ref_vx,ref_vy,"
           "path_x,path_y,path_vx,path_vy,path_ax,path_ay,"
           "obs,obs_x,obs_y,obs_h,obs_age_ms,dets";
}

/* Appends to buf at *pos; returns -1 once the line no longer fits. */
static int put(char *buf, size_t len, size_t *pos, const char *fmt, ...)
    __attribute__((format(printf, 4, 5)));

static int put(char *buf, size_t len, size_t *pos, const char *fmt, ...)
{
    va_list ap;
    int n;

    if (*pos >= len)
        return -1;
    va_start(ap, fmt);
    n = vsnprintf(buf + *pos, len - *pos, fmt, ap);
    va_end(ap);
    if (n < 0 || (size_t)n >= len - *pos)
        return -1;
    *pos += (size_t)n;
    return 0;
}

int track_trace_format(char *buf, size_t len, const track_trace_row_t *row)
{
    size_t pos = 0;
    int tick = row->tick >= TRACK_TRACE_TICK_DETECTION && row->tick <= TRACK_TRACE_TICK_IMMEDIATE ?
               (int)row->tick : TRACK_TRACE_TICK_IMMEDIATE;
    int r = 0;

    r |= put(buf, len, &pos, "%u,%u,%.6f,%s,%s,%d,%d,%d,", row->frame_seq, (unsigned int)row->v4l2_seq,
             row->capture_time, k_tick_names[tick], row->state ? row->state : "", row->isp_x, row->isp_y,
             row->prev_slip ? 1 : 0);
    if (row->active)
        r |= put(buf, len, &pos, "%.2f,%.2f,%.1f,%.1f,%.4f,%.2f,%.2f,%.1f,%.1f,%.2f,%.2f,%.1f,%.1f,%.0f,%.0f,",
                 row->kf_x, row->kf_y, row->kf_vx, row->kf_vy, row->kf_log_h,
                 row->ref_x, row->ref_y, row->ref_vx, row->ref_vy,
                 row->path_x, row->path_y, row->path_vx, row->path_vy, row->path_ax, row->path_ay);
    else
        r |= put(buf, len, &pos, ",,,,,,,,,,,,,,,");
    if (row->adopted)
        r |= put(buf, len, &pos, "1,%.2f,%.2f,%.1f,%.1f,", row->obs_x, row->obs_y, row->obs_h, row->obs_age_ms);
    else
        r |= put(buf, len, &pos, "0,,,,,");
    /* detections: id:x:y:w:h:score:publish:target, separated by ';' */
    for (int i = 0; i < row->n_det && i < TRACK_TRACE_MAX_DETS; i++) {
        const track_trace_det_t *d = &row->det[i];

        r |= put(buf, len, &pos, "%s%d:%.2f:%.2f:%.1f:%.1f:%.3f:%d:%d", i ? ";" : "", d->id, d->x, d->y,
                 d->w, d->h, d->score, d->publish ? 1 : 0, d->target ? 1 : 0);
    }
    r |= put(buf, len, &pos, "\n");
    return r ? -1 : (int)pos;
}

int track_trace_open(const char *path)
{
    int fd;

    track_trace_close();
    if (!path || path[0] == '\0')
        return 0;
    if (strncmp(path, TRACK_TRACE_DIR, strlen(TRACK_TRACE_DIR)) != 0 || strstr(path, "..") ||
        strlen(path) >= sizeof(g_path)) {
        LOG_W("tracker.trace = %s refused: the trace must be a file under %s", path, TRACK_TRACE_DIR);
        return -1;
    }
    fd = open(path, O_WRONLY | O_CREAT | O_TRUNC | O_NOFOLLOW | O_CLOEXEC, S_IRUSR | S_IWUSR);
    if (fd < 0) {
        LOG_W("tracker.trace = %s: %s; trace off", path, strerror(errno));
        return -1;
    }
    g_file = fdopen(fd, "w");
    if (!g_file) {
        LOG_W("tracker.trace = %s: %s; trace off", path, strerror(errno));
        close(fd);
        return -1;
    }
    setvbuf(g_file, g_buffer, _IOFBF, sizeof(g_buffer));
    snprintf(g_path, sizeof(g_path), "%s", path);
    g_rows = 0;
    g_bytes = 0;
    g_on = 1;
    if (fputs(track_trace_header(), g_file) == EOF || fputc('\n', g_file) == EOF) {
        LOG_W("tracker.trace %s: write failed; trace off", g_path);
        track_trace_close();
        return -1;
    }
    LOG_I("tracker.trace: numbers only, to %s (stops at %ld MiB)", g_path, TRACK_TRACE_MAX_BYTES >> 20);
    return 0;
}

int track_trace_on(void)
{
    return g_on;
}

void track_trace_write(const track_trace_row_t *row)
{
    char line[TRACK_TRACE_LINE_MAX];
    int n;

    if (!g_on)
        return;
    n = track_trace_format(line, sizeof(line), row);
    if (n < 0)
        return;
    if (g_bytes + n > TRACK_TRACE_MAX_BYTES) {
        LOG_W("tracker.trace %s reached %ld MiB; trace stopped", g_path, TRACK_TRACE_MAX_BYTES >> 20);
        track_trace_close();
        return;
    }
    if (fwrite(line, 1, (size_t)n, g_file) != (size_t)n) {
        LOG_W("tracker.trace %s: write failed: %s; trace stopped", g_path, strerror(errno));
        track_trace_close();
        return;
    }
    g_bytes += n;
    g_rows++;
}

unsigned long track_trace_rows(void)
{
    return g_rows;
}

void track_trace_close(void)
{
    if (!g_file)
        return;
    g_on = 0;
    if (fclose(g_file) != 0)
        LOG_W("tracker.trace %s: closing failed: %s", g_path, strerror(errno));
    else
        LOG_I("tracker.trace %s: %lu rows", g_path, g_rows);
    g_file = NULL;
}
