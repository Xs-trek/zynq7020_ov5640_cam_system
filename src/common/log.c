/* log.c - see log.h */
#define _POSIX_C_SOURCE 200809L
#include "common/log.h"

#include <pthread.h>
#include <stdarg.h>
#include <stdio.h>
#include <string.h>
#include <syslog.h>
#include <time.h>

#define LOG_LINE_MAX 1024

static cam_log_level_t g_level = CAM_LOG_INFO;
static cam_log_target_t g_target = CAM_LOG_TARGET_STDERR;
static pthread_mutex_t g_rl_lock = PTHREAD_MUTEX_INITIALIZER;

void cam_log_init(cam_log_level_t level, cam_log_target_t target, const char *ident)
{
    g_level = level;
    g_target = target;
    if (target == CAM_LOG_TARGET_SYSLOG)
        openlog(ident ? ident : "cam_system", LOG_PID | LOG_NDELAY, LOG_DAEMON);
}

void cam_log_close(void)
{
    if (g_target == CAM_LOG_TARGET_SYSLOG)
        closelog();
    g_target = CAM_LOG_TARGET_STDERR;
}

int cam_log_level_from_name(const char *name, cam_log_level_t *level)
{
    static const char *const names[] = {"error", "warn", "info", "debug"};

    for (int i = 0; i < 4; i++) {
        if (name && strcmp(name, names[i]) == 0) {
            *level = (cam_log_level_t)i;
            return 0;
        }
    }
    return -1;
}

int cam_log_enabled(cam_log_level_t level)
{
    return level <= g_level;
}

void cam_log_write(cam_log_level_t level, const char *tag, const char *fmt, ...)
{
    static const int prio[] = {LOG_ERR, LOG_WARNING, LOG_INFO, LOG_DEBUG};
    char msg[LOG_LINE_MAX];
    va_list ap;
    int n;

    if (!cam_log_enabled(level))
        return;
    va_start(ap, fmt);
    n = vsnprintf(msg, sizeof(msg), fmt, ap);
    va_end(ap);
    if (n < 0)
        return;
    /* Drop a trailing newline so callers converted from printf do not produce blank lines. */
    n = (int)strlen(msg);
    if (n > 0 && msg[n - 1] == '\n')
        msg[n - 1] = '\0';

    if (g_target == CAM_LOG_TARGET_SYSLOG)
        syslog(prio[level], "[%s] %s", tag, msg);
    else
        fprintf(stderr, "[%s] %s\n", tag, msg);  /* one call: stdio locks the stream per call */
}

int cam_log_ratelimit_at(cam_log_ratelimit_t *rl, double now, double interval, int burst,
                         unsigned *missed)
{
    if (missed)
        *missed = 0;
    if (interval <= 0.0 || burst <= 0)
        return 1;
    if (rl->emitted == 0 && rl->missed == 0 && rl->window_start == 0.0)
        rl->window_start = now;
    if (now - rl->window_start >= interval) {
        if (missed)
            *missed = rl->missed;
        rl->window_start = now;
        rl->emitted = 0;
        rl->missed = 0;
    }
    if (rl->emitted < burst) {
        rl->emitted++;
        return 1;
    }
    rl->missed++;
    return 0;
}

int cam_log_ratelimit(cam_log_ratelimit_t *rl, double interval, int burst, unsigned *missed)
{
    struct timespec ts;
    int ok;

    clock_gettime(CLOCK_MONOTONIC, &ts);
    pthread_mutex_lock(&g_rl_lock);
    ok = cam_log_ratelimit_at(rl, (double)ts.tv_sec + (double)ts.tv_nsec * 1e-9, interval, burst, missed);
    pthread_mutex_unlock(&g_rl_lock);
    return ok;
}
