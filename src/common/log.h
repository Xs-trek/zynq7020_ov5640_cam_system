/* log.h - leveled logging to stderr or syslog.
 *
 * Each source file defines LOG_TAG before including this header, then uses
 * LOG_E/LOG_W/LOG_I/LOG_D(fmt, ...).  On stderr a message is written as one
 * "[tag] message\n" line, the format the v0.1.1 log analyzer already parses.
 * LOG_*_RL variants allow at most `burst` messages per `interval` seconds per
 * call site and report how many were suppressed.
 */
#ifndef CAM_LOG_H
#define CAM_LOG_H

#ifdef __cplusplus
extern "C" {
#endif

typedef enum {
    CAM_LOG_ERROR = 0,
    CAM_LOG_WARN,
    CAM_LOG_INFO,
    CAM_LOG_DEBUG
} cam_log_level_t;

typedef enum {
    CAM_LOG_TARGET_STDERR = 0,
    CAM_LOG_TARGET_SYSLOG
} cam_log_target_t;

typedef struct {
    double   window_start;
    int      emitted;
    unsigned missed;
} cam_log_ratelimit_t;

#define CAM_LOG_RATELIMIT_INIT {0.0, 0, 0u}

/* Call once in main before any thread starts. */
void cam_log_init(cam_log_level_t level, cam_log_target_t target, const char *ident);
void cam_log_close(void);

/* Parses "error" | "warn" | "info" | "debug".  Returns 0, or -1 for an unknown name. */
int cam_log_level_from_name(const char *name, cam_log_level_t *level);

int cam_log_enabled(cam_log_level_t level);

void cam_log_write(cam_log_level_t level, const char *tag, const char *fmt, ...)
    __attribute__((format(printf, 3, 4)));

/* Rate-limit decision at time now (seconds).  Returns 1 if the message may be emitted.
 * When a new window opens, *missed receives the number suppressed in the previous one. */
int cam_log_ratelimit_at(cam_log_ratelimit_t *rl, double now, double interval, int burst,
                         unsigned *missed);

/* Same decision on CLOCK_MONOTONIC, serialized across threads. */
int cam_log_ratelimit(cam_log_ratelimit_t *rl, double interval, int burst, unsigned *missed);

#ifndef LOG_TAG
#define LOG_TAG "cam_system"
#endif

#define CAM_LOG_AT(level, fmt, ...)                                       \
    do {                                                                  \
        if (cam_log_enabled(level))                                       \
            cam_log_write((level), LOG_TAG, fmt, ##__VA_ARGS__);          \
    } while (0)

#define LOG_E(fmt, ...) CAM_LOG_AT(CAM_LOG_ERROR, fmt, ##__VA_ARGS__)
#define LOG_W(fmt, ...) CAM_LOG_AT(CAM_LOG_WARN, fmt, ##__VA_ARGS__)
#define LOG_I(fmt, ...) CAM_LOG_AT(CAM_LOG_INFO, fmt, ##__VA_ARGS__)
#define LOG_D(fmt, ...) CAM_LOG_AT(CAM_LOG_DEBUG, fmt, ##__VA_ARGS__)

#define CAM_LOG_RL_AT(level, interval, burst, fmt, ...)                                     \
    do {                                                                                    \
        static cam_log_ratelimit_t cam_log_rl_ = CAM_LOG_RATELIMIT_INIT;                    \
        unsigned cam_log_missed_ = 0;                                                       \
        if (cam_log_enabled(level) &&                                                       \
            cam_log_ratelimit(&cam_log_rl_, (interval), (burst), &cam_log_missed_)) {       \
            if (cam_log_missed_)                                                            \
                cam_log_write((level), LOG_TAG, "%u similar messages suppressed",           \
                              cam_log_missed_);                                             \
            cam_log_write((level), LOG_TAG, fmt, ##__VA_ARGS__);                            \
        }                                                                                   \
    } while (0)

#define LOG_E_RL(interval, burst, fmt, ...) CAM_LOG_RL_AT(CAM_LOG_ERROR, interval, burst, fmt, ##__VA_ARGS__)
#define LOG_W_RL(interval, burst, fmt, ...) CAM_LOG_RL_AT(CAM_LOG_WARN, interval, burst, fmt, ##__VA_ARGS__)
#define LOG_I_RL(interval, burst, fmt, ...) CAM_LOG_RL_AT(CAM_LOG_INFO, interval, burst, fmt, ##__VA_ARGS__)

#ifdef __cplusplus
}
#endif

#endif /* CAM_LOG_H */
