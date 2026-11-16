#include "launcher_policy.h"

#include <ctype.h>
#include <dirent.h>
#include <errno.h>
#include <fcntl.h>
#include <grp.h>
#include <limits.h>
#include <linux/input.h>
#include <poll.h>
#include <pwd.h>
#include <signal.h>
#include <stdarg.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/ioctl.h>
#include <sys/stat.h>
#include <sys/types.h>
#include <sys/wait.h>
#include <time.h>
#include <unistd.h>

#define DEFAULT_EVENT_NAME "gpio-keys"
#define DEFAULT_KEY_CODE 148
#define DEFAULT_CAM_BINARY "/usr/bin/cam_system"
#define DEFAULT_CAM_LOG "/tmp/cam_system.log"
#define DEFAULT_CAM_USER "camsys"
#define CAM_LOG_ROTATE_BYTES (1024 * 1024)
#define POLL_INTERVAL_MS 2000
#define STOP_POLL_MS 50
#define MAX_INPUT_DEVICES 32
#define INPUT_NAME_LEN 128
#define BIT_WORD_BITS ((unsigned int)(sizeof(unsigned long) * CHAR_BIT))

struct watched_input {
    int fd;
    char path[32];
    char name[INPUT_NAME_LEN];
};

static volatile sig_atomic_t g_stop = 0;
static pid_t g_child = -1;
static double g_child_started_at;
static double g_restart_at;     /* 0 when no restart is pending */
static launcher_policy_t g_policy;

static void on_signal(int signo)
{
    (void)signo;
    g_stop = 1;
}

/* Only interrupts poll; supervise() reaps the child. */
static void on_child(int signo)
{
    (void)signo;
}

static double monotonic_sec(void)
{
    struct timespec ts;

    clock_gettime(CLOCK_MONOTONIC, &ts);
    return (double)ts.tv_sec + (double)ts.tv_nsec / 1000000000.0;
}

static void log_msg(const char *fmt, ...)
{
    time_t now = time(NULL);
    struct tm tm_now;
    char stamp[32] = "unknown-time";

    if (now != (time_t)-1 && localtime_r(&now, &tm_now) != NULL) {
        strftime(stamp, sizeof(stamp), "%Y-%m-%d %H:%M:%S", &tm_now);
    }

    fprintf(stdout, "[%s] ", stamp);

    va_list ap;
    va_start(ap, fmt);
    vfprintf(stdout, fmt, ap);
    va_end(ap);

    fputc('\n', stdout);
    fflush(stdout);
}

static long env_long(const char *name, long fallback, long min_value, long max_value)
{
    const char *value = getenv(name);
    if (value == NULL || *value == '\0') {
        return fallback;
    }

    errno = 0;
    char *end = NULL;
    long parsed = strtol(value, &end, 10);
    if (errno != 0 || end == value || *end != '\0' ||
        parsed < min_value || parsed > max_value) {
        log_msg("invalid %s=%s, using %ld", name, value, fallback);
        return fallback;
    }

    return parsed;
}

/* CAM_LAUNCHER_AUTOSTART=1 starts cam_system once at launcher start, as a key press would.  Any
 * other value leaves it to the key. */
static int autostart_requested(void)
{
    return env_long("CAM_LAUNCHER_AUTOSTART", 0, 0, 1) == 1;
}

static int bit_is_set(unsigned int bit, const unsigned long *bits, size_t words)
{
    size_t word = bit / BIT_WORD_BITS;
    unsigned int shift = bit % BIT_WORD_BITS;

    if (word >= words) {
        return 0;
    }

    return ((bits[word] >> shift) & 1UL) != 0;
}

static int input_has_event(int fd, unsigned int event_type)
{
    unsigned long bits[(EV_MAX / BIT_WORD_BITS) + 1];
    memset(bits, 0, sizeof(bits));

    if (ioctl(fd, EVIOCGBIT(0, sizeof(bits)), bits) < 0) {
        return 0;
    }

    return bit_is_set(event_type, bits, sizeof(bits) / sizeof(bits[0]));
}

static int input_has_key(int fd, unsigned int key_code)
{
    unsigned long bits[(KEY_MAX / BIT_WORD_BITS) + 1];
    memset(bits, 0, sizeof(bits));

    if (ioctl(fd, EVIOCGBIT(EV_KEY, sizeof(bits)), bits) < 0) {
        return 0;
    }

    return bit_is_set(key_code, bits, sizeof(bits) / sizeof(bits[0]));
}

static int process_comm_equals(pid_t pid, const char *expected)
{
    char path[64];
    int n = snprintf(path, sizeof(path), "/proc/%ld/comm", (long)pid);
    if (n < 0 || (size_t)n >= sizeof(path)) {
        return 0;
    }

    FILE *fp = fopen(path, "r");
    if (fp == NULL) {
        return 0;
    }

    char comm[64];
    char *line = fgets(comm, sizeof(comm), fp);
    fclose(fp);

    if (line == NULL) {
        return 0;
    }

    comm[strcspn(comm, "\r\n")] = '\0';
    return strcmp(comm, expected) == 0;
}

static int cam_system_running(void)
{
    DIR *proc = opendir("/proc");
    if (proc == NULL) {
        log_msg("failed to open /proc: %s", strerror(errno));
        return 0;
    }

    struct dirent *entry;
    while ((entry = readdir(proc)) != NULL) {
        const char *name = entry->d_name;
        if (!isdigit((unsigned char)name[0])) {
            continue;
        }

        char *end = NULL;
        errno = 0;
        long pid_long = strtol(name, &end, 10);
        if (errno != 0 || end == name || *end != '\0' ||
            pid_long <= 0 || pid_long > INT_MAX) {
            continue;
        }

        if (process_comm_equals((pid_t)pid_long, "cam_system")) {
            closedir(proc);
            return 1;
        }
    }

    closedir(proc);
    return 0;
}

/* The log lives in RAM.  A start is the one moment it can be cut without a rotation daemon, and a
 * crash loop is what would make it grow. */
static void rotate_log(const char *log_path)
{
    struct stat st;
    char old_path[PATH_MAX];
    int n = snprintf(old_path, sizeof(old_path), "%s.old", log_path);

    if (n > 0 && (size_t)n < sizeof(old_path) && lstat(log_path, &st) == 0 &&
        S_ISREG(st.st_mode) && st.st_size >= CAM_LOG_ROTATE_BYTES) {
        (void)rename(log_path, old_path);
    }
}

static int redirect_child_stdio(const char *log_path)
{
    int null_fd = open("/dev/null", O_RDONLY | O_CLOEXEC);
    if (null_fd < 0) {
        return -1;
    }

    if (dup2(null_fd, STDIN_FILENO) < 0) {
        close(null_fd);
        return -1;
    }
    close(null_fd);

    rotate_log(log_path);

    /* Still root here, and the default path is in a world-writable directory. */
    int log_fd = open(log_path, O_WRONLY | O_CREAT | O_APPEND | O_CLOEXEC | O_NOFOLLOW, 0644);
    if (log_fd < 0) {
        return -1;
    }

    if (dup2(log_fd, STDOUT_FILENO) < 0 ||
        dup2(log_fd, STDERR_FILENO) < 0) {
        close(log_fd);
        return -1;
    }

    close(log_fd);
    return 0;
}

/* Child, before exec.  Returns -1 if the process would keep privileges it should not have. */
static int drop_privileges(const struct passwd *user)
{
    if (user == NULL) {
        return 0;
    }

    if (initgroups(user->pw_name, user->pw_gid) < 0 ||
        setgid(user->pw_gid) < 0 ||
        setuid(user->pw_uid) < 0) {
        return -1;
    }

    return setuid(0) == 0 ? -1 : 0;
}

static int launch_cam_system(const char *binary, const char *log_path, const char *user_name)
{
    if (g_child > 0 || cam_system_running()) {
        log_msg("cam_system already running; key press ignored");
        return 0;
    }

    if (access(binary, X_OK) != 0) {
        log_msg("cam_system binary is not executable: %s: %s",
                binary, strerror(errno));
        return -1;
    }

    /* The application needs no privileges.  Started as root, it must not keep them. */
    const struct passwd *user = NULL;
    if (geteuid() == 0) {
        user = getpwnam(user_name);
        if (user == NULL) {
            log_msg("user %s does not exist; refusing to run cam_system as root", user_name);
            return -1;
        }
    }

    pid_t pid = fork();
    if (pid < 0) {
        log_msg("fork failed: %s", strerror(errno));
        return -1;
    }

    if (pid == 0) {
        if (setsid() < 0) {
            _exit(126);
        }

        if (redirect_child_stdio(log_path) < 0) {
            _exit(126);
        }

        if (drop_privileges(user) < 0) {
            _exit(126);
        }

        execl(binary, binary, (char *)NULL);
        _exit(127);
    }

    g_child = pid;
    g_child_started_at = monotonic_sec();
    g_restart_at = 0.0;
    log_msg("started cam_system pid=%ld", (long)pid);
    return 0;
}

/* Reaps the child, decides about a restart and carries out one that is due. */
static void supervise(const char *binary, const char *log_path, const char *user_name)
{
    if (g_child > 0) {
        int status;
        pid_t pid = waitpid(g_child, &status, WNOHANG);

        if (pid == g_child) {
            const int exited = WIFEXITED(status);
            const int code = exited ? WEXITSTATUS(status) : 0;
            const double now = monotonic_sec();
            const double delay = launcher_restart_delay(&g_policy, exited, code,
                                                        now - g_child_started_at);

            g_child = -1;
            if (exited) {
                log_msg("cam_system exited with status %d", code);
            } else {
                log_msg("cam_system killed by signal %d", WIFSIGNALED(status) ? WTERMSIG(status) : 0);
            }
            if (delay >= 0.0) {
                g_restart_at = now + delay;
                log_msg("restarting cam_system in %.1f s", delay);
            }
        }
    }

    if (g_restart_at > 0.0 && monotonic_sec() >= g_restart_at) {
        g_restart_at = 0.0;
        (void)launch_cam_system(binary, log_path, user_name);
    }
}

/* SIGTERM, then SIGKILL if the process has not stopped in time. */
static void stop_child(void)
{
    if (g_child <= 0) {
        return;
    }

    const double deadline = monotonic_sec() + LAUNCHER_STOP_TIMEOUT_SEC;
    int killed = 0;

    log_msg("stopping cam_system pid=%ld", (long)g_child);
    (void)kill(g_child, SIGTERM);
    for (;;) {
        int status;
        pid_t pid = waitpid(g_child, &status, WNOHANG);

        if (pid == g_child || (pid < 0 && errno != EINTR)) {
            break;
        }
        if (!killed && monotonic_sec() >= deadline) {
            log_msg("cam_system did not stop in %.0f s; killing it", LAUNCHER_STOP_TIMEOUT_SEC);
            (void)kill(g_child, SIGKILL);
            killed = 1;
        }
        (void)poll(NULL, 0, STOP_POLL_MS);
    }
    g_child = -1;
    g_restart_at = 0.0;
}

/* Poll until the next pending restart, at most POLL_INTERVAL_MS. */
static int poll_timeout_ms(void)
{
    if (g_restart_at > 0.0) {
        double left_ms = (g_restart_at - monotonic_sec()) * 1000.0;

        if (left_ms < 0.0) {
            return 0;
        }
        if (left_ms < POLL_INTERVAL_MS) {
            return (int)left_ms + 1;
        }
    }
    return POLL_INTERVAL_MS;
}

static int scan_inputs(struct watched_input *inputs,
                       size_t capacity,
                       const char *name_filter,
                       unsigned int key_code)
{
    size_t count = 0;

    for (int i = 0; i < MAX_INPUT_DEVICES && count < capacity; ++i) {
        char path[32];
        int n = snprintf(path, sizeof(path), "/dev/input/event%d", i);
        if (n < 0 || (size_t)n >= sizeof(path)) {
            continue;
        }

        int fd = open(path, O_RDONLY | O_NONBLOCK | O_CLOEXEC);
        if (fd < 0) {
            continue;
        }

        char name[INPUT_NAME_LEN] = "";
        if (ioctl(fd, EVIOCGNAME(sizeof(name)), name) < 0) {
            close(fd);
            continue;
        }

        if (name_filter[0] != '\0' && strstr(name, name_filter) == NULL) {
            close(fd);
            continue;
        }

        int has_event = input_has_event(fd, EV_KEY);
        int has_key = input_has_key(fd, key_code);
        if (!has_event || !has_key) {
            log_msg("input candidate %s name='%s' lacks event/key: EV_KEY=%d key_%u=%d",
                    path, name, has_event, key_code, has_key);
            close(fd);
            continue;
        }

        inputs[count].fd = fd;
        snprintf(inputs[count].path, sizeof(inputs[count].path), "%s", path);
        snprintf(inputs[count].name, sizeof(inputs[count].name), "%s", name);
        ++count;
    }

    return (int)count;
}

static void close_inputs(struct watched_input *inputs, int count)
{
    for (int i = 0; i < count; ++i) {
        if (inputs[i].fd >= 0) {
            close(inputs[i].fd);
            inputs[i].fd = -1;
        }
    }
}

static void handle_input_event(const struct input_event *event,
                               unsigned int key_code,
                               const char *binary,
                               const char *log_path,
                               const char *user_name)
{
    if (event->type != EV_KEY || event->code != key_code) {
        return;
    }

    if (event->value == 1) {
        log_msg("key code %u pressed", key_code);
        (void)launch_cam_system(binary, log_path, user_name);
    }
}

static void watch_inputs(const char *name_filter,
                         unsigned int key_code,
                         const char *binary,
                         const char *log_path,
                         const char *user_name)
{
    while (!g_stop) {
        struct watched_input inputs[MAX_INPUT_DEVICES];
        for (int i = 0; i < MAX_INPUT_DEVICES; ++i) {
            inputs[i].fd = -1;
        }

        int count = scan_inputs(inputs, MAX_INPUT_DEVICES, name_filter, key_code);
        if (count <= 0) {
            log_msg("no input device found for name='%s' key=%u; rescanning",
                    name_filter, key_code);
            sleep(2);
            supervise(binary, log_path, user_name);
            continue;
        }

        for (int i = 0; i < count; ++i) {
            log_msg("watching %s name='%s' key=%u",
                    inputs[i].path, inputs[i].name, key_code);
        }

        struct pollfd fds[MAX_INPUT_DEVICES];
        for (int i = 0; i < count; ++i) {
            fds[i].fd = inputs[i].fd;
            fds[i].events = POLLIN;
            fds[i].revents = 0;
        }

        while (!g_stop) {
            supervise(binary, log_path, user_name);

            int rc = poll(fds, (nfds_t)count, poll_timeout_ms());
            if (rc < 0) {
                if (errno == EINTR) {
                    continue;
                }
                log_msg("poll failed: %s", strerror(errno));
                break;
            }

            if (rc == 0) {
                continue;
            }

            int need_rescan = 0;
            for (int i = 0; i < count; ++i) {
                if ((fds[i].revents & (POLLERR | POLLHUP | POLLNVAL)) != 0) {
                    need_rescan = 1;
                    continue;
                }

                if ((fds[i].revents & POLLIN) == 0) {
                    continue;
                }

                for (;;) {
                    struct input_event event;
                    ssize_t n = read(fds[i].fd, &event, sizeof(event));
                    if (n < 0) {
                        if (errno == EAGAIN || errno == EWOULDBLOCK) {
                            break;
                        }
                        log_msg("read %s failed: %s",
                                inputs[i].path, strerror(errno));
                        need_rescan = 1;
                        break;
                    }

                    if (n != (ssize_t)sizeof(event)) {
                        log_msg("short read from %s", inputs[i].path);
                        need_rescan = 1;
                        break;
                    }

                    handle_input_event(&event, key_code, binary, log_path, user_name);
                }
            }

            if (need_rescan) {
                break;
            }
        }

        close_inputs(inputs, count);
    }
}

int main(void)
{
    /* The init script may run from a login shell; its hangup must not end the service. */
    (void)setsid();

    struct sigaction sa;
    memset(&sa, 0, sizeof(sa));
    sa.sa_handler = on_signal;
    sigemptyset(&sa.sa_mask);
    sigaction(SIGTERM, &sa, NULL);
    sigaction(SIGINT, &sa, NULL);

    struct sigaction child_sa;
    memset(&child_sa, 0, sizeof(child_sa));
    child_sa.sa_handler = on_child;
    child_sa.sa_flags = SA_NOCLDSTOP;
    sigemptyset(&child_sa.sa_mask);
    sigaction(SIGCHLD, &child_sa, NULL);

    launcher_policy_init(&g_policy);

    const char *name_filter = getenv("CAM_LAUNCHER_EVENT_NAME");
    if (name_filter == NULL) {
        name_filter = DEFAULT_EVENT_NAME;
    }

    long key_code_long = env_long("CAM_LAUNCHER_KEY_CODE",
                                  DEFAULT_KEY_CODE, 0, KEY_MAX);
    unsigned int key_code = (unsigned int)key_code_long;

    const char *binary = getenv("CAM_LAUNCHER_CAM_BINARY");
    if (binary == NULL || *binary == '\0') {
        binary = DEFAULT_CAM_BINARY;
    }

    const char *log_path = getenv("CAM_LAUNCHER_CAM_LOG");
    if (log_path == NULL || *log_path == '\0') {
        log_path = DEFAULT_CAM_LOG;
    }

    const char *user_name = getenv("CAM_LAUNCHER_CAM_USER");
    if (user_name == NULL || *user_name == '\0') {
        user_name = DEFAULT_CAM_USER;
    }

    const int autostart = autostart_requested();

    log_msg("cam-launcher starting: event_name='%s' key=%u binary=%s user=%s autostart=%d",
            name_filter, key_code, binary, user_name, autostart);
    if (autostart) {
        log_msg("autostart: starting cam_system without a key press");
        (void)launch_cam_system(binary, log_path, user_name);
    }
    watch_inputs(name_filter, key_code, binary, log_path, user_name);
    stop_child();
    log_msg("cam-launcher exiting");

    return 0;
}
