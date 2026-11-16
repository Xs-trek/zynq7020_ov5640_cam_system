/* cam_launcher_test.c - restart policy, and supervision of real child processes.
 *
 * The launcher's source is compiled in with short delays.  The child is a shell script that logs
 * each start and exits as told, so fork, exec, waitpid and the exit status are the real ones.
 */
#define LAUNCHER_BACKOFF_INITIAL_SEC 0.02
#define LAUNCHER_BACKOFF_MAX_SEC 0.08
#define LAUNCHER_STABLE_RUN_SEC 0.5
#define LAUNCHER_STOP_TIMEOUT_SEC 0.2
#define main launcher_main
#include "cam-launcher.c"
#undef main

#include <stdlib.h>

static int g_checks;
static char g_dir[64];
static char g_app[96];
static char g_starts[96];
static char g_log[96];

static void require(int cond, const char *what)
{
    g_checks++;
    if (!cond) {
        fprintf(stderr, "cam_launcher_test: FAIL: %s\n", what);
        exit(1);
    }
}

static void test_policy(void)
{
    launcher_policy_t policy;

    launcher_policy_init(&policy);
    require(launcher_restart_delay(&policy, 1, 0, 1.0) < 0.0, "orderly exit stays down");
    require(launcher_restart_delay(&policy, 1, EX_USAGE, 1.0) < 0.0, "usage error stays down");
    require(launcher_restart_delay(&policy, 1, EX_CONFIG, 1.0) < 0.0, "config error stays down");

    require(launcher_restart_delay(&policy, 1, 70, 0.1) == 0.02, "first restart");
    require(launcher_restart_delay(&policy, 1, 1, 0.1) == 0.04, "delay doubles");
    require(launcher_restart_delay(&policy, 0, 0, 0.1) == 0.08, "a signal counts as a fault");
    require(launcher_restart_delay(&policy, 1, 1, 0.1) == 0.08, "delay stops growing");
    require(launcher_restart_delay(&policy, 1, 1, 0.6) == 0.02, "a stable run starts over");
    require(launcher_restart_delay(&policy, 1, 1, 0.1) == 0.04, "and doubles again");
    require(launcher_restart_delay(&policy, 1, 0, 0.1) < 0.0, "orderly exit");
    require(launcher_restart_delay(&policy, 1, 1, 0.1) == 0.02, "orderly exit clears the delay");
}

static int count_starts(void)
{
    FILE *file = fopen(g_starts, "r");
    int lines = 0;
    int c;

    if (!file)
        return 0;
    while ((c = fgetc(file)) != EOF)
        if (c == '\n')
            lines++;
    fclose(file);
    return lines;
}

/* The child logs a start, then exits with the code in the file "code", or kills itself. */
static void write_app(void)
{
    FILE *file = fopen(g_app, "w");

    require(file != NULL, "create the fake application");
    fprintf(file,
            "#!/bin/sh\n"
            "echo start >> %s\n"
            "code=$(cat %s/code)\n"
            "[ \"$code\" = kill ] && kill -9 $$\n"
            "exit $code\n", g_starts, g_dir);
    fclose(file);
    require(chmod(g_app, 0755) == 0, "fake application is executable");
}

static void set_exit(const char *code)
{
    char path[96];
    FILE *file;

    snprintf(path, sizeof(path), "%s/code", g_dir);
    file = fopen(path, "w");
    require(file != NULL, "write the exit code");
    fprintf(file, "%s\n", code);
    fclose(file);
}

/* Runs supervision for a while, as the poll loop would. */
static void supervise_for(double seconds)
{
    const double end = monotonic_sec() + seconds;

    while (monotonic_sec() < end) {
        supervise(g_app, g_log, "");
        usleep(2000);
    }
}

static void test_supervision(void)
{
    launcher_policy_init(&g_policy);

    set_exit("0");
    require(launch_cam_system(g_app, g_log, "") == 0, "start");
    supervise_for(0.3);
    require(count_starts() == 1, "exit 0 is not restarted");
    require(g_child < 0, "exit 0 is reaped");

    set_exit("78");
    require(launch_cam_system(g_app, g_log, "") == 0, "start");
    supervise_for(0.3);
    require(count_starts() == 2, "exit 78 is not restarted");

    set_exit("64");
    require(launch_cam_system(g_app, g_log, "") == 0, "start");
    supervise_for(0.3);
    require(count_starts() == 3, "exit 64 is not restarted");

    /* faults restart until the process stays up or exits cleanly */
    set_exit("70");
    require(launch_cam_system(g_app, g_log, "") == 0, "start");
    supervise_for(0.25);
    require(count_starts() >= 6, "exit 70 is restarted repeatedly");
    set_exit("0");
    supervise_for(0.2);
    {
        int starts = count_starts();

        supervise_for(0.2);
        require(count_starts() == starts, "restarting stops at an orderly exit");
    }

    set_exit("kill");
    require(launch_cam_system(g_app, g_log, "") == 0, "start");
    supervise_for(0.1);
    set_exit("0");
    supervise_for(0.2);
    require(g_child < 0 && g_restart_at == 0.0, "a killed process is restarted, then stays down");
}

/* Starts a script through the launcher and waits until it has installed its trap. */
static void start_trapping_app(const char *trap_action)
{
    char ready[96];
    FILE *file = fopen(g_app, "w");

    snprintf(ready, sizeof(ready), "%s/ready", g_dir);
    (void)unlink(ready);
    require(file != NULL, "create the trapping application");
    fprintf(file,
            "#!/bin/sh\n"
            "trap '%s' TERM\n"
            "touch %s\n"
            "while :; do sleep 0.01; done\n", trap_action, ready);
    fclose(file);
    require(launch_cam_system(g_app, g_log, "") == 0, "start the trapping application");
    for (int i = 0; i < 500 && access(ready, F_OK) != 0; i++)
        usleep(2000);
    require(access(ready, F_OK) == 0, "the trapping application came up");
}

static void test_stop(void)
{
    double start;

    start_trapping_app("exit 0");
    start = monotonic_sec();
    stop_child();
    require(g_child < 0, "a stopped child is reaped");
    require(monotonic_sec() - start < LAUNCHER_STOP_TIMEOUT_SEC, "SIGTERM is enough");

    start_trapping_app(":");
    start = monotonic_sec();
    stop_child();
    require(g_child < 0, "a child that ignores SIGTERM is reaped");
    require(monotonic_sec() - start >= LAUNCHER_STOP_TIMEOUT_SEC, "it gets the whole timeout");

    g_restart_at = 1.0;
    stop_child();
    require(g_restart_at == 1.0, "nothing to stop leaves the state alone");
}

static void test_log_rotation(void)
{
    char old_path[100];
    FILE *file = fopen(g_log, "w");

    require(file != NULL, "create the log");
    require(fseek(file, CAM_LOG_ROTATE_BYTES, SEEK_SET) == 0 && fputc('x', file) != EOF, "grow the log");
    fclose(file);
    snprintf(old_path, sizeof(old_path), "%s.old", g_log);
    rotate_log(g_log);
    require(access(old_path, F_OK) == 0 && access(g_log, F_OK) != 0, "a large log is set aside");

    file = fopen(g_log, "w");
    require(file != NULL, "create a small log");
    fclose(file);
    (void)unlink(old_path);
    rotate_log(g_log);
    require(access(g_log, F_OK) == 0 && access(old_path, F_OK) != 0, "a small log stays");
}

/* Only an explicit 1 starts cam_system without the key. */
static void test_autostart(void)
{
    (void)unsetenv("CAM_LAUNCHER_AUTOSTART");
    require(!autostart_requested(), "autostart is off when unset");
    setenv("CAM_LAUNCHER_AUTOSTART", "", 1);
    require(!autostart_requested(), "autostart is off when empty");
    setenv("CAM_LAUNCHER_AUTOSTART", "0", 1);
    require(!autostart_requested(), "autostart=0 is off");
    setenv("CAM_LAUNCHER_AUTOSTART", "1", 1);
    require(autostart_requested(), "autostart=1 is on");
    setenv("CAM_LAUNCHER_AUTOSTART", "2", 1);
    require(!autostart_requested(), "an out-of-range value is off");
    setenv("CAM_LAUNCHER_AUTOSTART", "yes", 1);
    require(!autostart_requested(), "a word is off");
    (void)unsetenv("CAM_LAUNCHER_AUTOSTART");
}

int main(void)
{
    snprintf(g_dir, sizeof(g_dir), "/tmp/cam_launcher_test.XXXXXX");
    require(mkdtemp(g_dir) != NULL, "temporary directory");
    snprintf(g_app, sizeof(g_app), "%s/app", g_dir);
    snprintf(g_starts, sizeof(g_starts), "%s/starts", g_dir);
    snprintf(g_log, sizeof(g_log), "%s/log", g_dir);
    write_app();

    test_policy();
    test_autostart();
    test_supervision();
    test_stop();
    test_log_rotation();

    {
        char command[128];

        snprintf(command, sizeof(command), "rm -rf %s", g_dir);
        if (system(command) != 0)
            fputs("cam_launcher_test: temporary directory left behind\n", stderr);
    }
    printf("cam_launcher_test: OK (%d checks)\n", g_checks);
    return 0;
}
