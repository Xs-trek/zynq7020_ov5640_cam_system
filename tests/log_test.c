#define _POSIX_C_SOURCE 200809L
#define LOG_TAG "logtest"

#include "common/log.h"

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>

static int g_checks;

static void require(int cond, const char *what)
{
    g_checks++;
    if (!cond) {
        fprintf(stderr, "log_test: FAIL: %s\n", what);
        exit(1);
    }
}

static void test_ratelimit(void)
{
    cam_log_ratelimit_t rl = CAM_LOG_RATELIMIT_INIT;
    unsigned missed = 99;

    require(cam_log_ratelimit_at(&rl, 10.0, 1.0, 2, &missed) == 1 && missed == 0, "first message passes");
    require(cam_log_ratelimit_at(&rl, 10.1, 1.0, 2, &missed) == 1, "burst of two passes");
    require(cam_log_ratelimit_at(&rl, 10.2, 1.0, 2, &missed) == 0, "third in window suppressed");
    require(cam_log_ratelimit_at(&rl, 10.9, 1.0, 2, &missed) == 0, "fourth in window suppressed");
    require(cam_log_ratelimit_at(&rl, 11.0, 1.0, 2, &missed) == 1 && missed == 2,
            "new window passes and reports suppressed count");
    require(cam_log_ratelimit_at(&rl, 11.1, 1.0, 2, &missed) == 1 && missed == 0, "count reported once");
    require(cam_log_ratelimit_at(&rl, 11.2, 0.0, 1, &missed) == 1, "zero interval disables limiting");
    require(cam_log_ratelimit_at(&rl, 11.3, 1.0, 0, &missed) == 1, "zero burst disables limiting");
}

static void test_levels(void)
{
    cam_log_level_t level = CAM_LOG_ERROR;

    require(cam_log_level_from_name("debug", &level) == 0 && level == CAM_LOG_DEBUG, "parse debug");
    require(cam_log_level_from_name("warn", &level) == 0 && level == CAM_LOG_WARN, "parse warn");
    require(cam_log_level_from_name("verbose", &level) == -1, "reject unknown level");
    require(cam_log_level_from_name(NULL, &level) == -1, "reject null level");
}

/* Runs the logging calls with stderr redirected to a file and returns what was written. */
static void capture_output(char *out, size_t len)
{
    char path[] = "/tmp/log_test_XXXXXX";
    int fd = mkstemp(path);
    int saved = dup(STDERR_FILENO);
    FILE *f;
    size_t n;

    require(fd >= 0 && saved >= 0, "redirect setup");
    fflush(stderr);
    require(dup2(fd, STDERR_FILENO) >= 0, "redirect stderr");

    cam_log_init(CAM_LOG_WARN, CAM_LOG_TARGET_STDERR, "log_test");
    LOG_E("error %d", 1);
    LOG_W("warn with newline\n");
    LOG_I("info is below the threshold");
    LOG_D("debug is below the threshold");
    for (int i = 0; i < 5; i++)
        LOG_W_RL(60.0, 2, "limited %d", i);
    cam_log_close();

    fflush(stderr);
    dup2(saved, STDERR_FILENO);
    close(saved);
    f = fdopen(fd, "r");
    require(f != NULL, "reopen capture");
    rewind(f);
    n = fread(out, 1, len - 1, f);
    out[n] = '\0';
    fclose(f);
    unlink(path);
}

static void test_output(void)
{
    char out[1024];

    capture_output(out, sizeof(out));
    require(strcmp(out,
                   "[logtest] error 1\n"
                   "[logtest] warn with newline\n"
                   "[logtest] limited 0\n"
                   "[logtest] limited 1\n") == 0,
            "stderr format, threshold and rate limit");
    require(cam_log_enabled(CAM_LOG_ERROR) && !cam_log_enabled(CAM_LOG_INFO), "threshold kept after close");
}

int main(void)
{
    test_ratelimit();
    test_levels();
    test_output();
    printf("log_test: OK (%d checks)\n", g_checks);
    return 0;
}
