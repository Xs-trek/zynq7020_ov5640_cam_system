#include "network/mjpeg_framing.h"

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
#define EXPECT_EQ(a, b) expect_true((a) == (b), #a " == " #b, __LINE__)

static const char DELIMITER[] = "--" MJPEG_BOUNDARY "\r\n";

static int ends_with(const char *s, const char *suffix)
{
    size_t len = strlen(s), suffix_len = strlen(suffix);

    return len >= suffix_len && strcmp(s + len - suffix_len, suffix) == 0;
}

static void test_first_part_opens_with_a_delimiter(void)
{
    EXPECT_TRUE(strncmp(MJPEG_FIRST_PART_HEAD, DELIMITER, strlen(DELIMITER)) == 0);
    EXPECT_TRUE(strstr(MJPEG_FIRST_PART_HEAD, "Content-Type: image/jpeg\r\n") != NULL);
    EXPECT_TRUE(ends_with(MJPEG_FIRST_PART_HEAD, "\r\n\r\n"));
}

static void test_tail_closes_the_part_and_opens_the_next(void)
{
    char tail[MJPEG_PART_TAIL_MAX];
    int n = mjpeg_format_part_tail(tail, sizeof(tail), 41, 3);

    EXPECT_EQ(n, (int)strlen(tail));
    EXPECT_TRUE(strncmp(tail, "\r\n", 2) == 0);
    EXPECT_TRUE(strncmp(tail + 2, DELIMITER, strlen(DELIMITER)) == 0);
    EXPECT_TRUE(strstr(tail, "Content-Type: image/jpeg\r\n") != NULL);
    EXPECT_TRUE(strstr(tail, "X-Frame-Seq: 41\r\n") != NULL);
    EXPECT_TRUE(strstr(tail, "X-Dropped-Frames: 3\r\n") != NULL);
    /* The length of the next frame is not known when this is sent. */
    EXPECT_TRUE(strstr(tail, "Content-Length") == NULL);
    EXPECT_TRUE(ends_with(tail, "\r\n\r\n"));
}

/* Splits a stream the way a client does; the payload of each part must come back unchanged. */
static void test_a_stream_parses_back_into_its_frames(void)
{
    static const char *const frames[] = {"\xff\xd8 first \xff\xd9", "\xff\xd8 second\r\n \xff\xd9"};
    char stream[1024] = MJPEG_FIRST_PART_HEAD;
    char tail[MJPEG_PART_TAIL_MAX];
    const char *p = stream;

    for (unsigned int i = 0; i < 2; i++) {
        EXPECT_TRUE(mjpeg_format_part_tail(tail, sizeof(tail), i, 0) > 0);
        strcat(stream, frames[i]);
        strcat(stream, tail);
    }
    for (unsigned int i = 0; i < 2; i++) {
        const char *body = strstr(p, "\r\n\r\n");
        const char *next;

        EXPECT_TRUE(body != NULL);
        body += 4;
        next = strstr(body, "\r\n--" MJPEG_BOUNDARY "\r\n");
        EXPECT_TRUE(next != NULL);
        EXPECT_EQ((size_t)(next - body), strlen(frames[i]));
        EXPECT_TRUE(strncmp(body, frames[i], strlen(frames[i])) == 0);
        p = next + 2;
    }
}

static void test_tail_size_limits(void)
{
    char tail[MJPEG_PART_TAIL_MAX];
    char small[16];

    EXPECT_TRUE(mjpeg_format_part_tail(tail, sizeof(tail), UINT_MAX, UINT_MAX) > 0);
    EXPECT_EQ(mjpeg_format_part_tail(small, sizeof(small), 1, 0), -1);
}

int main(void)
{
    test_first_part_opens_with_a_delimiter();
    test_tail_closes_the_part_and_opens_the_next();
    test_a_stream_parses_back_into_its_frames();
    test_tail_size_limits();
    if (failures) {
        fprintf(stderr, "mjpeg_framing_test: %d failure(s)\n", failures);
        return 1;
    }
    printf("mjpeg_framing_test: OK\n");
    return 0;
}
