#include "network/http_parse.h"

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

static int value_is(const char *req, const char *name, const char *want)
{
    size_t len = 0;
    const char *value = http_header_value(req, strstr(req, "\r\n\r\n"), name, &len);

    return value && len == strlen(want) && strncmp(value, want, len) == 0;
}

static void test_header_names_ignore_case(void)
{
    static const char req[] =
        "GET /ws HTTP/1.1\r\nHost: board\r\nsec-websocket-key: abc==\r\n"
        "SEC-WEBSOCKET-VERSION:13\r\nX-Pad:  \t spaced \t \r\n\r\n";

    EXPECT_TRUE(value_is(req, "Sec-WebSocket-Key", "abc=="));
    EXPECT_TRUE(value_is(req, "Sec-WebSocket-Version", "13"));
    EXPECT_TRUE(value_is(req, "x-pad", "spaced"));
}

static void test_missing_and_partial_names(void)
{
    static const char req[] = "GET /Sec-WebSocket-Key: no HTTP/1.1\r\nX-Sec-WebSocket-Key: no\r\n\r\nKey: body\r\n";
    size_t len = 0;

    EXPECT_TRUE(http_header_value(req, strstr(req, "\r\n\r\n"), "Sec-WebSocket-Key", &len) == NULL);
    EXPECT_TRUE(http_header_value(req, strstr(req, "\r\n\r\n"), "Key", &len) == NULL);
}

static void test_content_length(void)
{
    static const char some[] = "POST /api/x HTTP/1.1\r\ncontent-length: 12\r\n\r\n";
    static const char none[] = "POST /api/x HTTP/1.1\r\nHost: b\r\n\r\n";
    static const char bad[] = "POST /api/x HTTP/1.1\r\nContent-Length: 12x\r\n\r\n";
    static const char negative[] = "POST /api/x HTTP/1.1\r\nContent-Length: -1\r\n\r\n";

    EXPECT_EQ(http_content_length(some, strstr(some, "\r\n\r\n"), 100), 12);
    EXPECT_EQ(http_content_length(none, strstr(none, "\r\n\r\n"), 100), 0);
    EXPECT_EQ(http_content_length(bad, strstr(bad, "\r\n\r\n"), 100), -1);
    EXPECT_EQ(http_content_length(negative, strstr(negative, "\r\n\r\n"), 100), -1);
    EXPECT_EQ(http_content_length(some, strstr(some, "\r\n\r\n"), 11), -1);
}

static void test_request_complete(void)
{
    static const char post[] = "POST /api/x HTTP/1.1\r\nContent-Length: 2\r\n\r\n{}";
    static const char get[] = "GET /stream HTTP/1.1\r\n\r\n";
    char partial[64];

    EXPECT_EQ(http_request_complete(get, (int)strlen(get), 100), 1);
    EXPECT_EQ(http_request_complete(post, (int)strlen(post), 100), 1);
    EXPECT_EQ(http_request_complete(post, (int)strlen(post), 1), -1);

    snprintf(partial, sizeof(partial), "%.*s", (int)strlen(post) - 1, post);
    EXPECT_EQ(http_request_complete(partial, (int)strlen(partial), 100), 0);
    EXPECT_EQ(http_request_complete("GET /stream HTTP/1.1\r\n", 22, 100), 0);
}

int main(void)
{
    test_header_names_ignore_case();
    test_missing_and_partial_names();
    test_content_length();
    test_request_complete();
    if (failures) {
        fprintf(stderr, "http_parse_test: %d failure(s)\n", failures);
        return 1;
    }
    puts("http_parse_test: OK");
    return 0;
}
