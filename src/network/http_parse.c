/* http_parse.c - see http_parse.h */
#include "network/http_parse.h"

#include <errno.h>
#include <stdlib.h>
#include <string.h>
#include <strings.h>

const char *http_header_value(const char *headers, const char *headers_end, const char *name,
                              size_t *len)
{
    const size_t name_len = strlen(name);
    const char *line = strstr(headers, "\r\n");

    /* The first line is the request line. */
    while (line && line < headers_end) {
        const char *next;

        line += 2;
        next = strstr(line, "\r\n");
        if (!next || next > headers_end)
            next = headers_end;
        if ((size_t)(next - line) > name_len && line[name_len] == ':' &&
            strncasecmp(line, name, name_len) == 0) {
            const char *value = line + name_len + 1;
            const char *end = next;

            while (value < end && (*value == ' ' || *value == '\t'))
                value++;
            while (end > value && (end[-1] == ' ' || end[-1] == '\t'))
                end--;
            *len = (size_t)(end - value);
            return value;
        }
        line = next;
    }
    return NULL;
}

int http_content_length(const char *headers, const char *headers_end, int max)
{
    size_t len = 0;
    const char *value = http_header_value(headers, headers_end, "Content-Length", &len);
    char *end;
    long parsed;

    if (!value)
        return 0;
    errno = 0;
    parsed = strtol(value, &end, 10);
    if (errno || end != value + len || len == 0 || parsed < 0 || parsed > max)
        return -1;
    return (int)parsed;
}

int http_request_complete(const char *buf, int recv_len, int max_body)
{
    const char *headers_end = strstr(buf, "\r\n\r\n");
    int header_len;
    int body_len;

    if (!headers_end)
        return 0;
    if (strncmp(buf, "POST ", 5) != 0)
        return 1;

    body_len = http_content_length(buf, headers_end, max_body);
    if (body_len < 0)
        return -1;
    header_len = (int)(headers_end + 4 - buf);
    return recv_len >= header_len + body_len;
}
