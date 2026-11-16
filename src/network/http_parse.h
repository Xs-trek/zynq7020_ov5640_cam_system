/* http_parse.h - the part of an HTTP/1.1 request the server looks at.
 *
 * headers points at the request line, headers_end at the "\r\n\r\n" that ends the header block.
 * Pure functions, no I/O.
 */
#ifndef HTTP_PARSE_H
#define HTTP_PARSE_H

#include <stddef.h>

#ifdef __cplusplus
extern "C" {
#endif

/* Value of the named header, with surrounding blanks removed, and its length in *len.  Header names
 * are case-insensitive.  NULL when the request does not have it. */
const char *http_header_value(const char *headers, const char *headers_end, const char *name,
                              size_t *len);

/* Value of Content-Length: 0 without the header, -1 when it is malformed or above max. */
int http_content_length(const char *headers, const char *headers_end, int max);

/* buf holds recv_len bytes, NUL-terminated.  1 when the whole request has arrived, 0 when more is
 * needed, -1 when it can never be valid. */
int http_request_complete(const char *buf, int recv_len, int max_body);

#ifdef __cplusplus
}
#endif

#endif /* HTTP_PARSE_H */
