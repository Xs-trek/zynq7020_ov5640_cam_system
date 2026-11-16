/* net_server_test.c - the event loop against real sockets.
 *
 * net_server.c runs on a thread with its neighbours stubbed; the test is its clients.  The cases are
 * the ones the send path cannot show: a client that went away while backlogged, one that connected
 * and said nothing, and what the protocol leaves to the peer.
 */
#include "cam_system/app_state.h"
#include "cam_system/config.h"
#include "capture/capture.h"
#include "detect/detector.h"
#include "media/jpeg_worker.h"
#include "network/cmd_handler.h"
#include "network/net_server.h"

#include <arpa/inet.h>
#include <errno.h>
#include <netinet/in.h>
#include <pthread.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/socket.h>
#include <sys/time.h>
#include <time.h>
#include <unistd.h>

#define JPEG_BYTES (80 * 1024)

app_state_t g_state;

static volatile int g_demand;

void jpeg_worker_set_demand(int demand) { __atomic_store_n(&g_demand, demand, __ATOMIC_RELAXED); }
float capture_get_fps(double now) { (void)now; return 30.0f; }
unsigned long capture_get_isp_slips(void) { return 0; }
unsigned long capture_get_restarts(void) { return 0; }
void capture_get_health(capture_health_t *health) { memset(health, 0, sizeof(*health)); }
int capture_stalled(double now) { (void)now; return 0; }
void capture_get_stream_state(double now, unsigned int *run_sec, int *stopped_by_limit)
{
    (void)now;
    *run_sec = 0;
    *stopped_by_limit = 0;
}
const char *detector_health_name(detector_health_t health) { (void)health; return "running"; }

void detector_get_status(detector_status_t *status)
{
    memset(status, 0, sizeof(*status));
}

void cmd_handle_api(const char *request, const char *body, char *response, size_t resp_size)
{
    (void)request;
    (void)body;
    snprintf(response, resp_size, "HTTP/1.1 200 OK\r\nContent-Length: 0\r\n\r\n");
}

static int failures;

static void expect_true(int cond, const char *name, int line)
{
    if (!cond) {
        fprintf(stderr, "FAIL:%d: %s\n", line, name);
        failures++;
    }
}

#define EXPECT_TRUE(cond) expect_true((cond), #cond, __LINE__)

static void sleep_ms(int ms)
{
    struct timespec ts = { ms / 1000, (long)(ms % 1000) * 1000000L };

    nanosleep(&ts, NULL);
}

static int connect_board(int rcvbuf)
{
    struct sockaddr_in addr;
    struct timeval tv = { 2, 0 };
    int fd = socket(AF_INET, SOCK_STREAM, 0);

    if (fd < 0) {
        perror("socket");
        exit(1);
    }
    if (rcvbuf > 0)
        setsockopt(fd, SOL_SOCKET, SO_RCVBUF, &rcvbuf, sizeof(rcvbuf));
    setsockopt(fd, SOL_SOCKET, SO_RCVTIMEO, &tv, sizeof(tv));
    memset(&addr, 0, sizeof(addr));
    addr.sin_family = AF_INET;
    addr.sin_port = htons(HTTP_PORT);
    addr.sin_addr.s_addr = htonl(INADDR_LOOPBACK);
    for (int i = 0; i < 200; i++) {
        if (connect(fd, (struct sockaddr *)&addr, sizeof(addr)) == 0)
            return fd;
        sleep_ms(10);
    }
    perror("connect");
    exit(1);
}

/* Reads until the server closes, or for as long as the receive timeout allows. */
static int read_reply(int fd, char *buf, size_t size)
{
    size_t got = 0;

    while (got + 1 < size) {
        ssize_t n = recv(fd, buf + got, size - 1 - got, 0);

        if (n <= 0)
            break;
        got += (size_t)n;
    }
    buf[got] = '\0';
    return (int)got;
}

static void send_text(int fd, const char *text)
{
    if (send(fd, text, strlen(text), MSG_NOSIGNAL) < 0) {
        perror("send");
        exit(1);
    }
}

static void test_status_is_complete_and_closes(void)
{
    char reply[4096];
    int fd = connect_board(0);
    const char *body;
    int length = -1;

    send_text(fd, "GET /api/status HTTP/1.1\r\nHost: board\r\n\r\n");
    (void)read_reply(fd, reply, sizeof(reply));
    close(fd);

    EXPECT_TRUE(strncmp(reply, "HTTP/1.1 200 OK\r\n", 17) == 0);
    EXPECT_TRUE(strstr(reply, "Connection: close\r\n") != NULL);
    body = strstr(reply, "\r\n\r\n");
    EXPECT_TRUE(body != NULL);
    EXPECT_TRUE(sscanf(strstr(reply, "Content-Length:"), "Content-Length: %d", &length) == 1);
    EXPECT_TRUE(body && (int)strlen(body + 4) == length);
}

static void test_websocket_header_case(void)
{
    char reply[2048];
    int fd = connect_board(0);
    struct timeval tv = { 1, 200000 };

    setsockopt(fd, SOL_SOCKET, SO_RCVTIMEO, &tv, sizeof(tv));
    send_text(fd, "GET /ws HTTP/1.1\r\nhost: board\r\nupgrade: websocket\r\nconnection: upgrade\r\n"
                  "sec-websocket-key: dGhlIHNhbXBsZSBub25jZQ==\r\nsec-websocket-version: 13\r\n\r\n");
    (void)read_reply(fd, reply, sizeof(reply));
    close(fd);

    EXPECT_TRUE(strncmp(reply, "HTTP/1.1 101 ", 13) == 0);
    /* The server speaks first: the page learns the board's state without asking. */
    EXPECT_TRUE(strstr(reply, "{\"status\":{\"cam_enabled\":") != NULL);
    /* RFC 6455, section 1.3: this key gives this accept value. */
    EXPECT_TRUE(strstr(reply, "s3pPLMBiTxaQ9kYGzzhZRbK+xOo=") != NULL);
}

static void test_websocket_without_key(void)
{
    char reply[256];
    int fd = connect_board(0);

    send_text(fd, "GET /ws HTTP/1.1\r\nHost: board\r\n\r\n");
    (void)read_reply(fd, reply, sizeof(reply));
    close(fd);
    EXPECT_TRUE(strncmp(reply, "HTTP/1.1 400 ", 13) == 0);
    EXPECT_TRUE(strstr(reply, "Connection: close\r\n") != NULL);
}

static void test_silent_client_is_dropped(void)
{
    char reply[16];
    int fd = connect_board(0);

    EXPECT_TRUE(read_reply(fd, reply, sizeof(reply)) == 0);
    close(fd);
}

static void publish_frames(int count)
{
    for (int i = 0; i < count; i++) {
        pthread_mutex_lock(&g_state.jpeg_mutex);
        g_state.jpeg_seq++;
        pthread_mutex_unlock(&g_state.jpeg_mutex);
        sleep_ms(6);
    }
}

static int wait_for_demand(int want)
{
    for (int i = 0; i < 200; i++) {
        if (__atomic_load_n(&g_demand, __ATOMIC_RELAXED) == want)
            return 1;
        sleep_ms(5);
    }
    return 0;
}

/* The peer stops reading, the server starts dropping frames without touching the socket, and the
 * connection then dies.  SIOCOUTQ stays high, so only reading the socket shows it. */
static void test_backlogged_stream_client_that_died_is_removed(void)
{
    struct linger reset = { 1, 0 };
    int fd = connect_board(4096);

    send_text(fd, "GET /stream HTTP/1.1\r\nHost: board\r\n\r\n");
    EXPECT_TRUE(wait_for_demand(1));
    publish_frames(40);
    EXPECT_TRUE(__atomic_load_n(&g_demand, __ATOMIC_RELAXED) == 1);

    setsockopt(fd, SOL_SOCKET, SO_LINGER, &reset, sizeof(reset));
    close(fd);
    EXPECT_TRUE(wait_for_demand(0));
}

static void test_stream_client_can_leave_quietly(void)
{
    int fd = connect_board(0);

    send_text(fd, "GET /stream HTTP/1.1\r\nHost: board\r\n\r\n");
    EXPECT_TRUE(wait_for_demand(1));
    close(fd);
    EXPECT_TRUE(wait_for_demand(0));
}

static void *server_thread(void *arg)
{
    (void)arg;
    (void)net_server_run(-1);
    return NULL;
}

int main(void)
{
    pthread_t thread;

    pthread_mutex_init(&g_state.jpeg_mutex, NULL);
    pthread_mutex_init(&g_state.det_mutex, NULL);
    pthread_mutex_init(&g_state.pred_mutex, NULL);
    g_state.jpeg_buf = calloc(1, JPEG_BYTES);
    g_state.jpeg_size = JPEG_BYTES;
    g_state.jpeg_seq = 1;
    pthread_create(&thread, NULL, server_thread, NULL);

    test_status_is_complete_and_closes();
    test_websocket_header_case();
    test_websocket_without_key();
    test_silent_client_is_dropped();
    test_stream_client_can_leave_quietly();
    test_backlogged_stream_client_that_died_is_removed();

    net_server_stop();
    pthread_join(thread, NULL);
    free(g_state.jpeg_buf);
    if (failures) {
        fprintf(stderr, "net_server_test: %d failure(s)\n", failures);
        return 1;
    }
    puts("net_server_test: OK");
    return 0;
}
