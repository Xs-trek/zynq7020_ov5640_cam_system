/* net_server.c — HTTP MJPEG 推流 + REST API + WebSocket
 *
 * 单线程 select() 事件循环
 * - GET /stream      → MJPEG multipart 推流
 * - POST /api/xxx    → JSON 命令处理
 * - GET /ws          → WebSocket 升级 → 检测结果推送
 *
 * 依赖: cJSON (third_party/)
 */
#define LOG_TAG "net"

#include "network/net_server.h"
#include "network/cmd_handler.h"
#include "network/http_parse.h"
#include "network/mjpeg_framing.h"
#include "network/status_json.h"
#include "network/ws_protocol.h"
#include "cam_system/app_state.h"
#include "cam_system/config.h"
#include "cam_system/version.h"
#include "capture/capture.h"
#include "common/fps_meter.h"
#include "common/log.h"
#include "common/time_util.h"
#include "detect/detector.h"
#include "media/jpeg_worker.h"

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sysexits.h>
#include <unistd.h>
#include <errno.h>
#include <fcntl.h>
#include <strings.h>
#include <sys/ioctl.h>
#include <sys/socket.h>
#include <sys/select.h>
#include <sys/signalfd.h>
#include <linux/sockios.h>
#include <netinet/tcp.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <openssl/sha.h>

#ifdef CAM_SYSTEM_ENABLE_CPU_DETECTORS
/* ── COCO 类名 ── */
static const char *coco_names_tbl[80] = {
    "person","bicycle","car","motorcycle","airplane","bus","train","truck","boat",
    "traffic light","fire hydrant","stop sign","parking meter","bench","bird","cat",
    "dog","horse","sheep","cow","elephant","bear","zebra","giraffe","backpack",
    "umbrella","handbag","tie","suitcase","frisbee","skis","snowboard","sports ball",
    "kite","baseball bat","baseball glove","skateboard","surfboard","tennis racket",
    "bottle","wine glass","cup","fork","knife","spoon","bowl","banana","apple",
    "sandwich","orange","broccoli","carrot","hot dog","pizza","donut","cake","chair",
    "couch","potted plant","bed","dining table","toilet","tv","laptop","mouse",
    "remote","keyboard","cell phone","microwave","oven","toaster","sink",
    "refrigerator","book","clock","vase","scissors","teddy bear","hair drier",
    "toothbrush"
};
#endif

static const char *coco_names_lookup(int id) {
    if (id == DETECTION_CLASS_FACE) return "face";
#ifdef CAM_SYSTEM_ENABLE_CPU_DETECTORS
    if (id >= 0 && id < 80) return coco_names_tbl[id];
#endif
    return "unknown";
}

/* 简易 Base64 编码 */
static const char b64_table[] =
    "ABCDEFGHIJKLMNOPQRSTUVWXYZabcdefghijklmnopqrstuvwxyz0123456789+/";

static int base64_encode(const unsigned char *in, int len, char *out)
{
    int i, j = 0;
    for (i = 0; i < len - 2; i += 3) {
        out[j++] = b64_table[(in[i] >> 2) & 0x3F];
        out[j++] = b64_table[((in[i]&0x3)<<4) | ((in[i+1]>>4)&0xF)];
        out[j++] = b64_table[((in[i+1]&0xF)<<2) | ((in[i+2]>>6)&0x3)];
        out[j++] = b64_table[in[i+2] & 0x3F];
    }
    if (i < len) {
        out[j++] = b64_table[(in[i] >> 2) & 0x3F];
        if (i == len - 1) {
            out[j++] = b64_table[(in[i] & 0x3) << 4];
            out[j++] = '=';
        } else {
            out[j++] = b64_table[((in[i]&0x3)<<4) | ((in[i+1]>>4)&0xF)];
            out[j++] = b64_table[(in[i+1] & 0xF) << 2];
        }
        out[j++] = '=';
    }
    out[j] = '\0';
    return j;
}

/* ── 客户端类型 ── */
typedef enum {
    CLIENT_NEW = 0,
    CLIENT_MJPEG,
    CLIENT_WS,
    CLIENT_API
} client_type_t;

typedef struct {
    int fd;
    client_type_t type;
    uint32_t last_jpeg_seq;    /* MJPEG: 上次发送的帧序号 */
    char recv_buf[4096];
    int recv_len;
    int ws_handshake_done;
    uint8_t *mjpeg_buf;        /* 专属于这个客户端的推流缓冲区 */
    size_t mjpeg_buf_cap;      /* 缓冲区最大容量 */
    uint32_t mjpeg_drop_count; /* transport-backpressure dropped frames */
    double accepted_sec;       /* CLIENT_NEW: a request must arrive in time */
} client_t;

static int listen_fd = -1;
static client_t clients[MAX_HTTP_CLIENTS + MAX_WS_CLIENTS];
static int client_count = 0;
static int server_running = 0;

static int send_all_limited(int fd, const void *buf, size_t len,
                            int wait_budget_ms);

static int set_nonblock(int fd)
{
    int flags = fcntl(fd, F_GETFL, 0);

    if (flags < 0 || fcntl(fd, F_SETFL, flags | O_NONBLOCK) < 0)
        return -1;
    return 0;
}

static fps_meter_t g_stream_fps;
static unsigned int g_stream_last_seq;

static void client_remove(int idx)
{
    if (idx < 0 || idx >= client_count) return;
    /* 客户端断开时，回收专属缓冲区 */
    if (clients[idx].mjpeg_buf) {
        free(clients[idx].mjpeg_buf);
        clients[idx].mjpeg_buf = NULL;
    }
    close(clients[idx].fd);
    clients[idx] = clients[client_count - 1];
    client_count--;
}

static int send_all(int fd, const void *buf, size_t len)
{
    return send_all_limited(fd, buf, len, HTTP_SEND_WAIT_BUDGET_MS);
}

static int send_all_limited(int fd, const void *buf, size_t len, int wait_budget_ms)
{
    const char *p = (const char *)buf;
    size_t sent = 0;
    int waited_ms = 0;

    while (sent < len) {
        ssize_t n = send(fd, p + sent, len - sent, MSG_NOSIGNAL);
        if (n < 0) {
            if (errno == EAGAIN || errno == EWOULDBLOCK) {
                fd_set wr_set;
                struct timeval tv = { .tv_sec = 0, .tv_usec = 5000 };

                if (waited_ms >= wait_budget_ms)
                    return -1;

                FD_ZERO(&wr_set);
                FD_SET(fd, &wr_set);
                if (select(fd + 1, NULL, &wr_set, NULL, &tv) < 0 &&
                    errno != EINTR)
                    return -1;
                waited_ms += 5;
                continue;
            }
            return -1;
        }
        if (n == 0) return -1;
        sent += n;
    }
    return 0;
}

/* A phone that leaves the network sends nothing, not even a FIN.  Keepalive finds it on an idle
 * connection, the user timeout on one with data in flight. */
static void configure_client_socket(int fd)
{
    static const int enabled = 1;
    static const int idle_sec = CLIENT_KEEPALIVE_IDLE_SEC;
    static const int interval_sec = CLIENT_KEEPALIVE_INTERVAL_SEC;
    static const int probes = CLIENT_KEEPALIVE_PROBES;
    static const unsigned int user_timeout_ms = CLIENT_TCP_USER_TIMEOUT_MS;

    if (setsockopt(fd, SOL_SOCKET, SO_KEEPALIVE, &enabled, sizeof(enabled)) < 0 ||
        setsockopt(fd, IPPROTO_TCP, TCP_KEEPIDLE, &idle_sec, sizeof(idle_sec)) < 0 ||
        setsockopt(fd, IPPROTO_TCP, TCP_KEEPINTVL, &interval_sec, sizeof(interval_sec)) < 0 ||
        setsockopt(fd, IPPROTO_TCP, TCP_KEEPCNT, &probes, sizeof(probes)) < 0 ||
        setsockopt(fd, IPPROTO_TCP, TCP_USER_TIMEOUT, &user_timeout_ms, sizeof(user_timeout_ms)) < 0)
        LOG_W("TCP keepalive setup failed: %s", strerror(errno));
}

static void configure_mjpeg_socket(int fd)
{
    int opt = 1;
    int sndbuf = MJPEG_SOCKET_SNDBUF_BYTES;

    if (setsockopt(fd, IPPROTO_TCP, TCP_NODELAY, &opt, sizeof(opt)) < 0)
        LOG_W("TCP_NODELAY failed: %s", strerror(errno));
    if (setsockopt(fd, SOL_SOCKET, SO_SNDBUF, &sndbuf, sizeof(sndbuf)) < 0)
        LOG_W("SO_SNDBUF failed: %s", strerror(errno));
}

static int mjpeg_socket_backlogged(int fd)
{
    int outq = 0;

    if (ioctl(fd, SIOCOUTQ, &outq) < 0)
        return 0;

    return outq > MJPEG_OUTQ_DROP_THRESHOLD_BYTES;
}

/* ── MJPEG 帧发送 ── */
static int send_mjpeg_frame(client_t *c)
{
    /* 该客户端未成功分配缓冲区，放弃帧发送 */
    if (!c->mjpeg_buf) return 0; 

    pthread_mutex_lock(&g_state.jpeg_mutex);
    if (!g_state.jpeg_buf || g_state.jpeg_size == 0 ||
        g_state.jpeg_seq == c->last_jpeg_seq) {
        pthread_mutex_unlock(&g_state.jpeg_mutex);
        return 0;  /* 没有新帧 */
    }

    if (mjpeg_socket_backlogged(c->fd)) {
        unsigned int skipped = g_state.jpeg_seq - c->last_jpeg_seq;
        if (skipped == 0)
            skipped = 1;
        c->mjpeg_drop_count += skipped;
        c->last_jpeg_seq = g_state.jpeg_seq;
        pthread_mutex_unlock(&g_state.jpeg_mutex);
        return 0;
    }

    /* 拷贝 JPEG 数据 */
    size_t sz = g_state.jpeg_size;
    if (sz > c->mjpeg_buf_cap) sz = c->mjpeg_buf_cap;
    memcpy(c->mjpeg_buf, g_state.jpeg_buf, sz);
    c->last_jpeg_seq = g_state.jpeg_seq;
    pthread_mutex_unlock(&g_state.jpeg_mutex);

    /* Rate of distinct frames sent, however many clients receive each. */
    if (c->last_jpeg_seq != g_stream_last_seq) {
        g_stream_last_seq = c->last_jpeg_seq;
        fps_meter_tick(&g_stream_fps, monotonic_sec());
    }

    char tail[MJPEG_PART_TAIL_MAX];
    int tail_len = mjpeg_format_part_tail(tail, sizeof(tail), c->last_jpeg_seq,
                                          c->mjpeg_drop_count);
    if (tail_len < 0)
        return -1;

    int ret = 0;
    if (send_all_limited(c->fd, c->mjpeg_buf, sz, MJPEG_SEND_WAIT_BUDGET_MS) < 0 ||
        send_all_limited(c->fd, tail, tail_len, MJPEG_SEND_WAIT_BUDGET_MS) < 0) {
        ret = -1;
    }
    return ret;
}

/* ── WebSocket 帧发送 ── */
static int ws_send_frame(int fd, uint8_t opcode, const void *payload, size_t len)
{
    uint8_t hdr[10];
    size_t hdr_len;

    if (cam_ws_build_server_header(opcode, len, hdr, &hdr_len) < 0)
        return -1;
    if (send_all_limited(fd, hdr, hdr_len, 50) < 0)
        return -1;
    if (len > 0 && send_all_limited(fd, payload, len, 50) < 0)
        return -1;
    return 0;
}

static int ws_send_text(int fd, const char *text, size_t len)
{
    return ws_send_frame(fd, CAM_WS_OPCODE_TEXT, text, len);
}

static int ws_send_pong(int fd, const uint8_t *payload, size_t len)
{
    return ws_send_frame(fd, CAM_WS_OPCODE_PONG, payload, len);
}

static int ws_send_close_status(int fd, uint16_t code)
{
    uint8_t payload[2] = {
        (uint8_t)(code >> 8),
        (uint8_t)(code & 0xFF)
    };

    return ws_send_frame(fd, CAM_WS_OPCODE_CLOSE, payload, sizeof(payload));
}

static int ws_handle_text_message(client_t *c, const cam_ws_frame_t *frame)
{
    const char *msg = (const char *)frame->payload;
    const char *ping = strstr(msg, "\"ping\"");
    const char *colon;
    char *endp;
    double ping_ts;
    char pong[128];
    int plen;

    if (!ping)
        return 0;

    colon = strchr(ping, ':');
    if (!colon)
        return 0;

    errno = 0;
    ping_ts = strtod(colon + 1, &endp);
    if (errno || endp == colon + 1)
        return 0;

    plen = snprintf(pong, sizeof(pong), "{\"pong\":%.0f}", ping_ts);
    if (plen < 0 || plen >= (int)sizeof(pong))
        return -1;
    return ws_send_text(c->fd, pong, (size_t)plen);
}

static int ws_process_recv_buffer(client_t *c)
{
    size_t offset = 0U;
    size_t len = (size_t)c->recv_len;

    while (offset < len) {
        cam_ws_frame_t frame;
        cam_ws_parse_result_t ret =
            cam_ws_parse_client_frame((const uint8_t *)c->recv_buf + offset,
                                      len - offset,
                                      &frame);

        if (ret == CAM_WS_PARSE_INCOMPLETE)
            break;
        if (ret == CAM_WS_PARSE_PAYLOAD_TOO_LARGE) {
            ws_send_close_status(c->fd, 1009U);
            return -1;
        }
        if (ret != CAM_WS_PARSE_OK) {
            ws_send_close_status(c->fd, 1002U);
            return -1;
        }

        if (frame.opcode == CAM_WS_OPCODE_TEXT) {
            if (ws_handle_text_message(c, &frame) < 0)
                return -1;
        } else if (frame.opcode == CAM_WS_OPCODE_PING) {
            if (ws_send_pong(c->fd, frame.payload, frame.payload_len) < 0)
                return -1;
        } else if (frame.opcode == CAM_WS_OPCODE_CLOSE) {
            ws_send_close_status(c->fd, 1000U);
            return -1;
        }

        offset += frame.frame_len;
    }

    if (offset > 0U) {
        size_t remaining = len - offset;

        if (remaining > 0U)
            memmove(c->recv_buf, c->recv_buf + offset, remaining);
        c->recv_len = (int)remaining;
        c->recv_buf[c->recv_len] = '\0';
    }

    if ((size_t)c->recv_len >= sizeof(c->recv_buf) - 1U)
        return -1;
    return 0;
}

/* ── WebSocket 握手 ── */
static const char *WS_MAGIC = "258EAFA5-E914-47DA-95CA-C5AB0DC85B11";

static int ws_handshake(client_t *c, const char *key)
{
    /* SHA1(key + magic) */
    char concat[256];
    snprintf(concat, sizeof(concat), "%s%s", key, WS_MAGIC);

    unsigned char sha1_hash[20];
    SHA1((unsigned char *)concat, strlen(concat), sha1_hash);

    char accept_key[64];
    base64_encode(sha1_hash, 20, accept_key);

    char response[512];
    int rlen = snprintf(response, sizeof(response),
        "HTTP/1.1 101 Switching Protocols\r\n"
        "Upgrade: websocket\r\n"
        "Connection: Upgrade\r\n"
        "Sec-WebSocket-Accept: %s\r\n\r\n",
        accept_key);

    if (rlen < 0 || rlen >= (int)sizeof(response))
        return -1;
    if (send_all(c->fd, response, rlen) < 0) return -1;
    c->ws_handshake_done = 1;
    c->type = CLIENT_WS;
    return 0;
}

static void collect_status(status_fields_t *status)
{
    detector_status_t detector;
    capture_health_t health;
    const double now = monotonic_sec();

    detector_get_status(&detector);
    status->cam_enabled = app_state_load_int(&g_state.cam_enabled);
    status->detector_enabled = app_state_load_int(&g_state.detector_enabled);
    status->tracking = app_state_load_int(&g_state.tracking_active);
    status->target_id = app_state_load_int(&g_state.tracking_target_id);
    status->fps_capture = capture_get_fps(now);
    status->fps_inference = detector.fps;
    status->fps_stream = fps_meter_read(&g_stream_fps, now);
    status->uptime_sec = (unsigned int)(now - g_state.start_time_sec);
    status->status_seq = app_state_load_u32(&g_state.status_seq);
    status->detector_health = detector_health_name(detector.health);
    status->detector_failures = detector.failures;
    status->detector_last_error = detector.last_error;
    status->isp_slips = capture_get_isp_slips();
    status->capture_restarts = capture_get_restarts();
    capture_get_health(&health);
    status->dvp_frames = health.dvp_frames;
    status->dvp_bad_frames = health.dvp_bad_frames;
    status->i2c_write_failures = health.i2c_write_failures;
    status->v4l2_error_frames = health.v4l2_error_frames;
    status->version = CAM_SYSTEM_VERSION;
    status->git_commit = CAM_SYSTEM_GIT_COMMIT;
    status->driver_uapi = detector.identity.driver_uapi;
    status->hw_revision = detector.identity.hw_revision;
    /* The identity lives in the status snapshot copied above. */
    static char weights_sha256[sizeof(detector.identity.weight_sha256)];
    memcpy(weights_sha256, detector.identity.weight_sha256, sizeof(weights_sha256));
    status->weights_sha256 = weights_sha256;
    status->detector_input = detector.direct_input ? "direct" : "copy";
    {
        unsigned int stream_sec;
        int stopped_by_limit;

        capture_get_stream_state(now, &stream_sec, &stopped_by_limit);
        status->cam_off_reason = stopped_by_limit ? "stream_limit" : "none";
        status->cam_stream_sec = stream_sec;
    }
}

/* An answer the client cannot use, or one with nothing in it.  The connection ends here. */
static void http_reply_empty(client_t *c, const char *status_line)
{
    char resp[128];
    int n = snprintf(resp, sizeof(resp),
                     "HTTP/1.1 %s\r\nContent-Length: 0\r\nConnection: close\r\n\r\n", status_line);

    if (n > 0 && n < (int)sizeof(resp))
        (void)send_all(c->fd, resp, (size_t)n);
    c->type = CLIENT_API;
}

/* ── HTTP 请求解析 ── */
static void handle_http_request(client_t *c)
{
    char *req = c->recv_buf;

    if (strncmp(req, "GET /stream", 11) == 0) {
        /* MJPEG 推流 */
        const char *resp =
            "HTTP/1.1 200 OK\r\n"
            "Content-Type: multipart/x-mixed-replace;boundary=" MJPEG_BOUNDARY "\r\n"
            "Cache-Control: no-store, no-cache, must-revalidate, max-age=0\r\n"
            "Pragma: no-cache\r\n"
            "Connection: keep-alive\r\n"
            "Access-Control-Allow-Origin: *\r\n\r\n"
            MJPEG_FIRST_PART_HEAD;
        c->mjpeg_buf_cap = FRAME_SIZE;
        c->mjpeg_buf = (uint8_t *)malloc(c->mjpeg_buf_cap);
        if (!c->mjpeg_buf) {
            LOG_E("mjpeg_buf malloc failed");
            http_reply_empty(c, "503 Service Unavailable");
            return;
        }

        configure_mjpeg_socket(c->fd);
        if (send_all(c->fd, resp, strlen(resp)) == 0) {
            c->type = CLIENT_MJPEG;
            /* Encoding stops without clients, so the buffered JPEG may be old: start with the
             * next frame encoded after this client connected. */
            pthread_mutex_lock(&g_state.jpeg_mutex);
            c->last_jpeg_seq = g_state.jpeg_seq;
            pthread_mutex_unlock(&g_state.jpeg_mutex);
            c->mjpeg_drop_count = 0;
        } else {
            c->type = CLIENT_API;
        }

    } else if (strncmp(req, "GET /api/status", 15) == 0) {
        /* 设备状态 — APP 连接测试用 */
        status_fields_t status;
        char body[STATUS_BODY_MAX];
        char resp[STATUS_BODY_MAX + 256];
        int body_len;
        int resp_len;

        collect_status(&status);
        body_len = status_json_body(body, sizeof(body), &status);
        if (body_len < 0) {
            http_reply_empty(c, "500 Internal Server Error");
            return;
        }

        resp_len = snprintf(resp, sizeof(resp),
            "HTTP/1.1 200 OK\r\n"
            "Content-Type: application/json\r\n"
            "Access-Control-Allow-Origin: *\r\n"
            "Connection: close\r\n"
            "Content-Length: %d\r\n\r\n%s",
            body_len, body);
        if (resp_len > 0 && resp_len < (int)sizeof(resp))
            (void)send_all(c->fd, resp, (size_t)resp_len);
        c->type = CLIENT_API;

    } else if (strncmp(req, "GET /ws", 7) == 0) {
        /* WebSocket 升级 */
        const char *headers_end = strstr(req, "\r\n\r\n");
        size_t key_len = 0;
        const char *key_value = http_header_value(req, headers_end, "Sec-WebSocket-Key", &key_len);
        char key[128];
        int upgraded = 0;

        if (key_value && key_len < sizeof(key)) {
            memcpy(key, key_value, key_len);
            key[key_len] = '\0';
            upgraded = ws_handshake(c, key) == 0;
        }
        if (!upgraded)
            http_reply_empty(c, "400 Bad Request");

    } else if (strncmp(req, "POST /api/", 10) == 0) {
        /* REST API */
        char *body = strstr(req, "\r\n\r\n");
        if (body) {
            body += 4;
            char response[1024];
            cmd_handle_api(req, body, response, sizeof(response));
            send_all(c->fd, response, strlen(response));
        }
        /* API 请求处理完关闭连接 */
        c->type = CLIENT_API;

    } else if (strncmp(req, "OPTIONS ", 8) == 0) {
        /* CORS preflight */
        const char *resp =
            "HTTP/1.1 204 No Content\r\n"
            "Access-Control-Allow-Origin: *\r\n"
            "Access-Control-Allow-Methods: GET, POST, OPTIONS\r\n"
            "Access-Control-Allow-Headers: Content-Type\r\n"
            "Access-Control-Max-Age: 86400\r\n"
            "Connection: close\r\n\r\n";
        send_all(c->fd, resp, strlen(resp));
        c->type = CLIENT_API;

    } else {
        http_reply_empty(c, "404 Not Found");
    }
}

static void accept_client(double now)
{
    int cfd = accept(listen_fd, NULL, NULL);

    if (cfd < 0)
        return;
    if (client_count >= MAX_HTTP_CLIENTS + MAX_WS_CLIENTS) {
        LOG_W_RL(5.0, 3, "connection refused: all %d client slots in use", client_count);
        close(cfd);
        return;
    }
    if (set_nonblock(cfd) < 0) {
        LOG_W("O_NONBLOCK failed: %s", strerror(errno));
        close(cfd);
        return;
    }
    configure_client_socket(cfd);

    client_t *nc = &clients[client_count++];

    memset(nc, 0, sizeof(*nc));
    nc->fd = cfd;
    nc->type = CLIENT_NEW;
    nc->accepted_sec = now;
}

/* Reads what the client sent and acts on it.  Returns -1 when the connection is finished. */
static int client_receive(client_t *c)
{
    int space = (int)sizeof(c->recv_buf) - c->recv_len - 1;
    ssize_t n;

    if (space <= 0)
        return -1;

    n = recv(c->fd, c->recv_buf + c->recv_len, (size_t)space, 0);
    if (n == 0 || (n < 0 && errno != EAGAIN && errno != EINTR))
        return -1;
    if (n < 0)
        return 0;
    if (c->type == CLIENT_MJPEG) {
        c->recv_len = 0;    /* nothing it could say matters */
        return 0;
    }

    c->recv_len += (int)n;
    c->recv_buf[c->recv_len] = '\0';

    if (c->type == CLIENT_NEW) {
        int complete = http_request_complete(c->recv_buf, c->recv_len,
                                              (int)sizeof(c->recv_buf) - 1);

        if (complete < 0)
            return -1;
        if (complete > 0) {
            handle_http_request(c);
            c->recv_len = 0;
            if (c->type == CLIENT_API)
                return -1;  /* 请求处理完关闭连接 */
        }
    }
    if (c->type == CLIENT_WS && c->ws_handshake_done)
        return ws_process_recv_buffer(c);
    return 0;
}

/* ── 主事件循环 ── */
/* Returns 1 when a shutdown signal was read from signal_fd. */
static int read_shutdown_signal(int signal_fd)
{
    struct signalfd_siginfo info;
    ssize_t n = read(signal_fd, &info, sizeof(info));

    if (n != (ssize_t)sizeof(info))
        return 0;
    LOG_I("Signal %u (%s) requested shutdown", info.ssi_signo, strsignal((int)info.ssi_signo));
    return 1;
}

int net_server_run(int signal_fd)
{
    listen_fd = socket(AF_INET, SOCK_STREAM, 0);
    if (listen_fd < 0) { LOG_E("socket: %s", strerror(errno)); return -1; }

    int opt = 1;
    setsockopt(listen_fd, SOL_SOCKET, SO_REUSEADDR, &opt, sizeof(opt));

    struct sockaddr_in addr = {
        .sin_family = AF_INET,
        .sin_port = htons(HTTP_PORT),
        .sin_addr.s_addr = INADDR_ANY,
    };

    if (bind(listen_fd, (struct sockaddr *)&addr, sizeof(addr)) < 0) {
        LOG_E("bind: %s", strerror(errno)); close(listen_fd); return -1;
    }
    if (listen(listen_fd, 8) < 0) {
        LOG_E("listen: %s", strerror(errno)); close(listen_fd); return -1;
    }

    if (set_nonblock(listen_fd) < 0) {
        LOG_E("O_NONBLOCK: %s", strerror(errno)); close(listen_fd); return -1;
    }
    app_state_store_int(&server_running, 1);
    LOG_I("HTTP server listening on port %d", HTTP_PORT);

    int stream_demand = 0;

    while (app_state_load_int(&server_running) &&
           !app_state_load_int(&g_state.quit)) {
        fd_set rd_set;
        FD_ZERO(&rd_set);
        FD_SET(listen_fd, &rd_set);
        int max_fd = listen_fd;
        if (signal_fd >= 0) {
            FD_SET(signal_fd, &rd_set);
            if (signal_fd > max_fd) max_fd = signal_fd;
        }
        int mjpeg_clients = 0;

        /* An MJPEG client sends nothing after its request, so readable means it has gone.  The
         * send path cannot tell: a socket that was backlogged when the connection died keeps its
         * SIOCOUTQ count, and every frame is then dropped without touching the socket. */
        for (int i = 0; i < client_count; i++) {
            FD_SET(clients[i].fd, &rd_set);
            if (clients[i].type == CLIENT_MJPEG)
                mjpeg_clients++;
            if (clients[i].fd > max_fd) max_fd = clients[i].fd;
        }
        if (mjpeg_clients != stream_demand) {
            stream_demand = mjpeg_clients;
            jpeg_worker_set_demand(stream_demand);
            LOG_I("MJPEG clients: %d", stream_demand);
        }

        /* 33 ms 固定等待再叠加 send/copy 开销只能得到约 29.1 fps。
         * 有 MJPEG 客户端时缩短轮询等待；jpeg_seq 去重保证不会重复发帧。 */
        struct timeval tv = {
            .tv_sec = 0,
            .tv_usec = mjpeg_clients > 0 ? 5000 : 33000
        };
        int nready = select(max_fd + 1, &rd_set, NULL, NULL, &tv);
        const double now = monotonic_sec();

        if (nready < 0)
            FD_ZERO(&rd_set);   /* its contents are undefined after an error */

        if (capture_stalled(now)) {
            LOG_E("capture thread is stuck; exiting for a restart");
            cam_log_close();
            _exit(EX_SOFTWARE);
        }

        if (nready > 0 && signal_fd >= 0 && FD_ISSET(signal_fd, &rd_set) &&
            read_shutdown_signal(signal_fd)) {
            app_state_store_int(&g_state.quit, 1);
            app_state_store_int(&server_running, 0);
            break;
        }

        /* 新连接 */
        if (nready > 0 && FD_ISSET(listen_fd, &rd_set))
            accept_client(now);

        /* 广播检测结果或状态变化给 WS 客户端。先于 MJPEG：慢客户端的发送会占住这个循环。 */
        static unsigned int last_det_seq = 0;
        static unsigned int last_status_seq = 0;
        static double last_broadcast_sec = 0.0;
        unsigned int det_seq = app_state_load_u32(&g_state.det_seq);
        unsigned int status_seq = app_state_load_u32(&g_state.status_seq);

        /* The periodic message keeps the counters in the page moving, and gives a client that just
         * connected the state of the board. */
        if (det_seq != last_det_seq || status_seq != last_status_seq ||
            now - last_broadcast_sec >= WS_STATUS_PERIOD_SEC) {
            last_det_seq = det_seq;
            last_status_seq = status_seq;
            last_broadcast_sec = now;
            net_ws_broadcast_detections();
        }

        for (int i = 0; i < client_count; ) {
            client_t *c = &clients[i];
            int remove = FD_ISSET(c->fd, &rd_set) && client_receive(c) < 0;

            if (!remove && c->type == CLIENT_NEW &&
                now - c->accepted_sec > HTTP_REQUEST_TIMEOUT_SEC)
                remove = 1;

            if (!remove && c->type == CLIENT_MJPEG)
                remove = send_mjpeg_frame(c) < 0;

            if (remove) {
                client_remove(i);
            } else {
                i++;
            }
        }
    }

    /* 清理 */
    for (int i = 0; i < client_count; i++) {
        if (clients[i].mjpeg_buf) {
            free(clients[i].mjpeg_buf);
        }
        close(clients[i].fd);
    }
    client_count = 0;
    close(listen_fd);
    listen_fd = -1;
    return 0;
}

void net_ws_broadcast_detections(void)
{
    int has_ws = 0;
    for (int i = 0; i < client_count; i++) {
        if (clients[i].type == CLIENT_WS && clients[i].ws_handshake_done) {
            has_ws = 1;
            break;
        }
    }
    if (!has_ws) return;

    char json[4096];
    int pos = 0;
    status_fields_t status;
    detection_t det_snapshot[DETECTOR_MAX_DET];
    char frame_sequence_json[48];
    unsigned int det_frame_seq;
    int count;

    /* Snapshot under the detector lock, then format and send without holding
     * it.  A slow WebSocket client must never delay detector publication. */
    pthread_mutex_lock(&g_state.det_mutex);
    count = g_state.det_count;
    if (count < 0) count = 0;
    if (count > DETECTOR_MAX_DET) count = DETECTOR_MAX_DET;
    det_frame_seq = g_state.det_frame_seq;
    memcpy(det_snapshot, g_state.detections,
           (size_t)count * sizeof(detection_t));
    pthread_mutex_unlock(&g_state.det_mutex);
    if (cam_ws_format_detection_frame_seq(frame_sequence_json,
                                           sizeof(frame_sequence_json),
                                           det_frame_seq) < 0)
        return;

    collect_status(&status);
    pos = json_append(json, sizeof(json), pos, "{\"status\":{");
    pos = status_json_append_fields(json, sizeof(json), pos, &status);
    pos = json_append(json, sizeof(json), pos,
                      "},%s,\"detections\":[", frame_sequence_json);
    if (pos < 0)
        return;

    int published = 0;
    for (int i = 0; i < count && i < DETECTOR_MAX_DET; i++) {
        detection_t *d = &det_snapshot[i];
        if (!d->publish)
            continue;
        const char *class_name = coco_names_lookup(d->class_id);
        if (published > 0)
            pos = json_append(json, sizeof(json), pos, ",");
        pos = json_append(json, sizeof(json), pos,
            "{\"bbox\":[%.1f,%.1f,%.1f,%.1f],"
            "\"class_name\":\"%s\","
            "\"confidence\":%.3f,"
            "\"id\":%d,"
            "\"is_target\":%s}",
            d->bbox[0] * 640.0f, d->bbox[1] * 480.0f,
            d->bbox[2] * 640.0f, d->bbox[3] * 480.0f,
            class_name, d->confidence, d->track_id,
            d->is_target ? "true" : "false");
        published++;
        if (pos < 0)
            return;
    }

    pos = json_append(json, sizeof(json), pos, "]");
    if (pos < 0)
        return;

    /* 追加预测框 */
    int   pvalid;
    float pcx, pcy, pw, ph;
    int   pid, pcls;

    pthread_mutex_lock(&g_state.pred_mutex);
    pvalid = g_state.pred_box.valid;
    if (pvalid) {
        pcx  = g_state.pred_box.cx;
        pcy  = g_state.pred_box.cy;
        pw   = g_state.pred_box.w;
        ph   = g_state.pred_box.h;
        pid  = g_state.pred_box.track_id;
        pcls = g_state.pred_box.class_id;
    }
    pthread_mutex_unlock(&g_state.pred_mutex);

    /* 使用局部变量进行后续耗时的查表、浮点计算和字符串格式化 */
    if (pvalid) {
        const char *pname = coco_names_lookup(pcls);

        /* 转为像素坐标 (x,y,w,h) 与 detections 格式一致 */
        float px = (pcx - pw / 2.0f) * 640.0f;
        float py = (pcy - ph / 2.0f) * 480.0f;
        float ppw = pw * 640.0f;
        float pph = ph * 480.0f;

        pos = json_append(json, sizeof(json), pos,
            ",\"predicted_target\":{"
            "\"bbox\":[%.1f,%.1f,%.1f,%.1f],"
            "\"class_name\":\"%s\","
            "\"id\":%d,"
            "\"screen_cx\":%.3f,"
            "\"screen_cy\":%.3f}",
            px, py, ppw, pph,
            pname, pid, pcx, pcy);
        if (pos < 0)
            return;
    }

    pos = json_append(json, sizeof(json), pos, "}");
    if (pos < 0)
        return;

    for (int i = 0; i < client_count; ) {
        if (clients[i].type == CLIENT_WS && clients[i].ws_handshake_done &&
            ws_send_text(clients[i].fd, json, (size_t)pos) < 0) {
            client_remove(i);
            continue;
        }
        i++;
    }
}

void net_server_stop(void)
{
    app_state_store_int(&server_running, 0);
}
