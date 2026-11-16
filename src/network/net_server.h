#ifndef NET_SERVER_H
#define NET_SERVER_H

/* 启动 HTTP + WebSocket 服务器（阻塞，在主线程事件循环中运行）
 * 端口: HTTP_PORT (8080)。signal_fd 为 signalfd（-1 表示不处理信号），可读即有序退出。
 * 正常退出返回 0，启动失败返回 -1。 */
int net_server_run(int signal_fd);

/* 通知所有 WebSocket 客户端发送检测结果 */
void net_ws_broadcast_detections(void);

/* 停止服务器 */
void net_server_stop(void);

#endif
