#ifndef HAL_FRMBUF_H
#define HAL_FRMBUF_H

#include <stdint.h>

#include "cam_system/config.h"

typedef struct {
    uint32_t sequence;
    double timestamp_sec;
    uint32_t flags;
    int timestamp_monotonic;
} frmbuf_frame_info_t;

/* 初始化 V4L2 采集后端，返回 0 成功。缓冲已分配和映射，但不入队、不起流：流由 frmbuf_start 开始。 */
int frmbuf_init(void);

/* 返回后端持有的帧缓冲虚拟地址。 */
uint8_t *frmbuf_get_buffer(int buf_idx);

/* 本次只有带 V4L2_BUF_FLAG_ERROR 的帧（已丢弃并归还驱动）。 */
#define FRMBUF_ERR_NO_FRAME (-2)

/* 采集一帧，返回实际完成的 mmap buffer index；超时或设备错误返回 -1，
 * 只出队到错误帧时返回 FRMBUF_ERR_NO_FRAME。info 可为 NULL。 */
int frmbuf_capture_frame(int timeout_ms, frmbuf_frame_info_t *info);

/* 启动以来丢弃的错误帧数（任意线程可读）。 */
unsigned long frmbuf_error_frame_count(void);

/* PL 桥启动以来的帧数和尺寸错误帧数，返回 0 成功；只能在一个线程调用。 */
int frmbuf_get_bridge_counts(int32_t *frames, int32_t *bad_frames);

/* 释放已完成帧；V4L2 后端会重新 QBUF。 */
int frmbuf_release_frame(int buf_idx);

/* 导出 buffer 的 dma-buf，返回新文件描述符或负 errno。CPU 不写这些缓冲。 */
int frmbuf_export_dmabuf(int buf_idx);

/* 起流：把 held 未标记、也不在队列里的 buffer 入队，再 STREAMON。已在出流时什么也不做。
 * held 可为 NULL。 */
int frmbuf_start(const int held[CAPTURE_BUF_CNT]);

/* 重建流：STREAMOFF 后把 held 未标记的 buffer 重新入队再 STREAMON。不释放 buffer，
 * 已导出的 dma-buf 继续有效；held 标记的 buffer 之后由 frmbuf_release_frame 归还。 */
int frmbuf_restart(const int held[CAPTURE_BUF_CNT]);

/* 停流：STREAMOFF，所有 buffer 离开队列（mmap 和导出的 dma-buf 仍然有效）。没在出流时什么也不做。 */
void frmbuf_stop(void);

/* 关闭清理 */
void frmbuf_close(void);

#endif
