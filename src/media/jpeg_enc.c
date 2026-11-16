/* jpeg_enc.c — libjpeg-turbo JPEG 编码
 *
 * 使用 TurboJPEG API（NEON 加速）
 */
#define LOG_TAG "jpeg"

#include "media/jpeg_enc.h"
#include "cam_system/app_state.h"
#include "cam_system/config.h"
#include "common/app_config.h"
#include "common/log.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <turbojpeg.h>

static tjhandle tj_handle = NULL;
static unsigned char *tj_buf = NULL;
static unsigned long tj_buf_size = 0;
static int tj_quality = JPEG_QUALITY;   /* jpeg.quality, read once at init */

int jpeg_init(void)
{
    tj_quality = app_config()->jpeg_quality;
    tj_handle = tjInitCompress();
    if (!tj_handle) {
        LOG_E("tjInitCompress failed: %s", tjGetErrorStr());
        return -1;
    }
    
    /* 预分配最大可能的 JPEG 缓冲区 */
    tj_buf_size = tjBufSize(IMG_W, IMG_H, TJSAMP_420);
    tj_buf = (unsigned char *)malloc(tj_buf_size);
    if (!tj_buf) {
        LOG_E("Failed to allocate %lu bytes", tj_buf_size);
        tjDestroy(tj_handle);
        tj_handle = NULL;
        return -1;
    }
    
    /* 预分配全局 g_state 的 jpeg_buf */
    g_state.jpeg_buf = (uint8_t *)malloc(tj_buf_size);
    if (!g_state.jpeg_buf) {
        LOG_E("Failed to allocate g_state.jpeg_buf");
        free(tj_buf);
        tjDestroy(tj_handle);
        return -1;
    }

    LOG_I("Encoder initialized (TurboJPEG + NEON), quality=%d, buf=%lu bytes", tj_quality, tj_buf_size);
    return 0;
}

int jpeg_encode_frame(const uint8_t *rgb_data, unsigned int frame_seq)
{
    if (!tj_handle || !rgb_data || !tj_buf) return -1;

    unsigned long jpeg_size = tj_buf_size;

    int ret = tjCompress2(
        tj_handle,
        rgb_data,
        IMG_W, 0, IMG_H,
        TJPF_RGB,
        &tj_buf,
        &jpeg_size,
        TJSAMP_420,
        tj_quality,
        TJFLAG_FASTDCT
    );

    if (ret < 0) {
        LOG_E_RL(5.0, 3, "Compress error: %s", tjGetErrorStr());
        return -1;
    }

    /* 更新全局 JPEG 缓存 */
    pthread_mutex_lock(&g_state.jpeg_mutex);

    /* 重新分配并拷贝（避免共享 tj_buf） */
    if (g_state.jpeg_buf) {
        // 在 init 里分配的大小是最大的 tj_buf_size，所以这里不会越界
        memcpy(g_state.jpeg_buf, tj_buf, jpeg_size);
        g_state.jpeg_size = jpeg_size;
        g_state.jpeg_seq = frame_seq;
    }

    pthread_mutex_unlock(&g_state.jpeg_mutex);

    return 0;
}

void jpeg_close(void)
{
    if (tj_buf) {
        free(tj_buf);
        tj_buf = NULL;
    }
    if (tj_handle) {
        tjDestroy(tj_handle);
        tj_handle = NULL;
    }
}
