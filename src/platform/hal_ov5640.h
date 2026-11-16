#ifndef HAL_OV5640_H
#define HAL_OV5640_H

#include <stdint.h>

/* 初始化 OV5640 V4L2 控制后端，返回 0 成功，-1 失败。初始化后传感器处于 standby，开相机时由采集线程重新初始化。 */
int ov5640_init(void);

/* Hardware reset and the full register table; the sensor runs afterwards, with the scaler on,
 * the default offset and no colour bars.  Returns 0, or -1 when a write failed. */
int ov5640_initialize(void);

/* 进入/退出 standby（软件休眠） */
int ov5640_standby(int standby);

/* 原子写入 ISP offset；通过 packed V4L2 control 进入内核 group write。 */
int ov5640_apply_isp_offset(int x_offset, int y_offset);

/* 控制 ISP scaler bit。当前跟踪逻辑只需要开/关，不暴露任意 0x5001 写入。 */
int ov5640_set_scaler_enabled(int enabled);

/* 打开或关闭传感器彩条测试图，用于核对 DVP 数据采样。 */
int ov5640_set_test_pattern(int enabled);

/* 读取驱动的 I2C 写失败与 ISP offset 组写中止计数；驱动不支持时返回 -1。 */
int ov5640_get_error_counts(uint32_t *write_failures, uint32_t *group_aborts);

/* 关闭 V4L2 subdev fd。 */
void ov5640_close(void);

#endif
