#ifndef CAM_SYSTEM_FACE_ACCEL_UAPI_H
#define CAM_SYSTEM_FACE_ACCEL_UAPI_H

#include <linux/ioctl.h>
#include <linux/types.h>

#define CAM_FACE_ACCEL_DEVICE             "/dev/face_accel"
#define CAM_FACE_ACCEL_UAPI_VERSION       1u
#define CAM_FACE_ACCEL_IP_ID              0xFACE0004u
#define CAM_FACE_ACCEL_OUTPUT_MAGIC       0xFACE0D01u

#define CAM_FACE_ACCEL_INPUT_WIDTH        640u
#define CAM_FACE_ACCEL_INPUT_HEIGHT       480u
#define CAM_FACE_ACCEL_INPUT_STRIDE       1920u
#define CAM_FACE_ACCEL_INPUT_SIZE         921600u
#define CAM_FACE_ACCEL_WEIGHT_SIZE        358400u
#define CAM_FACE_ACCEL_SCRATCH_SIZE       311296u
#define CAM_FACE_ACCEL_OUTPUT_SIZE        26704u
#define CAM_FACE_ACCEL_OUTPUT_CAPACITY    26752u

#define CAM_FACE_ACCEL_INFO_WEIGHT_READY  (1u << 0)
#define CAM_FACE_ACCEL_INFO_IRQ_READY     (1u << 1)

struct cam_face_accel_info {
	__u32 uapi_version;
	__u32 ip_id;
	__u32 input_width;
	__u32 input_height;
	__u32 input_stride;
	__u32 input_size;
	__u32 output_size;
	__u32 output_capacity;
	__u32 weight_size;
	__u32 scratch_size;
	__u32 flags;
	__u32 reserved[5];
};

struct cam_face_accel_run {
	__aligned_u64 input_ptr;
	__aligned_u64 output_ptr;
	__u32 input_size;
	__u32 output_capacity;
	__u32 frame_id;
	__u32 timeout_ms;

	/* Driver results. Valid for diagnostics even when ioctl returns an error. */
	__u32 output_size;
	__u32 frame_id_done;
	__u32 status;
	__u32 error_code;
	__u32 perf_cycles;
	__u32 perf_dep_wait;
	__u32 perf_cmd_wait;
	__u32 perf_dma_active;
	__u32 perf_pre_active;
	__u32 perf_op_active;
	__u32 perf_merge_active;
	__u32 debug_progress;
	__u32 debug_dma;
	__u32 debug_transport;
	__u32 debug_dma_read;
	__u32 debug_dma_write;
};

#define CAM_FACE_ACCEL_IOC_MAGIC   'F'
#define CAM_FACE_ACCEL_IOC_GET_INFO \
	_IOR(CAM_FACE_ACCEL_IOC_MAGIC, 0x00, struct cam_face_accel_info)
#define CAM_FACE_ACCEL_IOC_RUN \
	_IOWR(CAM_FACE_ACCEL_IOC_MAGIC, 0x01, struct cam_face_accel_run)
#define CAM_FACE_ACCEL_IOC_RESET \
	_IO(CAM_FACE_ACCEL_IOC_MAGIC, 0x02)

#endif /* CAM_SYSTEM_FACE_ACCEL_UAPI_H */
