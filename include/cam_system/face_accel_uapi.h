#ifndef CAM_SYSTEM_FACE_ACCEL_UAPI_H
#define CAM_SYSTEM_FACE_ACCEL_UAPI_H

#include <linux/ioctl.h>
#include <linux/types.h>

/*
 * /dev/face_accel.  One job at a time, on a 640x480 RGB24 frame.
 *
 * Version 1: GET_INFO, RUN (input copied through the driver, blocking), RESET.
 * Version 2 adds the queue interface: GET_CAPS, IMPORT, SUBMIT, WAIT.  Version 1 is kept
 * unchanged: GET_INFO keeps reporting CAM_FACE_ACCEL_INFO_VERSION, which a version 1 client
 * compares for equality.  A client finds version 2 by calling GET_CAPS; a driver without it
 * answers ENOTTY.
 *
 * Reserved fields must be zero.
 */
#define CAM_FACE_ACCEL_DEVICE             "/dev/face_accel"
#define CAM_FACE_ACCEL_INFO_VERSION       1u
#define CAM_FACE_ACCEL_UAPI_VERSION       2u
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

/* The hardware can read an imported buffer without writing to it (R10 and later). */
#define CAM_FACE_ACCEL_CAP_DMABUF_INPUT   (1u << 0)

#define CAM_FACE_ACCEL_MAX_IMPORTS        8u
#define CAM_FACE_ACCEL_WEIGHT_SHA256_SIZE 32u

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

struct cam_face_accel_caps {
	__u32 uapi_version;
	__u32 flags;            /* CAM_FACE_ACCEL_CAP_* */
	__u32 hw_revision;      /* HW_REVISION register; 0 on R9 */
	__u32 hw_features;      /* HW_FEATURES register; 0 on R9 */
	__u32 queue_depth;      /* jobs SUBMIT accepts before answering EBUSY */
	__u32 max_imports;      /* per open file */
	__u32 reserved0[2];
	__u8 weight_sha256[CAM_FACE_ACCEL_WEIGHT_SHA256_SIZE];  /* verified at load */
	__u32 reserved[8];
};

/* Driver results.  Valid for diagnostics even when the ioctl returns an error. */
struct cam_face_accel_result {
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

struct cam_face_accel_run {
	__aligned_u64 input_ptr;
	__aligned_u64 output_ptr;
	__u32 input_size;
	__u32 output_capacity;
	__u32 frame_id;
	__u32 timeout_ms;
	struct cam_face_accel_result result;
};

/*
 * Imports a dma-buf for use as a SUBMIT input.  It must be physically contiguous, hold one
 * input frame, and never be written by the CPU through a cached mapping: the driver does no
 * cache maintenance on it.  The handle stays valid until the file is closed.
 */
struct cam_face_accel_import {
	__s32 fd;
	__u32 handle;           /* out */
	__u32 reserved[2];
};

struct cam_face_accel_submit {
	__u32 input_handle;
	__u32 frame_id;
	__u32 timeout_ms;       /* 0 = driver default; counts from SUBMIT */
	__u32 reserved;
};

/* Takes the result of the job this file submitted.  The buffer stays in use until it returns. */
struct cam_face_accel_wait {
	__aligned_u64 output_ptr;
	__u32 output_capacity;
	__u32 reserved;
	struct cam_face_accel_result result;
};

#define CAM_FACE_ACCEL_IOC_MAGIC   'F'
#define CAM_FACE_ACCEL_IOC_GET_INFO \
	_IOR(CAM_FACE_ACCEL_IOC_MAGIC, 0x00, struct cam_face_accel_info)
#define CAM_FACE_ACCEL_IOC_RUN \
	_IOWR(CAM_FACE_ACCEL_IOC_MAGIC, 0x01, struct cam_face_accel_run)
#define CAM_FACE_ACCEL_IOC_RESET \
	_IO(CAM_FACE_ACCEL_IOC_MAGIC, 0x02)
#define CAM_FACE_ACCEL_IOC_GET_CAPS \
	_IOR(CAM_FACE_ACCEL_IOC_MAGIC, 0x03, struct cam_face_accel_caps)
#define CAM_FACE_ACCEL_IOC_IMPORT \
	_IOWR(CAM_FACE_ACCEL_IOC_MAGIC, 0x04, struct cam_face_accel_import)
#define CAM_FACE_ACCEL_IOC_SUBMIT \
	_IOW(CAM_FACE_ACCEL_IOC_MAGIC, 0x05, struct cam_face_accel_submit)
#define CAM_FACE_ACCEL_IOC_WAIT \
	_IOWR(CAM_FACE_ACCEL_IOC_MAGIC, 0x06, struct cam_face_accel_wait)

#endif /* CAM_SYSTEM_FACE_ACCEL_UAPI_H */
