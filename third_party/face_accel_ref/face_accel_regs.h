/*
 * face_accel_regs.h — Zynq-7020 PL 人脸检测加速器 寄存器契约（单一真值源）
 *
 * 由 PS 用户态客户端和内核驱动共同引用，并须与 v0.1.1 XSA 中的 PL 实现一致。
 * 修改时须同步 UAPI、驱动、硬件交付物和契约版本。
 *
 * 契约版本: v0.6 (transport 诊断扩展)  日期: 2026-07-19
 */
#ifndef FACE_ACCEL_REGS_H
#define FACE_ACCEL_REGS_H

#ifdef __KERNEL__
#include <linux/types.h>
typedef u32 uint32_t;
typedef s32 int32_t;
#else
#include <stdint.h>
#endif

/* ── 基址（建议，避开现有 0x41200000/0x41600000/0x43C00000/0x43C10000）── */
#define FACE_ACCEL_BASE          0x43C20000u
#define FACE_ACCEL_SIZE          0x10000u      /* 64KB */

/* ── 固定参数 ── */
#define FACE_ACCEL_IP_ID_MAGIC   0xFACE0004u   /* IP_ID 期望值 = 魔数|ABI版本 */
#define FACE_ACCEL_IN_W          640
#define FACE_ACCEL_IN_H          480
#define FACE_ACCEL_IN_BPP        3             /* RGB24, 字节序 R,G,B */
#define FACE_ACCEL_IN_STRIDE_DEF 1920          /* 640*3, 无 padding 时 */
#define FACE_ACCEL_FRAME_SIZE    921600        /* 640*480*3 */
#define FACE_ACCEL_BUF_ALIGN     64            /* 所有 DMA buffer 64B 对齐 */
#define FACE_ACCEL_WEIGHT_SIZE   358400u
#define FACE_ACCEL_SCRATCH_SIZE  311296u

/* ── 寄存器偏移（相对 FACE_ACCEL_BASE，32 位对齐，小端）── */
#define FA_IP_ID                 0x00  /* RO   魔数+版本 */
#define FA_CTRL                  0x04  /* WO/W1P 见 CTRL 位 */
#define FA_STATUS                0x08  /* RO   见 STATUS 位 */
#define FA_IRQ_ENABLE            0x0C  /* RW   中断使能 */
#define FA_IRQ_STATUS            0x10  /* RW1C 中断状态 */
#define FA_IN_BUF_BASE           0x14  /* RW   输入帧 DMA 基址(64B 对齐) */
#define FA_IN_STRIDE             0x18  /* RW   每行字节数(默认 1920) */
#define FA_OUT_BUF_BASE          0x1C  /* RW   输出 tensor DMA 基址(64B 对齐) */
#define FA_OUT_BUF_CAP           0x20  /* RW   输出 buffer 容量(字节) */
#define FA_WT_BUF_BASE           0x24  /* RW   权重 blob DMA 基址(64B 对齐) */
#define FA_WT_BUF_SIZE           0x28  /* RW   权重 blob 字节数 */
#define FA_FRAME_ID              0x2C  /* RW   本次推理帧序号 */
#define FA_FRAME_ID_DONE         0x30  /* RO   最近完成帧序号 */
#define FA_IN_W_REG              0x34  /* RO   =640 */
#define FA_IN_H_REG              0x38  /* RO   =480 */
#define FA_OUT_LEN               0x3C  /* RO   本次实际写出字节数 */
#define FA_PERF_CYCLES           0x40  /* RO   上次推理耗用时钟周期 */
#define FA_MODEL_SEL             0x44  /* RW   多模型选择(默认 0) */
#define FA_SCRATCH_BUF_BASE      0x48  /* RW   PL私有feature scratch基址 */
#define FA_SCRATCH_BUF_SIZE      0x4C  /* RW   scratch容量，至少311296B */
#define FA_TIMEOUT_CYCLES        0x50  /* RW   无进展watchdog；0为关闭 */
#define FA_PERF_DEP_WAIT         0x54  /* RO   依赖等待周期 */
#define FA_PERF_CMD_WAIT         0x58  /* RO   DataMover命令反压周期 */
#define FA_PERF_DMA_ACTIVE       0x5C  /* RO   DMA在途周期 */
#define FA_PERF_PRE_ACTIVE       0x60  /* RO   预处理活动周期 */
#define FA_PERF_OP_ACTIVE        0x64  /* RO   operator活动周期 */
#define FA_PERF_MERGE_ACTIVE     0x68  /* RO   merge/head活动周期 */
#define FA_DEBUG_PROGRESS        0x6C  /* RO   [27:16]descriptor [15:0]event */
#define FA_DEBUG_DMA             0x70  /* RO   DataMover复位/命令/状态实时观测 */
#define FA_DEBUG_TRANSPORT       0x74  /* RO   sticky transport来源/route/status */
#define FA_DEBUG_DMA_READ        0x78  /* RO   MM2S tag/state/累计握手字节 */
#define FA_DEBUG_DMA_WRITE       0x7C  /* RO   S2MM tag/state/累计握手字节 */

/* ── CTRL 位 ── */
#define FA_CTRL_START            (1u << 0)  /* W1P 单周期命令 */
#define FA_CTRL_SOFT_RESET       (1u << 1)  /* W1P; 不清 buffer 配置 */

/* ── STATUS 位 ── */
#define FA_STATUS_IDLE           (1u << 0)
#define FA_STATUS_BUSY           (1u << 1)
#define FA_STATUS_DONE           (1u << 2)
#define FA_STATUS_ERROR          (1u << 3)
#define FA_STATUS_ERRCODE_SHIFT  8
#define FA_STATUS_ERRCODE_MASK   (0xFFu << FA_STATUS_ERRCODE_SHIFT)

/* ── DEBUG_DMA 位（实时状态，不是sticky事件）── */
#define FA_DEBUG_DMA_ARESETN             (1u << 0)
#define FA_DEBUG_DMA_RESET_REQ           (1u << 1)
#define FA_DEBUG_DMA_MM2S_CMD_VALID      (1u << 2)
#define FA_DEBUG_DMA_MM2S_CMD_READY      (1u << 3)
#define FA_DEBUG_DMA_S2MM_CMD_VALID      (1u << 4)
#define FA_DEBUG_DMA_S2MM_CMD_READY      (1u << 5)
#define FA_DEBUG_DMA_MM2S_STS_VALID      (1u << 6)
#define FA_DEBUG_DMA_S2MM_STS_VALID      (1u << 7)
#define FA_DEBUG_DMA_MM2S_DATA_VALID     (1u << 8)
#define FA_DEBUG_DMA_MM2S_DATA_READY     (1u << 9)
#define FA_DEBUG_DMA_S2MM_DATA_VALID     (1u << 10)
#define FA_DEBUG_DMA_S2MM_DATA_READY     (1u << 11)

/* ── DEBUG_TRANSPORT 位（首错源均由各子模块sticky保持到SOFT_RESET）── */
#define FA_DBG_TR_DM_STICKY              (1u << 0)
#define FA_DBG_TR_DM_PROTOCOL            (1u << 1)
#define FA_DBG_TR_ROUTE_PROTOCOL         (1u << 2)
#define FA_DBG_TR_WEIGHT_LOADER          (1u << 3)
#define FA_DBG_TR_FEATURE_ADAPTER        (1u << 4)
#define FA_DBG_TR_OUTPUT_COMMIT          (1u << 5)
#define FA_DBG_TR_PAYLOAD_ROUTE          (1u << 6)
#define FA_DBG_TR_CFG_CONFLICT           (1u << 7)
#define FA_DBG_TR_DM_STATUS_SHIFT        8
#define FA_DBG_TR_DM_STATUS_MASK         (0xFFu << FA_DBG_TR_DM_STATUS_SHIFT)
#define FA_DBG_TR_READ_BUSY              (1u << 16)
#define FA_DBG_TR_WRITE_BUSY             (1u << 17)
#define FA_DBG_TR_MM2S_ROUTE_VALID       (1u << 18)
#define FA_DBG_TR_S2MM_ROUTE_VALID       (1u << 19)
#define FA_DBG_TR_MM2S_KIND_SHIFT        20
#define FA_DBG_TR_S2MM_KIND_SHIFT        24

/* ── DEBUG_DMA_READ/WRITE 公共布局 ── */
#define FA_DBG_XFER_BYTES_MASK           0x007FFFFFu
#define FA_DBG_XFER_STREAM_BAD           (1u << 23)
#define FA_DBG_XFER_STREAM_DONE          (1u << 24)
#define FA_DBG_XFER_STATUS_SEEN          (1u << 25)
#define FA_DBG_XFER_ACTIVE               (1u << 26)
#define FA_DBG_XFER_PENDING              (1u << 27)
#define FA_DBG_XFER_TAG_SHIFT            28

/* ── IRQ_ENABLE / IRQ_STATUS 位 ── */
#define FA_IRQ_ENABLE_DONE       (1u << 0)
#define FA_IRQ_DONE              (1u << 0)

/* ── ERR_CODE（首错锁存）── */
#define FA_ERR_NONE              0x00
#define FA_ERR_CORE_BUSY_START   0x20
#define FA_ERR_CORE_TIMEOUT      0x21
#define FA_ERR_CORE_DESCRIPTOR   0x22
#define FA_ERR_CORE_TRANSPORT    0x23
#define FA_ERR_CORE_PREPROCESS   0x24
#define FA_ERR_CORE_OPERATOR     0x25
#define FA_ERR_CORE_MERGE        0x26
#define FA_ERR_BAD_PARAM         0x30
#define FA_ERR_BAD_STATE         0x31
#define FA_ERR_ADDRESS_MAP       0x32

/* 输出 buffer ABI v1；完整语义见 third_party/face_accel_ref/output_abi_v1.json。 */
#define FA_OUT_MAGIC             0xFACE0D01u   /* 输出 header magic (data v01) */
#define FA_OUT_TENSOR_COUNT      2u
#define FA_DTYPE_INT8            0u
#define FA_OUT_SCORES_OFFSET     128u
#define FA_OUT_SCORES_LEN        8840u         /* [1,4420,2], row-major int8 */
#define FA_OUT_BOXES_OFFSET      9024u
#define FA_OUT_BOXES_LEN         17680u        /* [1,4420,4], row-major int8 */
#define FA_OUT_LEN_V1            26704u
#define FA_OUT_CAP_MIN           26752u        /* FA_OUT_LEN_V1向上64B对齐 */
#define FA_OUT_SCORES_SCALE      0.06540081650018692f
#define FA_OUT_BOXES_SCALE       0.062080398201942444f
#define FA_OUT_ZERO_POINT        0

typedef struct {
    uint32_t magic;         /* = FA_OUT_MAGIC */
    uint32_t frame_id;      /* 与 FA_FRAME_ID 配对 */
    uint32_t tensor_count;
    uint32_t total_len;     /* = FA_OUT_LEN */
} fa_out_header_t;

typedef struct {
    uint32_t offset;        /* 相对输出 buffer 起始的字节偏移 */
    uint32_t dtype;
    uint32_t dims[4];       /* ABI JSON定义的逻辑维度 */
    float    scale;         /* 反量化: real = (q - zero_point) * scale */
    int32_t  zero_point;
} fa_tensor_desc_t;

#if defined(__STDC_VERSION__) && __STDC_VERSION__ >= 201112L
_Static_assert(sizeof(fa_out_header_t) == 16u, "fa_out_header_t ABI drift");
_Static_assert(sizeof(fa_tensor_desc_t) == 32u, "fa_tensor_desc_t ABI drift");
#endif

#endif /* FACE_ACCEL_REGS_H */
