#ifndef MCU_UART_CMDQ_H
#define MCU_UART_CMDQ_H

/*
 * mcu_uart_cmdq — UART 命令队列（SoC → MCU 发送通道）
 *
 * 设计要点：
 *  - 所有需要向 MCU 发送帧的上层业务（REQ/PUSH）均通过本队列统一入口
 *  - 单一 TX Thread 作为唯一 UART 写端，彻底消除多线程并发写 UART 导致的帧交叉问题
 *  - REQ：调用方提供 done_sem，TX Thread 发完帧并收到 RESP 后 sem_post，调用方 sem_wait 保持阻塞语义
 *  - PUSH：fire-and-forget，done_sem 置 NULL
 */

#include <stdint.h>
#include <semaphore.h>
#include <pthread.h>

#include "fw_link.h"

/* 队列深度，容纳 REQ + PUSH 并发入队 */
#define MCU_UART_CMDQ_DEPTH              (16u)

/* PUSH payload 最大长度（inline 拷贝进 cmd_t）
 * 当前最大 PUSH payload 为 fs_soc_sync_push_payload_t = 16 bytes，预留扩展空间 */
#define MCU_UART_CMDQ_PUSH_PAYLOAD_MAX   (256u)

/* 与 mcu_uart_session.h 保持一致 */
#define MCU_UART_CMDQ_MSG_REQ   (0x01u)
#define MCU_UART_CMDQ_MSG_PUSH  (0x03u)

/*
 * 单条命令描述符
 * REQ 字段（指针）：调用方保证 done_sem 触发前数据有效
 * PUSH 字段（inline 拷贝）：入队时完成数据拷贝，调用方无需保留原始数据
 */
typedef struct {
    uint8_t  msg_type;          /* MCU_UART_CMDQ_MSG_REQ / MCU_UART_CMDQ_MSG_PUSH */
    uint8_t  tag;               /* MCU_UART_LOAD_TAG_* */
    uint8_t  channel;           /* fw_link_channel_t */
    uint8_t  reserved;

    /* REQ-only */
    uint8_t         cmd_type;      /* MCU_PROT_TYPE_* */
    const uint8_t  *req_data;      /* 请求 payload（不含 type 字节），可为 NULL */
    uint16_t        req_data_len;
    uint8_t        *resp_buf;      /* 输出：RESP payload */
    uint16_t        resp_buf_size;
    uint16_t       *resp_len;      /* 输出：实际 RESP 长度 */
    uint32_t        timeout_ms;
    int            *result;        /* 输出：错误码 */
    sem_t          *done_sem;      /* TX Thread 完成后 sem_post；NULL 表示 fire-and-forget */

    /* PUSH-only */
    uint8_t  push_payload[MCU_UART_CMDQ_PUSH_PAYLOAD_MAX];
    uint16_t push_payload_len;
} mcu_uart_cmd_t;

typedef struct {
    uint32_t depth;
    uint32_t peak_depth;
    uint32_t req_wait_count;
    uint32_t req_wait_last_us;
    uint32_t req_wait_max_us;
    uint32_t push_drop;
} mcu_uart_cmdq_diag_t;

typedef struct {
    mcu_uart_cmd_t  ring[MCU_UART_CMDQ_DEPTH];
    uint32_t        head;
    uint32_t        tail;
    uint32_t        count;
    uint32_t        peak_count;
    uint32_t        req_wait_count;
    uint32_t        req_wait_last_us;
    uint32_t        req_wait_max_us;
    uint32_t        try_enqueue_drop;
    pthread_mutex_t mu;
    pthread_cond_t  not_empty;
    pthread_cond_t  not_full;
    int             stopped;    /* 1 = 队列已停止，不再接受新 cmd */
} mcu_uart_cmdq_t;

void mcu_uart_cmdq_init(mcu_uart_cmdq_t *q);
void mcu_uart_cmdq_destroy(mcu_uart_cmdq_t *q);

/* 停止队列：解除所有阻塞的 enqueue/dequeue */
void mcu_uart_cmdq_stop(mcu_uart_cmdq_t *q);

/*
 * 入队（队满时阻塞直到有空位或队列已停止）
 * 返回 FS_SUCCESS 或 FS_ERR_BUSY（队列已停止）
 */
int mcu_uart_cmdq_enqueue(mcu_uart_cmdq_t *q, const mcu_uart_cmd_t *cmd);

/*
 * 非阻塞入队（队满时立即返回 FS_ERR_BUSY，不等待空位）
 * 用于 PUSH 消息：允许丢弃，确保不阻塞调用方，避免 PUSH 洪泛饿死 REQ
 * 返回 FS_SUCCESS（入队成功）或 FS_ERR_BUSY（队满/队列已停止）
 */
int mcu_uart_cmdq_try_enqueue(mcu_uart_cmdq_t *q, const mcu_uart_cmd_t *cmd);

/*
 * 出队（无 cmd 时阻塞直到有 cmd 或超时或队列已停止）
 * 返回 FS_SUCCESS 或 FS_ERR_MCU_PROT_TIMEOUT
 */
int mcu_uart_cmdq_dequeue(mcu_uart_cmdq_t *q, mcu_uart_cmd_t *out, uint32_t timeout_ms);

void mcu_uart_cmdq_get_diag(mcu_uart_cmdq_t *q, mcu_uart_cmdq_diag_t *diag);

#endif /* MCU_UART_CMDQ_H */
