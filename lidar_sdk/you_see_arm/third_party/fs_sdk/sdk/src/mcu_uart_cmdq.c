#include "mcu_uart_cmdq.h"

#include <string.h>
#include <errno.h>
#include <time.h>

#include "fs_err.h"

static fw_link_channel_t cmdq_channel_or_default(const mcu_uart_cmd_t *cmd)
{
    if (cmd == NULL || cmd->channel >= FW_LINK_CHANNEL_COUNT) {
        return FW_LINK_CHANNEL_OTHER;
    }
    return (fw_link_channel_t)cmd->channel;
}

static uint64_t cmdq_mono_us(void)
{
    struct timespec ts;
    clock_gettime(CLOCK_MONOTONIC, &ts);
    return (uint64_t)ts.tv_sec * 1000000ull + (uint64_t)ts.tv_nsec / 1000ull;
}

void mcu_uart_cmdq_init(mcu_uart_cmdq_t *q)
{
    memset(q, 0, sizeof(*q));
    pthread_mutex_init(&q->mu, NULL);
    pthread_cond_init(&q->not_empty, NULL);
    pthread_cond_init(&q->not_full, NULL);
}

void mcu_uart_cmdq_destroy(mcu_uart_cmdq_t *q)
{
    pthread_mutex_destroy(&q->mu);
    pthread_cond_destroy(&q->not_empty);
    pthread_cond_destroy(&q->not_full);
}

void mcu_uart_cmdq_stop(mcu_uart_cmdq_t *q)
{
    pthread_mutex_lock(&q->mu);
    q->stopped = 1;
    pthread_cond_broadcast(&q->not_empty);
    pthread_cond_broadcast(&q->not_full);
    pthread_mutex_unlock(&q->mu);
}

int mcu_uart_cmdq_enqueue(mcu_uart_cmdq_t *q, const mcu_uart_cmd_t *cmd)
{
    uint64_t wait_start_us = 0u;
    uint8_t waited = 0u;

    pthread_mutex_lock(&q->mu);

    /* 队满时等待空位；队列已停止则立即返回 */
    while (q->count >= MCU_UART_CMDQ_DEPTH && !q->stopped) {
        if (waited == 0u) {
            wait_start_us = cmdq_mono_us();
            waited = 1u;
        }
        pthread_cond_wait(&q->not_full, &q->mu);
    }

    if (q->stopped) {
        pthread_mutex_unlock(&q->mu);
        return FS_ERR_BUSY;
    }

    if (waited != 0u) {
        uint64_t elapsed_us = cmdq_mono_us() - wait_start_us;
        if (elapsed_us > 0xffffffffull) {
            elapsed_us = 0xffffffffull;
        }
        q->req_wait_count++;
        q->req_wait_last_us = (uint32_t)elapsed_us;
        if ((uint32_t)elapsed_us > q->req_wait_max_us) {
            q->req_wait_max_us = (uint32_t)elapsed_us;
        }
    }

    q->ring[q->tail] = *cmd;
    q->tail = (q->tail + 1u) % MCU_UART_CMDQ_DEPTH;
    q->count++;
    if (q->count > q->peak_count) {
        q->peak_count = q->count;
    }

    pthread_cond_signal(&q->not_empty);
    pthread_mutex_unlock(&q->mu);
    return FS_SUCCESS;
}

int mcu_uart_cmdq_try_enqueue(mcu_uart_cmdq_t *q, const mcu_uart_cmd_t *cmd)
{
    fw_link_channel_policy_t policy;

    if (q == NULL || cmd == NULL) {
        return FS_ERR_BAD_PARAMS;
    }

    policy = fw_link_channel_policy(cmdq_channel_or_default(cmd));

    pthread_mutex_lock(&q->mu);

    if (q->stopped) {
        q->try_enqueue_drop++;
        pthread_mutex_unlock(&q->mu);
        return FS_ERR_BUSY;
    }

    if (q->count >= MCU_UART_CMDQ_DEPTH) {
        q->try_enqueue_drop++;
        pthread_mutex_unlock(&q->mu);
        return FS_ERR_BUSY;
    }

    if ((policy.drop_allowed != 0u)
        && ((q->count + policy.reserve_queue_slots) >= MCU_UART_CMDQ_DEPTH)) {
        q->try_enqueue_drop++;
        pthread_mutex_unlock(&q->mu);
        return FS_ERR_BUSY;
    }

    q->ring[q->tail] = *cmd;
    q->tail = (q->tail + 1u) % MCU_UART_CMDQ_DEPTH;
    q->count++;
    if (q->count > q->peak_count) {
        q->peak_count = q->count;
    }

    pthread_cond_signal(&q->not_empty);
    pthread_mutex_unlock(&q->mu);
    return FS_SUCCESS;
}

int mcu_uart_cmdq_dequeue(mcu_uart_cmdq_t *q, mcu_uart_cmd_t *out, uint32_t timeout_ms)
{
    struct timespec ts;
    int ret = FS_SUCCESS;

    clock_gettime(CLOCK_REALTIME, &ts);
    ts.tv_sec  += (time_t)(timeout_ms / 1000u);
    ts.tv_nsec += (long)((timeout_ms % 1000u) * 1000000L);
    if (ts.tv_nsec >= 1000000000L) {
        ts.tv_sec++;
        ts.tv_nsec -= 1000000000L;
    }

    pthread_mutex_lock(&q->mu);

    while (q->count == 0u && !q->stopped) {
        int cv = pthread_cond_timedwait(&q->not_empty, &q->mu, &ts);
        if (cv == ETIMEDOUT) {
            ret = FS_ERR_MCU_PROT_TIMEOUT;
            break;
        }
    }

    if (ret == FS_SUCCESS) {
        if (q->count > 0u) {
            *out = q->ring[q->head];
            q->head = (q->head + 1u) % MCU_UART_CMDQ_DEPTH;
            q->count--;
            pthread_cond_signal(&q->not_full);
        } else {
            /* stopped && count == 0 */
            ret = FS_ERR_MCU_PROT_TIMEOUT;
        }
    }

    pthread_mutex_unlock(&q->mu);
    return ret;
}

void mcu_uart_cmdq_get_diag(mcu_uart_cmdq_t *q, mcu_uart_cmdq_diag_t *diag)
{
    if (q == NULL || diag == NULL) {
        return;
    }

    pthread_mutex_lock(&q->mu);
    diag->depth = q->count;
    diag->peak_depth = q->peak_count;
    diag->req_wait_count = q->req_wait_count;
    diag->req_wait_last_us = q->req_wait_last_us;
    diag->req_wait_max_us = q->req_wait_max_us;
    diag->push_drop = q->try_enqueue_drop;
    pthread_mutex_unlock(&q->mu);
}
