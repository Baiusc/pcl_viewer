#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <pthread.h>
#include <sys/select.h>
#include <errno.h>

#include "board_cfg.h"
#include "mcu_log_reader.h"
#include "drivers/uart/uart.h"
#include "utils/fs_log.h"
#include "utils/fs_err.h"

#undef  LOG_TAG
#define LOG_TAG "mcu-log"

#define MCU_LOG_UART_SPEED  115200
#define MCU_LOG_BUF_SIZE    512
#define MCU_LOG_POLL_MS     100

static pthread_t g_thread;
static volatile bool g_running   = false;
static volatile bool g_suspended = false;
static volatile bool g_fd_released = false;  // 通知 suspend 调用者 FD 已释放

static pthread_mutex_t g_mutex = PTHREAD_MUTEX_INITIALIZER;
static pthread_cond_t  g_cond  = PTHREAD_COND_INITIALIZER;

static void *mcu_log_reader_thread(void *arg)
{
    (void)arg;
    int32_t fd = -1;
    char line_buf[MCU_LOG_BUF_SIZE];
    int line_len = 0;
    char read_buf[128];

    FS_LOGD("MCU log reader thread started");

    while (g_running) {
        // 检查是否被挂起
        pthread_mutex_lock(&g_mutex);
        while (g_suspended && g_running) {
            // 释放串口 FD
            if (fd >= 0) {
                close(fd);
                fd = -1;
                g_fd_released = true;
                pthread_cond_signal(&g_cond);
            }
            pthread_cond_wait(&g_cond, &g_mutex);
        }
        pthread_mutex_unlock(&g_mutex);

        if (!g_running) {
            break;
        }

        // 打开串口
        if (fd < 0) {
            fd = fs_uart_open_init(MCU_LOG_UART_DEV, MCU_LOG_UART_SPEED);
            if (fd < 0) {
                FS_LOGE("Failed to open %s for MCU log reading", MCU_LOG_UART_DEV);
                usleep(1000 * 1000);  // 等待 1 秒后重试
                continue;
            }
            line_len = 0;
            FS_LOGD("MCU log UART opened on %s", MCU_LOG_UART_DEV);
        }

        // 使用 select 等待数据，带超时以便检查 running/suspended 状态
        fd_set read_set;
        struct timeval tv;
        FD_ZERO(&read_set);
        FD_SET(fd, &read_set);
        tv.tv_sec = 0;
        tv.tv_usec = MCU_LOG_POLL_MS * 1000;

        int ret = select(fd + 1, &read_set, NULL, NULL, &tv);
        if (ret < 0) {
            if (errno == EINTR) {
                continue;
            }
            FS_LOGE("select error: %s", strerror(errno));
            close(fd);
            fd = -1;
            continue;
        }

        if (ret == 0 || !FD_ISSET(fd, &read_set)) {
            continue;
        }

        // 读取数据
        ssize_t n = read(fd, read_buf, sizeof(read_buf));
        if (n <= 0) {
            if (n < 0 && errno != EAGAIN && errno != EWOULDBLOCK && errno != EINTR) {
                FS_LOGE("read error: %s", strerror(errno));
                close(fd);
                fd = -1;
            }
            continue;
        }

        // 按行缓冲输出
        for (ssize_t i = 0; i < n; i++) {
            char ch = read_buf[i];

            if (ch == '\n' || line_len >= MCU_LOG_BUF_SIZE - 1) {
                line_buf[line_len] = '\0';
                if (line_len > 0) {
                    for (int j = 0; j < line_len; j++) {
                        if ((j + 2 < line_len) && (line_buf[j] == '$') && (line_buf[j + 1] == '>')) {
                            FS_LOGD("%s", &line_buf[j + 2]);
                            break;
                        }
                    }
                }
                line_len = 0;
            } else if (ch != '\r') {
                line_buf[line_len++] = ch;
            }
        }
    }

    // 清理
    if (fd >= 0) {
        if (line_len > 0) {
            line_buf[line_len] = '\0';

            for (int j = 0; j < line_len; j++) {
                if ((j + 2 < line_len) && (line_buf[j] == '$') && (line_buf[j + 1] == '>')) {
                    FS_LOGD("%s", &line_buf[j + 2]);
                    break;
                }
            }
        }
        close(fd);
    }

    FS_LOGD("MCU log reader thread stopped");
    return NULL;
}

int mcu_log_reader_start(void)
{
    if (g_running) {
        FS_LOGW("MCU log reader already running");
        return 0;
    }

    g_running = true;
    g_suspended = false;
    g_fd_released = false;

    if (pthread_create(&g_thread, NULL, mcu_log_reader_thread, NULL) != 0) {
        FS_LOGE("Failed to create MCU log reader thread");
        g_running = false;
        return FS_ERR_INTERNAL;
    }

    FS_LOGD("MCU log reader started");
    return FS_SUCCESS;
}

void mcu_log_reader_stop(void)
{
    if (!g_running) {
        return;
    }

    g_running = false;

    pthread_mutex_lock(&g_mutex);
    pthread_cond_signal(&g_cond);
    pthread_mutex_unlock(&g_mutex);

    pthread_join(g_thread, NULL);
    FS_LOGD("MCU log reader stopped");
}

bool mcu_log_reader_is_running(void)
{
    return g_running;
}

int mcu_log_reader_suspend(void)
{
    if (!g_running || g_suspended) {
        return 0;
    }

    FS_LOGD("Suspending MCU log reader...");

    pthread_mutex_lock(&g_mutex);
    g_suspended = true;
    g_fd_released = false;

    // 等待日志线程释放串口 FD
    while (g_running && !g_fd_released) {
        pthread_cond_wait(&g_cond, &g_mutex);
    }
    pthread_mutex_unlock(&g_mutex);

    FS_LOGD("MCU log reader suspended, UART FD released");
    return 0;
}

int mcu_log_reader_resume(void)
{
    if (!g_running || !g_suspended) {
        return 0;
    }

    FS_LOGD("Resuming MCU log reader...");

    pthread_mutex_lock(&g_mutex);
    g_suspended = false;
    pthread_cond_signal(&g_cond);
    pthread_mutex_unlock(&g_mutex);

    FS_LOGD("MCU log reader resumed");
    return 0;
}
