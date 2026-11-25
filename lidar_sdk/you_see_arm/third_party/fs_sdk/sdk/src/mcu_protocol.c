#include <pthread.h>
#include <stdlib.h>
#include <string.h>
#include <stdio.h>
#include <fcntl.h>
#include <unistd.h>
#include <termios.h>
#include <time.h>
#include <sys/select.h>
#include <errno.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <stddef.h>

#include "fs_log.h"
#include "fs_err.h"
#include "fs_top_module_cfg.h"
#include "fs_hw.h"
#include "utils/fs_version.h"
#include "drivers/i2c/i2c_master.h"
#include "board_cfg.h"
#include "mcu_protocol.h"
#include "soc_system_manager.h"
#include "mcu_uart_session.h"
#include "mcu_uart_cmdq.h"
#include "soc_fw_link.h"
#include "fw_link.h"
#include "fs_util.h"
#include "fw_if_route_group.h"
#include "pinctrl.h"

#ifdef LOG_TAG
#undef LOG_TAG
#endif
#define LOG_TAG "McuProt"


static int mcu_uart_read_int_file(const char *path, int *value);
static int mcu_uart_debug_enabled(void);
static int mcu_prot_write(uint8_t cmd_type, uint8_t* data, uint16_t data_len);
static int mcu_prot_read(uint8_t cmd_type, uint8_t* data, uint16_t data_len);
static int mcu_prot_wait_idle(uint32_t period_ms, uint32_t timeout_ms);
static void mcu_prot_bind_soc_fw_link_transport(void);
static void mcu_uart_consume_imu_push(void);
static void mcu_imu_forward_log_tap(const imu_sample_msg_t *samples, uint16_t count);

#define MCU_TIME_SYNC_DIAG_V2_SIZE ((uint16_t)offsetof(mcu_time_sync_diag_t, map_drift_ppb))
#define MCU_TIME_SYNC_DIAG_V4_SIZE ((uint16_t)offsetof(mcu_time_sync_diag_t, timesync_enabled))
#define MCU_TIME_SYNC_DIAG_V5_SIZE ((uint16_t)sizeof(mcu_time_sync_diag_t))
#define MCU_TIME_SYNC_DIAG_SIZE    MCU_TIME_SYNC_DIAG_V5_SIZE
#define MCU_IMU_PUSH_DIAG_V1_SIZE  ((uint16_t)offsetof(mcu_imu_push_diag_t, imu_ts_drop_extend_no_anchor))
#define MCU_IMU_PUSH_DIAG_V5_SIZE  ((uint16_t)offsetof(mcu_imu_push_diag_t, imu_ts_drop_fifo_duplicate))
#define MCU_IMU_PUSH_DIAG_V6_SIZE  ((uint16_t)offsetof(mcu_imu_push_diag_t, imu_ts_anchor_reject_count))
#define MCU_IMU_PUSH_DIAG_SIZE     ((uint16_t)sizeof(mcu_imu_push_diag_t))

#ifndef FS_MCU_UART_DEV_NAME
#define FS_MCU_UART_DEV_NAME           "/dev/ttyS2"
#endif
#define MCU_UART_DEFAULT_DEV            FS_MCU_UART_DEV_NAME
#define MCU_UART_DEFAULT_BAUD           (115200)
#define MCU_UART_REQ_TIMEOUT_MS          (800)
/* PROGRAM_EXIT may validate CFG storage after a large flash commit. */
#define MCU_UART_PROGRAM_EXIT_TIMEOUT_MS (30000)
/* Flash path may block longer during erase, verify, header commit, and backup sync. */
#define MCU_UART_FLASH_REQ_TIMEOUT_MS   (120000)
#define MCU_UART_STATUS_TIMEOUT_MS      (100)
#define MCU_UART_RESP_SLOT_NUM          (8)
#define MCU_UART_BOOT_TEST_FRAME_NUM    (3)
#define MCU_UART_FSYNC_DIAG_POLL_FILE   "/data/fslidar/config/mcu_uart_fsync_diag_poll"
#define MCU_UART_TIME_SYNC_DIAG_POLL_FILE "/data/fslidar/config/mcu_uart_time_sync_diag_poll"
#define MCU_UART_IMU_PUSH_DIAG_POLL_FILE "/data/fslidar/config/mcu_uart_imu_push_diag_poll"
#define MCU_UART_IMU_FORWARD_EN_FILE     "/data/fslidar/config/mcu_uart_imu_forward_en"
/** 非 0：IMU tap 每条 batch 打 FS_LOGI；0 或未创建：forward_en 开启时限 1Hz */
#define MCU_UART_IMU_PRINT_FILE          "/data/fslidar/config/mcu_uart_imu_print"
#define UART_LOAD_DIAG_EN_FILE           "/data/fslidar/config/uart_load_diag_en"
#define UART_LOAD_DIAG_PERIOD_MS_FILE    "/data/fslidar/config/uart_load_diag_period_ms"
#define UART_LOAD_ECHO_RATE_HZ_FILE      "/data/fslidar/config/uart_load_echo_rate_hz"
#define UART_LOAD_ECHO_LEN_FILE          "/data/fslidar/config/uart_load_echo_len"
#define UART_LOAD_ADC_RATE_HZ_FILE       "/data/fslidar/config/uart_load_adc_rate_hz"
#define UART_LOAD_DIAG_CSV_PATH          "/tmp/fs_uart_load_diag_summary.csv"
#define UART_LOAD_DIAG_CSV_PATH_OLD      "/tmp/fs_uart_load_diag_summary.csv.1"
#define UART_LOAD_DIAG_CSV_MAX_BYTES     (16u * 1024u * 1024u)
#define UART_LOAD_DIAG_PERIOD_DEFAULT_MS (5000u)
#define UART_LOAD_DIAG_PERIOD_MIN_MS     (1000u)
#define UART_LOAD_DIAG_PERIOD_MAX_MS     (60000u)
#define UART_LOAD_ECHO_LEN_DEFAULT       (32u)
#define UART_LOAD_ECHO_LEN_MAX           (256u)



#define MCU_LOG_TO_SOC_LINE_BUF_SIZE    (256u)

static pthread_mutex_t mcu_prot_mutex = PTHREAD_MUTEX_INITIALIZER;
static pthread_mutex_t g_mcu_prot_lifecycle_mutex = PTHREAD_MUTEX_INITIALIZER;
static int g_mcu_prot_inited = 0;
static int mcu_push_msg_run = 0;
static pthread_t mcu_push_msg_tid = 0;
static int g_mcu_uart_fd = -1;
/* g_mcu_uart_txid 仅由 TX Thread（或 boot-test 初始化阶段）访问，无需额外保护 */
static uint16_t g_mcu_uart_txid = 1u;

/* TX Thread — 唯一的 UART 写端 */
static mcu_uart_cmdq_t g_uart_cmdq;
static int             g_mcu_uart_tx_run = 0;
static pthread_t       g_mcu_uart_tx_tid = 0;
static uint8_t g_mcu_uart_resp_cache[MCU_PROT_TYPE_NUM][MCU_PROT_BUFFER_SIZE] = {{0}};
static uint16_t g_mcu_uart_resp_cache_len[MCU_PROT_TYPE_NUM] = {0};
static int g_mcu_uart_debug = -1;
static int g_mcu_uart_passive_listen = -1;
static int g_mcu_uart_periodic_req_test = -1;
static int g_mcu_uart_periodic_req_run = 0;
static pthread_t g_mcu_uart_periodic_req_tid = 0;

static pthread_mutex_t g_imu_forward_mutex = PTHREAD_MUTEX_INITIALIZER;
static mcu_imu_forward_cb_t g_imu_forward_cb = NULL;
static void *g_imu_forward_user = NULL;

static char g_mcu_log_to_soc_line[MCU_LOG_TO_SOC_LINE_BUF_SIZE];
static uint16_t g_mcu_log_to_soc_line_len = 0u;

static int mcu_log_to_soc_is_text_byte(uint8_t byte)
{
    return (byte == '\t') || (byte == '\n') || (byte == '\r')
           || (byte >= 0x20u && byte <= 0x7eu);
}

static void mcu_log_to_soc_flush(void)
{
    if (g_mcu_log_to_soc_line_len == 0u) {
        return;
    }

    g_mcu_log_to_soc_line[g_mcu_log_to_soc_line_len] = '\0';

    if ((g_mcu_log_to_soc_line_len >= 3) && (g_mcu_log_to_soc_line[0] == '$') && (g_mcu_log_to_soc_line[1] == '>')) {
        FS_LOGI("[MCU] %s", &g_mcu_log_to_soc_line[2]);
    }
    else {
        FS_LOGI("[MCU] %s", g_mcu_log_to_soc_line);
    }

    g_mcu_log_to_soc_line_len = 0u;
}

static void mcu_log_to_soc_append(uint8_t byte)
{
    if (!mcu_log_to_soc_is_text_byte(byte)) {
        mcu_log_to_soc_flush();
        return;
    }

    if (byte == '\r') {
        return;
    }

    if (byte == '\n') {
        mcu_log_to_soc_flush();
        return;
    }

    if (g_mcu_log_to_soc_line_len >= (MCU_LOG_TO_SOC_LINE_BUF_SIZE - 1u)) {
        mcu_log_to_soc_flush();
    }

    g_mcu_log_to_soc_line[g_mcu_log_to_soc_line_len++] = (char)byte;
}

static int mcu_log_to_soc_tap(mcu_uart_parser_t *parser, uint8_t byte)
{
    if (parser == NULL) {
        return 0;
    }

    if (parser->state == MCU_UART_PARSE_SYNC0) {
        if (byte != MCU_UART_SESSION_MAGIC0) {
            mcu_log_to_soc_append(byte);
        }
        return 0;
    }

    if (parser->state == MCU_UART_PARSE_SYNC1) {
        if (byte == MCU_UART_SESSION_MAGIC1) {
            return 0;
        }

        mcu_log_to_soc_append(MCU_UART_SESSION_MAGIC0);
        if (byte != MCU_UART_SESSION_MAGIC0) {
            mcu_log_to_soc_append(byte);
        }
        return 0;
    }

    if (parser->state == MCU_UART_PARSE_HEADER
        && parser->header_idx == 2u
        && byte != MCU_UART_SESSION_VERSION) {
        mcu_log_to_soc_append(MCU_UART_SESSION_MAGIC0);
        mcu_log_to_soc_append(MCU_UART_SESSION_MAGIC1);
        mcu_log_to_soc_append(byte);
        mcu_uart_parser_init(parser);
        return 1;
    }

    return 0;
}

typedef struct {
    uint8_t valid;
    uint16_t txid;
    uint16_t len;
    uint8_t data[MCU_PROT_BUFFER_SIZE];
} mcu_uart_resp_slot_t;

typedef struct {
    uint8_t valid;
    uint16_t len;
    uint32_t seq;
    uint32_t dropped;
    uint8_t data[MCU_PROT_BUFFER_SIZE];
} mcu_uart_push_cache_t;

static mcu_uart_resp_slot_t g_mcu_uart_resp_slots[MCU_UART_RESP_SLOT_NUM] = {{0}};
static uint8_t g_mcu_uart_resp_slot_rr = 0u;
static pthread_mutex_t g_mcu_uart_resp_slot_mutex = PTHREAD_MUTEX_INITIALIZER;
static mcu_uart_push_cache_t g_mcu_uart_push_cache = {0};
static mcu_uart_link_diag_t g_mcu_uart_diag = {0};
static pthread_mutex_t g_uart_load_mutex = PTHREAD_MUTEX_INITIALIZER;
static mcu_uart_load_bucket_t g_uart_load_soc_buckets[MCU_UART_LOAD_TAG_NUM] = {{0}};
static uint64_t g_uart_load_soc_start_ms = 0u;
static uint32_t g_uart_load_soc_rx_bytes = 0u;
static uint32_t g_uart_load_soc_tx_write_err = 0u;
static uint32_t g_uart_load_soc_tx_block_last_us = 0u;
static uint32_t g_uart_load_soc_tx_block_max_us = 0u;
static uint32_t g_uart_load_soc_tcdrain_last_us = 0u;
static uint32_t g_uart_load_soc_tcdrain_max_us = 0u;
static uint32_t g_uart_load_soc_rtt_last_us = 0u;
static uint32_t g_uart_load_soc_rtt_max_us = 0u;
static int g_uart_load_eval_run = 0;
static pthread_t g_uart_load_eval_tid = 0;

#define UART_LOAD_STATIC_ASSERT(name, expr) typedef char name[(expr) ? 1 : -1]
UART_LOAD_STATIC_ASSERT(uart_load_summary_fits_mcu_normal_payload,
                        sizeof(mcu_uart_load_diag_summary_t) <= 256u);
UART_LOAD_STATIC_ASSERT(uart_load_page_fits_mcu_normal_payload,
                        sizeof(mcu_uart_load_diag_page_t) <= 256u);

/**
 * @brief 读取 UART 调试日志开关
 */
static int mcu_uart_debug_enabled(void)
{
    if (g_mcu_uart_debug < 0) {
        const char *env = getenv("MCU_UART_DEBUG");
        if (env != NULL && atoi(env) > 0) {
            g_mcu_uart_debug = 1;
        } else {
            g_mcu_uart_debug = 0;
        }
    }
    return g_mcu_uart_debug;
}

/**
 * @brief 读取是否启用 UART 被动监听模式
 */
static int mcu_uart_passive_listen_enabled(void)
{
    if (g_mcu_uart_passive_listen < 0) {
        const char *env = getenv("MCU_UART_PASSIVE_LISTEN");
        if (env == NULL || env[0] == '\0') {
            g_mcu_uart_passive_listen = 1;
        } else if (atoi(env) > 0) {
            g_mcu_uart_passive_listen = 1;
        } else {
            g_mcu_uart_passive_listen = 0;
        }
    }
    return g_mcu_uart_passive_listen;
}

/**
 * @brief 周期业务请求测试开关（环境变量 MCU_UART_PERIODIC_REQ_TEST，默认关闭）
 */
static int mcu_uart_periodic_req_test_enabled(void)
{
    if (g_mcu_uart_periodic_req_test < 0) {
        const char *env = getenv("MCU_UART_PERIODIC_REQ_TEST");
        if (env != NULL && atoi(env) > 0) {
            g_mcu_uart_periodic_req_test = 1;
        } else {
            g_mcu_uart_periodic_req_test = 0;
        }
    }
    return g_mcu_uart_periodic_req_test;
}

static int mcu_uart_read_int_file(const char *path, int *value)
{
    FILE *fp;
    char buf[32];
    char *endptr;
    long parsed;

    if (path == NULL || value == NULL) {
        return FS_ERR_BAD_PARAMS;
    }

    fp = fopen(path, "r");
    if (fp == NULL) {
        return FS_ERR_NOT_FOUND;
    }

    if (fgets(buf, sizeof(buf), fp) == NULL) {
        fclose(fp);
        return FS_ERR_IO;
    }
    fclose(fp);

    parsed = strtol(buf, &endptr, 0);
    if (endptr == buf) {
        return FS_ERR_BAD_PARAMS;
    }

    *value = (int)parsed;
    return FS_SUCCESS;
}

static void mcu_uart_write_int_file(const char *path, int value)
{
    FILE *fp;

    if (path == NULL) {
        return;
    }

    fp = fopen(path, "w");
    if (fp == NULL) {
        return;
    }

    fprintf(fp, "%d\n", value);
    fclose(fp);
}

static uint64_t uart_load_mono_us(void)
{
    struct timespec ts;
    clock_gettime(CLOCK_MONOTONIC, &ts);
    return (uint64_t)ts.tv_sec * 1000000ull + (uint64_t)ts.tv_nsec / 1000ull;
}

static uint64_t uart_load_realtime_us(void)
{
    struct timespec ts;
    clock_gettime(CLOCK_REALTIME, &ts);
    return (uint64_t)ts.tv_sec * 1000000ull + (uint64_t)ts.tv_nsec / 1000ull;
}

static void uart_load_record_latency(mcu_uart_load_bucket_t *bucket, uint32_t latency_us)
{
    uint64_t sum;

    if (bucket == NULL) {
        return;
    }

    bucket->latency_sample_count++;
    sum = ((uint64_t)bucket->latency_sum_us_hi << 32) | (uint64_t)bucket->latency_sum_us_lo;
    sum += latency_us;
    bucket->latency_sum_us_lo = (uint32_t)(sum & 0xffffffffu);
    bucket->latency_sum_us_hi = (uint32_t)(sum >> 32);
    bucket->latency_last_us = latency_us;
    if (latency_us > bucket->latency_max_us) {
        bucket->latency_max_us = latency_us;
    }
}

static void uart_load_record(uint8_t tag,
                             uint32_t frames,
                             uint32_t bytes,
                             uint32_t drops,
                             uint32_t errors,
                             uint32_t latency_us,
                             int has_latency)
{
    mcu_uart_load_bucket_t *bucket;

    if (tag >= MCU_UART_LOAD_TAG_NUM) {
        tag = MCU_UART_LOAD_TAG_OTHER;
    }

    pthread_mutex_lock(&g_uart_load_mutex);
    bucket = &g_uart_load_soc_buckets[tag];
    bucket->frames += frames;
    bucket->bytes += bytes;
    bucket->drops += drops;
    bucket->errors += errors;
    if (has_latency) {
        uart_load_record_latency(bucket, latency_us);
    }
    pthread_mutex_unlock(&g_uart_load_mutex);
}

static void uart_load_note_tx_block(uint32_t tx_block_us)
{
    pthread_mutex_lock(&g_uart_load_mutex);
    g_uart_load_soc_tx_block_last_us = tx_block_us;
    if (tx_block_us > g_uart_load_soc_tx_block_max_us) {
        g_uart_load_soc_tx_block_max_us = tx_block_us;
    }
    pthread_mutex_unlock(&g_uart_load_mutex);
}

static void uart_load_note_rx_bytes(uint32_t rx_bytes)
{
    pthread_mutex_lock(&g_uart_load_mutex);
    g_uart_load_soc_rx_bytes += rx_bytes;
    pthread_mutex_unlock(&g_uart_load_mutex);
}

static void uart_load_note_tcdrain(uint32_t tcdrain_us)
{
    pthread_mutex_lock(&g_uart_load_mutex);
    g_uart_load_soc_tcdrain_last_us = tcdrain_us;
    if (tcdrain_us > g_uart_load_soc_tcdrain_max_us) {
        g_uart_load_soc_tcdrain_max_us = tcdrain_us;
    }
    pthread_mutex_unlock(&g_uart_load_mutex);
}

static void uart_load_note_rtt(uint32_t rtt_us)
{
    pthread_mutex_lock(&g_uart_load_mutex);
    g_uart_load_soc_rtt_last_us = rtt_us;
    if (rtt_us > g_uart_load_soc_rtt_max_us) {
        g_uart_load_soc_rtt_max_us = rtt_us;
    }
    pthread_mutex_unlock(&g_uart_load_mutex);
}

static uint32_t uart_load_clamp_us(uint64_t value_us)
{
    if (value_us > 0xffffffffull) {
        return 0xffffffffu;
    }
    return (uint32_t)value_us;
}

static uint8_t uart_load_classify_prot_head(const uint8_t *head, uint16_t head_len)
{
    uint8_t cmd_dev;
    fw_if_route_group_t group;
    uint32_t sub_cmd;

    if (head == NULL || head_len < 6u) {
        return MCU_UART_LOAD_TAG_OTHER;
    }

    cmd_dev = head[0];
    sub_cmd = ((uint32_t)head[2] << 24)
            | ((uint32_t)head[3] << 16)
            | ((uint32_t)head[4] << 8)
            | (uint32_t)head[5];

    if (cmd_dev == MCU_PROT_DEV_MCU) {
        group = fw_if_mcu_route_group_from_request(FW_IF_MCU_PAYLOAD_NORMAL,
                                                   cmd_dev,
                                                   head[1],
                                                   sub_cmd);
        if (sub_cmd == MCU_PROT_MCU_GET_ADC) {
            return MCU_UART_LOAD_TAG_ADC;
        }
        if (sub_cmd == MCU_PROT_MCU_UART_LOAD_ECHO) {
            return MCU_UART_LOAD_TAG_LOAD_ECHO;
        }
        switch (group) {
        case FW_IF_ROUTE_GROUP_TIME_SYNC:
            return MCU_UART_LOAD_TAG_TIME_SYNC;
        case FW_IF_ROUTE_GROUP_DIAGNOSTIC:
            return MCU_UART_LOAD_TAG_DIAG;
        case FW_IF_ROUTE_GROUP_UPGRADE:
            return MCU_UART_LOAD_TAG_UPDATE;
        case FW_IF_ROUTE_GROUP_CONTROL_CORE:
        default:
            return MCU_UART_LOAD_TAG_REG;
        }
    }

    if (cmd_dev == MCU_PROT_DEV_CALIDATA || cmd_dev == MCU_PROT_DEV_CFGDATA) {
        return MCU_UART_LOAD_TAG_FLASH;
    }

    return MCU_UART_LOAD_TAG_REG;
}

static uint8_t uart_load_classify_req_payload(const uint8_t *payload, uint16_t payload_len)
{
    if (payload == NULL || payload_len < 1u) {
        return MCU_UART_LOAD_TAG_OTHER;
    }

    if (payload[0] == MCU_PROT_TYPE_FLASH) {
        return MCU_UART_LOAD_TAG_FLASH;
    }

    if (payload[0] != MCU_PROT_TYPE_NORMAL) {
        return MCU_UART_LOAD_TAG_OTHER;
    }

    return uart_load_classify_prot_head(&payload[1], (uint16_t)(payload_len - 1u));
}

static uint8_t uart_load_classify_frame(uint8_t msg_type, const uint8_t *payload, uint16_t payload_len)
{
    if (msg_type == MCU_UART_MSG_PUSH) {
        if (payload != NULL && payload_len > 0u) {
            if (payload[0] == FS_SOC_SYNC_PUSH_TOPIC) {
                return MCU_UART_LOAD_TAG_TIME_SYNC;
            }
            if (payload[0] == MCU_IMU_PUSH_TOPIC_COMPACT) {
                return MCU_UART_LOAD_TAG_IMU_PUSH;
            }
        }
        return MCU_UART_LOAD_TAG_OTHER;
    }

    if (msg_type == MCU_UART_MSG_REQ) {
        return uart_load_classify_req_payload(payload, payload_len);
    }

    if (msg_type == MCU_UART_MSG_RESP) {
        return uart_load_classify_prot_head(payload, payload_len);
    }

    return MCU_UART_LOAD_TAG_OTHER;
}

static uint8_t uart_load_classify_cmd(uint8_t cmd_type, const uint8_t *data, uint16_t data_len)
{
    uint8_t payload[1u + sizeof(mcu_prot_cmd_t)];
    uint16_t copy_len;

    if (cmd_type == MCU_PROT_TYPE_FLASH) {
        return MCU_UART_LOAD_TAG_FLASH;
    }
    if (cmd_type != MCU_PROT_TYPE_NORMAL) {
        return MCU_UART_LOAD_TAG_OTHER;
    }

    payload[0] = cmd_type;
    copy_len = data_len;
    if (copy_len > sizeof(mcu_prot_cmd_t)) {
        copy_len = sizeof(mcu_prot_cmd_t);
    }
    if (copy_len > 0u && data != NULL) {
        memcpy(&payload[1], data, copy_len);
    }
    return uart_load_classify_req_payload(payload, (uint16_t)(1u + copy_len));
}

static fw_link_channel_t uart_load_channel_from_tag(uint8_t tag)
{
    switch (tag) {
    case MCU_UART_LOAD_TAG_FLASH:
        return FW_LINK_CHANNEL_STORAGE;
    case MCU_UART_LOAD_TAG_ADC:
    case MCU_UART_LOAD_TAG_DIAG:
    case MCU_UART_LOAD_TAG_LOAD_ECHO:
        return (tag == MCU_UART_LOAD_TAG_ADC) ? FW_LINK_CHANNEL_CONTROL : FW_LINK_CHANNEL_DIAGNOSTIC;
    case MCU_UART_LOAD_TAG_TIME_SYNC:
        return FW_LINK_CHANNEL_TIME_SYNC;
    case MCU_UART_LOAD_TAG_IMU_PUSH:
        return FW_LINK_CHANNEL_IMU_PUSH;
    case MCU_UART_LOAD_TAG_UPDATE:
        return FW_LINK_CHANNEL_UPDATE_BOOT;
    case MCU_UART_LOAD_TAG_REG:
        return FW_LINK_CHANNEL_CONTROL;
    default:
        return FW_LINK_CHANNEL_CONTROL;
    }
}

/*
 * MCU frame-sync诊断触发开关（默认关闭）。运行时可通过
 * /data/fslidar/config/mcu_uart_fsync_diag_poll 写入 1 触发一次诊断。
 * 查询完成后会自动写回 0，避免持续轮询影响 MCU UART 协议响应。
 */
static int mcu_uart_fsync_diag_poll_enabled(void)
{
    int file_value = 0;

    if (mcu_uart_read_int_file(MCU_UART_FSYNC_DIAG_POLL_FILE, &file_value) == FS_SUCCESS) {
        return (file_value > 0) ? 1 : 0;
    }

    return 0;
}

static int mcu_uart_time_sync_diag_poll_enabled(void)
{
    int file_value = 0;

    if (mcu_uart_read_int_file(MCU_UART_TIME_SYNC_DIAG_POLL_FILE, &file_value) == FS_SUCCESS) {
        return (file_value > 0) ? 1 : 0;
    }

    return 0;
}

static int mcu_uart_imu_push_diag_poll_enabled(void)
{
    int file_value = 0;

    if (mcu_uart_read_int_file(MCU_UART_IMU_PUSH_DIAG_POLL_FILE, &file_value) == FS_SUCCESS) {
        return (file_value > 0) ? 1 : 0;
    }

    return 0;
}

static int mcu_uart_passive_listener_running(void)
{
    return mcu_push_msg_run && (mcu_push_msg_tid != 0);
}

/**
 * @brief 获取 UART 周期请求间隔，周期毫秒数（环境变量 MCU_UART_PERIODIC_REQ_MS，默认 1000ms）
 */
static uint32_t mcu_uart_periodic_req_interval_ms(void)
{
    const char *env = getenv("MCU_UART_PERIODIC_REQ_MS");
    long interval_ms = 1000;

    if (env != NULL && env[0] != '\0') {
        interval_ms = strtol(env, NULL, 10);
    }

    if (interval_ms < 50) {
        interval_ms = 50;
    }

    return (uint32_t)interval_ms;
}

/**
 * @brief 获取 UART 周期请求寄存器地址
 */
static uint32_t mcu_uart_periodic_req_addr(void)
{
    const char *env = getenv("MCU_UART_PERIODIC_REQ_ADDR");
    unsigned long addr = 0x00000002ul;

    if (env != NULL && env[0] != '\0') {
        addr = strtoul(env, NULL, 0);
    }

    return (uint32_t)addr;
}

/**
 * @brief 获取 UART 周期请求设备号
 */
static uint8_t mcu_uart_periodic_req_dev(void)
{
    const char *env = getenv("MCU_UART_PERIODIC_REQ_DEV");
    long dev = MCU_PROT_DEV_FL9103_1;

    if (env != NULL && env[0] != '\0') {
        dev = strtol(env, NULL, 0);
    }

    if (dev < MCU_PROT_DEV_FL9003_1 || dev > MCU_PROT_DEV_FL9103_ALL) {
        dev = MCU_PROT_DEV_FL9103_1;
    }

    return (uint8_t)dev;
}

static uint32_t mcu_uart_fsync_diag_poll_interval_ms(void)
{
    const char *env = getenv("MCU_UART_FSYNC_DIAG_MS");
    long interval_ms = 1000;

    if (env != NULL && env[0] != '\0') {
        interval_ms = strtol(env, NULL, 10);
    }

    if (interval_ms < 100) {
        interval_ms = 100;
    }

    return (uint32_t)interval_ms;
}

/**
 * @brief 存储响应帧到 RESP 槽位
 * @param txid 响应对应的事务 ID
 * @param payload 响应载荷数据
 * @param payload_len 响应载荷长度
 */
static void mcu_uart_store_resp_slot(uint16_t txid, const uint8_t *payload, uint16_t payload_len)
{
    mcu_uart_resp_slot_t *slot;

    if (payload == NULL || payload_len == 0u || payload_len > MCU_PROT_BUFFER_SIZE) {
        return;
    }

    pthread_mutex_lock(&g_mcu_uart_resp_slot_mutex);
    slot = &g_mcu_uart_resp_slots[g_mcu_uart_resp_slot_rr % MCU_UART_RESP_SLOT_NUM];
    g_mcu_uart_resp_slot_rr = (uint8_t)((g_mcu_uart_resp_slot_rr + 1u) % MCU_UART_RESP_SLOT_NUM);

    slot->valid = 1u;
    slot->txid = txid;
    slot->len = payload_len;
    memcpy(slot->data, payload, payload_len);
    pthread_mutex_unlock(&g_mcu_uart_resp_slot_mutex);
}

/**
 * @brief 从 RESP 槽位提取指定 txid 的响应
 * @param txid 期望匹配的事务 ID
 * @param payload 输出载荷缓冲区
 * @param payload_buf_size 输出缓冲区大小
 * @param payload_len 输出载荷长度
 */
static int mcu_uart_take_resp_slot(uint16_t txid, uint8_t *payload, uint16_t payload_buf_size, uint16_t *payload_len)
{
    uint32_t i;

    if (payload == NULL || payload_len == NULL) {
        return FS_ERR_BAD_PARAMS;
    }

    pthread_mutex_lock(&g_mcu_uart_resp_slot_mutex);
    for (i = 0; i < MCU_UART_RESP_SLOT_NUM; i++) {
        mcu_uart_resp_slot_t *slot = &g_mcu_uart_resp_slots[i];
        uint16_t copy_len;

        if (slot->valid == 0u || slot->txid != txid) {
            continue;
        }

        copy_len = slot->len;
        if (copy_len > payload_buf_size) {
            pthread_mutex_unlock(&g_mcu_uart_resp_slot_mutex);
            return FS_ERR_BUF_OVERFLOW;
        }

        memcpy(payload, slot->data, copy_len);
        *payload_len = copy_len;
        slot->valid = 0u;
        g_mcu_uart_diag.resp_slot_hit++;
        pthread_mutex_unlock(&g_mcu_uart_resp_slot_mutex);
        return FS_SUCCESS;
    }

    pthread_mutex_unlock(&g_mcu_uart_resp_slot_mutex);
    return FS_ERR_NOT_FOUND;
}

/**
 * @brief 缓存 MCU PUSH 帧（新消息覆盖旧消息）
 * @param payload PUSH 载荷数据
 * @param payload_len PUSH 载荷长度
 */
static void mcu_uart_store_push_frame(const uint8_t *payload, uint16_t payload_len)
{
    uint8_t tag;

    if (payload == NULL || payload_len == 0u) {
        return;
    }

    tag = uart_load_classify_frame(MCU_UART_MSG_PUSH, payload, payload_len);
    if (payload_len > MCU_PROT_BUFFER_SIZE) {
        g_mcu_uart_push_cache.dropped++;
        g_mcu_uart_diag.push_drop++;
        uart_load_record(tag, 0u, 0u, 1u, 0u, 0u, 0);
        return;
    }

    if (g_mcu_uart_push_cache.valid != 0u) {
        g_mcu_uart_push_cache.dropped++;
        g_mcu_uart_diag.push_drop++;
        uart_load_record(tag, 0u, 0u, 1u, 0u, 0u, 0);
    }

    memcpy(g_mcu_uart_push_cache.data, payload, payload_len);
    g_mcu_uart_push_cache.len = payload_len;
    g_mcu_uart_push_cache.valid = 1u;
    g_mcu_uart_push_cache.seq++;
    g_mcu_uart_diag.rx_push++;
}

/**
 * @brief 将整数波特率映射为 termios 速率常量
 * @param baud 波特率数值
 */
static speed_t mcu_uart_to_speed(int baud)
{
    switch (baud) {
    case 9600: return B9600;
    case 19200: return B19200;
    case 38400: return B38400;
    case 57600: return B57600;
    case 115200: return B115200;
#ifdef B230400
    case 230400: return B230400;
#endif
    default: return B115200;
    }
}

/**
 * @brief 按需打开并初始化 UART 设备
 */
static int mcu_uart_open_if_needed(void)
{
    const char *dev = getenv("MCU_UART_DEV");
    const char *baud_str = getenv("MCU_UART_BAUD");
    int baud = MCU_UART_DEFAULT_BAUD;
    speed_t speed;
    struct termios tio;
    int flags;

    if (g_mcu_uart_fd >= 0) {
        return FS_SUCCESS;
    }

    if (dev == NULL || dev[0] == '\0') {
        dev = MCU_UART_DEFAULT_DEV;
    }

    if (baud_str != NULL && baud_str[0] != '\0') {
        int baud_env = atoi(baud_str);
        if (baud_env > 0) {
            baud = baud_env;
        }
    }

    g_mcu_uart_fd = open(dev, O_RDWR | O_NOCTTY | O_NONBLOCK);
    if (g_mcu_uart_fd < 0) {
        FS_LOGE("open uart failed: %s, errno=%d", dev, errno);
        return FS_ERR_IO;
    }

    memset(&tio, 0, sizeof(tio));
    if (tcgetattr(g_mcu_uart_fd, &tio) != 0) {
        FS_LOGE("tcgetattr failed, errno=%d", errno);
        close(g_mcu_uart_fd);
        g_mcu_uart_fd = -1;
        return FS_ERR_IO;
    }

    speed = mcu_uart_to_speed(baud);
    cfsetispeed(&tio, speed);
    cfsetospeed(&tio, speed);

    cfmakeraw(&tio);
    tio.c_cflag |= (CLOCAL | CREAD);
    tio.c_cflag &= ~CSTOPB;
    tio.c_cflag &= ~CRTSCTS;
    tio.c_cflag &= ~PARENB;
    tio.c_cflag &= ~CSIZE;
    tio.c_cflag |= CS8;
    tio.c_cc[VMIN] = 0;
    tio.c_cc[VTIME] = 0;

    if (tcsetattr(g_mcu_uart_fd, TCSANOW, &tio) != 0) {
        FS_LOGE("tcsetattr failed, errno=%d", errno);
        close(g_mcu_uart_fd);
        g_mcu_uart_fd = -1;
        return FS_ERR_IO;
    }

    flags = fcntl(g_mcu_uart_fd, F_GETFL, 0);
    if (flags >= 0) {
        (void)fcntl(g_mcu_uart_fd, F_SETFL, flags & (~O_NONBLOCK));
    }

    (void)tcflush(g_mcu_uart_fd, TCIOFLUSH);

    FS_LOGI("MCU UART opened: dev=%s baud=%d", dev, baud);
    return FS_SUCCESS;
}

/**
 * @brief 关闭已打开的 UART 设备
 */
static void mcu_uart_close_if_opened(void)
{
    if (g_mcu_uart_fd >= 0) {
        close(g_mcu_uart_fd);
        g_mcu_uart_fd = -1;
    }
}

/**
 * @brief 通过 UART 完整发送一段字节流
 * @param buf 待发送数据缓冲区
 * @param len 待发送字节数
 */
static int mcu_uart_write_all(const uint8_t *buf, uint16_t len)
{
    uint16_t sent = 0;

    if (buf == NULL) {
        return FS_ERR_BAD_PARAMS;
    }

    while (sent < len) {
        ssize_t n = write(g_mcu_uart_fd, buf + sent, len - sent);
        if (n < 0) {
            if (errno == EINTR) {
                continue;
            }
            return FS_ERR_IO;
        }
        sent += (uint16_t)n;
    }
    return FS_SUCCESS;
}

/**
 * @brief 带超时读取一个 UART 字节
 * @param byte 输出字节地址
 * @param timeout_ms 读取超时时间（毫秒）
 */
static int mcu_uart_read_byte_timeout(uint8_t *byte, uint32_t timeout_ms)
{
    fd_set rfds;
    struct timeval tv;
    int ret;
    ssize_t n;

    if (byte == NULL) {
        return FS_ERR_BAD_PARAMS;
    }

    FD_ZERO(&rfds);
    FD_SET(g_mcu_uart_fd, &rfds);
    tv.tv_sec = (time_t)(timeout_ms / 1000u);
    tv.tv_usec = (suseconds_t)((timeout_ms % 1000u) * 1000u);

    ret = select(g_mcu_uart_fd + 1, &rfds, NULL, NULL, &tv);
    if (ret == 0) {
        return FS_ERR_MCU_PROT_TIMEOUT;
    }
    if (ret < 0) {
        if (errno == EINTR) {
            return FS_ERR_MCU_PROT_TIMEOUT;
        }
        return FS_ERR_IO;
    }

    n = read(g_mcu_uart_fd, byte, 1);
    if (n == 1) {
        return FS_SUCCESS;
    }
    if (n < 0 && (errno == EINTR || errno == EAGAIN || errno == EWOULDBLOCK)) {
        return FS_ERR_MCU_PROT_TIMEOUT;
    }
    if (n == 0) {
        return FS_ERR_MCU_PROT_TIMEOUT;
    }
    return FS_ERR_IO;
}

/**
 * @brief 在 RESP 槽位中等待指定 txid 的响应
 * @param expect_txid 期望匹配的事务 ID
 * @param payload 输出载荷缓冲区
 * @param payload_buf_size 输出缓冲区大小
 * @param payload_len 输出载荷长度
 * @param timeout_ms 等待超时时间（毫秒）
 */
static int mcu_uart_wait_resp_slot(uint16_t expect_txid,
                                   uint8_t *payload,
                                   uint16_t payload_buf_size,
                                   uint16_t *payload_len,
                                   uint32_t timeout_ms)
{
    uint64_t start_ms;

    if (payload == NULL || payload_len == NULL) {
        return FS_ERR_BAD_PARAMS;
    }

    start_ms = fs_util_timestamp_ms();
    while ((fs_util_timestamp_ms() - start_ms) < timeout_ms) {
        int ret = mcu_uart_take_resp_slot(expect_txid, payload, payload_buf_size, payload_len);
        if (ret == FS_SUCCESS) {
            return FS_SUCCESS;
        }
        fs_hw_msleep(2);
    }

    return FS_ERR_MCU_PROT_TIMEOUT;
}

/**
 * @brief 发送请求并等待响应，期间会处理和缓存其他收到的响应和推送消息
 * @param expect_txid 期望匹配的事务 ID
 * @param payload 输出载荷缓冲区
 * @param payload_buf_size 输出缓冲区大小
 * @param payload_len 输出载荷长度
 * @param timeout_ms 等待超时时间（毫秒）
 */
static int mcu_uart_recv_resp(uint16_t expect_txid,
                              uint8_t *payload,
                              uint16_t payload_buf_size,
                              uint16_t *payload_len,
                              uint32_t timeout_ms)
{
    mcu_uart_parser_t parser;
    mcu_uart_frame_meta_t meta;
    uint8_t byte;
    uint64_t start_ms;
    int parse_ret;

    if (payload == NULL || payload_len == NULL) {
        return FS_ERR_BAD_PARAMS;
    }

    if (mcu_uart_take_resp_slot(expect_txid, payload, payload_buf_size, payload_len) == FS_SUCCESS) {
        if (mcu_uart_debug_enabled()) {
            FS_LOGI("UART resp hit slot: txid=%u len=%u", expect_txid, *payload_len);
        }
        return FS_SUCCESS;
    }

    mcu_uart_parser_init(&parser);
    start_ms = fs_util_timestamp_ms();
    while ((fs_util_timestamp_ms() - start_ms) < timeout_ms) {
        int ret = mcu_uart_read_byte_timeout(&byte, 20u);
        if (ret == FS_ERR_MCU_PROT_TIMEOUT) {
            g_mcu_uart_diag.rx_timeout++;
            continue;
        }
        if (ret != FS_SUCCESS) {
            return ret;
        }

        if (mcu_log_to_soc_tap(&parser, byte)) {
            continue;
        }

        parse_ret = mcu_uart_parser_input(&parser,
                                          byte,
                                          payload,
                                          payload_buf_size,
                                          &meta,
                                          payload_len);
        if (parse_ret < 0) {
            g_mcu_uart_diag.rx_frame_err++;
            uart_load_record(MCU_UART_LOAD_TAG_OTHER, 0u, 0u, 0u, 1u, 0u, 0);
            if (mcu_uart_debug_enabled()) {
                FS_LOGE("UART parser error while waiting txid=%u", expect_txid);
            }
            continue;
        }

        if (parse_ret == 0) {
            continue;
        }

        if (mcu_uart_debug_enabled()) {
            FS_LOGI("UART frame rx: type=%u txid=%u len=%u", meta.msg_type, meta.txid, *payload_len);
        }
        g_mcu_uart_diag.rx_frame_ok++;
        {
            uint8_t tag = uart_load_classify_frame(meta.msg_type, payload, *payload_len);
            uint16_t frame_bytes = (uint16_t)(MCU_UART_SESSION_HEAD_LEN
                                 + *payload_len
                                 + MCU_UART_SESSION_CRC_LEN);
            uart_load_note_rx_bytes(frame_bytes);
            uart_load_record(tag, 1u, frame_bytes, 0u, 0u, 0u, 0);
        }

        if (meta.msg_type == MCU_UART_MSG_RESP && meta.txid == expect_txid) {
            return FS_SUCCESS;
        }

        if (meta.msg_type == MCU_UART_MSG_RESP) {
            mcu_uart_store_resp_slot(meta.txid, payload, *payload_len);
            g_mcu_uart_diag.resp_slot_store++;
            continue;
        }

        if (meta.msg_type == MCU_UART_MSG_PUSH) {
            mcu_uart_store_push_frame(payload, *payload_len);
            mcu_uart_consume_imu_push();
            if (mcu_uart_debug_enabled()) {
                FS_LOGI("UART push cached: len=%u seq=%u drop=%u",
                        g_mcu_uart_push_cache.len,
                        g_mcu_uart_push_cache.seq,
                        g_mcu_uart_push_cache.dropped);
            }
            continue;
        }
    }

    return FS_ERR_MCU_PROT_TIMEOUT;
}

/**
 * @brief 接收任意一帧 UART 会话数据
 * @param payload 输出载荷缓冲区
 * @param payload_buf_size 输出缓冲区大小
 * @param payload_len 输出载荷长度
 * @param meta 输出帧元信息
 * @param timeout_ms 等待超时时间（毫秒）
 * @param ext_parser 外部持久化解析器（非 NULL 时跨调用保持状态；NULL 时使用内部临时解析器）
 */
static int mcu_uart_recv_any_frame(uint8_t *payload,
                                   uint16_t payload_buf_size,
                                   uint16_t *payload_len,
                                   mcu_uart_frame_meta_t *meta,
                                   uint32_t timeout_ms,
                                   mcu_uart_parser_t *ext_parser)
{
    mcu_uart_parser_t local_parser;
    mcu_uart_parser_t *parser;
    uint8_t byte;
    uint64_t start_ms;
    int parse_ret;

    if (payload == NULL || payload_len == NULL || meta == NULL) {
        return FS_ERR_BAD_PARAMS;
    }

    if (ext_parser != NULL) {
        parser = ext_parser;
    } else {
        mcu_uart_parser_init(&local_parser);
        parser = &local_parser;
    }
    start_ms = fs_util_timestamp_ms();
    while ((fs_util_timestamp_ms() - start_ms) < timeout_ms) {
        int ret = mcu_uart_read_byte_timeout(&byte, 20u);
        if (ret == FS_ERR_MCU_PROT_TIMEOUT) {
            continue;
        }
        if (ret != FS_SUCCESS) {
            return ret;
        }

        if (mcu_log_to_soc_tap(parser, byte)) {
            continue;
        }

        parse_ret = mcu_uart_parser_input(parser,
                                          byte,
                                          payload,
                                          payload_buf_size,
                                          meta,
                                          payload_len);
        if (parse_ret < 0) {
            g_mcu_uart_diag.rx_frame_err++;
            uart_load_record(MCU_UART_LOAD_TAG_OTHER, 0u, 0u, 0u, 1u, 0u, 0);
            if (mcu_uart_debug_enabled()) {
                FS_LOGE("UART parser error in passive listener");
            }
            continue;
        }

        if (parse_ret == 0) {
            continue;
        }

        g_mcu_uart_diag.rx_frame_ok++;
        {
            uint8_t tag = uart_load_classify_frame(meta->msg_type, payload, *payload_len);
            uint16_t frame_bytes = (uint16_t)(MCU_UART_SESSION_HEAD_LEN
                                 + *payload_len
                                 + MCU_UART_SESSION_CRC_LEN);
            uart_load_note_rx_bytes(frame_bytes);
            uart_load_record(tag, 1u, frame_bytes, 0u, 0u, 0u, 0);
        }
        return FS_SUCCESS;
    }

    return FS_ERR_MCU_PROT_TIMEOUT;
}

/**
 * @brief 发送一个协议请求并等待对应响应
 * @param cmd_type 协议类型（NORMAL/STATUS/PUSH/FLASH）
 * @param data 请求载荷（不含 type）
 * @param data_len 请求载荷长度
 * @param resp 输出响应载荷缓冲区
 * @param resp_buf_size 输出缓冲区大小
 * @param resp_len 输出响应长度
 * @param timeout_ms 等待超时时间（毫秒）
 */
static int mcu_uart_send_req_and_wait_resp(uint8_t cmd_type,
                                           const uint8_t *data,
                                           uint16_t data_len,
                                           uint8_t *resp,
                                           uint16_t resp_buf_size,
                                           uint16_t *resp_len,
                                           uint32_t timeout_ms)
{
    uint8_t req_payload[MCU_UART_SESSION_MAX_PAYLOAD] = {0};
    uint8_t tx_frame[MCU_UART_SESSION_HEAD_LEN + MCU_UART_SESSION_MAX_PAYLOAD + MCU_UART_SESSION_CRC_LEN] = {0};
    uint16_t tx_frame_len = 0;
    uint16_t txid;
    uint8_t tag;
    uint64_t req_start_us;
    uint64_t write_start_us;
    uint32_t write_us;
    uint32_t rtt_us;
    int resp_ret;

    if (resp == NULL || resp_len == NULL) {
        return FS_ERR_BAD_PARAMS;
    }

    if ((uint32_t)data_len + 1u > (uint32_t)sizeof(req_payload)) {
        return FS_ERR_BAD_PARAMS;
    }

    req_payload[0] = cmd_type;
    if (data_len > 0u && data != NULL) {
        memcpy(&req_payload[1], data, data_len);
    }

    txid = g_mcu_uart_txid++;
    if (g_mcu_uart_txid == 0u) {
        g_mcu_uart_txid = 1u;
    }

    if (mcu_uart_frame_pack(MCU_UART_MSG_REQ,
                            txid,
                            0u,
                            req_payload,
                            (uint16_t)(data_len + 1u),
                            tx_frame,
                            sizeof(tx_frame),
                            &tx_frame_len) != 0) {
        return FS_ERR_BAD_PROTOCOL;
    }

    if (mcu_uart_debug_enabled()) {
        FS_LOGI("UART frame tx: type=%u txid=%u len=%u", MCU_UART_MSG_REQ, txid, (uint16_t)(data_len + 1u));
    }

    tag = uart_load_classify_frame(MCU_UART_MSG_REQ, req_payload, (uint16_t)(data_len + 1u));
    g_mcu_uart_diag.tx_frame++;
    g_mcu_uart_diag.tx_bytes += tx_frame_len;

    req_start_us = uart_load_mono_us();
    write_start_us = req_start_us;
    if (mcu_uart_write_all(tx_frame, tx_frame_len) != FS_SUCCESS) {
        pthread_mutex_lock(&g_uart_load_mutex);
        g_uart_load_soc_tx_write_err++;
        pthread_mutex_unlock(&g_uart_load_mutex);
        uart_load_record(tag, 0u, 0u, 0u, 1u, 0u, 0);
        return FS_ERR_MCU_PROT_WRITE;
    }
    write_us = uart_load_clamp_us(uart_load_mono_us() - write_start_us);
    uart_load_note_tx_block(write_us);
    uart_load_record(tag, 1u, tx_frame_len, 0u, 0u, write_us, 1);

    /*
     * 被动监听模式下由后台线程统一读取串口并写入 resp slot。
     * 这里只等待目标 txid 出现在 slot，避免多线程竞争同一 UART 读口。
     */
    if (mcu_uart_passive_listen_enabled() && mcu_uart_passive_listener_running()) {
        resp_ret = mcu_uart_wait_resp_slot(txid, resp, resp_buf_size, resp_len, timeout_ms);
    } else {
        resp_ret = mcu_uart_recv_resp(txid, resp, resp_buf_size, resp_len, timeout_ms);
    }

    rtt_us = uart_load_clamp_us(uart_load_mono_us() - req_start_us);
    uart_load_note_rtt(rtt_us);
    uart_load_record(tag,
                     0u,
                     0u,
                     0u,
                     (resp_ret == FS_ERR_MCU_PROT_TIMEOUT) ? 1u : 0u,
                     rtt_us,
                     1);
    return resp_ret;
}

/**
 * @brief 发送 UART 上电测试帧
 */
static int mcu_uart_send_boot_test_frames(void)
{
    // 发送固定测试帧用于上电波形确认。
    static const uint8_t k_test_payloads[MCU_UART_BOOT_TEST_FRAME_NUM][2] = {
        {1u, 0xA1u},
        {1u, 0xA2u},
        {1u, 0xA3u},
    };
    uint8_t tx_frame[MCU_UART_SESSION_HEAD_LEN + MCU_UART_SESSION_MAX_PAYLOAD + MCU_UART_SESSION_CRC_LEN] = {0};
    uint16_t tx_frame_len = 0;
    uint16_t txid;
    uint32_t i;

    if (mcu_uart_open_if_needed() != FS_SUCCESS) {
        return FS_ERR_IO;
    }

    for (i = 0; i < MCU_UART_BOOT_TEST_FRAME_NUM; i++) {
        txid = g_mcu_uart_txid++;
        if (g_mcu_uart_txid == 0u) {
            g_mcu_uart_txid = 1u;
        }

        if (mcu_uart_frame_pack(MCU_UART_MSG_REQ,
                                txid,
                                0u,
                                k_test_payloads[i],
                                sizeof(k_test_payloads[i]),
                                tx_frame,
                                sizeof(tx_frame),
                                &tx_frame_len) != 0) {
            return FS_ERR_BAD_PROTOCOL;
        }

        if (mcu_uart_write_all(tx_frame, tx_frame_len) != FS_SUCCESS) {
            return FS_ERR_MCU_PROT_WRITE;
        }

        g_mcu_uart_diag.tx_frame++;
        g_mcu_uart_diag.tx_bytes += tx_frame_len;

        if (mcu_uart_debug_enabled()) {
            FS_LOGI("UART boot test frame tx: idx=%u txid=%u len=%u", i, txid, tx_frame_len);
        }

        fs_hw_msleep(10);
    }

    return FS_SUCCESS;
}

/* ------------------------------------------------------------------ */
/* TX Thread: 唯一向 UART fd 写入数据的线程                            */
/* ------------------------------------------------------------------ */

static void *mcu_uart_tx_thread(void *arg)
{
    mcu_uart_cmd_t cmd;
    (void)arg;

    FS_LOGI("UART TX thread started");

    while (g_mcu_uart_tx_run) {
        int dq = mcu_uart_cmdq_dequeue(&g_uart_cmdq, &cmd, 100u);
        if (dq == FS_ERR_MCU_PROT_TIMEOUT) {
            continue;
        }
        if (dq != FS_SUCCESS) {
            break;
        }

        if (cmd.msg_type == MCU_UART_CMDQ_MSG_REQ) {
            /* 发送 REQ 并等待对应 RESP，结果写回 *cmd.result */
            uint16_t resp_len = 0u;
            int tx_ret = mcu_uart_send_req_and_wait_resp(cmd.cmd_type,
                                                         cmd.req_data,
                                                         cmd.req_data_len,
                                                         cmd.resp_buf,
                                                         cmd.resp_buf_size,
                                                         &resp_len,
                                                         cmd.timeout_ms);
            if (cmd.resp_len != NULL) {
                *cmd.resp_len = resp_len;
            }
            if (cmd.result != NULL) {
                *cmd.result = tx_ret;
            }
            if (cmd.done_sem != NULL) {
                sem_post(cmd.done_sem);
            }

        } else {
            /* PUSH: 打包并直接发送，无需等待 RESP */
            uint8_t tx_frame[MCU_UART_SESSION_HEAD_LEN
                             + MCU_UART_CMDQ_PUSH_PAYLOAD_MAX
                             + MCU_UART_SESSION_CRC_LEN + 4u];
            uint16_t tx_frame_len = 0u;
            uint16_t cur_txid = g_mcu_uart_txid++;
            uint8_t tag = uart_load_classify_frame(MCU_UART_MSG_PUSH,
                                                    cmd.push_payload,
                                                    cmd.push_payload_len);
            if (g_mcu_uart_txid == 0u) {
                g_mcu_uart_txid = 1u;
            }

            if (mcu_uart_frame_pack(MCU_UART_MSG_PUSH,
                                    cur_txid,
                                    0u,
                                    cmd.push_payload,
                                    cmd.push_payload_len,
                                    tx_frame,
                                    sizeof(tx_frame),
                                    &tx_frame_len) == 0) {
                uint64_t write_start_us;
                uint64_t drain_start_us;
                uint32_t write_us;
                uint32_t drain_us;
                int write_ret;

                g_mcu_uart_diag.tx_frame++;
                g_mcu_uart_diag.tx_bytes += tx_frame_len;
                write_start_us = uart_load_mono_us();
                write_ret = mcu_uart_write_all(tx_frame, tx_frame_len);
                write_us = uart_load_clamp_us(uart_load_mono_us() - write_start_us);
                uart_load_note_tx_block(write_us);
                if (write_ret != FS_SUCCESS) {
                    pthread_mutex_lock(&g_uart_load_mutex);
                    g_uart_load_soc_tx_write_err++;
                    pthread_mutex_unlock(&g_uart_load_mutex);
                }
                uart_load_record(tag,
                                 (write_ret == FS_SUCCESS) ? 1u : 0u,
                                 (write_ret == FS_SUCCESS) ? tx_frame_len : 0u,
                                 0u,
                                 (write_ret == FS_SUCCESS) ? 0u : 1u,
                                 write_us,
                                 1);
                /* Wait for all PUSH bytes to be physically transmitted, then give
                 * the MCU main loop time to process the PUSH before the next frame
                 * arrives.  Without this gap, accumulated PUSH frames are drained
                 * back-to-back just before a REQ, and the MCU drops the REQ because
                 * g_uart_frame_ready is still set from the preceding PUSH. */
                drain_start_us = uart_load_mono_us();
                (void)tcdrain(g_mcu_uart_fd);
                drain_us = uart_load_clamp_us(uart_load_mono_us() - drain_start_us);
                uart_load_note_tcdrain(drain_us);
                uart_load_record(tag, 0u, 0u, 0u, 0u, drain_us, 1);
                fs_hw_msleep(2u);
            }
        }
    }

    FS_LOGI("UART TX thread exiting");
    return NULL;
}

/**
 * @brief 将一条 REQ 命令入队并阻塞等待 TX Thread 完成发送/接收
 *
 * 替代原 mcu_uart_send_req_and_wait_resp 的直接调用，确保所有 UART 写操作
 * 均序列化到 TX Thread，消除多线程并发写 UART 导致的帧字节交叉问题。
 */
static int mcu_uart_cmdq_send_req_wait(fw_link_channel_t channel,
                                       uint8_t tag,
                                       uint8_t cmd_type,
                                       const uint8_t *data,
                                       uint16_t data_len,
                                       uint8_t *resp_buf,
                                       uint16_t resp_buf_size,
                                       uint16_t *resp_len,
                                       uint32_t timeout_ms)
{
    sem_t done_sem;
    int result = FS_ERR_MCU_PROT_TIMEOUT;
    mcu_uart_cmd_t cmd;
    int ret;

    /* TX thread not yet started (e.g. pre-init upgrade version check).
     * Fall back to synchronous direct send — safe because no concurrency
     * exists at this stage. */
    if (g_mcu_uart_tx_tid == 0) {
        return mcu_uart_send_req_and_wait_resp(cmd_type, data, data_len,
                                               resp_buf, resp_buf_size,
                                               resp_len, timeout_ms);
    }

    if (sem_init(&done_sem, 0, 0) != 0) {
        return FS_ERR_IO;
    }

    memset(&cmd, 0, sizeof(cmd));
    cmd.msg_type      = MCU_UART_CMDQ_MSG_REQ;
    cmd.tag           = tag;
    cmd.channel       = (uint8_t)channel;
    cmd.cmd_type      = cmd_type;
    cmd.req_data      = data;
    cmd.req_data_len  = data_len;
    cmd.resp_buf      = resp_buf;
    cmd.resp_buf_size = resp_buf_size;
    cmd.resp_len      = resp_len;
    cmd.timeout_ms    = timeout_ms;
    cmd.result        = &result;
    cmd.done_sem      = &done_sem;

    ret = mcu_uart_cmdq_enqueue(&g_uart_cmdq, &cmd);
    if (ret != FS_SUCCESS) {
        sem_destroy(&done_sem);
        return ret;
    }

    sem_wait(&done_sem);
    sem_destroy(&done_sem);
    return result;
}

static int mcu_prot_soc_fw_link_send_req_wait(fw_link_channel_t channel,
                                              uint8_t tag,
                                              uint8_t cmd_type,
                                              const uint8_t *data,
                                              uint16_t data_len,
                                              uint8_t *resp_buf,
                                              uint16_t resp_buf_size,
                                              uint16_t *resp_len,
                                              uint32_t timeout_ms)
{
    return mcu_uart_cmdq_send_req_wait(channel,
                                       tag,
                                       cmd_type,
                                       data,
                                       data_len,
                                       resp_buf,
                                       resp_buf_size,
                                       resp_len,
                                       timeout_ms);
}

static int mcu_prot_soc_fw_link_send_push(fw_link_channel_t channel,
                                          uint8_t tag,
                                          const uint8_t *payload,
                                          uint16_t payload_len)
{
    mcu_uart_cmd_t cmd;
    int ret;

    if ((payload_len != 0u && payload == NULL)
        || payload_len > MCU_UART_CMDQ_PUSH_PAYLOAD_MAX) {
        return FS_ERR_BAD_PARAMS;
    }

    ret = mcu_uart_open_if_needed();
    if (ret != FS_SUCCESS) {
        return ret;
    }

    memset(&cmd, 0, sizeof(cmd));
    cmd.msg_type = MCU_UART_CMDQ_MSG_PUSH;
    cmd.tag = tag;
    cmd.channel = (uint8_t)channel;
    cmd.push_payload_len = payload_len;
    if (payload_len != 0u) {
        memcpy(cmd.push_payload, payload, payload_len);
    }

    return mcu_uart_cmdq_try_enqueue(&g_uart_cmdq, &cmd);
}

static void mcu_prot_bind_soc_fw_link_transport(void)
{
    static const soc_fw_link_transport_ops_t ops = {
        .send_req_wait = mcu_prot_soc_fw_link_send_req_wait,
        .send_push = mcu_prot_soc_fw_link_send_push,
    };

    soc_fw_link_set_transport_ops(&ops);
}

/**
 * @brief 通过 UART 发送 MCU 协议请求并缓存响应
 * @param cmd_type 协议类型
 * @param data 请求数据
 * @param data_len 请求数据长度
 */
static uint32_t mcu_prot_uart_default_timeout(uint8_t cmd_type)
{
    return (cmd_type == MCU_PROT_TYPE_FLASH) ? MCU_UART_FLASH_REQ_TIMEOUT_MS : MCU_UART_REQ_TIMEOUT_MS;
}

static int mcu_prot_uart_write_with_timeout(uint8_t cmd_type,
                                            uint8_t* data,
                                            uint16_t data_len,
                                            uint32_t timeout_ms)
{
    uint8_t tag;
    fw_link_channel_t channel;
    uint16_t resp_len = 0u;
    int ret;

    if (cmd_type >= MCU_PROT_TYPE_NUM) {
        return FS_ERR_BAD_PARAMS;
    }

    ret = mcu_uart_open_if_needed();
    if (ret != FS_SUCCESS) {
        return ret;
    }

    if (timeout_ms == 0u) {
        timeout_ms = mcu_prot_uart_default_timeout(cmd_type);
    }

    tag = uart_load_classify_cmd(cmd_type, data, data_len);
    channel = uart_load_channel_from_tag(tag);
    mcu_prot_bind_soc_fw_link_transport();
    ret = soc_fw_link_send_req_wait(channel,
                                    tag,
                                    cmd_type,
                                    data,
                                    data_len,
                                    g_mcu_uart_resp_cache[cmd_type],
                                    MCU_PROT_BUFFER_SIZE,
                                    &resp_len,
                                    timeout_ms);
    if (ret != FS_SUCCESS) {
        return ret;
    }

    g_mcu_uart_resp_cache_len[cmd_type] = resp_len;
    return FS_SUCCESS;
}

static int mcu_prot_uart_write(uint8_t cmd_type, uint8_t* data, uint16_t data_len)
{
    return mcu_prot_uart_write_with_timeout(cmd_type, data, data_len, 0u);
}

/**
 * @brief 从 UART 响应缓存读取指定类型数据
 * @param cmd_type 协议类型
 * @param data 输出缓冲区
 * @param data_len 输出缓冲区长度
 */
static int mcu_prot_uart_read(uint8_t cmd_type, uint8_t* data, uint16_t data_len)
{
    uint16_t copy_len;
    uint16_t resp_len = 0;
    int ret;

    if (data == NULL || data_len == 0u || cmd_type >= MCU_PROT_TYPE_NUM) {
        return FS_ERR_BAD_PARAMS;
    }

    ret = mcu_uart_open_if_needed();
    if (ret != FS_SUCCESS) {
        return ret;
    }

    if (cmd_type == MCU_PROT_TYPE_STATUS) {
        /* STATUS 读取：向 MCU 发一个最小 REQ，MCU 返回状态缓冲 */
        uint8_t tag = uart_load_classify_cmd(cmd_type, NULL, 0u);
        fw_link_channel_t channel = uart_load_channel_from_tag(tag);
        mcu_prot_bind_soc_fw_link_transport();
        ret = soc_fw_link_send_req_wait(channel,
                                        tag,
                                        cmd_type,
                                        NULL,
                                        0u,
                                        g_mcu_uart_resp_cache[cmd_type],
                                        MCU_PROT_BUFFER_SIZE,
                                        &resp_len,
                                        MCU_UART_STATUS_TIMEOUT_MS);
        if (ret != FS_SUCCESS) {
            return ret;
        }
        g_mcu_uart_resp_cache_len[cmd_type] = resp_len;
    }

    if (g_mcu_uart_resp_cache_len[cmd_type] == 0u) {
        return FS_ERR_MCU_PROT_READ;
    }

    copy_len = g_mcu_uart_resp_cache_len[cmd_type];
    if (copy_len > data_len) {
        copy_len = data_len;
    }
    memcpy(data, g_mcu_uart_resp_cache[cmd_type], copy_len);

    if (mcu_uart_debug_enabled()) {
        FS_LOGI("UART read cache: type=%u len=%u", cmd_type, copy_len);
    }

    return FS_SUCCESS;
}



int mcu_prot_uart_get_diag(mcu_uart_link_diag_t *diag)
{
    if (diag == NULL) {
        return FS_ERR_BAD_PARAMS;
    }

    *diag = g_mcu_uart_diag;
    return FS_SUCCESS;
}

int mcu_prot_uart_load_get_soc_snapshot(mcu_uart_load_diag_summary_t *summary,
                                        mcu_uart_load_bucket_t *buckets,
                                        uint16_t bucket_count)
{
    mcu_uart_cmdq_diag_t cmdq_diag;
    uint16_t i;

    if (summary == NULL) {
        return FS_ERR_BAD_PARAMS;
    }

    memset(summary, 0, sizeof(*summary));
    mcu_uart_cmdq_get_diag(&g_uart_cmdq, &cmdq_diag);

    pthread_mutex_lock(&g_uart_load_mutex);
    summary->schema_version = MCU_UART_LOAD_DIAG_SCHEMA_VERSION;
    summary->source = MCU_UART_LOAD_DIAG_SOURCE_SOC;
    summary->tag_count = MCU_UART_LOAD_TAG_NUM;
    if (g_uart_load_soc_start_ms != 0u) {
        summary->uptime_ms = (uint32_t)(fs_util_timestamp_ms() - g_uart_load_soc_start_ms);
    }
    summary->baud = MCU_UART_DEFAULT_BAUD;
    summary->rx_frame_ok = g_mcu_uart_diag.rx_frame_ok;
    summary->rx_frame_err = g_mcu_uart_diag.rx_frame_err;
    summary->rx_frame_drop = 0u;
    summary->rx_bytes = g_uart_load_soc_rx_bytes;
    summary->tx_frame = g_mcu_uart_diag.tx_frame;
    summary->tx_bytes = g_mcu_uart_diag.tx_bytes;
    summary->tx_write_err = g_uart_load_soc_tx_write_err;
    summary->rx_timeout = g_mcu_uart_diag.rx_timeout;
    summary->cmdq_depth = cmdq_diag.depth;
    summary->cmdq_peak = cmdq_diag.peak_depth;
    summary->cmdq_req_wait_count = cmdq_diag.req_wait_count;
    summary->cmdq_req_wait_last_us = cmdq_diag.req_wait_last_us;
    summary->cmdq_req_wait_max_us = cmdq_diag.req_wait_max_us;
    summary->cmdq_push_drop = cmdq_diag.push_drop;
    summary->resp_slot_hit = g_mcu_uart_diag.resp_slot_hit;
    summary->resp_slot_store = g_mcu_uart_diag.resp_slot_store;
    summary->push_drop = g_mcu_uart_diag.push_drop;
    summary->tx_block_last_us = g_uart_load_soc_tx_block_last_us;
    summary->tx_block_max_us = g_uart_load_soc_tx_block_max_us;
    summary->tcdrain_last_us = g_uart_load_soc_tcdrain_last_us;
    summary->tcdrain_max_us = g_uart_load_soc_tcdrain_max_us;
    summary->rtt_last_us = g_uart_load_soc_rtt_last_us;
    summary->rtt_max_us = g_uart_load_soc_rtt_max_us;

    if (buckets != NULL) {
        uint16_t copy_count = bucket_count;
        if (copy_count > MCU_UART_LOAD_TAG_NUM) {
            copy_count = MCU_UART_LOAD_TAG_NUM;
        }
        for (i = 0u; i < copy_count; i++) {
            buckets[i] = g_uart_load_soc_buckets[i];
        }
    }
    pthread_mutex_unlock(&g_uart_load_mutex);
    return FS_SUCCESS;
}

static int mcu_prot_uart_load_read_remote(uint8_t page_req,
                                          void *out,
                                          uint16_t out_size,
                                          uint16_t *out_len)
{
    uint8_t rdata[MCU_PROT_READ_COMMON_LEN + sizeof(mcu_uart_load_diag_page_t)] = {0};
    mcu_prot_read_head_t *rh = (mcu_prot_read_head_t *)rdata;
    mcu_prot_cmd_t cmd;
    uint16_t rdlen;
    uint8_t crc;
    int ret;

    if (out == NULL || out_len == NULL) {
        return FS_ERR_BAD_PARAMS;
    }

    memset(&cmd, 0, sizeof(cmd));
    cmd.cmd_dev = MCU_PROT_DEV_MCU;
    cmd.cmd_op = MCU_PROT_OP_READ;
    cmd.sub_cmd0 = (MCU_PROT_MCU_UART_LOAD_DIAG >> 24) & 0xff;
    cmd.sub_cmd1 = (MCU_PROT_MCU_UART_LOAD_DIAG >> 16) & 0xff;
    cmd.sub_cmd2 = (MCU_PROT_MCU_UART_LOAD_DIAG >> 8) & 0xff;
    cmd.sub_cmd3 = (MCU_PROT_MCU_UART_LOAD_DIAG >> 0) & 0xff;
    ((uint8_t *)&cmd.data)[0] = page_req;
    cmd.crc8 = fs_util_crc_8(&cmd, sizeof(cmd) - 1, 0);

    pthread_mutex_lock(&mcu_prot_mutex);
    ret = mcu_prot_write(MCU_PROT_TYPE_NORMAL, (uint8_t *)&cmd, sizeof(cmd));
    if (ret < 0) {
        pthread_mutex_unlock(&mcu_prot_mutex);
        return ret;
    }

    ret = mcu_prot_wait_idle(2, 200);
    if (ret < 0) {
        pthread_mutex_unlock(&mcu_prot_mutex);
        return ret;
    }

    ret = mcu_prot_read(MCU_PROT_TYPE_NORMAL, rdata, sizeof(rdata));
    if (ret < 0) {
        pthread_mutex_unlock(&mcu_prot_mutex);
        return ret;
    }

    rdlen = (uint16_t)(((uint16_t)rh->len0 << 8) | (uint16_t)rh->len1);
    if (rdlen > out_size || rdlen > (uint16_t)(sizeof(rdata) - sizeof(mcu_prot_read_head_t) - 1u)) {
        pthread_mutex_unlock(&mcu_prot_mutex);
        return FS_ERR_BUF_OVERFLOW;
    }

    crc = fs_util_crc_8(rdata + sizeof(mcu_prot_read_head_t), rdlen, 0);
    if (crc != rdata[sizeof(mcu_prot_read_head_t) + rdlen]) {
        pthread_mutex_unlock(&mcu_prot_mutex);
        return FS_ERR_MCU_PROT_CRC;
    }

    memcpy(out, rdata + sizeof(mcu_prot_read_head_t), rdlen);
    *out_len = rdlen;
    pthread_mutex_unlock(&mcu_prot_mutex);
    return FS_SUCCESS;
}

int mcu_prot_get_uart_load_diag(mcu_uart_load_diag_summary_t *summary,
                                mcu_uart_load_bucket_t *buckets,
                                uint16_t bucket_count)
{
    uint16_t out_len = 0u;
    uint16_t page_count;
    uint16_t page_idx;
    int ret;

    if (summary == NULL) {
        return FS_ERR_BAD_PARAMS;
    }

    memset(summary, 0, sizeof(*summary));
    if (buckets != NULL && bucket_count > 0u) {
        memset(buckets, 0, sizeof(buckets[0]) * bucket_count);
    }

    ret = mcu_prot_uart_load_read_remote(MCU_UART_LOAD_DIAG_PAGE_SUMMARY,
                                         summary,
                                         (uint16_t)sizeof(*summary),
                                         &out_len);
    if (ret != FS_SUCCESS) {
        return ret;
    }
    if (out_len != sizeof(*summary)
        || summary->schema_version != MCU_UART_LOAD_DIAG_SCHEMA_VERSION
        || summary->tag_count > MCU_UART_LOAD_TAG_NUM) {
        return FS_ERR_BAD_PROTOCOL;
    }

    if (buckets == NULL || bucket_count == 0u) {
        return FS_SUCCESS;
    }

    page_count = (uint16_t)((summary->tag_count + MCU_UART_LOAD_DIAG_BUCKETS_PER_PAGE - 1u)
               / MCU_UART_LOAD_DIAG_BUCKETS_PER_PAGE);
    for (page_idx = 0u; page_idx < page_count; page_idx++) {
        mcu_uart_load_diag_page_t page;
        uint16_t i;

        memset(&page, 0, sizeof(page));
        out_len = 0u;
        ret = mcu_prot_uart_load_read_remote((uint8_t)page_idx,
                                             &page,
                                             (uint16_t)sizeof(page),
                                             &out_len);
        if (ret != FS_SUCCESS) {
            return ret;
        }
        if (out_len != sizeof(page) || page.first_tag >= MCU_UART_LOAD_TAG_NUM) {
            return FS_ERR_BAD_PROTOCOL;
        }
        for (i = 0u; i < page.tag_count; i++) {
            uint16_t tag = (uint16_t)(page.first_tag + i);
            if (tag >= bucket_count || tag >= MCU_UART_LOAD_TAG_NUM) {
                continue;
            }
            buckets[tag] = page.buckets[i];
        }
    }

    return FS_SUCCESS;
}

int mcu_prot_uart_load_echo(const mcu_uart_load_echo_req_t *req,
                            uint8_t *resp_buf,
                            uint16_t resp_buf_size,
                            uint16_t *resp_len)
{
    uint8_t rdata[MCU_PROT_READ_COMMON_LEN + UART_LOAD_ECHO_LEN_MAX] = {0};
    mcu_prot_read_head_t *rh = (mcu_prot_read_head_t *)rdata;
    mcu_prot_cmd_t cmd;
    uint16_t rdlen;
    uint8_t crc;
    int ret;

    if (req == NULL || resp_len == NULL) {
        return FS_ERR_BAD_PARAMS;
    }
    if (req->resp_len > UART_LOAD_ECHO_LEN_MAX || req->resp_len > resp_buf_size) {
        return FS_ERR_BAD_PARAMS;
    }
    if (req->resp_len > 0u && resp_buf == NULL) {
        return FS_ERR_BAD_PARAMS;
    }

    memset(&cmd, 0, sizeof(cmd));
    cmd.cmd_dev = MCU_PROT_DEV_MCU;
    cmd.cmd_op = MCU_PROT_OP_READ;
    cmd.sub_cmd0 = (MCU_PROT_MCU_UART_LOAD_ECHO >> 24) & 0xff;
    cmd.sub_cmd1 = (MCU_PROT_MCU_UART_LOAD_ECHO >> 16) & 0xff;
    cmd.sub_cmd2 = (MCU_PROT_MCU_UART_LOAD_ECHO >> 8) & 0xff;
    cmd.sub_cmd3 = (MCU_PROT_MCU_UART_LOAD_ECHO >> 0) & 0xff;
    memcpy(&cmd.data, req, sizeof(*req));
    cmd.crc8 = fs_util_crc_8(&cmd, sizeof(cmd) - 1, 0);

    pthread_mutex_lock(&mcu_prot_mutex);
    ret = mcu_prot_write(MCU_PROT_TYPE_NORMAL, (uint8_t *)&cmd, sizeof(cmd));
    if (ret < 0) {
        pthread_mutex_unlock(&mcu_prot_mutex);
        return ret;
    }

    ret = mcu_prot_wait_idle(2, 200);
    if (ret < 0) {
        pthread_mutex_unlock(&mcu_prot_mutex);
        return ret;
    }

    ret = mcu_prot_read(MCU_PROT_TYPE_NORMAL, rdata, sizeof(rdata));
    if (ret < 0) {
        pthread_mutex_unlock(&mcu_prot_mutex);
        return ret;
    }

    rdlen = (uint16_t)(((uint16_t)rh->len0 << 8) | (uint16_t)rh->len1);
    if (rdlen > req->resp_len || rdlen > resp_buf_size) {
        pthread_mutex_unlock(&mcu_prot_mutex);
        return FS_ERR_BUF_OVERFLOW;
    }
    crc = fs_util_crc_8(rdata + sizeof(mcu_prot_read_head_t), rdlen, 0);
    if (crc != rdata[sizeof(mcu_prot_read_head_t) + rdlen]) {
        pthread_mutex_unlock(&mcu_prot_mutex);
        return FS_ERR_MCU_PROT_CRC;
    }

    if (rdlen > 0u) {
        memcpy(resp_buf, rdata + sizeof(mcu_prot_read_head_t), rdlen);
    }
    *resp_len = rdlen;
    pthread_mutex_unlock(&mcu_prot_mutex);
    return FS_SUCCESS;
}

int mcu_prot_send_soc_sync_pub(uint32_t sync_seq, uint64_t t_soc_sync_event_ns, uint8_t soc_sync_state)
{
#if !FS_CFG_FEATURE_MCU_TIMESYNC
    (void)sync_seq;
    (void)t_soc_sync_event_ns;
    (void)soc_sync_state;
    return FS_ERR_UNSUPPORT;
#else
    fs_soc_sync_push_payload_t payload;
    int ret;

    if (sync_seq == 0u) {
        return FS_ERR_BAD_PARAMS;
    }

    payload.topic = FS_SOC_SYNC_PUSH_TOPIC;
    payload.soc_sync_state = soc_sync_state;
    payload.reserved[0] = 0u;
    payload.reserved[1] = 0u;
    payload.sync_seq = sync_seq;
    payload.t_soc_sync_event_ns = t_soc_sync_event_ns;

    /* fire-and-forget PUSH: 使用非阻塞入队。队满时直接丢弃（下一次触发会补发），
     * 避免大量缓存的 trig 事件在启动时持续阻塞 cmdq，饿死 REQ 命令。*/
    mcu_prot_bind_soc_fw_link_transport();
    ret = soc_fw_link_send_push(FW_LINK_CHANNEL_TIME_SYNC,
                                MCU_UART_LOAD_TAG_TIME_SYNC,
                                (const uint8_t *)&payload,
                                (uint16_t)sizeof(payload));
    if (ret != FS_SUCCESS) {
        uart_load_record(MCU_UART_LOAD_TAG_TIME_SYNC, 0u, 0u, 1u, 0u, 0u, 0);
    }
    return ret;
#endif
}

/**
 * @brief 取出最近一帧 PUSH 缓存数据
 * @param buf 输出数据缓冲区
 * @param buf_size 输出缓冲区大小
 * @param out_len 输出数据长度
 * @param seq 输出 PUSH 序号
 * @param drop_cnt 输出丢帧计数
 */
int mcu_prot_uart_pop_push(uint8_t *buf,
                           uint16_t buf_size,
                           uint16_t *out_len,
                           uint32_t *seq,
                           uint32_t *drop_cnt)
{
    if (buf == NULL || out_len == NULL) {
        return FS_ERR_BAD_PARAMS;
    }

    if (g_mcu_uart_push_cache.valid == 0u) {
        return FS_ERR_NOT_FOUND;
    }

    if (buf_size < g_mcu_uart_push_cache.len) {
        return FS_ERR_BUF_OVERFLOW;
    }

    memcpy(buf, g_mcu_uart_push_cache.data, g_mcu_uart_push_cache.len);
    *out_len = g_mcu_uart_push_cache.len;
    if (seq != NULL) {
        *seq = g_mcu_uart_push_cache.seq;
    }
    if (drop_cnt != NULL) {
        *drop_cnt = g_mcu_uart_push_cache.dropped;
    }

    g_mcu_uart_push_cache.valid = 0u;
    return FS_SUCCESS;
}

static uint64_t mcu_imu_read_u64_le(const uint8_t *p)
{
    return (uint64_t)p[0] | ((uint64_t)p[1] << 8) | ((uint64_t)p[2] << 16) | ((uint64_t)p[3] << 24)
           | ((uint64_t)p[4] << 32) | ((uint64_t)p[5] << 40) | ((uint64_t)p[6] << 48) | ((uint64_t)p[7] << 56);
}

static int mcu_imu_decode_compact_v2(const uint8_t *payload,
                                     uint16_t payload_len,
                                     imu_sample_msg_t *samples,
                                     uint16_t max_samples,
                                     uint16_t *out_samples,
                                     uint32_t *seq_opt)
{
    uint8_t count;
    uint8_t quality;
    uint32_t frame_seq;
    uint64_t t_cur;
    uint16_t off;
    uint16_t i;

    if (payload_len < MCU_IMU_PUSH_COMPACT_HEAD_LEN + MCU_IMU_PUSH_COMPACT_SAMPLE_BLOCK0) {
        return FS_ERR_BAD_PROTOCOL;
    }
    if (payload[1] != MCU_IMU_PUSH_COMPACT_VER2) {
        return FS_ERR_BAD_PROTOCOL;
    }

    count = payload[6];
    quality = payload[7];
    if (count == 0u || count > MCU_IMU_PUSH_MAX_SAMPLES) {
        return FS_ERR_BAD_PROTOCOL;
    }
    if (quality != MCU_IMU_PUSH_TIMESTAMP_QUALITY_LOCKED
        && quality != MCU_IMU_PUSH_TIMESTAMP_QUALITY_HOLDOVER) {
        return FS_ERR_BAD_PROTOCOL;
    }
    if (count > max_samples) {
        return FS_ERR_BUF_OVERFLOW;
    }

    frame_seq = (uint32_t)payload[2]
                | ((uint32_t)payload[3] << 8)
                | ((uint32_t)payload[4] << 16)
                | ((uint32_t)payload[5] << 24);
    if (seq_opt != NULL) {
        *seq_opt = frame_seq;
    }

    t_cur = mcu_imu_read_u64_le(&payload[8]);
    off = MCU_IMU_PUSH_COMPACT_HEAD_LEN;

    for (i = 0u; i < count; i++) {
        const uint8_t *p;

        if (i > 0u) {
            uint16_t dt_us;

            if (payload_len < (uint16_t)(off + 2u + 12u)) {
                return FS_ERR_BAD_PROTOCOL;
            }
            dt_us = (uint16_t)payload[off] | ((uint16_t)payload[off + 1u] << 8);
            off = (uint16_t)(off + 2u);
            t_cur += (uint64_t)dt_us * 1000ull;
        }

        if (payload_len < (uint16_t)(off + 12u)) {
            return FS_ERR_BAD_PROTOCOL;
        }
        p = &payload[off];
        samples[i].t_imu_ns = t_cur;
        samples[i].timestamp_quality = quality;
        samples[i].acc_x = (int16_t)((uint16_t)p[0] | ((uint16_t)p[1] << 8));
        samples[i].acc_y = (int16_t)((uint16_t)p[2] | ((uint16_t)p[3] << 8));
        samples[i].acc_z = (int16_t)((uint16_t)p[4] | ((uint16_t)p[5] << 8));
        samples[i].gyro_x = (int16_t)((uint16_t)p[6] | ((uint16_t)p[7] << 8));
        samples[i].gyro_y = (int16_t)((uint16_t)p[8] | ((uint16_t)p[9] << 8));
        samples[i].gyro_z = (int16_t)((uint16_t)p[10] | ((uint16_t)p[11] << 8));
        off = (uint16_t)(off + 12u);
    }

    if (mcu_uart_debug_enabled() && count >= 2u) {
        FS_LOGI("IMU RX compact v2 seq=%u count=%u q=%u s0{t_ns=%llu} s1{t_ns=%llu}",
                frame_seq,
                count,
                quality,
                (unsigned long long)samples[0].t_imu_ns,
                (unsigned long long)samples[1].t_imu_ns);
    }

    *out_samples = count;
    return FS_SUCCESS;
}

/**
 * @brief 解析 PUSH 缓存为 IMU 样本数组
 * @param samples 输出样本数组
 * @param max_samples 输出数组可容纳样本数
 * @param out_samples 输出实际样本数
 * @param seq 输出 IMU 帧头序号（compact v2: Byte2~5）
 * @param drop_cnt 输出丢帧计数
 */
int mcu_prot_uart_pop_imu_batch(imu_sample_msg_t *samples,
                                uint16_t max_samples,
                                uint16_t *out_samples,
                                uint32_t *seq,
                                uint32_t *drop_cnt)
{
#if !FS_CFG_FEATURE_MCU_IMU_PUSH
    if (out_samples != NULL) {
        *out_samples = 0u;
    }
    if (seq != NULL) {
        *seq = 0u;
    }
    if (drop_cnt != NULL) {
        *drop_cnt = 0u;
    }
    (void)samples;
    (void)max_samples;
    return FS_ERR_UNSUPPORT;
#else
    uint8_t payload[MCU_PROT_BUFFER_SIZE] = {0};
    uint16_t payload_len = 0u;
    int ret;

    if (samples == NULL || out_samples == NULL) {
        return FS_ERR_BAD_PARAMS;
    }

    ret = mcu_prot_uart_pop_push(payload,
                                 sizeof(payload),
                                 &payload_len,
                                 NULL,
                                 drop_cnt);
    if (ret != FS_SUCCESS) {
        return ret;
    }

    if (payload_len < 1u) {
        return FS_ERR_BAD_PROTOCOL;
    }

    if (payload[0] == MCU_IMU_PUSH_TOPIC_COMPACT) {
        return mcu_imu_decode_compact_v2(payload, payload_len, samples, max_samples, out_samples, seq);
    }

    return FS_ERR_BAD_PROTOCOL;
#endif
}

static int mcu_uart_imu_forward_enabled(void)
{
    int v = 0;

    if (mcu_uart_read_int_file(MCU_UART_IMU_FORWARD_EN_FILE, &v) != FS_SUCCESS) {
        return 0;
    }

    return (v > 0) ? 1 : 0;
}

static void mcu_uart_consume_imu_push(void)
{
#if !FS_CFG_FEATURE_MCU_IMU_PUSH
    return;
#else
    imu_sample_msg_t samples[MCU_IMU_PUSH_MAX_SAMPLES];
    uint16_t out_samples = 0u;
    mcu_imu_forward_cb_t cb;
    void *user;
    int ret;

    ret = mcu_prot_uart_pop_imu_batch(samples,
                                      (uint16_t)(sizeof(samples) / sizeof(samples[0])),
                                      &out_samples,
                                      NULL,
                                      NULL);
    if (ret != FS_SUCCESS || out_samples == 0u) {
        return;
    }

    mcu_imu_forward_log_tap(samples, out_samples);

    pthread_mutex_lock(&g_imu_forward_mutex);
    cb = g_imu_forward_cb;
    user = g_imu_forward_user;
    pthread_mutex_unlock(&g_imu_forward_mutex);

    if (cb != NULL) {
        cb(samples, out_samples, user);
    }
#endif
}

static int mcu_uart_imu_print_verbose_enabled(void)
{
    int v = 0;

    if (mcu_uart_read_int_file(MCU_UART_IMU_PRINT_FILE, &v) != FS_SUCCESS) {
        return 0;
    }
    return (v > 0) ? 1 : 0;
}

static void mcu_imu_forward_log_tap(const imu_sample_msg_t *samples, uint16_t count)
{
    static uint64_t last_log_ms;
    uint64_t now_ms;
    struct timespec ts;
    int verbose;

    if (samples == NULL || count == 0u) {
        return;
    }

    verbose = mcu_uart_imu_print_verbose_enabled();
    if (!verbose && !mcu_uart_imu_forward_enabled()) {
        return;
    }
    if (!verbose) {
        (void)clock_gettime(CLOCK_MONOTONIC, &ts);
        now_ms = (uint64_t)ts.tv_sec * 1000ull + (uint64_t)ts.tv_nsec / 1000000ull;
        if (last_log_ms != 0u && (now_ms - last_log_ms) < 1000u) {
            return;
        }
        last_log_ms = now_ms;
    }

    if (verbose && count >= 2u) {
        FS_LOGI("IMU n=%u q=%u s0{t_ns=%llu ax=%d ay=%d az=%d gx=%d gy=%d gz=%d} s1{t_ns=%llu ax=%d ay=%d az=%d gx=%d gy=%d gz=%d}",
                (unsigned int)count,
                (unsigned int)samples[0].timestamp_quality,
                (unsigned long long)samples[0].t_imu_ns,
                (int)samples[0].acc_x,
                (int)samples[0].acc_y,
                (int)samples[0].acc_z,
                (int)samples[0].gyro_x,
                (int)samples[0].gyro_y,
                (int)samples[0].gyro_z,
                (unsigned long long)samples[1].t_imu_ns,
                (int)samples[1].acc_x,
                (int)samples[1].acc_y,
                (int)samples[1].acc_z,
                (int)samples[1].gyro_x,
                (int)samples[1].gyro_y,
                (int)samples[1].gyro_z);
    } else {
        FS_LOGI("IMU n=%u first{t_ns=%llu q=%u ax=%d ay=%d az=%d gx=%d gy=%d gz=%d}",
                (unsigned int)count,
                (unsigned long long)samples[0].t_imu_ns,
                (unsigned int)samples[0].timestamp_quality,
                (int)samples[0].acc_x,
                (int)samples[0].acc_y,
                (int)samples[0].acc_z,
                (int)samples[0].gyro_x,
                (int)samples[0].gyro_y,
                (int)samples[0].gyro_z);
    }
}

static void mcu_imu_forward_default_log_cb(const imu_sample_msg_t *samples, uint16_t count, void *user)
{
    (void)samples;
    (void)count;
    (void)user;
}

int mcu_prot_register_imu_forward_cb(mcu_imu_forward_cb_t cb, void *user)
{
#if !FS_CFG_FEATURE_MCU_IMU_PUSH
    (void)cb;
    (void)user;
    return FS_SUCCESS;
#else
    pthread_mutex_lock(&g_imu_forward_mutex);
    g_imu_forward_cb = cb;
    g_imu_forward_user = user;
    pthread_mutex_unlock(&g_imu_forward_mutex);
    return FS_SUCCESS;
#endif
}

/**
 * @brief 协议写接口（当前转发到 UART 实现）
 * @param cmd_type 协议类型
 * @param data 请求数据
 * @param data_len 请求数据长度
 */
static int mcu_prot_write(uint8_t cmd_type, uint8_t* data, uint16_t data_len)
{
    return mcu_prot_uart_write(cmd_type, data, data_len);
}

static int mcu_prot_write_with_timeout(uint8_t cmd_type,
                                       uint8_t* data,
                                       uint16_t data_len,
                                       uint32_t timeout_ms)
{
    return mcu_prot_uart_write_with_timeout(cmd_type, data, data_len, timeout_ms);
}

/**
 * @brief 协议读接口（当前转发到 UART 实现）
 * @param cmd_type 协议类型
 * @param data 输出缓冲区
 * @param data_len 输出缓冲区长度
 */
static int mcu_prot_read(uint8_t cmd_type, uint8_t* data, uint16_t data_len)
{
    return mcu_prot_uart_read(cmd_type, data, data_len);
}

/**
 * @brief 等待 MCU 空闲状态（UART 模式下为空操作）
 * @param period_ms 轮询周期（保留参数）
 * @param timeout_ms 超时时间（保留参数）
 */
static int mcu_prot_wait_idle(uint32_t period_ms, uint32_t timeout_ms)
{
    /* UART 会话层下每次 write 已完成 REQ/RESP 同步，无需 STATUS 轮询。 */
    (void)period_ms;
    (void)timeout_ms;
    return FS_SUCCESS;
}

int mcu_prot_set_lidarchip_reg(uint32_t addr, uint32_t val)
{
    int ret = 0;
    mcu_prot_cmd_t cmd;

    memset(&cmd, 0, sizeof(cmd));
    // FS_LOGD("set FL6X32[%d] 0x%08x = 0x%08x", 0, addr, val);

    cmd.cmd_dev = MCU_PROT_DEV_FL6X32_1;
    cmd.cmd_op = MCU_PROT_OP_WRITE;
    cmd.sub_cmd0 = (addr >> 24) & 0xff;
    cmd.sub_cmd1 = (addr >> 16) & 0xff;
    cmd.sub_cmd2 = (addr >> 8) & 0xff;
    cmd.sub_cmd3 = (addr >> 0) & 0xff;

    cmd.data0 = (val >> 24) & 0xff;
    cmd.data1 = (val >> 16) & 0xff;
    cmd.data2 = (val >> 8) & 0xff;
    cmd.data3 = (val >> 0) & 0xff;

    cmd.crc8 = fs_util_crc_8(&cmd, sizeof(cmd) - 1, 0);

    pthread_mutex_lock(&mcu_prot_mutex);
    ret = mcu_prot_write(MCU_PROT_TYPE_NORMAL, (uint8_t*)&cmd, sizeof(cmd));
    if (ret < 0) {
        FS_LOGE("mcu_prot_write failed: %d", ret);
        pthread_mutex_unlock(&mcu_prot_mutex);
        return ret;
    }
    ret = mcu_prot_wait_idle(2, 200);
    pthread_mutex_unlock(&mcu_prot_mutex);

    if (ret != 0) {
        FS_LOGE("mcu_prot_wait_idle failed: %d", ret);
    }

    return ret;
}

int mcu_prot_lidarchip_data_transparent(uint8_t *wdata, uint32_t wlen, uint8_t *rdata, uint32_t rlen)
{
    int ret = 0;
    uint8_t *buf = NULL;
    uint8_t *rbuf = NULL;
    uint32_t buf_len;
    uint32_t rdata_len;
    mcu_prot_transparent_cmd_head_t *head;
    mcu_prot_read_head_t *rh;
    uint32_t timeout_ms;
    uint16_t rdlen;
    uint8_t crc;

    if (wdata == NULL || wlen == 0) {
        return FS_ERR_BAD_PARAMS;
    }

    // 计算发送缓冲区大小: 头部 + 地址列表 + CRC
    buf_len = sizeof(mcu_prot_batch_cmd_head_t) + wlen + 1;
    buf = (uint8_t *)malloc(buf_len);
    if (buf == NULL) {
        return FS_ERR_NOMEM;
    }

    // 计算接收缓冲区大小: 头部 + 地址+数据列表 + CRC
    rdata_len = MCU_PROT_READ_COMMON_LEN + rlen;
    rbuf = (uint8_t *)malloc(rdata_len);
    if (rbuf == NULL) {
        free(buf);
        return FS_ERR_NOMEM;
    }

    memset(buf, 0, buf_len);
    head = (mcu_prot_transparent_cmd_head_t *)buf;
    head->cmd_dev = MCU_PROT_DEV_FL6X32_1;
    head->cmd_op = MCU_PROT_OP_RW_TRANSPARENT;
    head->wlen = wlen;
    head->rlen = rlen;

    // 复制数据到发送缓冲区
    memcpy(buf + sizeof(mcu_prot_batch_cmd_head_t), wdata, wlen);

    // CRC
    buf[buf_len - 1] = fs_util_crc_8(buf, buf_len - 1, 0);

    // 根据寄存器数量动态调整超时时间
    timeout_ms = 200 + wlen * 1;
    if (timeout_ms > 5000) {
        timeout_ms = 5000;
    }

    pthread_mutex_lock(&mcu_prot_mutex);
    ret = mcu_prot_write(MCU_PROT_TYPE_NORMAL, buf, buf_len);
    if (ret < 0) {
        FS_LOGE("mcu_prot_write batch read failed: %d", ret);
        pthread_mutex_unlock(&mcu_prot_mutex);
        free(buf);
        free(rdata);
        return ret;
    }

    ret = mcu_prot_wait_idle(2, timeout_ms);
    if (ret < 0) {
        pthread_mutex_unlock(&mcu_prot_mutex);
        free(buf);
        free(rdata);
        return ret;
    }
    if ( rdata == NULL || rlen == 0) {
        // 无需读取响应数据
        pthread_mutex_unlock(&mcu_prot_mutex);
        free(buf);
        free(rdata);
        return ret;
    }

    // 读取响应数据
    memset(rbuf, 0, rdata_len);
    ret = mcu_prot_read(MCU_PROT_TYPE_NORMAL, rbuf, rdata_len);
    if (ret < 0) {
        pthread_mutex_unlock(&mcu_prot_mutex);
        free(buf);
        free(rbuf);
        return ret;
    }

    pthread_mutex_unlock(&mcu_prot_mutex);

    // 验证响应数据
    rh = (mcu_prot_read_head_t*)rbuf;
    rdlen = (rh->len0 << 8) | rh->len1;
    crc = fs_util_crc_8(rbuf + sizeof(mcu_prot_read_head_t), rdlen, 0);
    if (crc != rbuf[sizeof(mcu_prot_read_head_t) + rdlen]) {
        FS_LOGE("batch read crc error");
        free(buf);
        free(rbuf);
        return FS_ERR_MCU_PROT_CRC;
    }

    memcpy(rdata, rbuf + sizeof(mcu_prot_read_head_t), rlen);

    free(buf);
    free(rbuf);

    return FS_SUCCESS;
}
int mcu_prot_set_lidarchip_reg_batch(const mcu_prot_batch_reg_t *regs, uint32_t count)
{
    int ret = 0;
    uint8_t *buf = NULL;
    uint32_t buf_len;
    mcu_prot_batch_cmd_head_t *head;
    uint8_t *data_ptr;
    uint32_t i;
    uint32_t timeout_ms;

    if (regs == NULL || count == 0) {
        return FS_ERR_BAD_PARAMS;
    }

    if (count > MCU_PROT_BATCH_MAX_REGS) {
        FS_LOGE("batch count %u exceeds max %d", count, MCU_PROT_BATCH_MAX_REGS);
        return FS_ERR_BAD_PARAMS;
    }

    // 计算缓冲区大小: 头部 + 寄存器数据 + CRC
    buf_len = sizeof(mcu_prot_batch_cmd_head_t) + count * sizeof(mcu_prot_batch_reg_t) + 1;
    buf = (uint8_t *)malloc(buf_len);
    if (buf == NULL) {
        return FS_ERR_NOMEM;
    }

    memset(buf, 0, buf_len);
    head = (mcu_prot_batch_cmd_head_t *)buf;
    head->cmd_dev = MCU_PROT_DEV_FL6X32_1;
    head->cmd_op = MCU_PROT_OP_WRITE_BATCH;
    head->reg_count = count;
    head->reserved = 0;

    // 填充寄存器数据（大端序）
    data_ptr = buf + sizeof(mcu_prot_batch_cmd_head_t);
    for (i = 0; i < count; i++) {
        // addr (big endian)
        data_ptr[0] = (regs[i].addr >> 24) & 0xff;
        data_ptr[1] = (regs[i].addr >> 16) & 0xff;
        data_ptr[2] = (regs[i].addr >> 8) & 0xff;
        data_ptr[3] = (regs[i].addr >> 0) & 0xff;
        // data (big endian)
        data_ptr[4] = (regs[i].data >> 24) & 0xff;
        data_ptr[5] = (regs[i].data >> 16) & 0xff;
        data_ptr[6] = (regs[i].data >> 8) & 0xff;
        data_ptr[7] = (regs[i].data >> 0) & 0xff;
        data_ptr += 8;
    }

    // CRC
    buf[buf_len - 1] = fs_util_crc_8(buf, buf_len - 1, 0);

    // 根据寄存器数量动态调整超时时间
    // 假设每个寄存器写入约需 0.1ms（SPI 传输），再加一些余量
    timeout_ms = 200 + count * 1;
    if (timeout_ms > 5000) {
        timeout_ms = 5000;
    }

    pthread_mutex_lock(&mcu_prot_mutex);
    ret = mcu_prot_write(MCU_PROT_TYPE_NORMAL, buf, buf_len);
    if (ret < 0) {
        FS_LOGE("mcu_prot_write batch failed: %d", ret);
        pthread_mutex_unlock(&mcu_prot_mutex);
        free(buf);
        return ret;
    }
    ret = mcu_prot_wait_idle(2, timeout_ms);
    pthread_mutex_unlock(&mcu_prot_mutex);

    free(buf);

    if (ret != 0) {
        FS_LOGE("mcu_prot_wait_idle batch failed: %d", ret);
    }

    return ret;
}

int mcu_prot_get_lidarchip_reg_batch(mcu_prot_batch_reg_t *regs, uint32_t count)
{
    int ret = 0;
    uint8_t *buf = NULL;
    uint8_t *rdata = NULL;
    uint32_t buf_len;
    uint32_t rdata_len;
    mcu_prot_batch_cmd_head_t *head;
    mcu_prot_read_head_t *rh;
    uint8_t *data_ptr;
    uint8_t *read_ptr;
    uint32_t i;
    uint32_t timeout_ms;
    uint16_t rdlen;
    uint8_t crc;

    if (regs == NULL || count == 0) {
        return FS_ERR_BAD_PARAMS;
    }

    if (count > MCU_PROT_BATCH_MAX_REGS) {
        FS_LOGE("batch count %u exceeds max %d", count, MCU_PROT_BATCH_MAX_REGS);
        return FS_ERR_BAD_PARAMS;
    }

    // 计算发送缓冲区大小: 头部 + 地址列表 + CRC
    buf_len = sizeof(mcu_prot_batch_cmd_head_t) + count * sizeof(uint32_t) + 1;
    buf = (uint8_t *)malloc(buf_len);
    if (buf == NULL) {
        return FS_ERR_NOMEM;
    }

    // 计算接收缓冲区大小: 头部 + 地址+数据列表 + CRC
    rdata_len = MCU_PROT_READ_COMMON_LEN + count * 8;
    rdata = (uint8_t *)malloc(rdata_len);
    if (rdata == NULL) {
        free(buf);
        return FS_ERR_NOMEM;
    }

    memset(buf, 0, buf_len);
    head = (mcu_prot_batch_cmd_head_t *)buf;
    head->cmd_dev = MCU_PROT_DEV_FL6X32_1;
    head->cmd_op = MCU_PROT_OP_READ_BATCH;
    head->reg_count = count;
    head->reserved = 0;

    // 填充寄存器地址列表（大端序）
    data_ptr = buf + sizeof(mcu_prot_batch_cmd_head_t);
    for (i = 0; i < count; i++) {
        data_ptr[0] = (regs[i].addr >> 24) & 0xff;
        data_ptr[1] = (regs[i].addr >> 16) & 0xff;
        data_ptr[2] = (regs[i].addr >> 8) & 0xff;
        data_ptr[3] = (regs[i].addr >> 0) & 0xff;
        data_ptr += 4;
    }

    // CRC
    buf[buf_len - 1] = fs_util_crc_8(buf, buf_len - 1, 0);

    // 根据寄存器数量动态调整超时时间
    timeout_ms = 200 + count * 1;
    if (timeout_ms > 5000) {
        timeout_ms = 5000;
    }

    pthread_mutex_lock(&mcu_prot_mutex);
    ret = mcu_prot_write(MCU_PROT_TYPE_NORMAL, buf, buf_len);
    if (ret < 0) {
        FS_LOGE("mcu_prot_write batch read failed: %d", ret);
        pthread_mutex_unlock(&mcu_prot_mutex);
        free(buf);
        free(rdata);
        return ret;
    }

    ret = mcu_prot_wait_idle(2, timeout_ms);
    if (ret < 0) {
        pthread_mutex_unlock(&mcu_prot_mutex);
        free(buf);
        free(rdata);
        return ret;
    }

    // 读取响应数据
    memset(rdata, 0, rdata_len);
    ret = mcu_prot_read(MCU_PROT_TYPE_NORMAL, rdata, rdata_len);
    if (ret < 0) {
        pthread_mutex_unlock(&mcu_prot_mutex);
        free(buf);
        free(rdata);
        return ret;
    }

    pthread_mutex_unlock(&mcu_prot_mutex);

    // 验证响应数据
    rh = (mcu_prot_read_head_t*)rdata;
    rdlen = (rh->len0 << 8) | rh->len1;
    crc = fs_util_crc_8(rdata + sizeof(mcu_prot_read_head_t), rdlen, 0);
    if (crc != rdata[sizeof(mcu_prot_read_head_t) + rdlen]) {
        FS_LOGE("batch read crc error");
        free(buf);
        free(rdata);
        return FS_ERR_MCU_PROT_CRC;
    }

    // 解析读取的数据（地址+数据对）
    read_ptr = rdata + sizeof(mcu_prot_read_head_t);
    for (i = 0; i < count; i++) {
        uint32_t addr = (read_ptr[0] << 24) | (read_ptr[1] << 16) | (read_ptr[2] << 8) | read_ptr[3];
        uint32_t data = (read_ptr[4] << 24) | (read_ptr[5] << 16) | (read_ptr[6] << 8) | read_ptr[7];

        if (addr != regs[i].addr) {
            FS_LOGE("batch read addr mismatch: expected 0x%08x, got 0x%08x", regs[i].addr, addr);
            free(buf);
            free(rdata);
            return FS_ERR_INTERNAL;
        }

        regs[i].data = data;
        read_ptr += 8;
    }

    free(buf);
    free(rdata);

    return FS_SUCCESS;
}

int mcu_prot_get_lidarchip_reg(uint32_t addr, uint32_t* val)
{
    int ret = 0;
    uint8_t rdata[MCU_PROT_READ_LIDARCHIP_LEN];
    uint16_t rdlen;
    mcu_prot_read_head_t* rh = (mcu_prot_read_head_t*)rdata;
    mcu_prot_cmd_t cmd;
    uint8_t crc;

    memset(&cmd, 0, sizeof(cmd));

    cmd.cmd_dev = MCU_PROT_DEV_FL6X32_1;
    cmd.cmd_op = MCU_PROT_OP_READ;
    cmd.sub_cmd0 = (addr >> 24) & 0xff;
    cmd.sub_cmd1 = (addr >> 16) & 0xff;
    cmd.sub_cmd2 = (addr >> 8) & 0xff;
    cmd.sub_cmd3 = (addr >> 0) & 0xff;

    cmd.crc8 = fs_util_crc_8(&cmd, sizeof(cmd) - 1, 0);

    pthread_mutex_lock(&mcu_prot_mutex);
    ret = mcu_prot_write(MCU_PROT_TYPE_NORMAL, (uint8_t*)&cmd, sizeof(cmd));
    if (ret < 0) {
        pthread_mutex_unlock(&mcu_prot_mutex);
        return ret;
    }

    ret = mcu_prot_wait_idle(2, 200);
    if (ret < 0) {
        pthread_mutex_unlock(&mcu_prot_mutex);
        return ret;
    }

    memset(&rdata[0], 0, sizeof(rdata));
    ret = mcu_prot_read(MCU_PROT_TYPE_NORMAL, &rdata[0], sizeof(rdata));
    if (ret < 0) {
        pthread_mutex_unlock(&mcu_prot_mutex);
        return ret;
    }

    rdlen = (rh->len0 << 8) | rh->len1;
    crc = fs_util_crc_8(rdata + sizeof(mcu_prot_read_head_t), rdlen, 0);
    if (crc != rdata[sizeof(mcu_prot_read_head_t) + rdlen]) {
        ret = FS_ERR_MCU_PROT_CRC;
        pthread_mutex_unlock(&mcu_prot_mutex);
        return ret;
    }
    *val = (rdata[8] << 24) | (rdata[9] << 16) | (rdata[10] << 8) | rdata[11];

    pthread_mutex_unlock(&mcu_prot_mutex);

    // FS_LOGI("get_lidarchip_reg 0x%08x", *val);
    // FS_LOGD("get FL6X32[%d] 0x%08x = 0x%08x", 0, addr, *val);

    return ret;
}

int mcu_prot_lidarchip_hw_power_ctrl(uint8_t dev_id, uint8_t power_on)
{
    int ret = 0;
    mcu_prot_cmd_t cmd;

    memset(&cmd, 0, sizeof(cmd));

    cmd.cmd_dev  = dev_id;
    cmd.cmd_op   = MCU_PROT_OP_HW_POWEROFF;
    cmd.sub_cmd0 = 0;
    cmd.sub_cmd1 = 0;
    cmd.sub_cmd2 = 0;
    cmd.sub_cmd3 = 0;
    cmd.data0    = power_on;
    cmd.data1    = 0;
    cmd.data2    = 0;
    cmd.data3    = 0;

    cmd.crc8 = fs_util_crc_8(&cmd, sizeof(cmd) - 1, 0);

    pthread_mutex_lock(&mcu_prot_mutex);
    ret = mcu_prot_write(MCU_PROT_TYPE_NORMAL, (uint8_t *)&cmd, sizeof(cmd));
    if (ret < 0) {
        pthread_mutex_unlock(&mcu_prot_mutex);
        FS_LOGE("mcu_prot_write HW_RESET failed: %d", ret);
        return ret;
    }

    // 给 MCU 处理复位和拉低/拉高控制线的时间
    ret = mcu_prot_wait_idle(2, 500);
    pthread_mutex_unlock(&mcu_prot_mutex);

    if (ret != 0) {
        FS_LOGE("FL6X32 hardware reset timeout/failed: %d", ret);
    }

    return ret;
}

int mcu_prot_lidarchip_hw_reset(uint8_t dev_id)
{
    int ret = 0;
    mcu_prot_cmd_t cmd;

    memset(&cmd, 0, sizeof(cmd));

    cmd.cmd_dev  = dev_id;
    cmd.cmd_op   = MCU_PROT_OP_HW_RESET;
    cmd.sub_cmd0 = 0;
    cmd.sub_cmd1 = 0;
    cmd.sub_cmd2 = 0;
    cmd.sub_cmd3 = 0;
    cmd.data0    = 0;
    cmd.data1    = 0;
    cmd.data2    = 0;
    cmd.data3    = 0;

    cmd.crc8 = fs_util_crc_8(&cmd, sizeof(cmd) - 1, 0);

    pthread_mutex_lock(&mcu_prot_mutex);
    ret = mcu_prot_write(MCU_PROT_TYPE_NORMAL, (uint8_t *)&cmd, sizeof(cmd));
    if (ret < 0) {
        pthread_mutex_unlock(&mcu_prot_mutex);
        FS_LOGE("mcu_prot_write HW_RESET failed: %d", ret);
        return ret;
    }

    // 给 MCU 处理复位和拉低/拉高控制线的时间
    ret = mcu_prot_wait_idle(2, 500);
    pthread_mutex_unlock(&mcu_prot_mutex);

    if (ret != 0) {
        FS_LOGE("FL6X32 hardware reset timeout/failed: %d", ret);
    }

    return ret;
}
#if 0
int mcu_prot_set_lidarchip_reg_continue(uint32_t addr, uint32_t *val, uint32_t reg_cnt)
{
    int ret = 0;
    mcu_prot_cmd_t cmd;

    memset(&cmd, 0, sizeof(cmd));

    cmd.cmd_dev = MCU_PROT_DEV_FL6X32_1;
    cmd.cmd_op = MCU_PROT_OP_WRITE_CONTINUE;
    cmd.sub_cmd0 = (addr >> 24) & 0xff;
    cmd.sub_cmd1 = (addr >> 16) & 0xff;
    cmd.sub_cmd2 = (addr >> 8) & 0xff;
    cmd.sub_cmd3 = (addr >> 0) & 0xff;

    cmd.data0 = (reg_cnt >> 24) & 0xff;
    cmd.data1 = (reg_cnt >> 16) & 0xff;
    cmd.data2 = (reg_cnt >> 8) & 0xff;
    cmd.data3 = (reg_cnt >> 0) & 0xff;

    cmd.crc8 = fs_util_crc_8(&cmd, sizeof(cmd) - 1, 0);

    pthread_mutex_lock(&mcu_prot_mutex);
    mcu_prot_write(MCU_PROT_TYPE_NORMAL, (uint8_t*)&cmd, sizeof(cmd));
    ret = mcu_prot_wait_idle(2, 200);
    pthread_mutex_unlock(&mcu_prot_mutex);

    return ret;
}
int mcu_prot_get_lidarchip_reg_continue(uint32_t addr, uint32_t* val, uint32_t reg_cnt)
{
    int ret = 0;
    // uint8_t rdata[MCU_PROT_READ_LIDARCHIP_LEN];
    uint8_t *rdata = malloc(MCU_PROT_READ_LIDARCHIP_LEN + reg_cnt * 4);
    uint16_t rdlen;
    mcu_prot_read_head_t* rh = (mcu_prot_read_head_t*)rdata;
    mcu_prot_cmd_t cmd;
    uint8_t crc;

    memset(&cmd, 0, sizeof(cmd));

    cmd.cmd_dev = MCU_PROT_DEV_FL6X32_1;
    cmd.cmd_op = MCU_PROT_OP_READ_CONTINUE;
    cmd.sub_cmd0 = (addr >> 24) & 0xff;
    cmd.sub_cmd1 = (addr >> 16) & 0xff;
    cmd.sub_cmd2 = (addr >> 8) & 0xff;
    cmd.sub_cmd3 = (addr >> 0) & 0xff;
    cmd.data0    = (reg_cnt >> 24) & 0xff;
    cmd.data1    = (reg_cnt >> 16) & 0xff;
    cmd.data2    = (reg_cnt >> 8) & 0xff;
    cmd.data3    = (reg_cnt >> 0) & 0xff;

    cmd.crc8 = fs_util_crc_8(&cmd, sizeof(cmd) - 1, 0);

    pthread_mutex_lock(&mcu_prot_mutex);
    ret = mcu_prot_write(MCU_PROT_TYPE_NORMAL, (uint8_t*)&cmd, sizeof(cmd));
    if (ret < 0) {
        pthread_mutex_unlock(&mcu_prot_mutex);
        return ret;
    }

    ret = mcu_prot_wait_idle(2, 200);
    if (ret < 0) {
        pthread_mutex_unlock(&mcu_prot_mutex);
        return ret;
    }

    memset(&rdata[0], 0, sizeof(rdata));
    ret = mcu_prot_read(MCU_PROT_TYPE_NORMAL, &rdata[0], sizeof(rdata));
    if (ret < 0) {
        pthread_mutex_unlock(&mcu_prot_mutex);
        return ret;
    }

    rdlen = (rh->len0 << 8) | rh->len1;
    crc = fs_util_crc_8(rdata + sizeof(mcu_prot_read_head_t), rdlen, 0);
    if (crc != rdata[sizeof(mcu_prot_read_head_t) + rdlen]) {
        ret = -1;
        pthread_mutex_unlock(&mcu_prot_mutex);
        return ret;
    }
    *val = (rdata[8] << 24) | (rdata[9] << 16) | (rdata[10] << 8) | rdata[11];

    pthread_mutex_unlock(&mcu_prot_mutex);

    printf("get_lidarchip_reg 0x%08x\n", *val);

    return ret;
}
#endif
int mcu_prot_set_fl9003_reg(uint8_t dev, uint32_t addr, uint32_t val)
{
    int ret = 0;
    mcu_prot_cmd_t cmd;

    memset(&cmd, 0, sizeof(cmd));

    if (dev > MCU_PROT_DEV_FL9003_ALL - MCU_PROT_DEV_FL9003_1) {
        return -1;
    }

    // FS_LOGD("set fl9003[%d] 0x%08x = 0x%08x", dev, addr, val);
    cmd.cmd_dev = dev + MCU_PROT_DEV_FL9003_1;
    cmd.cmd_op = MCU_PROT_OP_WRITE;
    cmd.sub_cmd0 = (addr >> 24) & 0xff;
    cmd.sub_cmd1 = (addr >> 16) & 0xff;
    cmd.sub_cmd2 = (addr >> 8) & 0xff;
    cmd.sub_cmd3 = (addr >> 0) & 0xff;

    cmd.data0 = (val >> 24) & 0xff;
    cmd.data1 = (val >> 16) & 0xff;
    cmd.data2 = (val >> 8) & 0xff;
    cmd.data3 = (val >> 0) & 0xff;

    cmd.crc8 = fs_util_crc_8(&cmd, sizeof(cmd) - 1, 0);

    pthread_mutex_lock(&mcu_prot_mutex);
    ret = mcu_prot_write(MCU_PROT_TYPE_NORMAL, (uint8_t*)&cmd, sizeof(cmd));
    if (ret < 0) {
        pthread_mutex_unlock(&mcu_prot_mutex);
        return ret;
    }
    ret = mcu_prot_wait_idle(2, 200);
    pthread_mutex_unlock(&mcu_prot_mutex);

    return ret;
}

int mcu_prot_get_fl9003_reg(uint8_t dev, uint32_t addr, uint32_t* val)
{
    int ret = 0;
    uint8_t rdata[MCU_PROT_READ_FL9003_LEN];
    uint16_t rdlen;
    mcu_prot_read_head_t* rh = (mcu_prot_read_head_t*)rdata;
    mcu_prot_cmd_t cmd;
    uint8_t crc;

    memset(&cmd, 0, sizeof(cmd));

    if (dev > MCU_PROT_DEV_FL9003_ALL - MCU_PROT_DEV_FL9003_1) {
        return -1;
    }

    cmd.cmd_dev = dev + MCU_PROT_DEV_FL9003_1;
    cmd.cmd_op = MCU_PROT_OP_READ;
    cmd.sub_cmd0 = (addr >> 24) & 0xff;
    cmd.sub_cmd1 = (addr >> 16) & 0xff;
    cmd.sub_cmd2 = (addr >> 8) & 0xff;
    cmd.sub_cmd3 = (addr >> 0) & 0xff;

    cmd.crc8 = fs_util_crc_8(&cmd, sizeof(cmd) - 1, 0);

    pthread_mutex_lock(&mcu_prot_mutex);
    ret = mcu_prot_write(MCU_PROT_TYPE_NORMAL, (uint8_t*)&cmd, sizeof(cmd));
    if (ret < 0) {
        pthread_mutex_unlock(&mcu_prot_mutex);
        return ret;
    }

    ret = mcu_prot_wait_idle(2, 200);
    if (ret < 0) {
        pthread_mutex_unlock(&mcu_prot_mutex);
        return ret;
    }

    memset(&rdata[0], 0, sizeof(rdata));
    ret = mcu_prot_read(MCU_PROT_TYPE_NORMAL, &rdata[0], sizeof(rdata));
    if (ret < 0) {
        pthread_mutex_unlock(&mcu_prot_mutex);
        return ret;
    }

    rdlen = (rh->len0 << 8) | rh->len1;
    crc = fs_util_crc_8(rdata + sizeof(mcu_prot_read_head_t), rdlen, 0);
    if (crc != rdata[sizeof(mcu_prot_read_head_t) + rdlen]) {
        ret = FS_ERR_MCU_PROT_CRC;
        pthread_mutex_unlock(&mcu_prot_mutex);
        return ret;
    }
    *val = (rdata[8] << 24) | (rdata[9] << 16) | (rdata[10] << 8) | rdata[11];

    pthread_mutex_unlock(&mcu_prot_mutex);

    // FS_LOGD("get fl9003[%d] 0x%08x = 0x%08x", dev, addr, *val);

    return ret;
}

int mcu_prot_set_fl9103_reg(uint8_t dev, uint32_t addr, uint32_t val)
{
    int ret = 0;
    mcu_prot_cmd_t cmd;

    memset(&cmd, 0, sizeof(cmd));

    if (dev > MCU_PROT_DEV_FL9103_ALL - MCU_PROT_DEV_FL9103_1) {
        return -1;
    }

    // FS_LOGD("set fl9103[%d] 0x%08x = 0x%08x", dev, addr, val);
    cmd.cmd_dev = dev + MCU_PROT_DEV_FL9103_1;
    cmd.cmd_op = MCU_PROT_OP_WRITE;
    cmd.sub_cmd0 = (addr >> 24) & 0xff;
    cmd.sub_cmd1 = (addr >> 16) & 0xff;
    cmd.sub_cmd2 = (addr >> 8) & 0xff;
    cmd.sub_cmd3 = (addr >> 0) & 0xff;

    cmd.data0 = (val >> 24) & 0xff;
    cmd.data1 = (val >> 16) & 0xff;
    cmd.data2 = (val >> 8) & 0xff;
    cmd.data3 = (val >> 0) & 0xff;

    cmd.crc8 = fs_util_crc_8(&cmd, sizeof(cmd) - 1, 0);

    pthread_mutex_lock(&mcu_prot_mutex);
    ret = mcu_prot_write(MCU_PROT_TYPE_NORMAL, (uint8_t*)&cmd, sizeof(cmd));
    if (ret < 0) {
        pthread_mutex_unlock(&mcu_prot_mutex);
        return ret;
    }
    ret = mcu_prot_wait_idle(2, 200);
    pthread_mutex_unlock(&mcu_prot_mutex);

    return ret;
}

int mcu_prot_get_fl9103_reg(uint8_t dev, uint32_t addr, uint32_t* val)
{
    int ret = 0;
    uint8_t rdata[MCU_PROT_READ_FL9103_LEN];
    uint16_t rdlen;
    mcu_prot_read_head_t* rh = (mcu_prot_read_head_t*)rdata;
    mcu_prot_cmd_t cmd;
    uint8_t crc;

    memset(&cmd, 0, sizeof(cmd));

    if (dev > MCU_PROT_DEV_FL9103_ALL - MCU_PROT_DEV_FL9103_1) {
        return -1;
    }

    cmd.cmd_dev = dev + MCU_PROT_DEV_FL9103_1;
    cmd.cmd_op = MCU_PROT_OP_READ;
    cmd.sub_cmd0 = (addr >> 24) & 0xff;
    cmd.sub_cmd1 = (addr >> 16) & 0xff;
    cmd.sub_cmd2 = (addr >> 8) & 0xff;
    cmd.sub_cmd3 = (addr >> 0) & 0xff;

    cmd.crc8 = fs_util_crc_8(&cmd, sizeof(cmd) - 1, 0);

    pthread_mutex_lock(&mcu_prot_mutex);
    ret = mcu_prot_write(MCU_PROT_TYPE_NORMAL, (uint8_t*)&cmd, sizeof(cmd));
    if (ret < 0) {
        pthread_mutex_unlock(&mcu_prot_mutex);
        return ret;
    }

    ret = mcu_prot_wait_idle(2, 200);
    if (ret < 0) {
        pthread_mutex_unlock(&mcu_prot_mutex);
        return ret;
    }

    memset(&rdata[0], 0, sizeof(rdata));
    ret = mcu_prot_read(MCU_PROT_TYPE_NORMAL, &rdata[0], sizeof(rdata));
    if (ret < 0) {
        pthread_mutex_unlock(&mcu_prot_mutex);
        return ret;
    }

    rdlen = (rh->len0 << 8) | rh->len1;
    crc = fs_util_crc_8(rdata + sizeof(mcu_prot_read_head_t), rdlen, 0);
    if (crc != rdata[sizeof(mcu_prot_read_head_t) + rdlen]) {
        ret = FS_ERR_MCU_PROT_CRC;
        pthread_mutex_unlock(&mcu_prot_mutex);
        return ret;
    }
    *val = (rdata[8] << 24) | (rdata[9] << 16) | (rdata[10] << 8) | rdata[11];

    pthread_mutex_unlock(&mcu_prot_mutex);

    // FS_LOGD("get fl9103[%d] 0x%08x = 0x%08x", dev, addr, *val);

    return ret;
}

int mcu_prot_set_workmode(uint32_t workmode, bool force)
{
    int ret = 0;
    mcu_prot_cmd_t cmd;

    memset(&cmd, 0, sizeof(cmd));

    cmd.cmd_dev = MCU_PROT_DEV_MCU;
    cmd.cmd_op = MCU_PROT_OP_WRITE;
    cmd.sub_cmd0 = (MCU_PROT_MCU_WORK_MODE >> 24) & 0xff;
    cmd.sub_cmd1 = (MCU_PROT_MCU_WORK_MODE >> 16) & 0xff;
    cmd.sub_cmd2 = (MCU_PROT_MCU_WORK_MODE >> 8) & 0xff;
    cmd.sub_cmd3 = (MCU_PROT_MCU_WORK_MODE >> 0) & 0xff;

    ((uint8_t *)&cmd.data)[0] = (workmode >> 24) & 0xFF;
    ((uint8_t *)&cmd.data)[1] = (workmode >> 16) & 0xFF;
    ((uint8_t *)&cmd.data)[2] = (workmode >>  8) & 0xFF;
    ((uint8_t *)&cmd.data)[3] = (workmode >>  0) & 0xFF;
    ((uint8_t *)&cmd.data)[4] =  force;

    cmd.crc8 = fs_util_crc_8(&cmd, sizeof(cmd) - 1, 0);

    pthread_mutex_lock(&mcu_prot_mutex);
    ret = mcu_prot_write(MCU_PROT_TYPE_NORMAL, (uint8_t*)&cmd, sizeof(cmd));
    if (ret < 0) {
        pthread_mutex_unlock(&mcu_prot_mutex);
        return ret;
    }
    ret = mcu_prot_wait_idle(2, 200);
    pthread_mutex_unlock(&mcu_prot_mutex);

    return ret;
}

int mcu_prot_set_vpp(float vpp, uint32_t *o_dac_code)
{
    int ret = 0;
    uint8_t rdata[MCU_PROT_READ_LIDARCHIP_LEN];
    uint16_t rdlen;
    mcu_prot_read_head_t* rh = (mcu_prot_read_head_t*)rdata;
    mcu_prot_cmd_t cmd;
    uint32_t vpp_int;
    uint8_t crc;

    memset(&cmd, 0, sizeof(cmd));
    memcpy(&vpp_int, &vpp, sizeof(vpp));

    cmd.cmd_dev = MCU_PROT_DEV_MCU;
    cmd.cmd_op = MCU_PROT_OP_READ;
    cmd.sub_cmd0 = (MCU_PROT_MCU_VPP >> 24) & 0xff;
    cmd.sub_cmd1 = (MCU_PROT_MCU_VPP >> 16) & 0xff;
    cmd.sub_cmd2 = (MCU_PROT_MCU_VPP >> 8) & 0xff;
    cmd.sub_cmd3 = (MCU_PROT_MCU_VPP >> 0) & 0xff;

    cmd.data0 = (vpp_int >> 24) & 0xff;
    cmd.data1 = (vpp_int >> 16) & 0xff;
    cmd.data2 = (vpp_int >> 8) & 0xff;
    cmd.data3 = (vpp_int >> 0) & 0xff;

    cmd.crc8 = fs_util_crc_8(&cmd, sizeof(cmd) - 1, 0);

    pthread_mutex_lock(&mcu_prot_mutex);
    ret = mcu_prot_write(MCU_PROT_TYPE_NORMAL, (uint8_t*)&cmd, sizeof(cmd));
    if (ret < 0) {
        pthread_mutex_unlock(&mcu_prot_mutex);
        return ret;
    }
    ret = mcu_prot_wait_idle(2, 200);
    if (ret < 0) {
        pthread_mutex_unlock(&mcu_prot_mutex);
        return ret;
    }

    memset(&rdata[0], 0, sizeof(rdata));
    ret = mcu_prot_read(MCU_PROT_TYPE_NORMAL, &rdata[0], sizeof(rdata));
    if (ret < 0) {
        pthread_mutex_unlock(&mcu_prot_mutex);
        return ret;
    }

    rdlen = (rh->len0 << 8) | rh->len1;
    if (rdlen < 4 || rdlen > (sizeof(rdata) - sizeof(mcu_prot_read_head_t) - 1)) {
        pthread_mutex_unlock(&mcu_prot_mutex);
        return FS_ERR_MCU_PROT_CRC;
    }

    crc = fs_util_crc_8(rdata + sizeof(mcu_prot_read_head_t), rdlen, 0);
    if (crc != rdata[sizeof(mcu_prot_read_head_t) + rdlen]) {
        ret = FS_ERR_MCU_PROT_CRC;
        pthread_mutex_unlock(&mcu_prot_mutex);
        return ret;
    }

    if (o_dac_code != NULL) {
        *o_dac_code = (rdata[8] << 24) | (rdata[9] << 16) | (rdata[10] << 8) | rdata[11];
    }

    pthread_mutex_unlock(&mcu_prot_mutex);

    return ret;
}

int mcu_prot_set_vbd(float vbd, uint32_t *o_dac_code)
{
    int ret = 0;
    uint8_t rdata[MCU_PROT_READ_LIDARCHIP_LEN];
    uint16_t rdlen;
    mcu_prot_read_head_t* rh = (mcu_prot_read_head_t*)rdata;
    mcu_prot_cmd_t cmd;
    uint32_t vbd_int;
    uint8_t crc;

    memset(&cmd, 0, sizeof(cmd));
    memcpy(&vbd_int, &vbd, sizeof(vbd));

    cmd.cmd_dev = MCU_PROT_DEV_MCU;
    cmd.cmd_op = MCU_PROT_OP_READ;
    cmd.sub_cmd0 = (MCU_PROT_MCU_VBD >> 24) & 0xff;
    cmd.sub_cmd1 = (MCU_PROT_MCU_VBD >> 16) & 0xff;
    cmd.sub_cmd2 = (MCU_PROT_MCU_VBD >> 8) & 0xff;
    cmd.sub_cmd3 = (MCU_PROT_MCU_VBD >> 0) & 0xff;

    cmd.data0 = (vbd_int >> 24) & 0xff;
    cmd.data1 = (vbd_int >> 16) & 0xff;
    cmd.data2 = (vbd_int >> 8) & 0xff;
    cmd.data3 = (vbd_int >> 0) & 0xff;

    cmd.crc8 = fs_util_crc_8(&cmd, sizeof(cmd) - 1, 0);

    pthread_mutex_lock(&mcu_prot_mutex);
    ret = mcu_prot_write(MCU_PROT_TYPE_NORMAL, (uint8_t*)&cmd, sizeof(cmd));
    if (ret < 0) {
        pthread_mutex_unlock(&mcu_prot_mutex);
        return ret;
    }
    ret = mcu_prot_wait_idle(2, 200);
    if (ret < 0) {
        pthread_mutex_unlock(&mcu_prot_mutex);
        return ret;
    }

    memset(&rdata[0], 0, sizeof(rdata));
    ret = mcu_prot_read(MCU_PROT_TYPE_NORMAL, &rdata[0], sizeof(rdata));
    if (ret < 0) {
        pthread_mutex_unlock(&mcu_prot_mutex);
        return ret;
    }

    rdlen = (rh->len0 << 8) | rh->len1;
    if (rdlen < 4 || rdlen > (sizeof(rdata) - sizeof(mcu_prot_read_head_t) - 1)) {
        pthread_mutex_unlock(&mcu_prot_mutex);
        return FS_ERR_MCU_PROT_CRC;
    }

    crc = fs_util_crc_8(rdata + sizeof(mcu_prot_read_head_t), rdlen, 0);
    if (crc != rdata[sizeof(mcu_prot_read_head_t) + rdlen]) {
        ret = FS_ERR_MCU_PROT_CRC;
        pthread_mutex_unlock(&mcu_prot_mutex);
        return ret;
    }

    if (o_dac_code != NULL) {
        *o_dac_code = (rdata[8] << 24) | (rdata[9] << 16) | (rdata[10] << 8) | rdata[11];
    }

    pthread_mutex_unlock(&mcu_prot_mutex);

    return ret;
}

int mcu_prot_set_enable_vbd(uint32_t vbdstate, bool force)
{
    int ret = 0;
    mcu_prot_cmd_t cmd;

    memset(&cmd, 0, sizeof(cmd));

    cmd.cmd_dev = MCU_PROT_DEV_MCU;
    cmd.cmd_op = MCU_PROT_OP_WRITE;
    cmd.sub_cmd0 = (MCU_PROT_MCU_ENABLE_VBD >> 24) & 0xff;
    cmd.sub_cmd1 = (MCU_PROT_MCU_ENABLE_VBD >> 16) & 0xff;
    cmd.sub_cmd2 = (MCU_PROT_MCU_ENABLE_VBD >> 8) & 0xff;
    cmd.sub_cmd3 = (MCU_PROT_MCU_ENABLE_VBD >> 0) & 0xff;

    ((uint8_t *)&cmd.data)[0] = (vbdstate >> 24) & 0xFF;
    ((uint8_t *)&cmd.data)[1] = (vbdstate >> 16) & 0xFF;
    ((uint8_t *)&cmd.data)[2] = (vbdstate >>  8) & 0xFF;
    ((uint8_t *)&cmd.data)[3] = (vbdstate >>  0) & 0xFF;
    ((uint8_t *)&cmd.data)[4] =  force;

    cmd.crc8 = fs_util_crc_8(&cmd, sizeof(cmd) - 1, 0);

    pthread_mutex_lock(&mcu_prot_mutex);
    ret = mcu_prot_write(MCU_PROT_TYPE_NORMAL, (uint8_t*)&cmd, sizeof(cmd));
    if (ret < 0) {
        pthread_mutex_unlock(&mcu_prot_mutex);
        return ret;
    }
    ret = mcu_prot_wait_idle(2, 200);
    pthread_mutex_unlock(&mcu_prot_mutex);

    return ret;
}

int mcu_prot_set_enable_vpp(uint32_t vppstate, bool force)
{
    int ret = 0;
    mcu_prot_cmd_t cmd;

    memset(&cmd, 0, sizeof(cmd));

    cmd.cmd_dev = MCU_PROT_DEV_MCU;
    cmd.cmd_op = MCU_PROT_OP_WRITE;
    cmd.sub_cmd0 = (MCU_PROT_MCU_ENABLE_VPP >> 24) & 0xff;
    cmd.sub_cmd1 = (MCU_PROT_MCU_ENABLE_VPP >> 16) & 0xff;
    cmd.sub_cmd2 = (MCU_PROT_MCU_ENABLE_VPP >> 8) & 0xff;
    cmd.sub_cmd3 = (MCU_PROT_MCU_ENABLE_VPP >> 0) & 0xff;

    ((uint8_t *)&cmd.data)[0] = (vppstate >> 24) & 0xFF;
    ((uint8_t *)&cmd.data)[1] = (vppstate >> 16) & 0xFF;
    ((uint8_t *)&cmd.data)[2] = (vppstate >>  8) & 0xFF;
    ((uint8_t *)&cmd.data)[3] = (vppstate >>  0) & 0xFF;
    ((uint8_t *)&cmd.data)[4] =  force;

    cmd.crc8 = fs_util_crc_8(&cmd, sizeof(cmd) - 1, 0);

    pthread_mutex_lock(&mcu_prot_mutex);
    ret = mcu_prot_write(MCU_PROT_TYPE_NORMAL, (uint8_t*)&cmd, sizeof(cmd));
    if (ret < 0) {
        pthread_mutex_unlock(&mcu_prot_mutex);
        return ret;
    }
    ret = mcu_prot_wait_idle(2, 200);
    pthread_mutex_unlock(&mcu_prot_mutex);

    return ret;
}

int mcu_prot_set_temp_comp(bool enable)
{
    int ret = 0;
    mcu_prot_cmd_t cmd;

    memset(&cmd, 0, sizeof(cmd));

    cmd.cmd_dev  = MCU_PROT_DEV_MCU;
    cmd.cmd_op   = MCU_PROT_OP_WRITE;
    cmd.sub_cmd0 = (MCU_PROT_MCU_ENABLE_TEMP_COMP >> 24) & 0xFF;
    cmd.sub_cmd1 = (MCU_PROT_MCU_ENABLE_TEMP_COMP >> 16) & 0xFF;
    cmd.sub_cmd2 = (MCU_PROT_MCU_ENABLE_TEMP_COMP >>  8) & 0xFF;
    cmd.sub_cmd3 = (MCU_PROT_MCU_ENABLE_TEMP_COMP >>  0) & 0xFF;

    ((uint8_t *)&cmd.data)[0] = (enable >> 24) & 0xFF;
    ((uint8_t *)&cmd.data)[1] = (enable >> 16) & 0xFF;
    ((uint8_t *)&cmd.data)[2] = (enable >>  8) & 0xFF;
    ((uint8_t *)&cmd.data)[3] = (enable >>  0) & 0xFF;

    cmd.crc8 = fs_util_crc_8(&cmd, sizeof(cmd) - 1, 0);

    pthread_mutex_lock(&mcu_prot_mutex);
    ret = mcu_prot_write(MCU_PROT_TYPE_NORMAL, (uint8_t*)&cmd, sizeof(cmd));

    if (ret < 0) {
        pthread_mutex_unlock(&mcu_prot_mutex);
        return ret;
    }

    ret = mcu_prot_wait_idle(2, 200);
    pthread_mutex_unlock(&mcu_prot_mutex);
    return ret;
}

int mcu_prot_set_voltage_calib(bool enable)
{
    int ret = 0;
    mcu_prot_cmd_t cmd;

    memset(&cmd, 0, sizeof(cmd));

    cmd.cmd_dev  = MCU_PROT_DEV_MCU;
    cmd.cmd_op   = MCU_PROT_OP_WRITE;
    cmd.sub_cmd0 = (MCU_PROT_MCU_ENABLE_VOLTAGE_CALIB >> 24) & 0xFF;
    cmd.sub_cmd1 = (MCU_PROT_MCU_ENABLE_VOLTAGE_CALIB >> 16) & 0xFF;
    cmd.sub_cmd2 = (MCU_PROT_MCU_ENABLE_VOLTAGE_CALIB >>  8) & 0xFF;
    cmd.sub_cmd3 = (MCU_PROT_MCU_ENABLE_VOLTAGE_CALIB >>  0) & 0xFF;

    ((uint8_t *)&cmd.data)[0] = (enable >> 24) & 0xFF;
    ((uint8_t *)&cmd.data)[1] = (enable >> 16) & 0xFF;
    ((uint8_t *)&cmd.data)[2] = (enable >>  8) & 0xFF;
    ((uint8_t *)&cmd.data)[3] = (enable >>  0) & 0xFF;

    cmd.crc8 = fs_util_crc_8(&cmd, sizeof(cmd) - 1, 0);

    pthread_mutex_lock(&mcu_prot_mutex);
    ret = mcu_prot_write(MCU_PROT_TYPE_NORMAL, (uint8_t*)&cmd, sizeof(cmd));

    if (ret < 0) {
        pthread_mutex_unlock(&mcu_prot_mutex);
        return ret;
    }

    ret = mcu_prot_wait_idle(2, 200);
    pthread_mutex_unlock(&mcu_prot_mutex);
    return ret;
}

// int mcu_prot_get_workmode_list(uint32_t *modes, uint32_t *count)
// {
//     int ret = 0;
//     uint8_t rdata[256];  // 足够容纳模式列表
//     uint16_t rdlen;
//     mcu_prot_read_head_t* rh = (mcu_prot_read_head_t*)rdata;
//     mcu_prot_cmd_t cmd;
//     uint8_t crc;
//     uint32_t mode_count = 0;
//     uint32_t i;

//     if (modes == NULL || count == NULL) {
//         return FS_ERR_BAD_PARAMS;
//     }

//     memset(&cmd, 0, sizeof(cmd));

//     cmd.cmd_dev = MCU_PROT_DEV_MCU;
//     cmd.cmd_op = MCU_PROT_OP_READ;
//     cmd.sub_cmd0 = (MCU_PROT_MCU_WORK_MODE_LIST >> 24) & 0xff;
//     cmd.sub_cmd1 = (MCU_PROT_MCU_WORK_MODE_LIST >> 16) & 0xff;
//     cmd.sub_cmd2 = (MCU_PROT_MCU_WORK_MODE_LIST >> 8) & 0xff;
//     cmd.sub_cmd3 = (MCU_PROT_MCU_WORK_MODE_LIST >> 0) & 0xff;

//     cmd.crc8 = fs_util_crc_8(&cmd, sizeof(cmd) - 1, 0);

//     pthread_mutex_lock(&mcu_prot_mutex);
//     ret = mcu_prot_write(MCU_PROT_TYPE_NORMAL, (uint8_t*)&cmd, sizeof(cmd));
//     if (ret < 0) {
//         pthread_mutex_unlock(&mcu_prot_mutex);
//         return ret;
//     }

//     ret = mcu_prot_wait_idle(2, 200);
//     if (ret < 0) {
//         pthread_mutex_unlock(&mcu_prot_mutex);
//         return ret;
//     }

//     memset(&rdata[0], 0, sizeof(rdata));
//     ret = mcu_prot_read(MCU_PROT_TYPE_NORMAL, &rdata[0], sizeof(rdata));
//     if (ret < 0) {
//         pthread_mutex_unlock(&mcu_prot_mutex);
//         return ret;
//     }

//     rdlen = (rh->len0 << 8) | rh->len1;
//     crc = fs_util_crc_8(rdata + sizeof(mcu_prot_read_head_t), rdlen, 0);
//     if (crc != rdata[sizeof(mcu_prot_read_head_t) + rdlen]) {
//         ret = FS_ERR_MCU_PROT_CRC;
//         pthread_mutex_unlock(&mcu_prot_mutex);
//         return ret;
//     }

//     // 解析模式列表：[count(1字节)] + [mode1(4字节)] + [mode2(4字节)] + ...
//     mode_count = rdata[8];
//     if (mode_count > *count) {
//         FS_LOGW("MCU mode count (%u) exceeds buffer size (%u), truncating", mode_count, *count);
//         mode_count = *count;
//     }

//     for (i = 0; i < mode_count; i++) {
//         uint32_t offset = 9 + i * sizeof(uint32_t);
//         modes[i] = (rdata[offset] << 24) | (rdata[offset + 1] << 16) |
//                    (rdata[offset + 2] << 8) | rdata[offset + 3];
//     }

//     *count = mode_count;

//     pthread_mutex_unlock(&mcu_prot_mutex);

//     FS_LOGI("get_workmode_list: count=%u", mode_count);

//     return ret;
// }

int mcu_prot_get_workmode(uint32_t* workmode)
{
    int ret = 0;
    uint8_t rdata[MCU_PROT_READ_LIDARCHIP_LEN];
    uint16_t rdlen;
    mcu_prot_read_head_t* rh = (mcu_prot_read_head_t*)rdata;
    mcu_prot_cmd_t cmd;
    uint8_t crc;

    memset(&cmd, 0, sizeof(cmd));

    cmd.cmd_dev = MCU_PROT_DEV_MCU;
    cmd.cmd_op = MCU_PROT_OP_READ;
    cmd.sub_cmd0 = (MCU_PROT_MCU_WORK_MODE >> 24) & 0xff;
    cmd.sub_cmd1 = (MCU_PROT_MCU_WORK_MODE >> 16) & 0xff;
    cmd.sub_cmd2 = (MCU_PROT_MCU_WORK_MODE >> 8) & 0xff;
    cmd.sub_cmd3 = (MCU_PROT_MCU_WORK_MODE >> 0) & 0xff;

    cmd.crc8 = fs_util_crc_8(&cmd, sizeof(cmd) - 1, 0);

    pthread_mutex_lock(&mcu_prot_mutex);
    ret = mcu_prot_write(MCU_PROT_TYPE_NORMAL, (uint8_t*)&cmd, sizeof(cmd));
    if (ret < 0) {
        pthread_mutex_unlock(&mcu_prot_mutex);
        return ret;
    }

    ret = mcu_prot_wait_idle(2, 200);
    if (ret < 0) {
        pthread_mutex_unlock(&mcu_prot_mutex);
        return ret;
    }

    memset(&rdata[0], 0, sizeof(rdata));
    ret = mcu_prot_read(MCU_PROT_TYPE_NORMAL, &rdata[0], sizeof(rdata));
    if (ret < 0) {
        pthread_mutex_unlock(&mcu_prot_mutex);
        return ret;
    }

    rdlen = (rh->len0 << 8) | rh->len1;
    crc = fs_util_crc_8(rdata + sizeof(mcu_prot_read_head_t), rdlen, 0);
    if (crc != rdata[sizeof(mcu_prot_read_head_t) + rdlen]) {
        ret = FS_ERR_MCU_PROT_CRC;
        pthread_mutex_unlock(&mcu_prot_mutex);
        return ret;
    }
    *workmode = (rdata[8] << 24) | (rdata[9] << 16) | (rdata[10] << 8) | rdata[11];

    pthread_mutex_unlock(&mcu_prot_mutex);

    FS_LOGI("get_workmode 0x%08x", *workmode);

    return ret;
}

int mcu_prot_set_txseqmode(uint8_t seqmode, uint8_t* chan, uint8_t chan_num)
{
    int ret = 0;
    mcu_prot_txseq_cmd_t cmd;

    memset(&cmd, 0, sizeof(cmd));

    cmd.cmd_dev = MCU_PROT_DEV_MCU;
    cmd.cmd_op = MCU_PROT_OP_WRITE;
    cmd.sub_cmd0 = (MCU_PROT_MCU_TX_SEQMODE >> 24) & 0xff;
    cmd.sub_cmd1 = (MCU_PROT_MCU_TX_SEQMODE >> 16) & 0xff;
    cmd.sub_cmd2 = (MCU_PROT_MCU_TX_SEQMODE >> 8) & 0xff;
    cmd.sub_cmd3 = (MCU_PROT_MCU_TX_SEQMODE >> 0) & 0xff;

    cmd.d0 = seqmode;

    if (chan_num > sizeof(cmd.data) - 2) {
        chan_num = sizeof(cmd.data) - 2;
    }
    cmd.d1 = chan_num;
    memcpy(&cmd.d2, chan, chan_num);

    cmd.crc8 = fs_util_crc_8(&cmd, sizeof(cmd) - 1, 0);

    pthread_mutex_lock(&mcu_prot_mutex);
    ret = mcu_prot_write(MCU_PROT_TYPE_NORMAL, (uint8_t*)&cmd, sizeof(cmd));
    if (ret < 0) {
        pthread_mutex_unlock(&mcu_prot_mutex);
        return ret;
    }
    ret = mcu_prot_wait_idle(2, 1000);
    pthread_mutex_unlock(&mcu_prot_mutex);

    return ret;
}

int mcu_prot_get_txseqmode(uint8_t* seqmode, uint8_t* chan, uint8_t* chan_num)
{
    int ret = 0;
    uint8_t rdata[MCU_PROT_READ_LIDARCHIP_LEN];
    uint16_t rdlen;
    mcu_prot_read_head_t* rh = (mcu_prot_read_head_t*)rdata;
    mcu_prot_txseq_cmd_t cmd;
    uint8_t crc;

    memset(&cmd, 0, sizeof(cmd));

    cmd.cmd_dev = MCU_PROT_DEV_MCU;
    cmd.cmd_op = MCU_PROT_OP_READ;
    cmd.sub_cmd0 = (MCU_PROT_MCU_TX_SEQMODE >> 24) & 0xff;
    cmd.sub_cmd1 = (MCU_PROT_MCU_TX_SEQMODE >> 16) & 0xff;
    cmd.sub_cmd2 = (MCU_PROT_MCU_TX_SEQMODE >> 8) & 0xff;
    cmd.sub_cmd3 = (MCU_PROT_MCU_TX_SEQMODE >> 0) & 0xff;

    cmd.crc8 = fs_util_crc_8(&cmd, sizeof(cmd) - 1, 0);

    pthread_mutex_lock(&mcu_prot_mutex);
    ret = mcu_prot_write(MCU_PROT_TYPE_NORMAL, (uint8_t*)&cmd, sizeof(cmd));
    if (ret < 0) {
        pthread_mutex_unlock(&mcu_prot_mutex);
        return ret;
    }

    ret = mcu_prot_wait_idle(2, 200);
    if (ret < 0) {
        pthread_mutex_unlock(&mcu_prot_mutex);
        return ret;
    }

    memset(&rdata[0], 0, sizeof(rdata));
    ret = mcu_prot_read(MCU_PROT_TYPE_NORMAL, &rdata[0], sizeof(rdata));
    if (ret < 0) {
        pthread_mutex_unlock(&mcu_prot_mutex);
        return ret;
    }

    rdlen = (rh->len0 << 8) | rh->len1;
    crc = fs_util_crc_8(rdata + sizeof(mcu_prot_read_head_t), rdlen, 0);
    if (crc != rdata[sizeof(mcu_prot_read_head_t) + rdlen]) {
        ret = FS_ERR_MCU_PROT_CRC;
        pthread_mutex_unlock(&mcu_prot_mutex);
        return ret;
    }
    *seqmode = rdata[8];
    *chan_num = rdata[9];
    memcpy(chan, &rdata[10], *chan_num);

    pthread_mutex_unlock(&mcu_prot_mutex);

    FS_LOGI("get_txseqmode 0x%02x: %02x, %02x, %02x", *seqmode, chan[0], chan[1], chan[2]);

    return ret;
}

int mcu_prot_get_mcu_version(uint8_t *data, uint16_t data_len)
{
    int ret = 0;
    uint8_t rdata[MCU_PROT_READ_MCU_VER_LEN];
    uint16_t rdlen;
    uint8_t crc;
    mcu_prot_read_head_t* rh = (mcu_prot_read_head_t*)rdata;
    mcu_prot_cmd_t cmd;

    memset(&cmd, 0, sizeof(cmd));

    cmd.cmd_dev = MCU_PROT_DEV_MCU;
    cmd.cmd_op = MCU_PROT_OP_READ;
    cmd.sub_cmd0 = (MCU_PROT_MCU_MCU_VER >> 24) & 0xff;
    cmd.sub_cmd1 = (MCU_PROT_MCU_MCU_VER >> 16) & 0xff;
    cmd.sub_cmd2 = (MCU_PROT_MCU_MCU_VER >> 8) & 0xff;
    cmd.sub_cmd3 = (MCU_PROT_MCU_MCU_VER >> 0) & 0xff;
    cmd.crc8 = fs_util_crc_8(&cmd, sizeof(cmd) - 1, 0);

    pthread_mutex_lock(&mcu_prot_mutex);
    ret = mcu_prot_write(MCU_PROT_TYPE_NORMAL, (uint8_t*)&cmd, sizeof(cmd));
    if (ret < 0) {
        pthread_mutex_unlock(&mcu_prot_mutex);
        return ret;
    }

    ret = mcu_prot_wait_idle(2, 200);
    if (ret < 0) {
        pthread_mutex_unlock(&mcu_prot_mutex);
        return ret;
    }

    memset(&rdata[0], 0, 73);
    ret = mcu_prot_read(MCU_PROT_TYPE_NORMAL, &rdata[0], sizeof(rdata));
    if (ret < 0) {
        pthread_mutex_unlock(&mcu_prot_mutex);
        return ret;
    }

    rdlen = (rh->len0 << 8) | rh->len1;
    crc = fs_util_crc_8(rdata + sizeof(mcu_prot_read_head_t), rdlen, 0);
    if (crc != rdata[sizeof(mcu_prot_read_head_t) + rdlen]) {
        ret = FS_ERR_MCU_PROT_CRC;
    }
    pthread_mutex_unlock(&mcu_prot_mutex);

    if (ret == FS_SUCCESS) {
        FS_LOGI("MCU Version: %s", &rdata[8]);
        memcpy(data, &rdata[8], data_len);
        if (data_len > 0u) {
            data[data_len - 1u] = '\0';
        }
    }

    return ret;
}

/**
 * @brief 获取 MCU 初始化状态
 * @param state 输出参数，返回 MCU 初始化状态 (MCU_INIT_STATE_xxx)
 * @return 0 成功，其他值表示失败
 */
int mcu_prot_get_init_state(uint8_t *state)
{
    int ret = 0;
    uint8_t rdata[MCU_PROT_READ_COMMON_LEN + 1];  // 1 字节状态数据
    uint16_t rdlen;
    uint8_t crc;
    mcu_prot_read_head_t* rh = (mcu_prot_read_head_t*)rdata;
    mcu_prot_cmd_t cmd;

    if (state == NULL) {
        return FS_ERR_BAD_PARAMS;
    }

    memset(&cmd, 0, sizeof(cmd));

    cmd.cmd_dev = MCU_PROT_DEV_MCU;
    cmd.cmd_op = MCU_PROT_OP_READ;
    cmd.sub_cmd0 = (MCU_PROT_MCU_INIT_STATE >> 24) & 0xff;
    cmd.sub_cmd1 = (MCU_PROT_MCU_INIT_STATE >> 16) & 0xff;
    cmd.sub_cmd2 = (MCU_PROT_MCU_INIT_STATE >> 8) & 0xff;
    cmd.sub_cmd3 = (MCU_PROT_MCU_INIT_STATE >> 0) & 0xff;
    cmd.crc8 = fs_util_crc_8(&cmd, sizeof(cmd) - 1, 0);

    pthread_mutex_lock(&mcu_prot_mutex);
    ret = mcu_prot_write(MCU_PROT_TYPE_NORMAL, (uint8_t*)&cmd, sizeof(cmd));
    if (ret < 0) {
        pthread_mutex_unlock(&mcu_prot_mutex);
        return ret;
    }

    ret = mcu_prot_wait_idle(2, 200);
    if (ret < 0) {
        pthread_mutex_unlock(&mcu_prot_mutex);
        return ret;
    }

    memset(&rdata[0], 0, sizeof(rdata));
    ret = mcu_prot_read(MCU_PROT_TYPE_NORMAL, &rdata[0], sizeof(rdata));
    if (ret < 0) {
        pthread_mutex_unlock(&mcu_prot_mutex);
        return ret;
    }

    rdlen = (rh->len0 << 8) | rh->len1;
    if (rdlen != 1) {
        pthread_mutex_unlock(&mcu_prot_mutex);
        FS_LOGE("Unexpected init state data length: %u", rdlen);
        return FS_ERR_BAD_PROTOCOL;
    }

    crc = fs_util_crc_8(rdata + sizeof(mcu_prot_read_head_t), rdlen, 0);
    if (crc != rdata[sizeof(mcu_prot_read_head_t) + rdlen]) {
        pthread_mutex_unlock(&mcu_prot_mutex);
        FS_LOGE("Init state CRC mismatch");
        return FS_ERR_MCU_PROT_CRC;
    }
    pthread_mutex_unlock(&mcu_prot_mutex);

    *state = rdata[8];  // 状态数据在偏移 8 处
    FS_LOGD("MCU init state: %u", *state);

    return 0;
}

/**
 * @brief 等待 MCU 初始化完成
 * @param timeout_ms 超时时间（毫秒），0 表示无限等待
 * @return 0 成功（MCU 已就绪），FS_ERR_TIME_OUT 表示超时，其他值表示通信错误
 */
int mcu_prot_wait_ready(uint32_t timeout_ms)
{
    int ret;
    uint8_t state;
    uint32_t elapsed_ms = 0;
    const uint32_t poll_interval_ms = 100;  // 轮询间隔 100ms

    FS_LOGI("Waiting for MCU to be ready (timeout=%u ms)...", timeout_ms);

    while (1) {
        ret = mcu_prot_get_init_state(&state);
        if (ret != 0) {
            // 通信错误，可能 MCU 还在启动过程中，继续重试
            FS_LOGD("Failed to get MCU init state: %d, retrying...", ret);
        } else if (state == MCU_INIT_STATE_READY) {
            FS_LOGI("MCU is ready (elapsed %u ms)", elapsed_ms);
            return 0;
        } else if (state == MCU_INIT_STATE_ERROR) {
            FS_LOGE("MCU initialization failed!");
            return FS_ERR_INTERNAL;
        } else {
            FS_LOGD("MCU init state: %u, waiting...", state);
        }

        // 检查超时
        if (timeout_ms > 0 && elapsed_ms >= timeout_ms) {
            FS_LOGE("Timeout waiting for MCU ready (elapsed %u ms)", elapsed_ms);
            return FS_ERR_TIME_OUT;
        }

        usleep(poll_interval_ms * 1000);
        elapsed_ms += poll_interval_ms;
    }
}

int mcu_prot_get_spad_sn(uint8_t *data, uint16_t data_len)
{
    int ret = 0;
    uint8_t rdata[MCU_PROT_READ_SPAD_SN_LEN];
    uint16_t rdlen;
    uint8_t crc;
    mcu_prot_read_head_t* rh = (mcu_prot_read_head_t*)rdata;
    mcu_prot_cmd_t cmd;

    memset(&cmd, 0, sizeof(cmd));

    cmd.cmd_dev = MCU_PROT_DEV_MCU;
    cmd.cmd_op = MCU_PROT_OP_READ;
    cmd.sub_cmd0 = (MCU_PROT_MCU_SPAD_SN >> 24) & 0xff;
    cmd.sub_cmd1 = (MCU_PROT_MCU_SPAD_SN >> 16) & 0xff;
    cmd.sub_cmd2 = (MCU_PROT_MCU_SPAD_SN >> 8) & 0xff;
    cmd.sub_cmd3 = (MCU_PROT_MCU_SPAD_SN >> 0) & 0xff;
    cmd.crc8 = fs_util_crc_8(&cmd, sizeof(cmd) - 1, 0);

    pthread_mutex_lock(&mcu_prot_mutex);
    ret = mcu_prot_write(MCU_PROT_TYPE_NORMAL, (uint8_t*)&cmd, sizeof(cmd));
    if (ret < 0) {
        pthread_mutex_unlock(&mcu_prot_mutex);
        return ret;
    }

    ret = mcu_prot_wait_idle(2, 200);
    if (ret < 0) {
        pthread_mutex_unlock(&mcu_prot_mutex);
        return ret;
    }

    memset(&rdata[0], 0, 64 + 9);
    ret = mcu_prot_read(MCU_PROT_TYPE_NORMAL, &rdata[0], sizeof(rdata));
    rdlen = (rh->len0 << 8) | rh->len1;
    crc = fs_util_crc_8(rdata + sizeof(mcu_prot_read_head_t), rdlen, 0);
    if (crc != rdata[sizeof(mcu_prot_read_head_t) + rdlen]) {
        ret = FS_ERR_MCU_PROT_ERR;
    }

    pthread_mutex_unlock(&mcu_prot_mutex);

    if (ret == 0) {
        FS_LOGI("%s", &rdata[8]);
        memcpy(data, &rdata[8], data_len);
    }

    return 0;
}

int mcu_prot_get_board_temp(void)
{
    int ret = 0;
    uint8_t* rdata;
    uint16_t rdlen;
    uint8_t crc;
    mcu_prot_read_head_t* rh;
    mcu_prot_cmd_t cmd;
    fs_temp_info_t temp;

    memset(&cmd, 0, sizeof(cmd));

    cmd.cmd_dev = MCU_PROT_DEV_MCU;
    cmd.cmd_op = MCU_PROT_OP_READ;
    cmd.sub_cmd0 = (MCU_PROT_MCU_BOARD_TEMP >> 24) & 0xff;
    cmd.sub_cmd1 = (MCU_PROT_MCU_BOARD_TEMP >> 16) & 0xff;
    cmd.sub_cmd2 = (MCU_PROT_MCU_BOARD_TEMP >> 8) & 0xff;
    cmd.sub_cmd3 = (MCU_PROT_MCU_BOARD_TEMP >> 0) & 0xff;
    cmd.crc8 = fs_util_crc_8(&cmd, sizeof(cmd) - 1, 0);

    pthread_mutex_lock(&mcu_prot_mutex);
    ret = mcu_prot_write(MCU_PROT_TYPE_NORMAL, (uint8_t*)&cmd, sizeof(cmd));
    if (ret < 0) {
        pthread_mutex_unlock(&mcu_prot_mutex);
        return ret;
    }

    ret = mcu_prot_wait_idle(2, 200);
    if (ret < 0) {
        pthread_mutex_unlock(&mcu_prot_mutex);
        return ret;
    }

    uint32_t buf_len = sizeof(temp) + MCU_PROT_READ_COMMON_LEN;
    rdata = (uint8_t*)malloc(buf_len);
    if (!rdata) {
        pthread_mutex_unlock(&mcu_prot_mutex);
        return FS_ERR_NOMEM;
    }

    rh = (mcu_prot_read_head_t*)rdata;
    memset(&rdata[0], 0, buf_len);
    ret = mcu_prot_read(MCU_PROT_TYPE_NORMAL, &rdata[0], buf_len);
    rdlen = (rh->len0 << 8) | rh->len1;

    // 边界检查：确保 rdlen 不超过实际分配的缓冲区大小
    uint32_t max_rdlen = buf_len - sizeof(mcu_prot_read_head_t) - 1;
    if (rdlen > max_rdlen) {
        FS_LOGE("mcu_prot_get_board_temp: invalid rdlen %u (max %u)", rdlen, max_rdlen);
        pthread_mutex_unlock(&mcu_prot_mutex);
        free(rdata);
        return FS_ERR_MCU_PROT_CRC;
    }

    crc = fs_util_crc_8(rdata + sizeof(mcu_prot_read_head_t), rdlen, 0);
    if (crc != rdata[sizeof(mcu_prot_read_head_t) + rdlen]) {
        ret = FS_ERR_MCU_PROT_CRC;
    }

    pthread_mutex_unlock(&mcu_prot_mutex);

    if (ret == 0) {
        memcpy(&temp, &rdata[sizeof(mcu_prot_read_head_t)], rdlen);
        FS_LOGI("temp:%.03f, %.03f, %.03f, %.03f", temp.rx_temp, temp.tx1_temp, temp.tx2_temp, temp.pwr_temp);
    }

    free(rdata);

    return 0;
}

int mcu_prot_get_voltages(fs_voltage_info_t *v)
{
    int ret = 0;
    uint8_t* rdata;
    uint16_t rdlen;
    uint8_t crc;
    mcu_prot_read_head_t* rh;
    mcu_prot_cmd_t cmd;

    memset(&cmd, 0, sizeof(cmd));

    cmd.cmd_dev = MCU_PROT_DEV_MCU;
    cmd.cmd_op = MCU_PROT_OP_READ;
    cmd.sub_cmd0 = (MCU_PROT_MCU_VOLTAGE >> 24) & 0xff;
    cmd.sub_cmd1 = (MCU_PROT_MCU_VOLTAGE >> 16) & 0xff;
    cmd.sub_cmd2 = (MCU_PROT_MCU_VOLTAGE >> 8) & 0xff;
    cmd.sub_cmd3 = (MCU_PROT_MCU_VOLTAGE >> 0) & 0xff;
    cmd.crc8 = fs_util_crc_8(&cmd, sizeof(cmd) - 1, 0);

    pthread_mutex_lock(&mcu_prot_mutex);
    ret = mcu_prot_write(MCU_PROT_TYPE_NORMAL, (uint8_t*)&cmd, sizeof(cmd));
    if (ret < 0) {
        pthread_mutex_unlock(&mcu_prot_mutex);
        return ret;
    }

    ret = mcu_prot_wait_idle(2, 200);
    if (ret < 0) {
        pthread_mutex_unlock(&mcu_prot_mutex);
        return ret;
    }

    uint32_t buf_len = sizeof(fs_voltage_info_t) + MCU_PROT_READ_COMMON_LEN;
    rdata = (uint8_t*)malloc(buf_len);
    if (!rdata) {
        pthread_mutex_unlock(&mcu_prot_mutex);
        return FS_ERR_NOMEM;
    }

    rh = (mcu_prot_read_head_t*)rdata;
    memset(&rdata[0], 0, buf_len);
    ret = mcu_prot_read(MCU_PROT_TYPE_NORMAL, &rdata[0], buf_len);
    rdlen = (rh->len0 << 8) | rh->len1;

    // 边界检查：确保 rdlen 不超过实际分配的缓冲区大小
    uint32_t max_rdlen = buf_len - sizeof(mcu_prot_read_head_t) - 1;
    if (rdlen > max_rdlen) {
        FS_LOGE("mcu_prot_get_voltages: invalid rdlen %u (max %u)", rdlen, max_rdlen);
        pthread_mutex_unlock(&mcu_prot_mutex);
        free(rdata);
        return FS_ERR_MCU_PROT_CRC;
    }

    crc = fs_util_crc_8(rdata + sizeof(mcu_prot_read_head_t), rdlen, 0);
    if (crc != rdata[sizeof(mcu_prot_read_head_t) + rdlen]) {
        ret = FS_ERR_MCU_PROT_CRC;
    }

    pthread_mutex_unlock(&mcu_prot_mutex);

    if (ret == 0) {
        memcpy(v, &rdata[sizeof(mcu_prot_read_head_t)], rdlen);
        FS_LOGI("voltage:%.03f, %.03f, %.03f", v->vbd, v->vpp, v->vex);
    }

    free(rdata);

    return 0;
}

int mcu_prot_get_adc(uint8_t type, uint8_t fmt, int32_t *io_value, uint8_t *io_cnt, uint32_t *o_raw_adc_mV)
{
    int ret = 0;
    uint8_t* rdata;
    uint16_t rdlen;
    uint8_t crc;
    mcu_prot_read_head_t* rh;
    mcu_prot_cmd_t cmd;

    if (!io_value || !io_cnt) {
        return FS_ERR_BAD_PARAMS;
    }

    memset(&cmd, 0, sizeof(cmd));

    cmd.cmd_dev = MCU_PROT_DEV_MCU;
    cmd.cmd_op = MCU_PROT_OP_READ;
    cmd.sub_cmd0 = (MCU_PROT_MCU_GET_ADC >> 24) & 0xff;
    cmd.sub_cmd1 = (MCU_PROT_MCU_GET_ADC >> 16) & 0xff;
    cmd.sub_cmd2 = (MCU_PROT_MCU_GET_ADC >> 8) & 0xff;
    cmd.sub_cmd3 = (MCU_PROT_MCU_GET_ADC >> 0) & 0xff;

    // 将 type 和 fmt 参数放入 data 字段
    cmd.data0 = type;
    cmd.data1 = fmt;
    cmd.data2 = 0;
    cmd.data3 = 0;

    cmd.crc8 = fs_util_crc_8(&cmd, sizeof(cmd) - 1, 0);

    pthread_mutex_lock(&mcu_prot_mutex);
    ret = mcu_prot_write(MCU_PROT_TYPE_NORMAL, (uint8_t*)&cmd, sizeof(cmd));
    if (ret < 0) {
        pthread_mutex_unlock(&mcu_prot_mutex);
        return ret;
    }

    ret = mcu_prot_wait_idle(2, 200);
    if (ret < 0) {
        pthread_mutex_unlock(&mcu_prot_mutex);
        return ret;
    }

    // 接收固定 8 字节数据: [volt_mV(int32)][adc_code(uint32)]
    rdata = (uint8_t*)malloc(MCU_PROT_READ_COMMON_LEN + 8 + 1);
    if (!rdata) {
        pthread_mutex_unlock(&mcu_prot_mutex);
        return FS_ERR_NOMEM;
    }

    rh = (mcu_prot_read_head_t*)rdata;
    uint32_t buf_len = MCU_PROT_READ_COMMON_LEN + 8 + 1;
    memset(&rdata[0], 0, buf_len);
    ret = mcu_prot_read(MCU_PROT_TYPE_NORMAL, &rdata[0], buf_len);
    rdlen = (rh->len0 << 8) | rh->len1;

    uint32_t max_rdlen = buf_len - sizeof(mcu_prot_read_head_t) - 1;
    if (rdlen > max_rdlen) {
        FS_LOGE("mcu_prot_get_adc: invalid rdlen %u (max %u), buf_len %u",
                rdlen, max_rdlen, buf_len);
        pthread_mutex_unlock(&mcu_prot_mutex);
        free(rdata);
        return FS_ERR_MCU_PROT_CRC;
    }

    crc = fs_util_crc_8(rdata + sizeof(mcu_prot_read_head_t), rdlen, 0);
    if (crc != rdata[sizeof(mcu_prot_read_head_t) + rdlen]) {
        ret = FS_ERR_MCU_PROT_CRC;
    }

    pthread_mutex_unlock(&mcu_prot_mutex);

    if (ret == 0) {
        *io_cnt = 1;
        io_value[0] = (int32_t)(((uint32_t)rdata[8] << 24) | ((uint32_t)rdata[9] << 16) |
                                ((uint32_t)rdata[10] << 8) | (uint32_t)rdata[11]);
        if (o_raw_adc_mV) {
            *o_raw_adc_mV = ((uint32_t)rdata[12] << 24) | ((uint32_t)rdata[13] << 16) |
                            ((uint32_t)rdata[14] << 8) | (uint32_t)rdata[15];
        }
    }

    free(rdata);

    return ret;
}

static int get_adc_calib_subcmd(uint8_t adc_type, bool remove, uint32_t *sub_cmd)
{
    if (!sub_cmd) {
        return FS_ERR_BAD_PARAMS;
    }

    if (adc_type == ADC_TYPE_VBD) {
        *sub_cmd = remove ? MCU_PROT_MCU_REMOVE_ADC_VBD_CALIB : MCU_PROT_MCU_ADC_VBD_CALIB;
        return FS_SUCCESS;
    }

    if (adc_type == ADC_TYPE_VPP) {
        *sub_cmd = remove ? MCU_PROT_MCU_REMOVE_ADC_VPP_CALIB : MCU_PROT_MCU_ADC_VPP_CALIB;
        return FS_SUCCESS;
    }

    return FS_ERR_BAD_PARAMS;
}

static int get_dac_calib_subcmd(uint8_t dac_type, bool remove, uint32_t *sub_cmd)
{
    if (!sub_cmd) {
        return FS_ERR_BAD_PARAMS;
    }

    if (dac_type == DAC_TYPE_VBD) {
        *sub_cmd = remove ? MCU_PROT_MCU_REMOVE_DAC_VBD_CALIB : MCU_PROT_MCU_DAC_VBD_CALIB;
        return FS_SUCCESS;
    }

    if (dac_type == DAC_TYPE_VPP) {
        *sub_cmd = remove ? MCU_PROT_MCU_REMOVE_DAC_VPP_CALIB : MCU_PROT_MCU_DAC_VPP_CALIB;
        return FS_SUCCESS;
    }

    return FS_ERR_BAD_PARAMS;
}

static int mcu_prot_calib_program_enter(int *acquired_local)
{
    fw_if_reject_reason_t reason = FW_IF_REJECT_NONE;

    if (acquired_local == NULL) {
        return FS_ERR_BAD_PARAMS;
    }

    if (soc_mgr_program_token_held()) {
        *acquired_local = 0;
        return FS_SUCCESS;
    }

    *acquired_local = 1;
    return soc_mgr_program_token_acquire(&reason);
}

static void mcu_prot_calib_program_exit(int acquired_local, int *ret)
{
    fw_if_reject_reason_t reason = FW_IF_REJECT_NONE;
    int release_ret;

    if (acquired_local == 0 || ret == NULL) {
        return;
    }

    release_ret = soc_mgr_program_token_release(&reason);
    if (release_ret != FS_SUCCESS) {
        FS_LOGW("calib program exit ret=%d reason=%d", release_ret, (int)reason);
        if (*ret == FS_SUCCESS) {
            *ret = release_ret;
        }
    }
}

static int mcu_prot_store_calib_by_subcmd(uint32_t sub_cmd, float k, float b)
{
    int ret;
    int acquired_local = 0;
    mcu_prot_cmd_t cmd;

    ret = mcu_prot_calib_program_enter(&acquired_local);
    if (ret != FS_SUCCESS) {
        FS_LOGW("store calib rejected: program enter ret=%d", ret);
        return ret;
    }

    memset(&cmd, 0, sizeof(cmd));

    cmd.cmd_dev = MCU_PROT_DEV_MCU;
    cmd.cmd_op = MCU_PROT_OP_WRITE;
    cmd.sub_cmd0 = (sub_cmd >> 24) & 0xff;
    cmd.sub_cmd1 = (sub_cmd >> 16) & 0xff;
    cmd.sub_cmd2 = (sub_cmd >> 8) & 0xff;
    cmd.sub_cmd3 = (sub_cmd >> 0) & 0xff;
    memcpy((uint8_t *)&cmd.data, &k, sizeof(float));
    memcpy(((uint8_t *)&cmd.data) + sizeof(float), &b, sizeof(float));
    cmd.crc8 = fs_util_crc_8(&cmd, sizeof(cmd) - 1, 0);

    pthread_mutex_lock(&mcu_prot_mutex);
    ret = mcu_prot_write(MCU_PROT_TYPE_NORMAL, (uint8_t *)&cmd, sizeof(cmd));
    if (ret < 0) {
        pthread_mutex_unlock(&mcu_prot_mutex);
        mcu_prot_calib_program_exit(acquired_local, &ret);
        return ret;
    }

    ret = mcu_prot_wait_idle(2, 500);
    pthread_mutex_unlock(&mcu_prot_mutex);

    mcu_prot_calib_program_exit(acquired_local, &ret);
    return ret;
}

static int mcu_prot_read_calib_by_subcmd(uint32_t sub_cmd, float *k, float *b)
{
    int ret;
    uint8_t rdata[MCU_PROT_READ_COMMON_LEN + 8 + 1];
    uint16_t rdlen;
    uint8_t crc;
    mcu_prot_read_head_t *rh = (mcu_prot_read_head_t *)rdata;
    mcu_prot_cmd_t cmd;

    if (!k || !b) {
        return FS_ERR_BAD_PARAMS;
    }

    memset(&cmd, 0, sizeof(cmd));
    cmd.cmd_dev = MCU_PROT_DEV_MCU;
    cmd.cmd_op = MCU_PROT_OP_READ;
    cmd.sub_cmd0 = (sub_cmd >> 24) & 0xff;
    cmd.sub_cmd1 = (sub_cmd >> 16) & 0xff;
    cmd.sub_cmd2 = (sub_cmd >> 8) & 0xff;
    cmd.sub_cmd3 = (sub_cmd >> 0) & 0xff;
    cmd.crc8 = fs_util_crc_8(&cmd, sizeof(cmd) - 1, 0);

    pthread_mutex_lock(&mcu_prot_mutex);
    ret = mcu_prot_write(MCU_PROT_TYPE_NORMAL, (uint8_t *)&cmd, sizeof(cmd));
    if (ret < 0) {
        pthread_mutex_unlock(&mcu_prot_mutex);
        return ret;
    }

    ret = mcu_prot_wait_idle(2, 500);
    if (ret < 0) {
        pthread_mutex_unlock(&mcu_prot_mutex);
        return ret;
    }

    memset(rdata, 0, sizeof(rdata));
    ret = mcu_prot_read(MCU_PROT_TYPE_NORMAL, rdata, sizeof(rdata));
    if (ret < 0) {
        pthread_mutex_unlock(&mcu_prot_mutex);
        return ret;
    }

    rdlen = (rh->len0 << 8) | rh->len1;
    if (rdlen != 8u) {
        pthread_mutex_unlock(&mcu_prot_mutex);
        return FS_ERR_BAD_PROTOCOL;
    }

    crc = fs_util_crc_8(rdata + sizeof(mcu_prot_read_head_t), rdlen, 0);
    if (crc != rdata[sizeof(mcu_prot_read_head_t) + rdlen]) {
        pthread_mutex_unlock(&mcu_prot_mutex);
        return FS_ERR_MCU_PROT_CRC;
    }

    pthread_mutex_unlock(&mcu_prot_mutex);

    memcpy(k, &rdata[sizeof(mcu_prot_read_head_t)], sizeof(float));
    memcpy(b, &rdata[sizeof(mcu_prot_read_head_t) + sizeof(float)], sizeof(float));

    return FS_SUCCESS;
}

static int mcu_prot_remove_calib_by_subcmd(uint32_t sub_cmd)
{
    int ret;
    int acquired_local = 0;
    mcu_prot_cmd_t cmd;

    ret = mcu_prot_calib_program_enter(&acquired_local);
    if (ret != FS_SUCCESS) {
        FS_LOGW("remove calib rejected: program enter ret=%d", ret);
        return ret;
    }

    memset(&cmd, 0, sizeof(cmd));

    cmd.cmd_dev = MCU_PROT_DEV_MCU;
    cmd.cmd_op = MCU_PROT_OP_WRITE;
    cmd.sub_cmd0 = (sub_cmd >> 24) & 0xff;
    cmd.sub_cmd1 = (sub_cmd >> 16) & 0xff;
    cmd.sub_cmd2 = (sub_cmd >> 8) & 0xff;
    cmd.sub_cmd3 = (sub_cmd >> 0) & 0xff;
    cmd.crc8 = fs_util_crc_8(&cmd, sizeof(cmd) - 1, 0);

    pthread_mutex_lock(&mcu_prot_mutex);
    ret = mcu_prot_write(MCU_PROT_TYPE_NORMAL, (uint8_t *)&cmd, sizeof(cmd));
    if (ret < 0) {
        pthread_mutex_unlock(&mcu_prot_mutex);
        mcu_prot_calib_program_exit(acquired_local, &ret);
        return ret;
    }

    ret = mcu_prot_wait_idle(2, 500);
    pthread_mutex_unlock(&mcu_prot_mutex);

    mcu_prot_calib_program_exit(acquired_local, &ret);
    return ret;
}

int mcu_prot_store_adc_calib(uint8_t adc_type, float k, float b)
{
    uint32_t sub_cmd;
    int ret = get_adc_calib_subcmd(adc_type, false, &sub_cmd);

    if (ret != FS_SUCCESS) {
        return ret;
    }

    return mcu_prot_store_calib_by_subcmd(sub_cmd, k, b);
}

int mcu_prot_read_adc_calib(uint8_t adc_type, float *k, float *b)
{
    uint32_t sub_cmd;
    int ret = get_adc_calib_subcmd(adc_type, false, &sub_cmd);

    if (ret != FS_SUCCESS) {
        return ret;
    }

    return mcu_prot_read_calib_by_subcmd(sub_cmd, k, b);
}

int mcu_prot_remove_adc_calib(uint8_t adc_type)
{
    uint32_t sub_cmd;
    int ret = get_adc_calib_subcmd(adc_type, true, &sub_cmd);

    if (ret != FS_SUCCESS) {
        return ret;
    }

    return mcu_prot_remove_calib_by_subcmd(sub_cmd);
}

int mcu_prot_store_dac_calib(uint8_t dac_type, float k, float b)
{
    uint32_t sub_cmd;
    int ret = get_dac_calib_subcmd(dac_type, false, &sub_cmd);

    if (ret != FS_SUCCESS) {
        return ret;
    }

    return mcu_prot_store_calib_by_subcmd(sub_cmd, k, b);
}

int mcu_prot_read_dac_calib(uint8_t dac_type, float *k, float *b)
{
    uint32_t sub_cmd;
    int ret = get_dac_calib_subcmd(dac_type, false, &sub_cmd);

    if (ret != FS_SUCCESS) {
        return ret;
    }

    return mcu_prot_read_calib_by_subcmd(sub_cmd, k, b);
}

int mcu_prot_remove_dac_calib(uint8_t dac_type)
{
    uint32_t sub_cmd;
    int ret = get_dac_calib_subcmd(dac_type, true, &sub_cmd);

    if (ret != FS_SUCCESS) {
        return ret;
    }

    return mcu_prot_remove_calib_by_subcmd(sub_cmd);
}

int mcu_prot_get_fsync_diag(mcu_fsync_diag_t *diag)
{
    int ret = 0;
    uint8_t *rdata;
    uint16_t rdlen;
    uint8_t crc;
    uint32_t buf_len;
    mcu_prot_read_head_t *rh;
    mcu_prot_cmd_t cmd;

    if (diag == NULL) {
        return FS_ERR_BAD_PARAMS;
    }

    memset(&cmd, 0, sizeof(cmd));
    memset(diag, 0, sizeof(*diag));

    cmd.cmd_dev = MCU_PROT_DEV_MCU;
    cmd.cmd_op = MCU_PROT_OP_READ;
    cmd.sub_cmd0 = (MCU_PROT_MCU_FSYNC_DIAG >> 24) & 0xff;
    cmd.sub_cmd1 = (MCU_PROT_MCU_FSYNC_DIAG >> 16) & 0xff;
    cmd.sub_cmd2 = (MCU_PROT_MCU_FSYNC_DIAG >> 8) & 0xff;
    cmd.sub_cmd3 = (MCU_PROT_MCU_FSYNC_DIAG >> 0) & 0xff;
    cmd.crc8 = fs_util_crc_8(&cmd, sizeof(cmd) - 1, 0);

    pthread_mutex_lock(&mcu_prot_mutex);
    ret = mcu_prot_write(MCU_PROT_TYPE_NORMAL, (uint8_t *)&cmd, sizeof(cmd));
    if (ret < 0) {
        pthread_mutex_unlock(&mcu_prot_mutex);
        return ret;
    }

    ret = mcu_prot_wait_idle(2, 200);
    if (ret < 0) {
        pthread_mutex_unlock(&mcu_prot_mutex);
        return ret;
    }

    buf_len = MCU_PROT_READ_COMMON_LEN + sizeof(mcu_fsync_diag_t);
    rdata = (uint8_t *)malloc(buf_len);
    if (rdata == NULL) {
        pthread_mutex_unlock(&mcu_prot_mutex);
        return FS_ERR_NOMEM;
    }

    memset(rdata, 0, buf_len);
    rh = (mcu_prot_read_head_t *)rdata;
    ret = mcu_prot_read(MCU_PROT_TYPE_NORMAL, rdata, buf_len);
    if (ret < 0) {
        pthread_mutex_unlock(&mcu_prot_mutex);
        free(rdata);
        return ret;
    }

    rdlen = (rh->len0 << 8) | rh->len1;
    if (rdlen != sizeof(mcu_fsync_diag_t)) {
        pthread_mutex_unlock(&mcu_prot_mutex);
        free(rdata);
        return FS_ERR_MCU_PROT_CRC;
    }

    crc = fs_util_crc_8(rdata + sizeof(mcu_prot_read_head_t), rdlen, 0);
    if (crc != rdata[sizeof(mcu_prot_read_head_t) + rdlen]) {
        pthread_mutex_unlock(&mcu_prot_mutex);
        free(rdata);
        return FS_ERR_MCU_PROT_CRC;
    }

    memcpy(diag, rdata + sizeof(mcu_prot_read_head_t), sizeof(*diag));

    pthread_mutex_unlock(&mcu_prot_mutex);
    free(rdata);

    return FS_SUCCESS;
}

int mcu_prot_get_time_sync_diag(mcu_time_sync_diag_t *diag)
{
#if !FS_CFG_FEATURE_MCU_TIMESYNC
    if (diag != NULL) {
        memset(diag, 0, sizeof(*diag));
    }
    return FS_ERR_UNSUPPORT;
#else
    int ret = 0;
    uint8_t *rdata;
    uint16_t rdlen;
    uint8_t crc;
    uint32_t buf_len;
    mcu_prot_read_head_t *rh;
    mcu_prot_cmd_t cmd;

    if (diag == NULL) {
        return FS_ERR_BAD_PARAMS;
    }

    memset(&cmd, 0, sizeof(cmd));
    memset(diag, 0, sizeof(*diag));

    cmd.cmd_dev = MCU_PROT_DEV_MCU;
    cmd.cmd_op = MCU_PROT_OP_READ;
    cmd.sub_cmd0 = (MCU_PROT_MCU_TIME_SYNC_DIAG >> 24) & 0xff;
    cmd.sub_cmd1 = (MCU_PROT_MCU_TIME_SYNC_DIAG >> 16) & 0xff;
    cmd.sub_cmd2 = (MCU_PROT_MCU_TIME_SYNC_DIAG >> 8) & 0xff;
    cmd.sub_cmd3 = (MCU_PROT_MCU_TIME_SYNC_DIAG >> 0) & 0xff;
    cmd.crc8 = fs_util_crc_8(&cmd, sizeof(cmd) - 1, 0);

    pthread_mutex_lock(&mcu_prot_mutex);
    ret = mcu_prot_write(MCU_PROT_TYPE_NORMAL, (uint8_t *)&cmd, sizeof(cmd));
    if (ret < 0) {
        pthread_mutex_unlock(&mcu_prot_mutex);
        return ret;
    }

    ret = mcu_prot_wait_idle(2, 200);
    if (ret < 0) {
        pthread_mutex_unlock(&mcu_prot_mutex);
        return ret;
    }

    buf_len = MCU_PROT_READ_COMMON_LEN + sizeof(mcu_time_sync_diag_t);
    rdata = (uint8_t *)malloc(buf_len);
    if (rdata == NULL) {
        pthread_mutex_unlock(&mcu_prot_mutex);
        return FS_ERR_NOMEM;
    }

    memset(rdata, 0, buf_len);
    rh = (mcu_prot_read_head_t *)rdata;
    ret = mcu_prot_read(MCU_PROT_TYPE_NORMAL, rdata, buf_len);
    if (ret < 0) {
        pthread_mutex_unlock(&mcu_prot_mutex);
        free(rdata);
        return ret;
    }

    rdlen = (rh->len0 << 8) | rh->len1;
    if (rdlen != MCU_TIME_SYNC_DIAG_V2_SIZE &&
        rdlen != MCU_TIME_SYNC_DIAG_V4_SIZE &&
        rdlen != MCU_TIME_SYNC_DIAG_V5_SIZE) {
        pthread_mutex_unlock(&mcu_prot_mutex);
        free(rdata);
        return FS_ERR_MCU_PROT_CRC;
    }

    crc = fs_util_crc_8(rdata + sizeof(mcu_prot_read_head_t), rdlen, 0);
    if (crc != rdata[sizeof(mcu_prot_read_head_t) + rdlen]) {
        pthread_mutex_unlock(&mcu_prot_mutex);
        free(rdata);
        return FS_ERR_MCU_PROT_CRC;
    }

    memcpy(diag, rdata + sizeof(mcu_prot_read_head_t), rdlen);

    pthread_mutex_unlock(&mcu_prot_mutex);
    free(rdata);

    return FS_SUCCESS;
#endif
}

int mcu_prot_get_time_sync_health(mcu_time_sync_health_t *health)
{
#if !FS_CFG_FEATURE_MCU_TIMESYNC
    if (health != NULL) {
        memset(health, 0, sizeof(*health));
    }
    return FS_ERR_UNSUPPORT;
#else
    int ret = 0;
    uint8_t *rdata;
    uint16_t rdlen;
    uint8_t crc;
    uint32_t buf_len;
    mcu_prot_read_head_t *rh;
    mcu_prot_cmd_t cmd;

    if (health == NULL) {
        return FS_ERR_BAD_PARAMS;
    }

    memset(&cmd, 0, sizeof(cmd));
    memset(health, 0, sizeof(*health));

    cmd.cmd_dev = MCU_PROT_DEV_MCU;
    cmd.cmd_op = MCU_PROT_OP_READ;
    cmd.sub_cmd0 = (MCU_PROT_MCU_TIME_SYNC_HEALTH >> 24) & 0xff;
    cmd.sub_cmd1 = (MCU_PROT_MCU_TIME_SYNC_HEALTH >> 16) & 0xff;
    cmd.sub_cmd2 = (MCU_PROT_MCU_TIME_SYNC_HEALTH >> 8) & 0xff;
    cmd.sub_cmd3 = (MCU_PROT_MCU_TIME_SYNC_HEALTH >> 0) & 0xff;
    cmd.crc8 = fs_util_crc_8(&cmd, sizeof(cmd) - 1, 0);

    pthread_mutex_lock(&mcu_prot_mutex);
    ret = mcu_prot_write(MCU_PROT_TYPE_NORMAL, (uint8_t *)&cmd, sizeof(cmd));
    if (ret < 0) {
        pthread_mutex_unlock(&mcu_prot_mutex);
        return ret;
    }

    ret = mcu_prot_wait_idle(2, 200);
    if (ret < 0) {
        pthread_mutex_unlock(&mcu_prot_mutex);
        return ret;
    }

    buf_len = MCU_PROT_READ_COMMON_LEN + sizeof(mcu_time_sync_health_t);
    rdata = (uint8_t *)malloc(buf_len);
    if (rdata == NULL) {
        pthread_mutex_unlock(&mcu_prot_mutex);
        return FS_ERR_NOMEM;
    }

    memset(rdata, 0, buf_len);
    rh = (mcu_prot_read_head_t *)rdata;
    ret = mcu_prot_read(MCU_PROT_TYPE_NORMAL, rdata, buf_len);
    if (ret < 0) {
        pthread_mutex_unlock(&mcu_prot_mutex);
        free(rdata);
        return ret;
    }

    rdlen = (rh->len0 << 8) | rh->len1;
    if (rdlen != sizeof(mcu_time_sync_health_t)) {
        pthread_mutex_unlock(&mcu_prot_mutex);
        free(rdata);
        return FS_ERR_MCU_PROT_CRC;
    }

    crc = fs_util_crc_8(rdata + sizeof(mcu_prot_read_head_t), rdlen, 0);
    if (crc != rdata[sizeof(mcu_prot_read_head_t) + rdlen]) {
        pthread_mutex_unlock(&mcu_prot_mutex);
        free(rdata);
        return FS_ERR_MCU_PROT_CRC;
    }

    memcpy(health, rdata + sizeof(mcu_prot_read_head_t), sizeof(*health));

    pthread_mutex_unlock(&mcu_prot_mutex);
    free(rdata);

    return FS_SUCCESS;
#endif
}

int mcu_prot_time_sync_resync(void)
{
#if !FS_CFG_FEATURE_MCU_TIMESYNC
    return FS_ERR_UNSUPPORT;
#else
    int ret = 0;
    mcu_prot_cmd_t cmd;

    memset(&cmd, 0, sizeof(cmd));
    cmd.cmd_dev = MCU_PROT_DEV_MCU;
    cmd.cmd_op = MCU_PROT_OP_WRITE;
    cmd.sub_cmd0 = (MCU_PROT_MCU_TIME_SYNC_RESYNC >> 24) & 0xff;
    cmd.sub_cmd1 = (MCU_PROT_MCU_TIME_SYNC_RESYNC >> 16) & 0xff;
    cmd.sub_cmd2 = (MCU_PROT_MCU_TIME_SYNC_RESYNC >> 8) & 0xff;
    cmd.sub_cmd3 = (MCU_PROT_MCU_TIME_SYNC_RESYNC >> 0) & 0xff;
    cmd.crc8 = fs_util_crc_8(&cmd, sizeof(cmd) - 1, 0);

    pthread_mutex_lock(&mcu_prot_mutex);
    ret = mcu_prot_write(MCU_PROT_TYPE_NORMAL, (uint8_t *)&cmd, sizeof(cmd));
    if (ret < 0) {
        pthread_mutex_unlock(&mcu_prot_mutex);
        return ret;
    }

    ret = mcu_prot_wait_idle(2, 200);
    pthread_mutex_unlock(&mcu_prot_mutex);

    return ret;
#endif
}

int mcu_prot_time_sync_ctrl(uint8_t ctrl)
{
#if !FS_CFG_FEATURE_POINTCLOUD_TIMESYNC || !FS_CFG_FEATURE_MCU_TIMESYNC
    (void)ctrl;
    return FS_ERR_UNSUPPORT;
#else
    int ret = 0;
    mcu_prot_cmd_t cmd;

    memset(&cmd, 0, sizeof(cmd));
    cmd.cmd_dev  = MCU_PROT_DEV_MCU;
    cmd.cmd_op   = MCU_PROT_OP_WRITE;
    cmd.sub_cmd0 = (MCU_PROT_MCU_TIME_SYNC_CTRL >> 24) & 0xff;
    cmd.sub_cmd1 = (MCU_PROT_MCU_TIME_SYNC_CTRL >> 16) & 0xff;
    cmd.sub_cmd2 = (MCU_PROT_MCU_TIME_SYNC_CTRL >> 8) & 0xff;
    cmd.sub_cmd3 = (MCU_PROT_MCU_TIME_SYNC_CTRL >> 0) & 0xff;
    cmd.data0    = ctrl;
    cmd.crc8     = fs_util_crc_8(&cmd, sizeof(cmd) - 1, 0);

    pthread_mutex_lock(&mcu_prot_mutex);
    ret = mcu_prot_write(MCU_PROT_TYPE_NORMAL, (uint8_t *)&cmd, sizeof(cmd));
    if (ret < 0) {
        pthread_mutex_unlock(&mcu_prot_mutex);
        return ret;
    }

    ret = mcu_prot_wait_idle(2, 200);
    pthread_mutex_unlock(&mcu_prot_mutex);

    return ret;
#endif
}

static int mcu_prot_program_read_status_payload(mcu_prot_program_status_t *out_status)
{
    int ret;
    uint8_t *rdata;
    uint16_t rdlen;
    uint8_t crc;
    uint32_t buf_len;
    mcu_prot_read_head_t *rh;

    if (out_status == NULL) {
        return FS_ERR_BAD_PARAMS;
    }

    buf_len = MCU_PROT_READ_COMMON_LEN + (uint32_t)sizeof(mcu_prot_program_status_t);
    rdata = (uint8_t *)malloc(buf_len);
    if (rdata == NULL) {
        return FS_ERR_NOMEM;
    }

    memset(rdata, 0, buf_len);
    rh = (mcu_prot_read_head_t *)rdata;
    ret = mcu_prot_read(MCU_PROT_TYPE_NORMAL, rdata, buf_len);
    if (ret < 0) {
        free(rdata);
        return ret;
    }

    rdlen = (uint16_t)((rh->len0 << 8) | rh->len1);
    if (rdlen != sizeof(mcu_prot_program_status_t)) {
        free(rdata);
        return FS_ERR_MCU_PROT_CRC;
    }

    crc = fs_util_crc_8(rdata + sizeof(mcu_prot_read_head_t), rdlen, 0);
    if (crc != rdata[sizeof(mcu_prot_read_head_t) + rdlen]) {
        free(rdata);
        return FS_ERR_MCU_PROT_CRC;
    }

    memcpy(out_status, rdata + sizeof(mcu_prot_read_head_t), sizeof(*out_status));
    free(rdata);
    return FS_SUCCESS;
}

static int mcu_prot_program_cmd_write(uint32_t sub_cmd, mcu_prot_program_status_t *out_status)
{
    int ret;
    mcu_prot_cmd_t cmd;
    uint32_t timeout_ms = MCU_UART_REQ_TIMEOUT_MS;

    if (out_status == NULL) {
        return FS_ERR_BAD_PARAMS;
    }

    memset(&cmd, 0, sizeof(cmd));
    memset(out_status, 0, sizeof(*out_status));

    cmd.cmd_dev = MCU_PROT_DEV_MCU;
    cmd.cmd_op = MCU_PROT_OP_WRITE;
    cmd.sub_cmd0 = (sub_cmd >> 24) & 0xff;
    cmd.sub_cmd1 = (sub_cmd >> 16) & 0xff;
    cmd.sub_cmd2 = (sub_cmd >> 8) & 0xff;
    cmd.sub_cmd3 = (sub_cmd >> 0) & 0xff;
    cmd.crc8 = fs_util_crc_8(&cmd, sizeof(cmd) - 1, 0);
    if (sub_cmd == MCU_PROT_MCU_PROGRAM_EXIT) {
        timeout_ms = MCU_UART_PROGRAM_EXIT_TIMEOUT_MS;
    }

    pthread_mutex_lock(&mcu_prot_mutex);
    ret = mcu_prot_write_with_timeout(MCU_PROT_TYPE_NORMAL, (uint8_t *)&cmd, sizeof(cmd), timeout_ms);
    if (ret < 0) {
        pthread_mutex_unlock(&mcu_prot_mutex);
        return ret;
    }

    ret = mcu_prot_wait_idle(2, 200);
    if (ret < 0) {
        pthread_mutex_unlock(&mcu_prot_mutex);
        return ret;
    }

    ret = mcu_prot_program_read_status_payload(out_status);
    pthread_mutex_unlock(&mcu_prot_mutex);
    return ret;
}

static int mcu_prot_program_status_from_payload(const mcu_prot_program_status_t *status)
{
    if (status == NULL) {
        return FS_ERR_BAD_PARAMS;
    }
    if (status->schema_version != MCU_PROT_PROGRAM_STATUS_SCHEMA_VERSION) {
        return FS_ERR_INTERNAL;
    }
    if (status->result != 0u) {
        if (status->reject_reason == (uint8_t)FW_IF_REJECT_BUSY) {
            return FS_ERR_BUSY;
        }
        if (status->reject_reason == (uint8_t)FW_IF_REJECT_INVALID_STATE) {
            return FS_ERR_UNSUPPORT;
        }
        return FS_ERR_INTERNAL;
    }
    return FS_SUCCESS;
}

int mcu_prot_program_enter(mcu_prot_program_status_t *out_status)
{
    int ret;
    mcu_prot_program_status_t local;

    if (out_status == NULL) {
        out_status = &local;
    }

    ret = mcu_prot_program_cmd_write(MCU_PROT_MCU_PROGRAM_ENTER, out_status);
    if (ret != FS_SUCCESS) {
        return ret;
    }
    return mcu_prot_program_status_from_payload(out_status);
}

int mcu_prot_program_exit(mcu_prot_program_status_t *out_status)
{
    int ret;
    mcu_prot_program_status_t local;

    if (out_status == NULL) {
        out_status = &local;
    }

    ret = mcu_prot_program_cmd_write(MCU_PROT_MCU_PROGRAM_EXIT, out_status);
    if (ret != FS_SUCCESS) {
        return ret;
    }
    return mcu_prot_program_status_from_payload(out_status);
}

int mcu_prot_program_get_status(mcu_prot_program_status_t *out_status)
{
    int ret;
    mcu_prot_cmd_t cmd;

    if (out_status == NULL) {
        return FS_ERR_BAD_PARAMS;
    }

    memset(&cmd, 0, sizeof(cmd));
    memset(out_status, 0, sizeof(*out_status));

    cmd.cmd_dev = MCU_PROT_DEV_MCU;
    cmd.cmd_op = MCU_PROT_OP_READ;
    cmd.sub_cmd0 = (MCU_PROT_MCU_PROGRAM_STATUS >> 24) & 0xff;
    cmd.sub_cmd1 = (MCU_PROT_MCU_PROGRAM_STATUS >> 16) & 0xff;
    cmd.sub_cmd2 = (MCU_PROT_MCU_PROGRAM_STATUS >> 8) & 0xff;
    cmd.sub_cmd3 = (MCU_PROT_MCU_PROGRAM_STATUS >> 0) & 0xff;
    cmd.crc8 = fs_util_crc_8(&cmd, sizeof(cmd) - 1, 0);

    pthread_mutex_lock(&mcu_prot_mutex);
    ret = mcu_prot_write(MCU_PROT_TYPE_NORMAL, (uint8_t *)&cmd, sizeof(cmd));
    if (ret < 0) {
        pthread_mutex_unlock(&mcu_prot_mutex);
        return ret;
    }

    ret = mcu_prot_wait_idle(2, 200);
    if (ret < 0) {
        pthread_mutex_unlock(&mcu_prot_mutex);
        return ret;
    }

    ret = mcu_prot_program_read_status_payload(out_status);
    pthread_mutex_unlock(&mcu_prot_mutex);
    if (ret != FS_SUCCESS) {
        return ret;
    }
    return mcu_prot_program_status_from_payload(out_status);
}

int mcu_prot_get_imu_push_diag(mcu_imu_push_diag_t *diag)
{
#if !FS_CFG_FEATURE_MCU_IMU_PUSH
    if (diag == NULL) {
        return FS_ERR_BAD_PARAMS;
    }

    memset(diag, 0, sizeof(*diag));
    return FS_ERR_UNSUPPORT;
#else
    int ret = 0;
    uint8_t *rdata;
    uint16_t rdlen;
    uint8_t crc;
    uint32_t buf_len;
    mcu_prot_read_head_t *rh;
    mcu_prot_cmd_t cmd;

    if (diag == NULL) {
        return FS_ERR_BAD_PARAMS;
    }

    memset(&cmd, 0, sizeof(cmd));
    memset(diag, 0, sizeof(*diag));

    cmd.cmd_dev = MCU_PROT_DEV_MCU;
    cmd.cmd_op = MCU_PROT_OP_READ;
    cmd.sub_cmd0 = (MCU_PROT_MCU_IMU_PUSH_DIAG >> 24) & 0xff;
    cmd.sub_cmd1 = (MCU_PROT_MCU_IMU_PUSH_DIAG >> 16) & 0xff;
    cmd.sub_cmd2 = (MCU_PROT_MCU_IMU_PUSH_DIAG >> 8) & 0xff;
    cmd.sub_cmd3 = (MCU_PROT_MCU_IMU_PUSH_DIAG >> 0) & 0xff;
    cmd.crc8 = fs_util_crc_8(&cmd, sizeof(cmd) - 1, 0);

    pthread_mutex_lock(&mcu_prot_mutex);
    ret = mcu_prot_write(MCU_PROT_TYPE_NORMAL, (uint8_t *)&cmd, sizeof(cmd));
    if (ret < 0) {
        pthread_mutex_unlock(&mcu_prot_mutex);
        return ret;
    }

    ret = mcu_prot_wait_idle(2, 200);
    if (ret < 0) {
        pthread_mutex_unlock(&mcu_prot_mutex);
        return ret;
    }

    buf_len = MCU_PROT_READ_COMMON_LEN + MCU_IMU_PUSH_DIAG_SIZE;
    rdata = (uint8_t *)malloc(buf_len);
    if (rdata == NULL) {
        pthread_mutex_unlock(&mcu_prot_mutex);
        return FS_ERR_NOMEM;
    }

    memset(rdata, 0, buf_len);
    rh = (mcu_prot_read_head_t *)rdata;
    ret = mcu_prot_read(MCU_PROT_TYPE_NORMAL, rdata, buf_len);
    if (ret < 0) {
        pthread_mutex_unlock(&mcu_prot_mutex);
        free(rdata);
        return ret;
    }

    rdlen = (rh->len0 << 8) | rh->len1;
    if (rdlen != MCU_IMU_PUSH_DIAG_V1_SIZE
        && rdlen != MCU_IMU_PUSH_DIAG_V5_SIZE
        && rdlen != MCU_IMU_PUSH_DIAG_V6_SIZE
        && rdlen != MCU_IMU_PUSH_DIAG_SIZE) {
        pthread_mutex_unlock(&mcu_prot_mutex);
        free(rdata);
        return FS_ERR_MCU_PROT_CRC;
    }

    crc = fs_util_crc_8(rdata + sizeof(mcu_prot_read_head_t), rdlen, 0);
    if (crc != rdata[sizeof(mcu_prot_read_head_t) + rdlen]) {
        pthread_mutex_unlock(&mcu_prot_mutex);
        free(rdata);
        return FS_ERR_MCU_PROT_CRC;
    }

    memcpy(diag, rdata + sizeof(mcu_prot_read_head_t), rdlen);

    pthread_mutex_unlock(&mcu_prot_mutex);
    free(rdata);

    return FS_SUCCESS;
#endif
}


int mcu_prot_mcu_enter_upgrade_mode_with_status(mcu_prot_program_status_t *out_status)
{
    return mcu_prot_program_cmd_write(MCU_PROT_MCU_UPGRADE, out_status);
}

int mcu_prot_mcu_enter_upgrade_mode(void)
{
    int ret;
    mcu_prot_program_status_t status;

    ret = mcu_prot_mcu_enter_upgrade_mode_with_status(&status);
    if (ret != FS_SUCCESS) {
        return ret;
    }

    return mcu_prot_program_status_from_payload(&status);
}

int mcu_prot_flash_write_session_begin(int data_type)
{
    return soc_mgr_cfg_cali_transfer_begin(data_type);
}

void mcu_prot_flash_write_session_end(void)
{
    soc_mgr_cfg_cali_transfer_end();
}

static int mcu_prot_flash_cmd_dev_from_type(int data_type, uint8_t *cmd_dev)
{
    if (cmd_dev == NULL) {
        return FS_ERR_BAD_PARAMS;
    }

    if (data_type == MCU_PROT_FLASH_DT_CFG) {
        *cmd_dev = MCU_PROT_DEV_CFGDATA;
        return FS_SUCCESS;
    }

    if (data_type == MCU_PROT_FLASH_DT_CALI) {
        *cmd_dev = MCU_PROT_DEV_CALIDATA;
        return FS_SUCCESS;
    }

    return FS_ERR_BAD_PARAMS;
}

int mcu_prot_flash_write_start(int data_type, mcu_prot_flash_info_t* info)
{
    int ret = 0;
    uint8_t rdata[13];
    mcu_prot_flash_cmd_t cmd;
    uint8_t crc;
    uint16_t dlen;

    if (info == NULL) {
        return FS_ERR_BAD_PARAMS;
    }
    if (info->total_len == 0u) {
        return FS_ERR_BAD_PARAMS;
    }
    if (!soc_mgr_cfg_cali_transfer_busy()) {
        return FS_ERR_BUSY;
    }

    memset(&cmd, 0, sizeof(cmd));

    ret = mcu_prot_flash_cmd_dev_from_type(data_type, &cmd.cmd_dev);
    if (ret != FS_SUCCESS) {
        return ret;
    }

    cmd.cmd_op = MCU_PROT_OP_WRITE;
    cmd.sub_cmd0 = (MCU_PROT_FLASH_WRITE_START >> 24) & 0xff;
    cmd.sub_cmd1 = (MCU_PROT_FLASH_WRITE_START >> 16) & 0xff;
    cmd.sub_cmd2 = (MCU_PROT_FLASH_WRITE_START >> 8) & 0xff;
    cmd.sub_cmd3 = (MCU_PROT_FLASH_WRITE_START >> 0) & 0xff;
    // total length
    cmd.d0 = (info->total_len >> 24) & 0xff;
    cmd.d1 = (info->total_len >> 16) & 0xff;
    cmd.d2 = (info->total_len >> 8) & 0xff;
    cmd.d3 = (info->total_len >> 0) & 0xff;
    cmd.crc8 = fs_util_crc_8(&cmd, sizeof(cmd) - 1, 0);

    pthread_mutex_lock(&mcu_prot_mutex);
    ret = mcu_prot_write(MCU_PROT_TYPE_FLASH, (uint8_t*)&cmd, sizeof(cmd));
    if (ret < 0) {
        pthread_mutex_unlock(&mcu_prot_mutex);
        goto write_fail;
    }

    ret = mcu_prot_wait_idle(200, 20000);
    if (ret < 0) {
        pthread_mutex_unlock(&mcu_prot_mutex);
        goto write_fail;
    }

    memset(&rdata[0], 0, 13);
    ret = mcu_prot_read(MCU_PROT_TYPE_FLASH, &rdata[0], 13);
    if (ret < 0) {
        pthread_mutex_unlock(&mcu_prot_mutex);
        goto write_fail;
    }

    dlen = (rdata[6] << 8) | rdata[7];
    if (dlen != 4u) {
        ret = FS_ERR_MCU_PROT_ERR;
        pthread_mutex_unlock(&mcu_prot_mutex);
        return ret;
    }

    crc = fs_util_crc_8(&rdata[8], dlen, 0);
    if (crc != rdata[12]) {
        ret = FS_ERR_MCU_PROT_CRC;
        pthread_mutex_unlock(&mcu_prot_mutex);
        goto write_fail;
    }

    info->pkt_num = (rdata[8] << 8) | rdata[9];
    info->pkt_len = (rdata[10] << 8) | rdata[11];
    if (info->pkt_num == 0u || info->pkt_len == 0u) {
        ret = FS_ERR_MCU_PROT_ERR;
    }
    pthread_mutex_unlock(&mcu_prot_mutex);

    return ret;

write_fail:
    return ret;
}

int mcu_prot_flash_write_pkt_data(int data_type, mcu_prot_flash_pkt_info_t* pkt_info)
{
    int ret = 0;
    uint8_t* data;
    uint8_t rdata[13];
    mcu_prot_flash_wcmd_head_t* h;
    uint8_t crc;
    uint16_t dlen;
    uint16_t pkt_idx;
    uint16_t status;
    uint16_t hlen;

    if (pkt_info == NULL || pkt_info->data == NULL || pkt_info->pkt_len == 0u) {
        return FS_ERR_BAD_PARAMS;
    }

    FS_LOGI("prot_flash_write_pkt_data: %u %u",
            (unsigned int)pkt_info->pkt_idx,
            (unsigned int)pkt_info->pkt_len);

    hlen = sizeof(mcu_prot_flash_wcmd_head_t);
    dlen = pkt_info->pkt_len + hlen + 1;
    data = (uint8_t*)malloc(dlen);
    if (!data) {
        return FS_ERR_NOMEM;
    }
    memset(data, 0, dlen);

    h = (mcu_prot_flash_wcmd_head_t*)data;
    ret = mcu_prot_flash_cmd_dev_from_type(data_type, &h->cmd_dev);
    if (ret != FS_SUCCESS) {
        free(data);
        return ret;
    }
    h->cmd_op = MCU_PROT_OP_WRITE;
    h->sub_cmd0 = (MCU_PROT_FLASH_WRITE_DATA >> 24) & 0xff;
    h->sub_cmd1 = (MCU_PROT_FLASH_WRITE_DATA >> 16) & 0xff;
    h->sub_cmd2 = (MCU_PROT_FLASH_WRITE_DATA >> 8) & 0xff;
    h->sub_cmd3 = (MCU_PROT_FLASH_WRITE_DATA >> 0) & 0xff;
    h->d0 = (pkt_info->pkt_idx >> 8) & 0xff;
    h->d1 = (pkt_info->pkt_idx >> 0) & 0xff;
    h->d2 = (pkt_info->pkt_len >> 8) & 0xff;
    h->d3 = (pkt_info->pkt_len >> 0) & 0xff;
    memcpy(data + hlen, pkt_info->data, pkt_info->pkt_len);

    data[dlen - 1] = fs_util_crc_8(data, dlen - 1, 0);

    pthread_mutex_lock(&mcu_prot_mutex);
    ret = mcu_prot_write(MCU_PROT_TYPE_FLASH, data, dlen);
    if (ret < 0) {
        free(data);
        pthread_mutex_unlock(&mcu_prot_mutex);
        return ret;
    }

    ret = mcu_prot_wait_idle(3, 5000);
    if (ret < 0) {
        free(data);
        pthread_mutex_unlock(&mcu_prot_mutex);
        return ret;
    }

    memset(&rdata[0], 0, 13);
    ret = mcu_prot_read(MCU_PROT_TYPE_FLASH, &rdata[0], 13);
    if (ret < 0) {
        free(data);
        pthread_mutex_unlock(&mcu_prot_mutex);
        return ret;
    }

    dlen = (rdata[6] << 8) | rdata[7];
    if (dlen != 4u) {
        ret = FS_ERR_MCU_PROT_ERR;
        free(data);
        pthread_mutex_unlock(&mcu_prot_mutex);
        return ret;
    }

    crc = fs_util_crc_8(&rdata[8], dlen, 0);
    if (crc != rdata[12]) {
        ret = FS_ERR_MCU_PROT_CRC;
        free(data);
        pthread_mutex_unlock(&mcu_prot_mutex);
        return ret;
    }

    pkt_idx = (rdata[8] << 8) | rdata[9];
    status = (rdata[10] << 8) | rdata[11];
    if (pkt_idx != pkt_info->pkt_idx || status != 0) {
        ret = FS_ERR_MCU_PROT_ERR;
    }
    pthread_mutex_unlock(&mcu_prot_mutex);
    free(data);

    return ret;
}

int mcu_prot_flash_read_start(int data_type, mcu_prot_flash_info_t* info)
{
    int ret = 0;
    uint8_t rdata[17];
    mcu_prot_flash_cmd_t cmd;
    uint8_t crc;
    uint16_t dlen;

    if (info == NULL) {
        return FS_ERR_BAD_PARAMS;
    }

    memset(&cmd, 0, sizeof(cmd));

    ret = mcu_prot_flash_cmd_dev_from_type(data_type, &cmd.cmd_dev);
    if (ret != FS_SUCCESS) {
        return ret;
    }

    cmd.cmd_op = MCU_PROT_OP_WRITE;
    cmd.sub_cmd0 = (MCU_PROT_FLASH_READ_START >> 24) & 0xff;
    cmd.sub_cmd1 = (MCU_PROT_FLASH_READ_START >> 16) & 0xff;
    cmd.sub_cmd2 = (MCU_PROT_FLASH_READ_START >> 8) & 0xff;
    cmd.sub_cmd3 = (MCU_PROT_FLASH_READ_START >> 0) & 0xff;
    cmd.crc8 = fs_util_crc_8(&cmd, sizeof(cmd) - 1, 0);

    pthread_mutex_lock(&mcu_prot_mutex);
    ret = mcu_prot_write(MCU_PROT_TYPE_FLASH, (uint8_t*)&cmd, sizeof(cmd));
    if (ret < 0) {
        pthread_mutex_unlock(&mcu_prot_mutex);
        return ret;
    }

    ret = mcu_prot_wait_idle(10, 2000);
    if (ret < 0) {
        pthread_mutex_unlock(&mcu_prot_mutex);
        return ret;
    }

    memset(&rdata[0], 0, 17);
    ret = mcu_prot_read(MCU_PROT_TYPE_FLASH, &rdata[0], 17);
    if (ret < 0) {
        pthread_mutex_unlock(&mcu_prot_mutex);
        return ret;
    }

    dlen = (rdata[6] << 8) | rdata[7];
    if (dlen != 8u) {
        ret = FS_ERR_MCU_PROT_ERR;
        pthread_mutex_unlock(&mcu_prot_mutex);
        return ret;
    }

    crc = fs_util_crc_8(&rdata[8], dlen, 0);
    if (crc != rdata[dlen + 8]) {
        ret = FS_ERR_MCU_PROT_CRC;
        pthread_mutex_unlock(&mcu_prot_mutex);
        return ret;
    }

    info->total_len = (rdata[8] << 24) | (rdata[9] << 16) | (rdata[10] << 8) | rdata[11];
    info->pkt_num = (rdata[12] << 8) | rdata[13];
    info->pkt_len = (rdata[14] << 8) | rdata[15];

    if (info->total_len == 0u && info->pkt_num == 0u && info->pkt_len == 0u) {
        ret = FS_ERR_NOT_FOUND;
    }
    else if (info->total_len == 0u || info->pkt_num == 0u || info->pkt_len == 0u) {
        ret = FS_ERR_MCU_PROT_ERR;
    }

    pthread_mutex_unlock(&mcu_prot_mutex);

    return ret;
}

int mcu_prot_flash_read_pkt_data(int data_type, mcu_prot_flash_pkt_info_t* pkt_info)
{
    int ret = 0;
    uint8_t* rdata;
    uint16_t rlen;
    mcu_prot_flash_cmd_t cmd;
    uint8_t crc;
    uint16_t dlen;
    uint16_t pkt_idx;
    uint16_t pkt_len;
    uint16_t hlen = 12;
    const uint16_t log_interval = 64u;

    if (pkt_info == NULL || pkt_info->data == NULL || pkt_info->pkt_len == 0u) {
        return FS_ERR_BAD_PARAMS;
    }

    if (pkt_info->pkt_idx == 0u || (pkt_info->pkt_idx % log_interval) == 0u) {
        FS_LOGI("prot_flash_read_pkt_data: %u %u",
                (unsigned int)pkt_info->pkt_idx,
                (unsigned int)pkt_info->pkt_len);
    }

    memset(&cmd, 0, sizeof(cmd));
    ret = mcu_prot_flash_cmd_dev_from_type(data_type, &cmd.cmd_dev);
    if (ret != FS_SUCCESS) {
        return ret;
    }
    cmd.cmd_op = MCU_PROT_OP_READ;
    cmd.sub_cmd0 = (MCU_PROT_FLASH_READ_DATA >> 24) & 0xff;
    cmd.sub_cmd1 = (MCU_PROT_FLASH_READ_DATA >> 16) & 0xff;
    cmd.sub_cmd2 = (MCU_PROT_FLASH_READ_DATA >> 8) & 0xff;
    cmd.sub_cmd3 = (MCU_PROT_FLASH_READ_DATA >> 0) & 0xff;
    cmd.d0 = (pkt_info->pkt_idx >> 8) & 0xff;
    cmd.d1 = (pkt_info->pkt_idx >> 0) & 0xff;
    cmd.d2 = (pkt_info->pkt_len >> 8) & 0xff;
    cmd.d3 = (pkt_info->pkt_len >> 0) & 0xff;
    cmd.crc8 = fs_util_crc_8(&cmd, sizeof(cmd) - 1, 0);

    rlen = pkt_info->pkt_len + hlen + 1;
    rdata = (uint8_t*)malloc(rlen);
    if (!rdata) {
        return FS_ERR_NOMEM;
    }

    pthread_mutex_lock(&mcu_prot_mutex);
    ret = mcu_prot_write(MCU_PROT_TYPE_FLASH, (uint8_t*)&cmd, sizeof(cmd));
    if (ret < 0) {
        free(rdata);
        pthread_mutex_unlock(&mcu_prot_mutex);
        return ret;
    }

    ret = mcu_prot_wait_idle(3, 1000);
    if (ret < 0) {
        free(rdata);
        pthread_mutex_unlock(&mcu_prot_mutex);
        return ret;
    }

    memset(rdata, 0, rlen);
    ret = mcu_prot_read(MCU_PROT_TYPE_FLASH, rdata, rlen);
    if (ret < 0) {
        free(rdata);
        pthread_mutex_unlock(&mcu_prot_mutex);
        return ret;
    }

    dlen = (rdata[6] << 8) | rdata[7];
    if (dlen != (uint16_t)(pkt_info->pkt_len + 4u)) {
        ret = FS_ERR_MCU_PROT_ERR;
        free(rdata);
        pthread_mutex_unlock(&mcu_prot_mutex);
        return ret;
    }

    crc = fs_util_crc_8(&rdata[8], dlen, 0);
    if (crc != rdata[rlen - 1]) {
        ret = FS_ERR_MCU_PROT_CRC;
        free(rdata);
        pthread_mutex_unlock(&mcu_prot_mutex);
        return ret;
    }

    pkt_idx = (rdata[8] << 8) | rdata[9];
    pkt_len = (rdata[10] << 8) | rdata[11];
    if (pkt_idx != pkt_info->pkt_idx || pkt_len != pkt_info->pkt_len) {
        FS_LOGE("pkt_idx or pkt_len not matched");
        ret = FS_ERR_MCU_PROT_ERR;
        free(rdata);
        pthread_mutex_unlock(&mcu_prot_mutex);
        return ret;
    }

    memcpy(pkt_info->data, &rdata[12], pkt_len);

    free(rdata);
    pthread_mutex_unlock(&mcu_prot_mutex);

    return 0;
}

int mcu_prot_send_flash_data(uint8_t data_type, uint8_t* data, uint32_t len)
{
    int ret = 0;
    uint16_t pkt_idx;
    mcu_prot_flash_info_t info;
    mcu_prot_flash_pkt_info_t pkt_info;
    uint8_t retry;

    if (data == NULL || len == 0u) {
        return FS_ERR_BAD_PARAMS;
    }

    ret = soc_mgr_cfg_cali_transfer_begin((int)data_type);
    if (ret != FS_SUCCESS) {
        return ret;
    }

    info.total_len = len;
    ret = mcu_prot_flash_write_start(data_type, &info);
    if (ret < 0) {
        goto flash_done;
    }

    FS_LOGI("[type:%d] pkt_num=%d, pkt_len=%d", data_type, info.pkt_num, info.pkt_len);
    if (info.pkt_num == 0u || info.pkt_len == 0u) {
        ret = FS_ERR_MCU_PROT_ERR;
        goto flash_done;
    }

    for (pkt_idx = 0; pkt_idx < info.pkt_num; pkt_idx++) {
        retry = 5;
        while (retry > 0) {
            uint32_t remain;
            pkt_info.pkt_idx = pkt_idx;
            remain = len - info.pkt_len * pkt_idx;

            if (remain >= info.pkt_len) {
                pkt_info.pkt_len = info.pkt_len;
            } else {
                pkt_info.pkt_len = remain;
            }
            pkt_info.data = data + info.pkt_len * pkt_idx;

            ret = mcu_prot_flash_write_pkt_data(data_type, &pkt_info);
            if (ret == 0) {
                break;
            } else {
                FS_LOGE("flash write pkt %d error, retry:%d, ret=%d", pkt_idx, retry, ret);
            }
            retry--;
        }

        if (ret) {
            break;
        }
    }

flash_done:
    soc_mgr_cfg_cali_transfer_end();
    return ret;
}

/* data的内存需要在调用这个函数返回成功后释放 */
int mcu_prot_recv_flash_data(uint8_t data_type, uint8_t** data, uint32_t* len)
{
    int ret;
    mcu_prot_flash_info_t info;
    mcu_prot_flash_pkt_info_t pkt_info;
    uint16_t i;
    uint32_t remain;
    int retry;

    if (data == NULL || len == NULL) {
        return FS_ERR_BAD_PARAMS;
    }

    *data = NULL;
    *len = 0u;

    ret = mcu_prot_flash_read_start(data_type, &info);
    if (ret < 0) {
        return ret;
    }

    FS_LOGI("[type:%d]:total_len=%d, pkt_num=%d,pkt_len=%d", data_type, info.total_len, info.pkt_num, info.pkt_len);

    if (info.total_len == 0) {
        return FS_ERR_BAD_PARAMS;
    }
    if (info.pkt_num == 0u || info.pkt_len == 0u) {
        return FS_ERR_MCU_PROT_ERR;
    }

    *len = info.total_len;
    *data = (uint8_t*)malloc(*len);
    if (!(*data)) {
        *len = 0u;
        return FS_ERR_NOMEM;
    }

    for (i = 0; i < info.pkt_num; i++) {
        retry = 5;

        while (retry) {
            retry--;
            pkt_info.pkt_idx = i;
            remain = info.total_len - info.pkt_len * i;

            if (remain >= info.pkt_len) {
                pkt_info.pkt_len = info.pkt_len;
            } else {
                pkt_info.pkt_len = remain;
            }
            pkt_info.data = (*data) + info.pkt_len * i;
            ret = mcu_prot_flash_read_pkt_data(data_type, &pkt_info);
            if (ret == 0) {
                break;
            } else {
                FS_LOGE("flash read pkt %d error, retry:%d, ret=%d", i, retry, ret);
            }
        };

        if (ret) {
            break;
        }
    }

    if (ret) {
        free(*data);
        *data = NULL;
        *len = 0u;
    }

    return ret;
}

static void *mcu_prot_push_msg_handler(void *param)
{
    int ret;

    uint8_t payload[MCU_PROT_BUFFER_SIZE] = {0};
    uint16_t payload_len = 0u;
    mcu_uart_frame_meta_t meta;
    /* 跨调用持久化解析器：避免100ms超时后丢弃半帧导致后续帧字节被误判为ASCII文本 */
    mcu_uart_parser_t passive_parser;

    (void)param;

    mcu_uart_parser_init(&passive_parser);

    while (mcu_push_msg_run) {
        if (mcu_uart_passive_listen_enabled()) {
            ret = mcu_uart_recv_any_frame(payload,
                                          sizeof(payload),
                                          &payload_len,
                                          &meta,
                                          100u,
                                          &passive_parser);
            if (ret == FS_ERR_MCU_PROT_TIMEOUT) {
                continue;
            }
            if (ret != FS_SUCCESS) {
                FS_LOGE("uart passive listener failed: %d", ret);
                fs_hw_msleep(20);
                continue;
            }

            if (mcu_uart_debug_enabled()) {
                FS_LOGI("UART passive rx: type=%u txid=%u len=%u", meta.msg_type, meta.txid, payload_len);
            }

            if (meta.msg_type == MCU_UART_MSG_PUSH) {
                mcu_uart_store_push_frame(payload, payload_len);
                mcu_uart_consume_imu_push();
                continue;
            }

            if (meta.msg_type == MCU_UART_MSG_RESP) {
                mcu_uart_store_resp_slot(meta.txid, payload, payload_len);
                g_mcu_uart_diag.resp_slot_store++;
            }
            continue;
        }

        // 使用较短超时，快速响应退出信号
        ret = mcu_int_wait_timeout(100);
        if (ret == 0) {
            // FS_LOGI("Detect MCU Push Msg interrup");
        }
    }

    return NULL;
}

static void *mcu_prot_periodic_req_test_handler(void *param)
{
    uint32_t val = 0;
    uint8_t dev = mcu_uart_periodic_req_dev();
    uint32_t addr = mcu_uart_periodic_req_addr();
    uint32_t interval_ms = mcu_uart_periodic_req_interval_ms();
    int periodic_req_en = mcu_uart_periodic_req_test_enabled();
    int last_fsync_diag_en = -1;
    int last_time_sync_diag_en = -1;
    int last_imu_push_diag_en = -1;

    (void)param;

    FS_LOGI("UART periodic helper start: req_en=%d fsync_req=%d time_sync_req=%d imu_push_req=%d req_interval=%ums fsync_interval=%ums",
            periodic_req_en,
            mcu_uart_fsync_diag_poll_enabled(),
            mcu_uart_time_sync_diag_poll_enabled(),
            mcu_uart_imu_push_diag_poll_enabled(),
            interval_ms,
            mcu_uart_fsync_diag_poll_interval_ms());

    if (periodic_req_en) {
        FS_LOGI("UART periodic req target: dev=%u addr=0x%08x", dev, addr);
    }

    while (g_mcu_uart_periodic_req_run) {
        int fsync_diag_en = mcu_uart_fsync_diag_poll_enabled();
        int time_sync_diag_en = mcu_uart_time_sync_diag_poll_enabled();
        int imu_push_diag_en = mcu_uart_imu_push_diag_poll_enabled();

        if (fsync_diag_en != last_fsync_diag_en && fsync_diag_en == 0) {
            FS_LOGI("MCU fsync diag runtime trigger %s (file=%s)",
                    fsync_diag_en ? "enabled" : "disabled",
                    MCU_UART_FSYNC_DIAG_POLL_FILE);
            last_fsync_diag_en = fsync_diag_en;
        }
        if (time_sync_diag_en != last_time_sync_diag_en && time_sync_diag_en == 0) {
            FS_LOGI("MCU time sync diag runtime trigger %s (file=%s)",
                    time_sync_diag_en ? "enabled" : "disabled",
                    MCU_UART_TIME_SYNC_DIAG_POLL_FILE);
            last_time_sync_diag_en = time_sync_diag_en;
        }
        if (imu_push_diag_en != last_imu_push_diag_en && imu_push_diag_en == 0) {
            FS_LOGI("MCU imu push diag runtime trigger %s (file=%s)",
                    imu_push_diag_en ? "enabled" : "disabled",
                    MCU_UART_IMU_PUSH_DIAG_POLL_FILE);
            last_imu_push_diag_en = imu_push_diag_en;
        }

        if (periodic_req_en) {
            int ret;

            if (dev >= MCU_PROT_DEV_FL9103_1 && dev <= MCU_PROT_DEV_FL9103_ALL) {
                ret = mcu_prot_get_fl9103_reg((uint8_t)(dev - MCU_PROT_DEV_FL9103_1), addr, &val);
            } else {
                ret = mcu_prot_get_fl9003_reg(dev, addr, &val);
            }

            if (ret == FS_SUCCESS) {
                FS_LOGI("UART periodic req ok: dev=%u addr=0x%08x val=0x%08x", dev, addr, val);
            } else {
                FS_LOGE("UART periodic req failed: dev=%u addr=0x%08x ret=%d", dev, addr, ret);
            }
        }

        if (fsync_diag_en) {
            mcu_fsync_diag_t diag;
            int ret;

            last_fsync_diag_en = 1;
            FS_LOGI("MCU fsync diag one-shot trigger (file=%s)", MCU_UART_FSYNC_DIAG_POLL_FILE);
            ret = mcu_prot_get_fsync_diag(&diag);
            if (ret == FS_SUCCESS) {
                FS_LOGI("MCU fsync diag[poll]: trigger_irq=%u trigger_ig=%u busy_irq=%u busy_ig=%u fsync_hi=%u fsync_lo=%u flags=0x%08x",
                        diag.trigger_irq_total,
                        diag.trigger_ignored_total,
                        diag.busy_irq_total,
                        diag.busy_ignored_total,
                        diag.fsync_high_total,
                        diag.fsync_low_total,
                        diag.flags);
            } else {
                FS_LOGE("MCU fsync diag poll failed: %d", ret);
            }

            mcu_uart_write_int_file(MCU_UART_FSYNC_DIAG_POLL_FILE, 0);
        }

        if (time_sync_diag_en) {
            mcu_time_sync_diag_t diag;
            int ret;

            last_time_sync_diag_en = 1;
            FS_LOGI("MCU time sync diag one-shot trigger (file=%s)", MCU_UART_TIME_SYNC_DIAG_POLL_FILE);
            ret = mcu_prot_get_time_sync_diag(&diag);
            if (ret == FS_SUCCESS) {
                struct timespec ts_now;
                (void)clock_gettime(CLOCK_REALTIME, &ts_now);
                uint64_t t_soc_now_ns = (uint64_t)ts_now.tv_sec * 1000000000ull
                                        + (uint64_t)ts_now.tv_nsec;
                uint64_t now_soc_est = ((uint64_t)diag.now_soc_est_ns_hi << 32)
                                       | (uint64_t)diag.now_soc_est_ns_lo;
                uint64_t anchor_soc  = ((uint64_t)diag.anchor_soc_ns_hi << 32)
                                       | (uint64_t)diag.anchor_soc_ns_lo;
                uint32_t soc_sync_drop_total = diag.soc_sync_drop_invalid_count
                                             + diag.soc_sync_drop_queue_full_count;
                uint32_t sync_pair_drop_total = diag.sync_pair_drop_trigger_old_count
                                              + diag.sync_pair_drop_soc_old_count
                                              + diag.sync_pair_drop_rx_early_count
                                              + diag.sync_pair_drop_rx_late_count
                                              + diag.sync_pair_drop_invalid_count
                                              + diag.sync_pair_drop_align_mismatch_count;

                if (now_soc_est > 0u) {
                    /* offset > 0: MCU estimate is in the past relative to SoC now.
                     * Expected ~5-15 ms (UART latency + processing).
                     * Large or drifting offset indicates mapping error. */
                    int64_t offset_ns = (int64_t)(t_soc_now_ns - now_soc_est);
                    int64_t offset_ms = offset_ns / 1000000;

                    /* anchor_age: how many ms ago was the last map update anchor.
                     * Small value (< 200 ms) means we are near the anchor, low extrapolation risk. */
                    int32_t anchor_age_cnt32 = (int32_t)(diag.now_mcu_cnt32 - diag.anchor_mcu_cnt32);
                    int32_t anchor_age_ms = anchor_age_cnt32 / 48000;

                    FS_LOGI("MCU time sync quality: map_locked=%u holdover=%u now_cnt32=0x%08x soc_est_ns=%llu soc_realtime_now_ns=%llu offset_ms=%lld anchor_age_ms=%ld residual_us=%u drift_ppb=%d rx_age_us(last/max)=%u/%u",
                            diag.map_locked,
                            diag.holdover_active,
                            diag.now_mcu_cnt32,
                            (unsigned long long)now_soc_est,
                            (unsigned long long)t_soc_now_ns,
                            (long long)offset_ms,
                            (long)anchor_age_ms,
                            diag.holdover_last_residual_us,
                            diag.map_drift_ppb,
                            diag.pair_rx_age_last_us,
                            diag.pair_rx_age_max_us);
                } else {
                    FS_LOGI("MCU time sync quality: map_locked=%u holdover=%u (no estimate available, now_cnt32=0x%08x anchor_soc_ns=%llu residual_us=%u drift_ppb=%d rx_age_us(last/max)=%u/%u)",
                            diag.map_locked,
                            diag.holdover_active,
                            diag.now_mcu_cnt32,
                            (unsigned long long)anchor_soc,
                            diag.holdover_last_residual_us,
                            diag.map_drift_ppb,
                            diag.pair_rx_age_last_us,
                            diag.pair_rx_age_max_us);
                }

                FS_LOGI("MCU time sync diag[poll]: trigger_irq=%u fsync_pulse=%u trigger_ring_ovf=%u fsync_overlap=%u soc_rx=%u soc_drop=%u(invalid=%u full=%u) pair=%u pair_drop=%u(trigger_old=%u soc_old=%u rx_early=%u rx_late=%u invalid=%u align=%u) pair_gap=%u cap_gap=%u resync=%u map_lock=%u map_lost=%u drift_ppb=%d rx_age(sample/sum/last/max/gt50/gt80)=%u/%llu/%u/%u/%u/%u hold=%u hold_stale=%u hold_good=%u hold_bad=%u hold_rec=%u hold_to=%u residual_hold=%u residual_reject=%u hold_active=%u hold_res_us=%u hold_res_signed_us=%d imu_wrap=%u imu_anchor=%u imu_soc_ok=%u imu_hold=%u",
                        diag.trigger_irq_count,
                        diag.fsync_pulse_count,
                        diag.trigger_ring_overflow_count,
                        diag.fsync_overlap_count,
                        diag.soc_sync_rx_count,
                        soc_sync_drop_total,
                        diag.soc_sync_drop_invalid_count,
                        diag.soc_sync_drop_queue_full_count,
                        diag.sync_pair_count,
                        sync_pair_drop_total,
                        diag.sync_pair_drop_trigger_old_count,
                        diag.sync_pair_drop_soc_old_count,
                        diag.sync_pair_drop_rx_early_count,
                        diag.sync_pair_drop_rx_late_count,
                        diag.sync_pair_drop_invalid_count,
                        diag.sync_pair_drop_align_mismatch_count,
                        diag.sync_pair_seq_gap,
                        diag.sync_pair_capture_gap,
                        diag.resync_count,
                        diag.time_map_locked_count,
                        diag.time_map_lost_count,
                        diag.map_drift_ppb,
                        diag.pair_rx_age_sample_count,
                        (unsigned long long)(((uint64_t)diag.pair_rx_age_sum_us_hi << 32)
                                             | (uint64_t)diag.pair_rx_age_sum_us_lo),
                        diag.pair_rx_age_last_us,
                        diag.pair_rx_age_max_us,
                        diag.pair_rx_age_gt50ms_count,
                        diag.pair_rx_age_gt80ms_count,
                        diag.holdover_count,
                        diag.holdover_stale_count,
                        diag.holdover_good_count,
                        diag.holdover_bad_count,
                        diag.holdover_recover_count,
                        diag.holdover_timeout_count,
                        diag.residual_hold_count,
                        diag.residual_reject_count,
                        diag.holdover_active,
                        diag.holdover_last_residual_us,
                        diag.holdover_last_residual_signed_us,
                        diag.imu_timestamp_epoch_count,
                        diag.imu_ts_anchor_update_count,
                        diag.imu_to_soc_ok_count,
                        diag.imu_to_soc_fallback_count);
            } else if (ret == -2) {
                FS_LOGE("MCU time sync diag poll failed: %d (MCU diag response buffer too small, check PROT_TX_BUF_LEN_NORMAL)", ret);
            } else {
                FS_LOGE("MCU time sync diag poll failed: %d", ret);
            }

            mcu_uart_write_int_file(MCU_UART_TIME_SYNC_DIAG_POLL_FILE, 0);
        }

        if (imu_push_diag_en) {
            mcu_imu_push_diag_t diag;
            int ret;

            last_imu_push_diag_en = 1;
            FS_LOGI("MCU imu push diag one-shot trigger (file=%s)", MCU_UART_IMU_PUSH_DIAG_POLL_FILE);
            ret = mcu_prot_get_imu_push_diag(&diag);
            if (ret == FS_SUCCESS) {
                FS_LOGI("MCU imu push diag[poll]: enabled=%u mcu_log=%u mcu_log_time_sync=%u soc_log_tap=%u inited=%u raw_q=%u/%u push_seq=%u push_ok=%u push_err=%u poll_err=%u sample_enq=%u sample_sent=%u last_read_bytes=%u q_ovw_drop=%u map_drop=%u ts_drop=%u map_locked=%u imu_soc_ok=%u imu_fb=%u uart_rx_push=%u uart_push_drop=%u uart_rx_frame_err=%u",
                        diag.enabled,
                        diag.mcu_log_enabled,
                        diag.mcu_log_time_sync_enabled,
                        1u,
                        diag.inited,
                        diag.raw_q_count,
                        diag.raw_q_capacity,
                        diag.push_seq,
                        diag.push_ok,
                        diag.push_err,
                        diag.poll_err,
                        diag.sample_enqueued,
                        diag.sample_sent,
                        diag.last_read_bytes,
                        diag.queue_overwrite_drop,
                        diag.map_drop,
                        diag.ts_drop,
                        diag.map_locked,
                        diag.imu_to_soc_ok_count,
                        diag.imu_to_soc_fallback_count,
                        g_mcu_uart_diag.rx_push,
                        g_mcu_uart_diag.push_drop,
                        g_mcu_uart_diag.rx_frame_err);
                FS_LOGI("MCU imu fifo diag[poll]: poll=%u read=%u bytes=%u overflow=%u ts_count=%u ts_decode_err=%u level=%u peak_level=%u max_level=%u last_error=%d last_ts_valid=%u last_ts_raw24=0x%06x last_ts_us=%u",
                        diag.fifo_poll_count,
                        diag.fifo_read_count,
                        diag.fifo_bytes_read_total,
                        diag.fifo_overflow_count,
                        diag.fifo_ts_count,
                        diag.fifo_ts_decode_err,
                        diag.fifo_last_level_word,
                        diag.fifo_last_peak_level_word,
                        diag.fifo_max_level_word,
                        diag.fifo_last_error,
                        diag.last_ts_valid,
                        diag.last_ts_raw_24,
                        diag.last_ts_us);
                FS_LOGI("MCU imu ts diag[poll]: drop_reason(no_anchor/no_candidate/map_anchor/past/future/nonmono/large_step/dup/fifo_gap)=%u/%u/%u/%u/%u/%u/%u/%u/%u tracker(reseed/wrap/gap_us/fifo_reseed/fifo_gap_us/output_step_us)=%u/%u/%u/%u/%u/%u anchor(reject/raw_delta_us/elapsed_us/valid_wrap)=%u/%u/%u/%u",
                        diag.imu_ts_drop_extend_no_anchor,
                        diag.imu_ts_drop_extend_no_candidate,
                        diag.imu_ts_drop_map_anchor,
                        diag.imu_ts_drop_plausible_past,
                        diag.imu_ts_drop_plausible_future,
                        diag.imu_ts_drop_plausible_nonmono,
                        diag.imu_ts_drop_plausible_large_step,
                        diag.imu_ts_drop_fifo_duplicate,
                        diag.imu_ts_drop_fifo_large_gap,
                        diag.imu_ts_tracker_reseed_count,
                        diag.imu_ts_tracker_wrap_count,
                        diag.imu_ts_tracker_last_gap_us,
                        diag.imu_ts_fifo_stream_reseed_count,
                        diag.imu_ts_fifo_forward_gap_us,
                        diag.imu_ts_output_step_us,
                        diag.imu_ts_anchor_reject_count,
                        diag.imu_ts_anchor_raw_delta_us,
                        diag.imu_ts_anchor_elapsed_us,
                        diag.imu_ts_anchor_valid_wrap_count);
            } else if (ret == -2) {
                FS_LOGE("MCU imu push diag poll failed: %d (MCU diag response buffer too small, check PROT_TX_BUF_LEN_NORMAL)", ret);
            } else {
                FS_LOGE("MCU imu push diag poll failed: %d", ret);
            }

            mcu_uart_write_int_file(MCU_UART_IMU_PUSH_DIAG_POLL_FILE, 0);
        }

        fs_hw_msleep(interval_ms);
    }

    return NULL;
}

static uint32_t uart_load_cfg_u32(const char *path, uint32_t def_value, uint32_t min_value, uint32_t max_value)
{
    int value = 0;

    if (mcu_uart_read_int_file(path, &value) != FS_SUCCESS) {
        return def_value;
    }
    if (value < (int)min_value) {
        return min_value;
    }
    if ((uint32_t)value > max_value) {
        return max_value;
    }
    return (uint32_t)value;
}

static int uart_load_eval_enabled(void)
{
    int value = 0;

    if (mcu_uart_read_int_file(UART_LOAD_DIAG_EN_FILE, &value) != FS_SUCCESS) {
        return 0;
    }
    return (value > 0) ? 1 : 0;
}

static const char *uart_load_tag_name(uint16_t tag)
{
    static const char *names[MCU_UART_LOAD_TAG_NUM] = {
        "adc",
        "reg",
        "imu_push",
        "time_sync",
        "diag",
        "flash",
        "load_echo",
        "update",
        "other",
    };

    if (tag >= MCU_UART_LOAD_TAG_NUM) {
        return "unknown";
    }
    return names[tag];
}

static void uart_load_csv_rotate_if_needed(void)
{
    struct stat st;

    if (stat(UART_LOAD_DIAG_CSV_PATH, &st) != 0) {
        return;
    }
    if ((uint64_t)st.st_size <= UART_LOAD_DIAG_CSV_MAX_BYTES) {
        return;
    }

    (void)unlink(UART_LOAD_DIAG_CSV_PATH_OLD);
    if (rename(UART_LOAD_DIAG_CSV_PATH, UART_LOAD_DIAG_CSV_PATH_OLD) != 0) {
        FS_LOGW("rotate %s failed: %s", UART_LOAD_DIAG_CSV_PATH, strerror(errno));
    }
}

static void uart_load_csv_write_header(FILE *fp)
{
    uint16_t i;

    fprintf(fp,
            "schema_version,soc_realtime_us,soc_monotonic_ms,period_ms,echo_rate_hz,echo_len,adc_rate_hz,mcu_diag_ret,"
            "soc_cmdq_depth,soc_cmdq_peak,soc_cmdq_push_drop,soc_req_wait_count,soc_req_wait_last_us,soc_req_wait_max_us,"
            "soc_tx_frame,soc_tx_bytes,soc_tx_write_err,soc_rx_frame_ok,soc_rx_frame_err,soc_rx_bytes,soc_rx_timeout,soc_resp_slot_hit,soc_resp_slot_store,soc_push_drop,"
            "soc_tx_block_last_us,soc_tx_block_max_us,soc_tcdrain_last_us,soc_tcdrain_max_us,soc_rtt_last_us,soc_rtt_max_us,"
            "mcu_rx_frame_ok,mcu_rx_frame_err,mcu_rx_frame_drop,mcu_rx_bytes,mcu_tx_frame,mcu_tx_bytes,mcu_tx_write_err,"
            "mcu_tx_ring_depth,mcu_tx_ring_peak,mcu_tx_ring_push_reject,mcu_tx_ring_frame_reject,"
            "mcu_ready_wait_last_us,mcu_ready_wait_max_us,mcu_process_last_us,mcu_process_max_us,mcu_tx_block_last_us,mcu_tx_block_max_us");

    for (i = 0u; i < MCU_UART_LOAD_TAG_NUM; i++) {
        const char *name = uart_load_tag_name(i);
        fprintf(fp, ",soc_%s_frames,soc_%s_bytes,soc_%s_drops,soc_%s_errors,soc_%s_lat_last_us,soc_%s_lat_max_us",
                name, name, name, name, name, name);
    }
    for (i = 0u; i < MCU_UART_LOAD_TAG_NUM; i++) {
        const char *name = uart_load_tag_name(i);
        fprintf(fp, ",mcu_%s_frames,mcu_%s_bytes,mcu_%s_drops,mcu_%s_errors,mcu_%s_lat_last_us,mcu_%s_lat_max_us",
                name, name, name, name, name, name);
    }
    fprintf(fp, "\n");
}

static void uart_load_csv_write_buckets(FILE *fp, const mcu_uart_load_bucket_t *buckets)
{
    uint16_t i;

    for (i = 0u; i < MCU_UART_LOAD_TAG_NUM; i++) {
        const mcu_uart_load_bucket_t *b = &buckets[i];
        fprintf(fp, ",%u,%u,%u,%u,%u,%u",
                b->frames,
                b->bytes,
                b->drops,
                b->errors,
                b->latency_last_us,
                b->latency_max_us);
    }
}

static void uart_load_write_csv_row(uint32_t period_ms,
                                    uint32_t echo_rate_hz,
                                    uint32_t echo_len,
                                    uint32_t adc_rate_hz)
{
    mcu_uart_load_diag_summary_t soc_summary;
    mcu_uart_load_diag_summary_t mcu_summary;
    mcu_uart_load_bucket_t soc_buckets[MCU_UART_LOAD_TAG_NUM];
    mcu_uart_load_bucket_t mcu_buckets[MCU_UART_LOAD_TAG_NUM];
    struct stat st;
    int need_header = 0;
    FILE *fp;
    int mcu_ret;

    memset(&soc_summary, 0, sizeof(soc_summary));
    memset(&mcu_summary, 0, sizeof(mcu_summary));
    memset(soc_buckets, 0, sizeof(soc_buckets));
    memset(mcu_buckets, 0, sizeof(mcu_buckets));

    (void)mcu_prot_uart_load_get_soc_snapshot(&soc_summary,
                                              soc_buckets,
                                              (uint16_t)(sizeof(soc_buckets) / sizeof(soc_buckets[0])));
    mcu_ret = mcu_prot_get_uart_load_diag(&mcu_summary,
                                          mcu_buckets,
                                          (uint16_t)(sizeof(mcu_buckets) / sizeof(mcu_buckets[0])));

    uart_load_csv_rotate_if_needed();
    if (stat(UART_LOAD_DIAG_CSV_PATH, &st) != 0 || st.st_size == 0) {
        need_header = 1;
    }

    fp = fopen(UART_LOAD_DIAG_CSV_PATH, "a");
    if (fp == NULL) {
        FS_LOGW("open %s failed: %s", UART_LOAD_DIAG_CSV_PATH, strerror(errno));
        return;
    }
    if (need_header) {
        uart_load_csv_write_header(fp);
    }

    fprintf(fp,
            "%u,%llu,%llu,%u,%u,%u,%u,%d,"
            "%u,%u,%u,%u,%u,%u,"
            "%u,%u,%u,%u,%u,%u,%u,%u,%u,%u,"
            "%u,%u,%u,%u,%u,%u,"
            "%u,%u,%u,%u,%u,%u,%u,"
            "%u,%u,%u,%u,"
            "%u,%u,%u,%u,%u,%u",
            MCU_UART_LOAD_DIAG_SCHEMA_VERSION,
            (unsigned long long)uart_load_realtime_us(),
            (unsigned long long)(uart_load_mono_us() / 1000ull),
            period_ms,
            echo_rate_hz,
            echo_len,
            adc_rate_hz,
            mcu_ret,
            soc_summary.cmdq_depth,
            soc_summary.cmdq_peak,
            soc_summary.cmdq_push_drop,
            soc_summary.cmdq_req_wait_count,
            soc_summary.cmdq_req_wait_last_us,
            soc_summary.cmdq_req_wait_max_us,
            soc_summary.tx_frame,
            soc_summary.tx_bytes,
            soc_summary.tx_write_err,
            soc_summary.rx_frame_ok,
            soc_summary.rx_frame_err,
            soc_summary.rx_bytes,
            soc_summary.rx_timeout,
            soc_summary.resp_slot_hit,
            soc_summary.resp_slot_store,
            soc_summary.push_drop,
            soc_summary.tx_block_last_us,
            soc_summary.tx_block_max_us,
            soc_summary.tcdrain_last_us,
            soc_summary.tcdrain_max_us,
            soc_summary.rtt_last_us,
            soc_summary.rtt_max_us,
            mcu_summary.rx_frame_ok,
            mcu_summary.rx_frame_err,
            mcu_summary.rx_frame_drop,
            mcu_summary.rx_bytes,
            mcu_summary.tx_frame,
            mcu_summary.tx_bytes,
            mcu_summary.tx_write_err,
            mcu_summary.tx_ring_depth,
            mcu_summary.tx_ring_peak,
            mcu_summary.tx_ring_push_reject,
            mcu_summary.tx_ring_frame_reject,
            mcu_summary.ready_wait_last_us,
            mcu_summary.ready_wait_max_us,
            mcu_summary.process_last_us,
            mcu_summary.process_max_us,
            mcu_summary.tx_block_last_us,
            mcu_summary.tx_block_max_us);

    uart_load_csv_write_buckets(fp, soc_buckets);
    uart_load_csv_write_buckets(fp, mcu_buckets);
    fprintf(fp, "\n");
    fclose(fp);
}

static uint8_t uart_load_echo_byte(const mcu_uart_load_echo_req_t *req, uint16_t idx)
{
    uint16_t seed = (uint16_t)(req->pattern_seed ^ req->seq);
    return (uint8_t)((seed + idx * 31u + (idx >> 1)) & 0xffu);
}

static void uart_load_run_echo_once(uint16_t resp_len, uint16_t *seq)
{
    mcu_uart_load_echo_req_t req;
    uint8_t resp[UART_LOAD_ECHO_LEN_MAX];
    uint16_t out_len = 0u;
    uint16_t i;
    int ret;

    memset(&req, 0, sizeof(req));
    req.resp_len = resp_len;
    req.pattern_seed = 0x4c44u;
    req.seq = *seq;
    (*seq)++;

    memset(resp, 0, sizeof(resp));
    ret = mcu_prot_uart_load_echo(&req, resp, sizeof(resp), &out_len);
    if (ret != FS_SUCCESS || out_len != req.resp_len) {
        uart_load_record(MCU_UART_LOAD_TAG_LOAD_ECHO, 0u, 0u, 0u, 1u, 0u, 0);
        return;
    }

    for (i = 0u; i < out_len; i++) {
        if (resp[i] != uart_load_echo_byte(&req, i)) {
            uart_load_record(MCU_UART_LOAD_TAG_LOAD_ECHO, 0u, 0u, 0u, 1u, 0u, 0);
            return;
        }
    }
}

static void uart_load_run_adc_once(void)
{
    int32_t value = 0;
    uint8_t cnt = 1u;
    uint32_t raw_mv = 0u;
    int ret;

    ret = mcu_prot_get_adc(ADC_TYPE_VBD, ADC_DATA_CALIB_PARA, &value, &cnt, &raw_mv);
    if (ret != FS_SUCCESS) {
        uart_load_record(MCU_UART_LOAD_TAG_ADC, 0u, 0u, 0u, 1u, 0u, 0);
    }
}

static void *uart_load_eval_thread(void *arg)
{
    uint64_t next_csv_ms = 0u;
    uint64_t next_echo_ms = 0u;
    uint64_t next_adc_ms = 0u;
    uint16_t echo_seq = 1u;

    (void)arg;

    while (g_uart_load_eval_run) {
        uint64_t now_ms = fs_util_timestamp_ms();
        uint32_t period_ms;
        uint32_t echo_rate_hz;
        uint32_t echo_len;
        uint32_t adc_rate_hz;

        if (!uart_load_eval_enabled()) {
            next_csv_ms = 0u;
            next_echo_ms = 0u;
            next_adc_ms = 0u;
            fs_hw_msleep(200);
            continue;
        }

        period_ms = uart_load_cfg_u32(UART_LOAD_DIAG_PERIOD_MS_FILE,
                                      UART_LOAD_DIAG_PERIOD_DEFAULT_MS,
                                      UART_LOAD_DIAG_PERIOD_MIN_MS,
                                      UART_LOAD_DIAG_PERIOD_MAX_MS);
        echo_rate_hz = uart_load_cfg_u32(UART_LOAD_ECHO_RATE_HZ_FILE, 0u, 0u, 200u);
        echo_len = uart_load_cfg_u32(UART_LOAD_ECHO_LEN_FILE,
                                     UART_LOAD_ECHO_LEN_DEFAULT,
                                     0u,
                                     UART_LOAD_ECHO_LEN_MAX);
        adc_rate_hz = uart_load_cfg_u32(UART_LOAD_ADC_RATE_HZ_FILE, 0u, 0u, 50u);

        if (echo_rate_hz > 0u) {
            uint32_t interval_ms = 1000u / echo_rate_hz;
            if (interval_ms == 0u) {
                interval_ms = 1u;
            }
            if (next_echo_ms == 0u || now_ms >= next_echo_ms) {
                uart_load_run_echo_once((uint16_t)echo_len, &echo_seq);
                next_echo_ms = now_ms + interval_ms;
            }
        }

        if (adc_rate_hz > 0u) {
            uint32_t interval_ms = 1000u / adc_rate_hz;
            if (interval_ms == 0u) {
                interval_ms = 1u;
            }
            if (next_adc_ms == 0u || now_ms >= next_adc_ms) {
                uart_load_run_adc_once();
                next_adc_ms = now_ms + interval_ms;
            }
        }

        if (next_csv_ms == 0u || now_ms >= next_csv_ms) {
            uart_load_write_csv_row(period_ms, echo_rate_hz, echo_len, adc_rate_hz);
            next_csv_ms = now_ms + period_ms;
        }

        fs_hw_msleep(10);
    }

    return NULL;
}

static void uart_load_eval_start(void)
{
    if (g_uart_load_eval_tid != 0) {
        return;
    }
    g_uart_load_eval_run = 1;
    g_uart_load_soc_start_ms = fs_util_timestamp_ms();
    if (pthread_create(&g_uart_load_eval_tid, NULL, uart_load_eval_thread, NULL) != 0) {
        FS_LOGW("failed to create uart load eval thread");
        g_uart_load_eval_tid = 0;
        g_uart_load_eval_run = 0;
    }
}

static void uart_load_eval_stop(void)
{
    g_uart_load_eval_run = 0;
    if (g_uart_load_eval_tid != 0) {
        pthread_join(g_uart_load_eval_tid, NULL);
        g_uart_load_eval_tid = 0;
    }
}

static void mcu_prot_deinit_internal(void)
{
    uart_load_eval_stop();

    /* 先停止 TX Thread，防止 deinit 期间仍有 PUSH 入队写 UART */
    g_mcu_uart_tx_run = 0;
    mcu_uart_cmdq_stop(&g_uart_cmdq);
    if (g_mcu_uart_tx_tid != 0) {
        pthread_join(g_mcu_uart_tx_tid, NULL);
        g_mcu_uart_tx_tid = 0;
    }
    mcu_uart_cmdq_destroy(&g_uart_cmdq);

    g_mcu_uart_periodic_req_run = 0;
    if (g_mcu_uart_periodic_req_tid != 0) {
        pthread_join(g_mcu_uart_periodic_req_tid, NULL);
        g_mcu_uart_periodic_req_tid = 0;
    }

    mcu_push_msg_run = 0;
    if (mcu_push_msg_tid != 0) {
        pthread_join(mcu_push_msg_tid, NULL);
        mcu_push_msg_tid = 0;
    }
    mcu_uart_close_if_opened();
}

/**
 * @brief 启动阶段查询 MCU 固件版本并写入 g_version.app_mcu
 * @param version_buf 版本字符串缓冲区
 * @param buf_len 缓冲区长度
 * @param allow_fail 为真时读版本失败不阻断 init（被动监听等场景）
 */
static int mcu_prot_read_mcu_version_at_boot(char *version_buf, size_t buf_len, int allow_fail)
{
    int err = FS_ERR_MCU_PROT_READ;
    int retry = 5;

    if (version_buf == NULL || buf_len == 0u) {
        return FS_ERR_BAD_PARAMS;
    }

    if (mcu_uart_passive_listen_enabled() && mcu_uart_passive_listener_running()) {
        fs_hw_msleep(50);
    }

    do {
        memset(version_buf, 0, buf_len);
        err = mcu_prot_get_mcu_version((uint8_t *)version_buf, (uint16_t)buf_len);
        if (err == FS_SUCCESS && version_buf[0] != '\0') {
            snprintf(g_version.app_mcu, sizeof(g_version.app_mcu), "%s", version_buf);
            FS_LOGI("MCU firmware version: %s", version_buf);
            return FS_SUCCESS;
        }

        if (err != FS_SUCCESS) {
            FS_LOGW("MCU firmware version read failed: %d, retrying(%d)...", err, retry);
        } else {
            FS_LOGW("MCU firmware version empty, retrying(%d)...", retry);
            err = FS_ERR_MCU_PROT_READ;
        }
        fs_hw_msleep(100);
    }
    while (--retry > 0);

    snprintf(g_version.app_mcu, sizeof(g_version.app_mcu), "%s", "unknown");
    FS_LOGE("MCU firmware version unavailable after retries (last err=%d)", err);
    return allow_fail ? FS_SUCCESS : err;
}

int mcu_prot_init(void)
{
    int32_t err = FS_SUCCESS;
    char version_data[FS_VERSION_MAX_LEN * 2] = {0};
    int passive_listen;

    pthread_mutex_lock(&g_mcu_prot_lifecycle_mutex);
    if (g_mcu_prot_inited) {
        pthread_mutex_unlock(&g_mcu_prot_lifecycle_mutex);
        FS_LOGI("mcu_prot_init skipped: already initialized");
        return FS_SUCCESS;
    }

    FS_LOGI("MCU IMU push gate: always enabled (system state gated)");

    // 固定使用 UART 传输，打开串口设备并完成 termios 配置。
    err = mcu_uart_open_if_needed();
    if (err != FS_SUCCESS) {
        FS_LOGE("open uart failed: %d", err);
        pthread_mutex_unlock(&g_mcu_prot_lifecycle_mutex);
        return err;
    }

    passive_listen = mcu_uart_passive_listen_enabled();

    /* 先启动 TX Thread，所有后续发送操作均经过队列 */
    mcu_uart_cmdq_init(&g_uart_cmdq);
    g_mcu_uart_tx_run = 1;
    pthread_create(&g_mcu_uart_tx_tid, NULL, mcu_uart_tx_thread, NULL);

    mcu_push_msg_run = 1;
    pthread_create(&mcu_push_msg_tid, NULL, mcu_prot_push_msg_handler, NULL);

    g_mcu_uart_periodic_req_run = 1;
    pthread_create(&g_mcu_uart_periodic_req_tid, NULL, mcu_prot_periodic_req_test_handler, NULL);
    uart_load_eval_start();

    if (passive_listen) {
        mcu_fsync_diag_t fsync_diag;

        FS_LOGI("MCU UART passive listen mode enabled");
        err = mcu_prot_get_fsync_diag(&fsync_diag);
        if (err == FS_SUCCESS) {
            FS_LOGI("MCU fsync diag: trigger_irq=%u trigger_ig=%u busy_irq=%u busy_ig=%u fsync_hi=%u fsync_lo=%u flags=0x%08x",
                    fsync_diag.trigger_irq_total,
                    fsync_diag.trigger_ignored_total,
                    fsync_diag.busy_irq_total,
                    fsync_diag.busy_ignored_total,
                    fsync_diag.fsync_high_total,
                    fsync_diag.fsync_low_total,
                    fsync_diag.flags);
        } else {
            FS_LOGE("MCU fsync diag query failed: %d", err);
        }
        err = mcu_prot_read_mcu_version_at_boot(version_data, sizeof(version_data), 1);
        (void)mcu_prot_register_imu_forward_cb(mcu_imu_forward_default_log_cb, NULL);
        g_mcu_prot_inited = 1;
        pthread_mutex_unlock(&g_mcu_prot_lifecycle_mutex);
        return err;
    }

    err = mcu_uart_send_boot_test_frames();
    if (err != FS_SUCCESS) {
        FS_LOGE("send uart boot test frames failed: %d", err);
        mcu_prot_deinit_internal();
        pthread_mutex_unlock(&g_mcu_prot_lifecycle_mutex);
        return err;
    }

    err = mcu_prot_read_mcu_version_at_boot(version_data, sizeof(version_data), 0);
    if (err != FS_SUCCESS) {
        mcu_prot_deinit_internal();
        pthread_mutex_unlock(&g_mcu_prot_lifecycle_mutex);
        return err;
    }

    (void)mcu_prot_register_imu_forward_cb(mcu_imu_forward_default_log_cb, NULL);

    g_mcu_prot_inited = 1;
    pthread_mutex_unlock(&g_mcu_prot_lifecycle_mutex);
    return FS_SUCCESS;
}

void mcu_prot_deinit(void)
{
    pthread_mutex_lock(&g_mcu_prot_lifecycle_mutex);
    if (!g_mcu_prot_inited) {
        pthread_mutex_unlock(&g_mcu_prot_lifecycle_mutex);
        return;
    }

    mcu_prot_deinit_internal();
    g_mcu_prot_inited = 0;
    pthread_mutex_unlock(&g_mcu_prot_lifecycle_mutex);
}

void mcu_uart_release(void)
{
    pthread_mutex_lock(&g_mcu_prot_lifecycle_mutex);
    if (g_mcu_prot_inited) {
        mcu_prot_deinit_internal();
        g_mcu_prot_inited = 0;
    } else {
        mcu_uart_close_if_opened();
    }
    pthread_mutex_unlock(&g_mcu_prot_lifecycle_mutex);
}
