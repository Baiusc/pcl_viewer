
#ifndef __MCU_PROTOCOL_H__
#define __MCU_PROTOCOL_H__

#include <stdint.h>
#include <stdbool.h>

#include "fw_if.h"

#define MCU_PROT_BUFFER_SIZE                2070

#define MCU_PROT_READ_COMMON_LEN            (9)

#define MCU_PROT_READ_LIDARCHIP_DATA_LEN       (4)
#define MCU_PROT_READ_LIDARCHIP_LEN            (MCU_PROT_READ_COMMON_LEN + MCU_PROT_READ_LIDARCHIP_DATA_LEN)

#define MCU_PROT_READ_FL9003_DATA_LEN       (4)
#define MCU_PROT_READ_FL9003_LEN            (MCU_PROT_READ_COMMON_LEN + MCU_PROT_READ_FL9003_DATA_LEN)

#define MCU_PROT_READ_FL9103_DATA_LEN       (4)
#define MCU_PROT_READ_FL9103_LEN            (MCU_PROT_READ_COMMON_LEN + MCU_PROT_READ_FL9103_DATA_LEN)

#define MCU_PROT_READ_MCU_VER_DATA_LEN      (64)
#define MCU_PROT_READ_MCU_VER_LEN           (MCU_PROT_READ_COMMON_LEN + MCU_PROT_READ_MCU_VER_DATA_LEN)
#define MCU_PROT_READ_SPAD_SN_DATA_LEN      (64)
#define MCU_PROT_READ_SPAD_SN_LEN           (MCU_PROT_READ_COMMON_LEN + MCU_PROT_READ_SPAD_SN_DATA_LEN)

// 单次批量操作最大寄存器数量(N): cmd_dev (1B) + cmd_op (1B) + reg_count (2B) + reserved (2B) + [(addr(4B) + data(4B)) * N] + crc8 (1B)
#define MCU_PROT_BATCH_MAX_REGS   ((uint32_t)((MCU_PROT_BUFFER_SIZE - sizeof(mcu_prot_batch_cmd_head_t) - 1) / sizeof(mcu_prot_batch_reg_t)))

#define MCU_PROT_DEV_FL6X32_1     ((uint8_t)FW_IF_MCU_DEV_FL6X32_1)
#define MCU_PROT_DEV_FL6X32_ALL   ((uint8_t)FW_IF_MCU_DEV_FL6X32_ALL)
#define MCU_PROT_DEV_FL9003_1     ((uint8_t)FW_IF_MCU_DEV_FL9003_1)
#define MCU_PROT_DEV_FL9003_2     ((uint8_t)FW_IF_MCU_DEV_FL9003_2)
#define MCU_PROT_DEV_FL9003_3     ((uint8_t)FW_IF_MCU_DEV_FL9003_3)
#define MCU_PROT_DEV_FL9003_ALL   ((uint8_t)FW_IF_MCU_DEV_FL9003_ALL)
#define MCU_PROT_DEV_FL9103_1     ((uint8_t)FW_IF_MCU_DEV_FL9103_1)
#define MCU_PROT_DEV_FL9103_2     ((uint8_t)FW_IF_MCU_DEV_FL9103_2)
#define MCU_PROT_DEV_FL9103_ALL   ((uint8_t)FW_IF_MCU_DEV_FL9103_ALL)
#define MCU_PROT_DEV_MCU          ((uint8_t)FW_IF_MCU_DEV_MCU)
#define MCU_PROT_DEV_CALIDATA     ((uint8_t)FW_IF_MCU_DEV_CALIDATA)
#define MCU_PROT_DEV_CFGDATA      ((uint8_t)FW_IF_MCU_DEV_CFGDATA)
#define MCU_PROT_DEV_STATUS       ((uint8_t)FW_IF_MCU_DEV_STATUS)

#define MCU_PROT_OP_WRITE          ((uint8_t)FW_IF_MCU_OP_WRITE)
#define MCU_PROT_OP_READ           ((uint8_t)FW_IF_MCU_OP_READ)
#define MCU_PROT_OP_WRITE_CONTINUE ((uint8_t)FW_IF_MCU_OP_WRITE_CONTINUE)
#define MCU_PROT_OP_READ_CONTINUE  ((uint8_t)FW_IF_MCU_OP_READ_CONTINUE)
#define MCU_PROT_OP_WRITE_BATCH    ((uint8_t)FW_IF_MCU_OP_WRITE_BATCH)
#define MCU_PROT_OP_READ_BATCH     ((uint8_t)FW_IF_MCU_OP_READ_BATCH)
#define MCU_PROT_OP_RW_TRANSPARENT ((uint8_t)FW_IF_MCU_OP_RW_TRANSPARENT)
#define MCU_PROT_OP_HW_RESET       ((uint8_t)FW_IF_MCU_OP_HW_RESET)
#define MCU_PROT_OP_HW_POWEROFF    ((uint8_t)FW_IF_MCU_OP_HW_POWEROFF)
#define MCU_PROT_OP_HW_TRX_DAC_EN  ((uint8_t)FW_IF_MCU_OP_HW_TRX_DAC_EN)

#define MCU_PROT_MCU_MCU_VER              FW_IF_MCU_SUBCMD_MCU_VER
#define MCU_PROT_MCU_SPAD_SN              FW_IF_MCU_SUBCMD_SPAD_SN
#define MCU_PROT_MCU_BOARD_TEMP           FW_IF_MCU_SUBCMD_BOARD_TEMP
#define MCU_PROT_MCU_SPAD_TEMP            FW_IF_MCU_SUBCMD_SPAD_TEMP
#define MCU_PROT_MCU_VOLTAGE              FW_IF_MCU_SUBCMD_VOLTAGE
#define MCU_PROT_MCU_GET_ADC              FW_IF_MCU_SUBCMD_GET_ADC
#define MCU_PROT_MCU_INIT_STATE           FW_IF_MCU_SUBCMD_INIT_STATE
#define MCU_PROT_MCU_WORK_MODE            FW_IF_MCU_SUBCMD_WORK_MODE
#define MCU_PROT_MCU_TX_SEQMODE           FW_IF_MCU_SUBCMD_TX_SEQMODE
#define MCU_PROT_MCU_VBD                  FW_IF_MCU_SUBCMD_VBD
#define MCU_PROT_MCU_VPP                  FW_IF_MCU_SUBCMD_VPP
#define MCU_PROT_MCU_UPGRADE              FW_IF_MCU_SUBCMD_UPGRADE
#define MCU_PROT_MCU_ENABLE_VBD           FW_IF_MCU_SUBCMD_ENABLE_VBD
#define MCU_PROT_MCU_ENABLE_VPP           FW_IF_MCU_SUBCMD_ENABLE_VPP
#define MCU_PROT_MCU_ENABLE_TEMP_COMP     FW_IF_MCU_SUBCMD_ENABLE_TEMP_COMP
#define MCU_PROT_MCU_GET_TEMP_COMP_STATE  FW_IF_MCU_SUBCMD_GET_TEMP_COMP_STATE
#define MCU_PROT_MCU_ENABLE_VOLTAGE_CALIB FW_IF_MCU_SUBCMD_ENABLE_VOLTAGE_CALIB
#define MCU_PROT_MCU_ADC_VBD_CALIB        FW_IF_MCU_SUBCMD_ADC_VBD_CALIB
#define MCU_PROT_MCU_ADC_VPP_CALIB        FW_IF_MCU_SUBCMD_ADC_VPP_CALIB
#define MCU_PROT_MCU_DAC_VBD_CALIB        FW_IF_MCU_SUBCMD_DAC_VBD_CALIB
#define MCU_PROT_MCU_DAC_VPP_CALIB        FW_IF_MCU_SUBCMD_DAC_VPP_CALIB
#define MCU_PROT_MCU_REMOVE_ADC_VBD_CALIB FW_IF_MCU_SUBCMD_REMOVE_ADC_VBD_CALIB
#define MCU_PROT_MCU_REMOVE_ADC_VPP_CALIB FW_IF_MCU_SUBCMD_REMOVE_ADC_VPP_CALIB
#define MCU_PROT_MCU_REMOVE_DAC_VBD_CALIB FW_IF_MCU_SUBCMD_REMOVE_DAC_VBD_CALIB
#define MCU_PROT_MCU_REMOVE_DAC_VPP_CALIB FW_IF_MCU_SUBCMD_REMOVE_DAC_VPP_CALIB
#define MCU_PROT_MCU_FSYNC_DIAG           FW_IF_MCU_SUBCMD_FSYNC_DIAG
#define MCU_PROT_MCU_TIME_SYNC_DIAG       FW_IF_MCU_SUBCMD_TIME_SYNC_DIAG
#define MCU_PROT_MCU_IMU_PUSH_DIAG        FW_IF_MCU_SUBCMD_IMU_PUSH_DIAG
#define MCU_PROT_MCU_UART_LOAD_DIAG       FW_IF_MCU_SUBCMD_UART_LOAD_DIAG
#define MCU_PROT_MCU_UART_LOAD_ECHO       FW_IF_MCU_SUBCMD_UART_LOAD_ECHO
#define MCU_PROT_MCU_TIME_SYNC_HEALTH     FW_IF_MCU_SUBCMD_TIME_SYNC_HEALTH
#define MCU_PROT_MCU_TIME_SYNC_RESYNC     FW_IF_MCU_SUBCMD_TIME_SYNC_RESYNC
#define MCU_PROT_MCU_TIME_SYNC_CTRL       FW_IF_MCU_SUBCMD_TIME_SYNC_CTRL
#define MCU_PROT_MCU_PROGRAM_ENTER        FW_IF_MCU_SUBCMD_PROGRAM_ENTER
#define MCU_PROT_MCU_PROGRAM_EXIT         FW_IF_MCU_SUBCMD_PROGRAM_EXIT
#define MCU_PROT_MCU_PROGRAM_STATUS       FW_IF_MCU_SUBCMD_PROGRAM_STATUS

#define MCU_PROT_PROGRAM_STATUS_SCHEMA_VERSION 1u

typedef fw_if_mcu_program_status_t mcu_prot_program_status_t;

enum {
    MCU_TIME_SYNC_HEALTH_SCHEMA_VERSION = 1,
};

enum {
    MCU_TIME_SYNC_HEALTH_EVT_NONE = 0,
    MCU_TIME_SYNC_HEALTH_EVT_NOT_LOCKED = 1,
    MCU_TIME_SYNC_HEALTH_EVT_DEGRADED = 2,
    MCU_TIME_SYNC_HEALTH_EVT_LOST = 3,
    MCU_TIME_SYNC_HEALTH_EVT_LINK_UNSTABLE = 4,
    MCU_TIME_SYNC_HEALTH_EVT_DIAG_UNAVAILABLE = 5,
};

enum {
    MCU_TIME_SYNC_HEALTH_LEVEL_NONE = 0,
    MCU_TIME_SYNC_HEALTH_LEVEL_WARNING = 1,
    MCU_TIME_SYNC_HEALTH_LEVEL_ERROR = 2,
};

enum {
    MCU_TIME_SYNC_HEALTH_REASON_NONE = 0,
    MCU_TIME_SYNC_HEALTH_REASON_TRIGGER_TIMER_INIT_FAIL = 1,
    MCU_TIME_SYNC_HEALTH_REASON_TRIGGER_MISSING = 2,
    MCU_TIME_SYNC_HEALTH_REASON_SOC_PUSH_MISSING = 3,
    MCU_TIME_SYNC_HEALTH_REASON_PAIR_RX_LATE = 4,
    MCU_TIME_SYNC_HEALTH_REASON_PAIR_RX_EARLY = 5,
    MCU_TIME_SYNC_HEALTH_REASON_SEQ_GAP = 6,
    MCU_TIME_SYNC_HEALTH_REASON_CAPTURE_GAP = 7,
    MCU_TIME_SYNC_HEALTH_REASON_QUEUE_OVERFLOW = 8,
    MCU_TIME_SYNC_HEALTH_REASON_RESIDUAL_BAD = 9,
    MCU_TIME_SYNC_HEALTH_REASON_HOLDOVER_ENTER = 10,
    MCU_TIME_SYNC_HEALTH_REASON_HOLDOVER_TIMEOUT = 11,
    MCU_TIME_SYNC_HEALTH_REASON_BOOT_LOCK_TIMEOUT = 12,
    MCU_TIME_SYNC_HEALTH_REASON_SOC_SYNC_UNLOCKED = 13,
    MCU_TIME_SYNC_HEALTH_REASON_MAP_LOST = 14,
    MCU_TIME_SYNC_HEALTH_REASON_DIAG_READ_FAIL = 15,
};

enum {
    MCU_TIME_SYNC_HEALTH_ACTION_NONE = 0,
    MCU_TIME_SYNC_HEALTH_ACTION_USER_NOTICE = 1,
    MCU_TIME_SYNC_HEALTH_ACTION_TIME_SYNC_RESYNC = 2,
    MCU_TIME_SYNC_HEALTH_ACTION_RESTART_SOC_PUBLISHER = 3,
    MCU_TIME_SYNC_HEALTH_ACTION_RESET_CANDIDATE = 4,
    MCU_TIME_SYNC_HEALTH_ACTION_CHECK_LINK = 5,
};

enum {
    MCU_PROT_FSYNC_FLAG_ACTIVE                 = (1u << 0),
    MCU_PROT_FSYNC_FLAG_TRIGGER_LEVEL          = (1u << 1),
    MCU_PROT_FSYNC_FLAG_BUSY_LEVEL             = (1u << 2),
    MCU_PROT_FSYNC_FLAG_FSYNC_LEVEL            = (1u << 3),
    MCU_PROT_FSYNC_FLAG_TRIGGER_PENDING        = (1u << 4),
    MCU_PROT_FSYNC_FLAG_BUSY_PENDING           = (1u << 5),
    MCU_PROT_FSYNC_FLAG_BUSY_IRQ_SEEN          = (1u << 6),
    MCU_PROT_FSYNC_FLAG_BUSY_IRQ_LOW_SEEN      = (1u << 7),
    MCU_PROT_FSYNC_FLAG_BUSY_IRQ_HIGH_SEEN     = (1u << 8),
    MCU_PROT_FSYNC_FLAG_BUSY_IRQ_WHEN_ACTIVE  = (1u << 9),
    MCU_PROT_FSYNC_FLAG_BUSY_IRQ_WHEN_IDLE    = (1u << 10),
};

#define MCU_PROT_TYPE_NORMAL   ((uint8_t)FW_IF_MCU_PAYLOAD_NORMAL)
#define MCU_PROT_TYPE_STATUS   ((uint8_t)FW_IF_MCU_PAYLOAD_STATUS)
#define MCU_PROT_TYPE_PUSH_MSG ((uint8_t)FW_IF_MCU_PAYLOAD_PUSH_MSG)
#define MCU_PROT_TYPE_FLASH    ((uint8_t)FW_IF_MCU_PAYLOAD_FLASH)
#define MCU_PROT_TYPE_NUM      ((uint8_t)FW_IF_MCU_PAYLOAD_COUNT)

#define MCU_PROT_FLASH_WRITE_START FW_IF_MCU_FLASH_WRITE_START
#define MCU_PROT_FLASH_WRITE_DATA  FW_IF_MCU_FLASH_WRITE_DATA
#define MCU_PROT_FLASH_READ_START  FW_IF_MCU_FLASH_READ_START
#define MCU_PROT_FLASH_READ_DATA   FW_IF_MCU_FLASH_READ_DATA

#define MCU_PROT_FLASH_DT_CFG  ((uint8_t)FW_IF_MCU_FLASH_DT_CFG)
#define MCU_PROT_FLASH_DT_CALI ((uint8_t)FW_IF_MCU_FLASH_DT_CALI)

enum {
    MCU_PROT_STATUS_IDLE = 0,
    MCU_PROT_STATUS_BUSY,
    MCU_PROT_STATUS_ERROR,
};

#define MCU_INIT_STATE_BOOTING   FW_IF_MCU_INIT_STATE_BOOTING
#define MCU_INIT_STATE_HW_INIT   FW_IF_MCU_INIT_STATE_HW_INIT
#define MCU_INIT_STATE_CFG_LOAD  FW_IF_MCU_INIT_STATE_CFG_LOAD
#define MCU_INIT_STATE_HAL_INIT  FW_IF_MCU_INIT_STATE_HAL_INIT
#define MCU_INIT_STATE_CORE_INIT FW_IF_MCU_INIT_STATE_CORE_INIT
#define MCU_INIT_STATE_READY     FW_IF_MCU_INIT_STATE_READY
#define MCU_INIT_STATE_ERROR     FW_IF_MCU_INIT_STATE_ERROR

#pragma pack(push)
#pragma pack(1)  // one byte-aligned

typedef fw_if_mcu_cmd_t mcu_prot_cmd_t;

// 批量写入的单个寄存器配置
typedef struct {
    uint32_t addr;
    uint32_t data;
} mcu_prot_batch_reg_t;

// 批量写入命令头
typedef struct {
    union {
        uint16_t cmd;
        struct {
            uint8_t cmd_dev;
            uint8_t cmd_op;
        };
    };
    uint16_t reg_count;     // 寄存器数量
    uint16_t reserved;      // 保留
    // mcu_prot_batch_reg_t regs[]; // 变长数组，后跟 crc8
} mcu_prot_batch_cmd_head_t;

typedef struct {
    union {
        uint16_t cmd;
        struct {
            uint8_t cmd_dev;
            uint8_t cmd_op;
        };
    };
    uint16_t wlen;      // 写数据的长度
    uint16_t rlen;      // 读数据的长度
    uint8_t  wdata[];   // 写数据
} mcu_prot_transparent_cmd_head_t;

typedef struct {
    union {
        uint16_t cmd;
        struct {
            uint8_t cmd_dev;
            uint8_t cmd_op;
        };
    };

    union {
        uint32_t sub_cmd;
        struct {
            uint8_t sub_cmd0;
            uint8_t sub_cmd1;
            uint8_t sub_cmd2;
            uint8_t sub_cmd3;
        };
    };

    union {
        uint64_t data;
        struct {
            uint8_t d0;
            uint8_t d1;
            uint8_t d2;
            uint8_t d3;
            uint8_t d4;
            uint8_t d5;
            uint8_t d6;
            uint8_t d7;
        };
    };

    uint8_t crc8;
} mcu_prot_flash_cmd_t;

typedef mcu_prot_flash_cmd_t mcu_prot_txseq_cmd_t;
typedef struct {
    union {
        uint16_t cmd;
        struct {
            uint8_t cmd_dev;
            uint8_t cmd_op;
        };
    };

    union {
        uint32_t sub_cmd;
        struct {
            uint8_t sub_cmd0;
            uint8_t sub_cmd1;
            uint8_t sub_cmd2;
            uint8_t sub_cmd3;
        };
    };

    union {
        uint64_t data;
        struct {
            uint8_t d0;
            uint8_t d1;
            uint8_t d2;
            uint8_t d3;
            uint8_t d4;
            uint8_t d5;
            uint8_t d6;
            uint8_t d7;
        };
    };
} mcu_prot_flash_wcmd_head_t;

typedef struct {
    union {
        uint16_t cmd;
        struct {
            uint8_t cmd_dev;
            uint8_t cmd_op;
        };
    };

    union {
        uint32_t sub_cmd;
        struct {
            uint8_t sub_cmd0;
            uint8_t sub_cmd1;
            uint8_t sub_cmd2;
            uint8_t sub_cmd3;
        };
    };

    union {
        uint16_t len;
        struct {
            uint8_t len0;
            uint8_t len1;
        };
    };
} mcu_prot_read_head_t;

typedef struct {
    union {
        uint16_t addr;
        struct {
            uint8_t dummy;
            uint8_t type;
        };
    };
} mcu_prot_type_t;

typedef struct {
    float rx_temp_init;     //rx开机温度
    float rx_temp;	        //rx实时温度
    float tx1_temp_init;    //tx1开机温度
    float tx1_temp;	        //tx1实时温度
    float tx2_temp_init;    //tx2开机温度
    float tx2_temp;	        //tx2实时温度
    float pwr_temp_init;    //pwr板开机温度
    float pwr_temp;	        //pwr板实时温度
} fs_temp_info_t;

typedef struct {
    float vbd;
    float vpp;
    float vex;
} fs_voltage_info_t;

typedef struct {
    uint32_t trigger_irq_total;
    uint32_t trigger_ignored_total;
    uint32_t busy_irq_total;
    uint32_t busy_ignored_total;
    uint32_t fsync_high_total;
    uint32_t fsync_low_total;
    uint32_t scan_trigger_irq_total;
    uint32_t time_ref_irq_total;
    uint32_t flags;
} mcu_fsync_diag_t;

typedef struct {
    uint32_t trigger_irq_count;
    uint32_t time_ref_irq_count;
    uint32_t fsync_pulse_count;
    uint32_t trigger_ring_overflow_count;
    uint32_t fsync_overlap_count;
    uint32_t soc_sync_rx_count;
    uint32_t soc_sync_drop_invalid_count;
    uint32_t soc_sync_drop_queue_full_count;
    uint32_t sync_pair_count;
    uint32_t sync_pair_drop_trigger_old_count;
    uint32_t sync_pair_drop_soc_old_count;
    uint32_t sync_pair_drop_rx_early_count;
    uint32_t sync_pair_drop_rx_late_count;
    uint32_t sync_pair_drop_invalid_count;
    uint32_t sync_pair_drop_align_mismatch_count;
    uint32_t sync_pair_seq_gap;
    uint32_t sync_pair_capture_gap;
    uint32_t resync_count;
    uint32_t time_map_locked_count;
    uint32_t time_map_lost_count;
    uint32_t holdover_active;
    uint32_t holdover_count;
    uint32_t holdover_recover_count;
    uint32_t holdover_timeout_count;
    uint32_t holdover_last_residual_us;
    int32_t holdover_last_residual_signed_us;
    uint32_t residual_hold_count;
    uint32_t residual_reject_count;
    uint32_t imu_timestamp_epoch_count;
    uint32_t imu_ts_anchor_update_count;
    uint32_t imu_to_soc_ok_count;
    uint32_t imu_to_soc_fallback_count;
    uint32_t map_locked;
    uint32_t now_mcu_cnt32;
    uint32_t now_soc_est_ns_lo;
    uint32_t now_soc_est_ns_hi;
    uint32_t anchor_mcu_cnt32;
    uint32_t anchor_soc_ns_lo;
    uint32_t anchor_soc_ns_hi;
    uint32_t holdover_stale_count;
    uint32_t holdover_good_count;
    uint32_t holdover_bad_count;
    int32_t map_drift_ppb;
    uint32_t pair_rx_age_sample_count;
    uint32_t pair_rx_age_sum_us_lo;
    uint32_t pair_rx_age_sum_us_hi;
    uint32_t pair_rx_age_last_us;
    uint32_t pair_rx_age_max_us;
    uint32_t pair_rx_age_gt50ms_count;
    uint32_t pair_rx_age_gt80ms_count;
    uint32_t timesync_enabled;
    uint32_t current_map_valid;
    uint32_t last_map_locked_count;
    uint32_t last_sync_pair_count;
    uint32_t last_soc_sync_rx_count;
    uint32_t last_trigger_irq_count;
} mcu_time_sync_diag_t;

typedef struct {
    uint32_t schema_version;
    uint32_t event_id;
    uint32_t level;
    uint32_t detail_reason;
    uint32_t action_hint;
    uint32_t state;
    uint32_t map_locked;
    uint32_t holdover_active;
    uint32_t last_residual_us;
    int32_t last_residual_signed_us;
    uint32_t anchor_age_ms;
    uint32_t event_count;
    uint32_t last_event_mcu_cnt32;
} mcu_time_sync_health_t;

typedef struct {
    uint32_t enabled;
    uint32_t mcu_log_enabled;
    uint32_t mcu_log_time_sync_enabled;
    uint32_t inited;
    uint32_t raw_q_count;
    uint32_t raw_q_capacity;
    uint32_t push_seq;
    uint32_t push_ok;
    uint32_t push_err;
    uint32_t poll_err;
    uint32_t sample_sent;
    uint32_t last_read_bytes;
    uint32_t sample_enqueued;
    uint32_t queue_overwrite_drop;
    uint32_t map_drop;
    uint32_t ts_drop;
    uint32_t fifo_poll_count;
    uint32_t fifo_read_count;
    uint32_t fifo_bytes_read_total;
    uint32_t fifo_overflow_count;
    uint32_t fifo_ts_count;
    uint32_t fifo_ts_decode_err;
    uint32_t fifo_last_level_word;
    uint32_t fifo_last_peak_level_word;
    uint32_t fifo_max_level_word;
    int32_t fifo_last_error;
    uint32_t last_ts_raw_24;
    uint32_t last_ts_us;
    uint32_t last_ts_valid;
    uint32_t map_locked;
    uint32_t imu_to_soc_ok_count;
    uint32_t imu_to_soc_fallback_count;
    uint32_t imu_ts_drop_extend_no_anchor;
    uint32_t imu_ts_drop_extend_no_candidate;
    uint32_t imu_ts_drop_map_anchor;
    uint32_t imu_ts_drop_plausible_past;
    uint32_t imu_ts_drop_plausible_future;
    uint32_t imu_ts_drop_plausible_nonmono;
    uint32_t imu_ts_drop_plausible_large_step;
    uint32_t imu_ts_tracker_reseed_count;
    uint32_t imu_ts_tracker_wrap_count;
    uint32_t imu_ts_tracker_last_gap_us;
    uint32_t imu_ts_drop_fifo_duplicate;
    uint32_t imu_ts_drop_fifo_large_gap;
    uint32_t imu_ts_fifo_stream_reseed_count;
    uint32_t imu_ts_fifo_forward_gap_us;
    uint32_t imu_ts_output_step_us;
    uint32_t imu_ts_anchor_reject_count;
    uint32_t imu_ts_anchor_raw_delta_us;
    uint32_t imu_ts_anchor_elapsed_us;
    uint32_t imu_ts_anchor_valid_wrap_count;
} mcu_imu_push_diag_t;

enum {
    MCU_UART_LOAD_TAG_ADC = 0,
    MCU_UART_LOAD_TAG_REG,
    MCU_UART_LOAD_TAG_IMU_PUSH,
    MCU_UART_LOAD_TAG_TIME_SYNC,
    MCU_UART_LOAD_TAG_DIAG,
    MCU_UART_LOAD_TAG_FLASH,
    MCU_UART_LOAD_TAG_LOAD_ECHO,
    MCU_UART_LOAD_TAG_UPDATE,
    MCU_UART_LOAD_TAG_OTHER,
    MCU_UART_LOAD_TAG_NUM,
};

enum {
    MCU_UART_LOAD_DIAG_SCHEMA_VERSION = 2,
    MCU_UART_LOAD_DIAG_SOURCE_MCU = 1,
    MCU_UART_LOAD_DIAG_SOURCE_SOC = 2,
    MCU_UART_LOAD_DIAG_PAGE_SUMMARY = 0xff,
    MCU_UART_LOAD_DIAG_BUCKETS_PER_PAGE = 4,
};

typedef struct {
    uint32_t frames;
    uint32_t bytes;
    uint32_t drops;
    uint32_t errors;
    uint32_t latency_sample_count;
    uint32_t latency_sum_us_lo;
    uint32_t latency_sum_us_hi;
    uint32_t latency_last_us;
    uint32_t latency_max_us;
} mcu_uart_load_bucket_t;

typedef struct {
    uint16_t schema_version;
    uint8_t source;
    uint8_t tag_count;
    uint32_t uptime_ms;
    uint32_t baud;
    uint32_t rx_frame_ok;
    uint32_t rx_frame_err;
    uint32_t rx_frame_drop;
    uint32_t rx_bytes;
    uint32_t tx_frame;
    uint32_t tx_bytes;
    uint32_t tx_write_err;
    uint32_t tx_ring_depth;
    uint32_t tx_ring_peak;
    uint32_t tx_ring_push_reject;
    uint32_t tx_ring_frame_reject;
    uint32_t rx_timeout;
    uint32_t cmdq_depth;
    uint32_t cmdq_peak;
    uint32_t cmdq_req_wait_count;
    uint32_t cmdq_req_wait_last_us;
    uint32_t cmdq_req_wait_max_us;
    uint32_t cmdq_push_drop;
    uint32_t resp_slot_hit;
    uint32_t resp_slot_store;
    uint32_t push_drop;
    uint32_t ready_wait_last_us;
    uint32_t ready_wait_max_us;
    uint32_t process_last_us;
    uint32_t process_max_us;
    uint32_t tx_block_last_us;
    uint32_t tx_block_max_us;
    uint32_t tcdrain_last_us;
    uint32_t tcdrain_max_us;
    uint32_t rtt_last_us;
    uint32_t rtt_max_us;
    uint32_t reserved[8];
} mcu_uart_load_diag_summary_t;

typedef struct {
    uint16_t page_index;
    uint16_t page_count;
    uint16_t first_tag;
    uint16_t tag_count;
    mcu_uart_load_bucket_t buckets[MCU_UART_LOAD_DIAG_BUCKETS_PER_PAGE];
} mcu_uart_load_diag_page_t;

typedef struct {
    uint16_t resp_len;
    uint16_t pattern_seed;
    uint16_t seq;
    uint8_t flags;
    uint8_t reserved;
} mcu_uart_load_echo_req_t;

#pragma pack(pop)

typedef struct {
    uint32_t total_len; // 数据总长度
    uint16_t pkt_num;   // 数据总包数
    uint16_t pkt_len;   // 每包数据最大长度
} mcu_prot_flash_info_t;

typedef struct {
    uint16_t pkt_idx;
    uint16_t pkt_len;
    uint8_t *data;
} mcu_prot_flash_pkt_info_t;

typedef struct {
    uint32_t tx_frame;
    uint32_t tx_bytes;
    uint32_t rx_frame_ok;
    uint32_t rx_frame_err;
    uint32_t rx_timeout;
    uint32_t rx_push;
    uint32_t resp_slot_hit;
    uint32_t resp_slot_store;
    uint32_t push_drop;
} mcu_uart_link_diag_t;

#pragma pack(push)
#pragma pack(1)
typedef struct {
    uint64_t t_imu_ns;
    uint8_t timestamp_quality;
    int16_t acc_x;
    int16_t acc_y;
    int16_t acc_z;
    int16_t gyro_x;
    int16_t gyro_y;
    int16_t gyro_z;
} imu_sample_msg_t;
#pragma pack(pop)

#define MCU_IMU_PUSH_TOPIC_COMPACT (0x11u)
#define MCU_IMU_PUSH_COMPACT_VER2  (0x02u)
#define MCU_IMU_PUSH_TIMESTAMP_QUALITY_LOCKED   (1u)
#define MCU_IMU_PUSH_TIMESTAMP_QUALITY_HOLDOVER (2u)
#define MCU_IMU_PUSH_COMPACT_HEAD_LEN (16u)
#define MCU_IMU_PUSH_COMPACT_SAMPLE_BLOCK0 (12u)
#define MCU_IMU_PUSH_COMPACT_TAIL_PER_SAMPLE (14u)
#define MCU_IMU_PUSH_MAX_SAMPLES   (8u)
#define FS_SOC_SYNC_PUSH_TOPIC     (0x20u)


#pragma pack(push)
#pragma pack(1)
typedef struct {
    uint8_t topic;
    uint8_t soc_sync_state;
    uint8_t reserved[2];
    uint32_t sync_seq;
    uint64_t t_soc_sync_event_ns;
} fs_soc_sync_push_payload_t;
#pragma pack(pop)

#ifdef __cplusplus
extern "C" {
#endif

int mcu_prot_set_lidarchip_reg(uint32_t addr, uint32_t val);
int mcu_prot_get_lidarchip_reg(uint32_t addr, uint32_t* val);
// 如果需要读取数据，wlen和wdata需要包含读数据的dummy
int mcu_prot_lidarchip_data_transparent(uint8_t *wdata, uint32_t wlen, uint8_t *rdata, uint32_t rlen);
// int mcu_prot_get_lidarchip_reg_continue(uint32_t addr, uint32_t* val, uint32_t reg_cnt);
int mcu_prot_set_lidarchip_reg_batch(const mcu_prot_batch_reg_t *regs, uint32_t count);
int mcu_prot_get_lidarchip_reg_batch(mcu_prot_batch_reg_t *regs, uint32_t count);
int mcu_prot_lidarchip_hw_reset(uint8_t dev_id);
int mcu_prot_lidarchip_hw_power_ctrl(uint8_t dev_id, uint8_t power_on); // power_on: 1 - on 0 - off
int mcu_prot_set_fl9003_reg(uint8_t dev, uint32_t addr, uint32_t val);
int mcu_prot_get_fl9003_reg(uint8_t dev, uint32_t addr, uint32_t* val);
int mcu_prot_set_fl9103_reg(uint8_t dev, uint32_t addr, uint32_t val);
int mcu_prot_get_fl9103_reg(uint8_t dev, uint32_t addr, uint32_t* val);
int mcu_prot_set_workmode(uint32_t workmode, bool force);
int mcu_prot_set_vpp(float vpp, uint32_t *o_dac_code);
int mcu_prot_set_vbd(float vbd, uint32_t *o_dac_code);
int mcu_prot_set_enable_vpp(uint32_t vbdstate, bool force);
int mcu_prot_set_enable_vbd(uint32_t vbdstate, bool force);
int mcu_prot_set_temp_comp(bool enable);
int mcu_prot_set_voltage_calib(bool enable);
int mcu_prot_get_workmode(uint32_t* workmode);
// int mcu_prot_get_workmode_list(uint32_t *modes, uint32_t *count);
int mcu_prot_set_txseqmode(uint8_t seqmode, uint8_t* chan, uint8_t chan_num);
int mcu_prot_get_txseqmode(uint8_t* seqmode, uint8_t* chan, uint8_t* chan_num);
int mcu_prot_get_mcu_version(uint8_t *data, uint16_t data_len);
int mcu_prot_get_init_state(uint8_t *state);
int mcu_prot_wait_ready(uint32_t timeout_ms);
int mcu_prot_get_spad_sn(uint8_t *data, uint16_t data_len);
int mcu_prot_get_board_temp(void);
int mcu_prot_get_voltages(fs_voltage_info_t *v);
int mcu_prot_get_adc(uint8_t type, uint8_t fmt, int32_t *io_value, uint8_t *io_cnt, uint32_t *o_raw_adc_mV);
int mcu_prot_get_fsync_diag(mcu_fsync_diag_t *diag);
int mcu_prot_get_time_sync_diag(mcu_time_sync_diag_t *diag);
int mcu_prot_get_time_sync_health(mcu_time_sync_health_t *health);
int mcu_prot_time_sync_resync(void);
int mcu_prot_time_sync_ctrl(uint8_t ctrl);

int mcu_prot_program_enter(mcu_prot_program_status_t *out_status);
int mcu_prot_program_exit(mcu_prot_program_status_t *out_status);
int mcu_prot_program_get_status(mcu_prot_program_status_t *out_status);
int mcu_prot_get_imu_push_diag(mcu_imu_push_diag_t *diag);
int mcu_prot_get_uart_load_diag(mcu_uart_load_diag_summary_t *summary,
                                mcu_uart_load_bucket_t *buckets,
                                uint16_t bucket_count);
int mcu_prot_uart_load_echo(const mcu_uart_load_echo_req_t *req,
                            uint8_t *resp_buf,
                            uint16_t resp_buf_size,
                            uint16_t *resp_len);
int mcu_prot_uart_load_get_soc_snapshot(mcu_uart_load_diag_summary_t *summary,
                                        mcu_uart_load_bucket_t *buckets,
                                        uint16_t bucket_count);
int mcu_prot_store_adc_calib(uint8_t adc_type, float k, float b);
int mcu_prot_read_adc_calib(uint8_t adc_type, float *k, float *b);
int mcu_prot_remove_adc_calib(uint8_t adc_type);
int mcu_prot_store_dac_calib(uint8_t dac_type, float k, float b);
int mcu_prot_read_dac_calib(uint8_t dac_type, float *k, float *b);
int mcu_prot_remove_dac_calib(uint8_t dac_type);
int mcu_prot_mcu_enter_upgrade_mode_with_status(mcu_prot_program_status_t *out_status);
int mcu_prot_mcu_enter_upgrade_mode(void);
int mcu_prot_flash_write_session_begin(int data_type);
void mcu_prot_flash_write_session_end(void);
int mcu_prot_flash_write_start(int data_type, mcu_prot_flash_info_t* info);
int mcu_prot_flash_write_pkt_data(int data_type, mcu_prot_flash_pkt_info_t* pkt_info);
int mcu_prot_flash_read_start(int data_type, mcu_prot_flash_info_t* info);
int mcu_prot_flash_read_pkt_data(int data_type, mcu_prot_flash_pkt_info_t* pkt_info);
int mcu_prot_send_flash_data(uint8_t data_type, uint8_t* data, uint32_t len);
int mcu_prot_recv_flash_data(uint8_t data_type, uint8_t** data, uint32_t* len);

int mcu_prot_send_lidar_cfg_data(void);
int mcu_prot_recv_lidar_cfg_data(uint8_t** data, uint32_t* len);

int mcu_prot_uart_get_diag(mcu_uart_link_diag_t *diag);
int mcu_prot_uart_pop_push(uint8_t *buf,
                           uint16_t buf_size,
                           uint16_t *out_len,
                           uint32_t *seq,
                           uint32_t *drop_cnt);
int mcu_prot_uart_pop_imu_batch(imu_sample_msg_t *samples,
                                uint16_t max_samples,
                                uint16_t *out_samples,
                                uint32_t *seq,
                                uint32_t *drop_cnt);
typedef void (*mcu_imu_forward_cb_t)(const imu_sample_msg_t *samples, uint16_t count, void *user);
int mcu_prot_register_imu_forward_cb(mcu_imu_forward_cb_t cb, void *user);
int mcu_prot_send_soc_sync_pub(uint32_t sync_seq, uint64_t t_soc_sync_event_ns, uint8_t soc_sync_state);

int mcu_prot_init(void);
void mcu_prot_deinit(void);
/** Stop protocol threads and close MCU UART (including after partial open). */
void mcu_uart_release(void);

#ifdef __cplusplus
}
#endif

#endif //__MCU_PROTOCOL_H__
