/**
 * Firmware interface - MCU legacy wire protocol ABI.
 *
 * This header is the shared SoC/MCU source for wire-stable MCU protocol
 * identifiers that new architecture code needs. Existing legacy headers keep
 * their public names; this file provides fw_if-prefixed ABI vocabulary and
 * packed helper payloads.
 */

#ifndef FW_IF_MCU_PROTOCOL_H
#define FW_IF_MCU_PROTOCOL_H

#include <stddef.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef enum {
    FW_IF_MCU_DEV_FL6X32_1 = 0,
    FW_IF_MCU_DEV_FL6X32_ALL,
    FW_IF_MCU_DEV_FL9003_1,
    FW_IF_MCU_DEV_FL9003_2,
    FW_IF_MCU_DEV_FL9003_3,
    FW_IF_MCU_DEV_FL9003_ALL,
    FW_IF_MCU_DEV_FL9103_1,
    FW_IF_MCU_DEV_FL9103_2,
    FW_IF_MCU_DEV_FL9103_ALL,
    FW_IF_MCU_DEV_MCU,
    FW_IF_MCU_DEV_CALIDATA,
    FW_IF_MCU_DEV_CFGDATA,
    FW_IF_MCU_DEV_STATUS = 0x40
} fw_if_mcu_dev_t;

typedef enum {
    FW_IF_MCU_OP_WRITE          = 0x01,
    FW_IF_MCU_OP_READ           = 0x02,
    FW_IF_MCU_OP_WRITE_CONTINUE = 0x03,
    FW_IF_MCU_OP_READ_CONTINUE  = 0x04,
    FW_IF_MCU_OP_WRITE_BATCH    = 0x05,
    FW_IF_MCU_OP_READ_BATCH     = 0x06,
    FW_IF_MCU_OP_RW_TRANSPARENT = 0x07,
    FW_IF_MCU_OP_HW_RESET       = 0x10,
    FW_IF_MCU_OP_HW_POWEROFF    = 0x11,
    FW_IF_MCU_OP_HW_TRX_DAC_EN  = 0x12
} fw_if_mcu_op_t;

#define FW_IF_MCU_SUBCMD_MCU_VER               ((uint32_t)0x0001u)
#define FW_IF_MCU_SUBCMD_SPAD_SN               ((uint32_t)0x0002u)
#define FW_IF_MCU_SUBCMD_BOARD_TEMP            ((uint32_t)0x0003u)
#define FW_IF_MCU_SUBCMD_SPAD_TEMP             ((uint32_t)0x0004u)
#define FW_IF_MCU_SUBCMD_VOLTAGE               ((uint32_t)0x0005u)
#define FW_IF_MCU_SUBCMD_GET_ADC               ((uint32_t)0x0006u)
#define FW_IF_MCU_SUBCMD_INIT_STATE            ((uint32_t)0x0007u)
#define FW_IF_MCU_SUBCMD_WORK_MODE             ((uint32_t)0x0101u)
#define FW_IF_MCU_SUBCMD_TX_SEQMODE            ((uint32_t)0x0102u)
#define FW_IF_MCU_SUBCMD_VBD                   ((uint32_t)0x0103u)
#define FW_IF_MCU_SUBCMD_VPP                   ((uint32_t)0x0104u)
#define FW_IF_MCU_SUBCMD_UPGRADE               ((uint32_t)0x0105u)
#define FW_IF_MCU_SUBCMD_ENABLE_VBD            ((uint32_t)0x0110u)
#define FW_IF_MCU_SUBCMD_ENABLE_VPP            ((uint32_t)0x0111u)
#define FW_IF_MCU_SUBCMD_ENABLE_TEMP_COMP      ((uint32_t)0x0112u)
#define FW_IF_MCU_SUBCMD_GET_TEMP_COMP_STATE   ((uint32_t)0x0113u)
#define FW_IF_MCU_SUBCMD_ENABLE_VOLTAGE_CALIB  ((uint32_t)0x0114u)
#define FW_IF_MCU_SUBCMD_ADC_VBD_CALIB         ((uint32_t)0x0120u)
#define FW_IF_MCU_SUBCMD_ADC_VPP_CALIB         ((uint32_t)0x0121u)
#define FW_IF_MCU_SUBCMD_DAC_VBD_CALIB         ((uint32_t)0x0122u)
#define FW_IF_MCU_SUBCMD_DAC_VPP_CALIB         ((uint32_t)0x0123u)
#define FW_IF_MCU_SUBCMD_REMOVE_ADC_VBD_CALIB  ((uint32_t)0x0124u)
#define FW_IF_MCU_SUBCMD_REMOVE_ADC_VPP_CALIB  ((uint32_t)0x0125u)
#define FW_IF_MCU_SUBCMD_REMOVE_DAC_VBD_CALIB  ((uint32_t)0x0126u)
#define FW_IF_MCU_SUBCMD_REMOVE_DAC_VPP_CALIB  ((uint32_t)0x0127u)
#define FW_IF_MCU_SUBCMD_FSYNC_DIAG            ((uint32_t)0x0130u)
#define FW_IF_MCU_SUBCMD_TIME_SYNC_DIAG        ((uint32_t)0x0131u)
#define FW_IF_MCU_SUBCMD_IMU_PUSH_DIAG         ((uint32_t)0x0132u)
#define FW_IF_MCU_SUBCMD_UART_LOAD_DIAG        ((uint32_t)0x0133u)
#define FW_IF_MCU_SUBCMD_UART_LOAD_ECHO        ((uint32_t)0x0134u)
#define FW_IF_MCU_SUBCMD_TIME_SYNC_HEALTH      ((uint32_t)0x0135u)
#define FW_IF_MCU_SUBCMD_TIME_SYNC_RESYNC      ((uint32_t)0x0136u)
#define FW_IF_MCU_SUBCMD_PROGRAM_ENTER         ((uint32_t)0x0137u)
#define FW_IF_MCU_SUBCMD_PROGRAM_EXIT          ((uint32_t)0x0138u)
#define FW_IF_MCU_SUBCMD_PROGRAM_STATUS        ((uint32_t)0x0139u)
#define FW_IF_MCU_SUBCMD_TIME_SYNC_CTRL        ((uint32_t)0x013Au)

typedef enum {
    FW_IF_MCU_PAYLOAD_NORMAL   = 0,
    FW_IF_MCU_PAYLOAD_STATUS   = 1,
    FW_IF_MCU_PAYLOAD_PUSH_MSG = 2,
    FW_IF_MCU_PAYLOAD_FLASH    = 3,
    FW_IF_MCU_PAYLOAD_COUNT    = 4
} fw_if_mcu_payload_type_t;

#define FW_IF_MCU_FLASH_WRITE_START ((uint8_t)0x01u)
#define FW_IF_MCU_FLASH_WRITE_DATA  ((uint8_t)0x02u)
#define FW_IF_MCU_FLASH_READ_START  ((uint8_t)0x10u)
#define FW_IF_MCU_FLASH_READ_DATA   ((uint8_t)0x11u)

typedef enum {
    FW_IF_MCU_FLASH_DT_CFG = 0,
    FW_IF_MCU_FLASH_DT_CALI = 1
} fw_if_mcu_flash_data_type_t;

typedef enum {
    FW_IF_MCU_INIT_STATE_BOOTING = 0,
    FW_IF_MCU_INIT_STATE_HW_INIT = 1,
    FW_IF_MCU_INIT_STATE_CFG_LOAD = 2,
    FW_IF_MCU_INIT_STATE_HAL_INIT = 3,
    FW_IF_MCU_INIT_STATE_CORE_INIT = 4,
    FW_IF_MCU_INIT_STATE_READY = 5,
    FW_IF_MCU_INIT_STATE_ERROR = 0xFF
} fw_if_mcu_init_state_t;

#pragma pack(push, 1)
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
            uint8_t data0;
            uint8_t data1;
            uint8_t data2;
            uint8_t data3;
        };
    };
    uint8_t crc8;
} fw_if_mcu_cmd_t;

typedef struct {
    uint8_t schema_version;
    uint8_t mcu_state;
    uint8_t reject_reason;
    uint8_t result;
} fw_if_mcu_program_status_t;

#define FW_IF_MCU_REJECT_RESP_SCHEMA_VERSION 1u
typedef struct {
    uint8_t schema_version;
    uint8_t reject_reason;
    uint8_t cmd_dev;
    uint8_t cmd_op;
    uint8_t sub_cmd0;
    uint8_t sub_cmd1;
    uint8_t sub_cmd2;
    uint8_t sub_cmd3;
} fw_if_mcu_reject_resp_t;
#pragma pack(pop)

static inline uint32_t fw_if_mcu_cmd_subcmd(const fw_if_mcu_cmd_t *cmd)
{
    if (cmd == NULL) {
        return 0u;
    }

    return ((uint32_t)cmd->sub_cmd0 << 24)
         | ((uint32_t)cmd->sub_cmd1 << 16)
         | ((uint32_t)cmd->sub_cmd2 << 8)
         | (uint32_t)cmd->sub_cmd3;
}

#ifndef FW_IF_STATIC_ASSERT
#define FW_IF_STATIC_ASSERT(name, expr) \
    typedef char fw_if_assert_##name[(expr) ? 1 : -1]
#endif

FW_IF_STATIC_ASSERT(mcu_cmd_size_15, sizeof(fw_if_mcu_cmd_t) == 15u);
FW_IF_STATIC_ASSERT(mcu_program_status_size_4, sizeof(fw_if_mcu_program_status_t) == 4u);
FW_IF_STATIC_ASSERT(mcu_reject_resp_size_8, sizeof(fw_if_mcu_reject_resp_t) == 8u);
FW_IF_STATIC_ASSERT(mcu_dev_mcu_is_9, FW_IF_MCU_DEV_MCU == 9);
FW_IF_STATIC_ASSERT(mcu_op_write_is_1, FW_IF_MCU_OP_WRITE == 0x01);
FW_IF_STATIC_ASSERT(mcu_op_read_is_2, FW_IF_MCU_OP_READ == 0x02);

#ifdef __cplusplus
}
#endif

#endif /* FW_IF_MCU_PROTOCOL_H */
