#ifndef FW_IF_ROUTE_GROUP_H
#define FW_IF_ROUTE_GROUP_H

#include <stdint.h>

#include "fw_if_mcu_protocol.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef enum {
    FW_IF_ROUTE_GROUP_NONE = 0,
    FW_IF_ROUTE_GROUP_CONTROL_CORE,
    FW_IF_ROUTE_GROUP_CFG_STORAGE,
    FW_IF_ROUTE_GROUP_TIME_SYNC,
    FW_IF_ROUTE_GROUP_IMU_PUSH,
    FW_IF_ROUTE_GROUP_DIAGNOSTIC,
    FW_IF_ROUTE_GROUP_LOG,
    FW_IF_ROUTE_GROUP_UPGRADE,
} fw_if_route_group_t;

static inline int fw_if_mcu_is_sensor_dev(uint8_t cmd_dev)
{
    switch (cmd_dev) {
    case FW_IF_MCU_DEV_FL6X32_1:
    case FW_IF_MCU_DEV_FL6X32_ALL:
    case FW_IF_MCU_DEV_FL9003_1:
    case FW_IF_MCU_DEV_FL9003_2:
    case FW_IF_MCU_DEV_FL9003_3:
    case FW_IF_MCU_DEV_FL9003_ALL:
    case FW_IF_MCU_DEV_FL9103_1:
    case FW_IF_MCU_DEV_FL9103_2:
    case FW_IF_MCU_DEV_FL9103_ALL:
        return 1;
    default:
        return 0;
    }
}

static inline fw_if_route_group_t fw_if_mcu_route_group_from_request(uint8_t payload_type,
                                                                     uint8_t cmd_dev,
                                                                     uint8_t cmd_op,
                                                                     uint32_t sub_cmd)
{
    (void)cmd_op;

    if (payload_type == FW_IF_MCU_PAYLOAD_FLASH) {
        return FW_IF_ROUTE_GROUP_CFG_STORAGE;
    }

    if (payload_type == FW_IF_MCU_PAYLOAD_STATUS) {
        return FW_IF_ROUTE_GROUP_CONTROL_CORE;
    }

    if (cmd_dev == FW_IF_MCU_DEV_CFGDATA || cmd_dev == FW_IF_MCU_DEV_CALIDATA) {
        return FW_IF_ROUTE_GROUP_CFG_STORAGE;
    }

    if (fw_if_mcu_is_sensor_dev(cmd_dev)) {
        return FW_IF_ROUTE_GROUP_CONTROL_CORE;
    }

    if (cmd_dev != FW_IF_MCU_DEV_MCU) {
        return FW_IF_ROUTE_GROUP_NONE;
    }

    switch (sub_cmd) {
    case FW_IF_MCU_SUBCMD_UPGRADE:
    case FW_IF_MCU_SUBCMD_PROGRAM_ENTER:
    case FW_IF_MCU_SUBCMD_PROGRAM_EXIT:
    case FW_IF_MCU_SUBCMD_PROGRAM_STATUS:
        return FW_IF_ROUTE_GROUP_UPGRADE;

    case FW_IF_MCU_SUBCMD_TIME_SYNC_HEALTH:
    case FW_IF_MCU_SUBCMD_TIME_SYNC_RESYNC:
    case FW_IF_MCU_SUBCMD_TIME_SYNC_CTRL:
        return FW_IF_ROUTE_GROUP_TIME_SYNC;

    case FW_IF_MCU_SUBCMD_FSYNC_DIAG:
    case FW_IF_MCU_SUBCMD_TIME_SYNC_DIAG:
    case FW_IF_MCU_SUBCMD_IMU_PUSH_DIAG:
    case FW_IF_MCU_SUBCMD_UART_LOAD_DIAG:
    case FW_IF_MCU_SUBCMD_UART_LOAD_ECHO:
        return FW_IF_ROUTE_GROUP_DIAGNOSTIC;

    default:
        return FW_IF_ROUTE_GROUP_CONTROL_CORE;
    }
}

#ifdef __cplusplus
}
#endif

#endif /* FW_IF_ROUTE_GROUP_H */
