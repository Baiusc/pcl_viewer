/**
 * Firmware interface — MCU runtime system state vocabulary.
 *
 * Distinct from legacy MCU_INIT_STATE_* initialization progress readback.
 * S01 defines names only; no storage or enforcement in product code.
 */

#ifndef FW_IF_SYSTEM_STATE_H
#define FW_IF_SYSTEM_STATE_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/**
 * MCU system state — reviewed lifecycle model for App / SoC managers.
 */
typedef enum {
    FW_IF_MCU_STATE_UNKNOWN = 0,
    FW_IF_MCU_STATE_INIT = 1,
    FW_IF_MCU_STATE_IDLE = 2,
    FW_IF_MCU_STATE_NORMAL_RUN = 3,
    FW_IF_MCU_STATE_PROGRAM = 4,
    FW_IF_MCU_STATE_PROVISIONING_REQUIRED = 5, /**< ABI reserved; v0.6 App does not actively enter. */
    FW_IF_MCU_STATE_UPDATE_PREPARE = 6,
    FW_IF_MCU_STATE_FAULT = 7,
    /* 0x08–0xFE reserved */
    FW_IF_MCU_STATE_RESERVED_BASE = 0x80u,
    FW_IF_MCU_STATE_RESERVED_END = 0xFEu,
    FW_IF_MCU_STATE_INVALID = 0xFFu
} fw_if_mcu_system_state_t;

#ifdef __cplusplus
}
#endif

#endif /* FW_IF_SYSTEM_STATE_H */
