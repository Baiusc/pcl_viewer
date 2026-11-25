/**
 * Firmware interface — protocol ABI version, command class, and reject reason.
 *
 * Platform-neutral; stdint/stdbool only. No transport framing or on-wire IDs.
 */

#ifndef FW_IF_PROTOCOL_H
#define FW_IF_PROTOCOL_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/** Overall fw_if ABI version (independent of UART payload layout). */
#define FW_IF_ABI_VERSION_MAJOR 1u
#define FW_IF_ABI_VERSION_MINOR 0u

/** Packed ABI version for storage or comparison: (major << 16) | minor. */
#define FW_IF_ABI_VERSION_PACKED \
    ((uint32_t)(((uint32_t)FW_IF_ABI_VERSION_MAJOR << 16) | (uint32_t)FW_IF_ABI_VERSION_MINOR))

/**
 * Logical protocol vocabulary version for future gate and manager alignment.
 * Does not replace existing Layer-1 device/op/subcmd values in product headers.
 */
#define FW_IF_PROTOCOL_VERSION 1u

/**
 * Command class — future command-gate classification (not Layer-1 cmd_dev/cmd_op).
 */
typedef enum {
    FW_IF_CMD_CLASS_UNKNOWN = 0,
    FW_IF_CMD_CLASS_NONE = 1,
    FW_IF_CMD_CLASS_CONTROL = 2,       /**< Register, mode, reset, power */
    FW_IF_CMD_CLASS_STORAGE = 3,       /**< CFGDATA / CALIDATA transfer */
    FW_IF_CMD_CLASS_DIAGNOSTIC = 4,    /**< Read-only diagnostics */
    FW_IF_CMD_CLASS_UPDATE = 5,        /**< Firmware update / boot entry */
    FW_IF_CMD_CLASS_TIME_SYNC = 6,
    FW_IF_CMD_CLASS_IMU_PUSH = 7,
    FW_IF_CMD_CLASS_LOG = 8,
    /* 0x09–0x7F reserved for future classes */
    FW_IF_CMD_CLASS_RESERVED_BASE = 0x80u,
    FW_IF_CMD_CLASS_RESERVED_END = 0xFEu,
    FW_IF_CMD_CLASS_INVALID = 0xFFu
} fw_if_cmd_class_t;

/**
 * Reject reason — future command/service gate responses and diagnostics.
 * Distinct from legacy PROT_STATUS_* and CFG_SLOT_ERR_* numeric spaces.
 */
typedef enum {
    FW_IF_REJECT_NONE = 0,
    FW_IF_REJECT_OK = 1,
    FW_IF_REJECT_BUSY = 2,
    FW_IF_REJECT_UNSUPPORTED = 3,
    FW_IF_REJECT_INVALID_STATE = 4,
    FW_IF_REJECT_INVALID_ARGUMENT = 5,
    FW_IF_REJECT_LENGTH = 6,
    FW_IF_REJECT_CRC = 7,
    FW_IF_REJECT_STORAGE = 8,
    FW_IF_REJECT_TIMEOUT = 9,
    FW_IF_REJECT_PERMISSION = 10,      /**< Auth / policy (reserved) */
    FW_IF_REJECT_UPDATE = 11,
    FW_IF_REJECT_INTERNAL = 12,
    FW_IF_REJECT_UNKNOWN = 13,
    /* 0x0E–0x7F reserved */
    FW_IF_REJECT_RESERVED_BASE = 0x80u,
    FW_IF_REJECT_RESERVED_END = 0xFEu,
    FW_IF_REJECT_INVALID = 0xFFu
} fw_if_reject_reason_t;

#ifdef __cplusplus
}
#endif

#endif /* FW_IF_PROTOCOL_H */
