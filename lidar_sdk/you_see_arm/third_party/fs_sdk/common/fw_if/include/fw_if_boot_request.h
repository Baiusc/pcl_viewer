/**
 * Firmware interface — boot request record written to metadata flash.
 *
 * Little-endian scalar fields. Fixed 64-byte record for metadata region storage.
 * Consumed by Custom Boot on reset; unused in standalone-app layout until enabled.
 */

#ifndef FW_IF_BOOT_REQUEST_H
#define FW_IF_BOOT_REQUEST_H

#include <stdint.h>

#include "fw_if_image.h"

#ifdef __cplusplus
extern "C" {
#endif

/** Magic 'FBRQ' (Fortsense boot request). */
#define FW_IF_BOOT_REQUEST_MAGIC 0x46425251u

#define FW_IF_BOOT_REQUEST_STRUCT_VERSION 1u
#define FW_IF_BOOT_REQUEST_SIZE_BYTES 64u

/** Boot request command (metadata command field). */
typedef enum {
    FW_IF_BOOT_REQUEST_CMD_NONE = 0,
    FW_IF_BOOT_REQUEST_CMD_UPDATE_APP = 1,
    FW_IF_BOOT_REQUEST_CMD_RUN_BOOTUPDATER = 2, /**< Reserved */
    FW_IF_BOOT_REQUEST_CMD_CLEAR = 3,
    /* 0x04–0x7F reserved */
    FW_IF_BOOT_REQUEST_CMD_RESERVED_BASE = 0x80u,
    FW_IF_BOOT_REQUEST_CMD_RESERVED_END = 0xFEu,
    FW_IF_BOOT_REQUEST_CMD_INVALID = 0xFFu
} fw_if_boot_request_cmd_t;

/** Boot request flags (bitmask). */
#define FW_IF_BOOT_REQUEST_FLAG_NONE 0x00000000u
#define FW_IF_BOOT_REQUEST_FLAG_FORCE 0x00000001u /**< Skip some soft checks (reserved policy) */
#define FW_IF_BOOT_REQUEST_FLAG_VERIFY_ONLY 0x00000002u /**< Reserved */
/* Bits 2–31 reserved */

#pragma pack(push, 1)

/**
 * Boot request record — App writes, Boot reads on reset.
 * crc32 covers all bytes before the crc32 field (standard CRC-32 policy TBD by integrator).
 */
typedef struct {
    uint32_t magic;
    uint16_t struct_version;
    uint8_t command;   /**< fw_if_boot_request_cmd_t */
    uint8_t target;    /**< fw_if_update_target_t */
    uint32_t flags;
    uint32_t image_size_bytes;
    uint32_t image_crc32;
    uint32_t backend;  /**< fw_if_upgrade_backend_t */
    uint8_t reserved[36];
    uint32_t crc32;
} fw_if_boot_request_t;

#pragma pack(pop)

#ifdef __cplusplus
}
static_assert(sizeof(fw_if_boot_request_t) == FW_IF_BOOT_REQUEST_SIZE_BYTES,
              "fw_if_boot_request_t must be 64 bytes");
#endif

#endif /* FW_IF_BOOT_REQUEST_H */
