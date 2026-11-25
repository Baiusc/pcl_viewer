/**
 * Firmware interface — image type, update target, and upgrade backend.
 *
 * Separates what is being updated from how the update is performed.
 */

#ifndef FW_IF_IMAGE_H
#define FW_IF_IMAGE_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/** Image payload kind (image header and boot validation). */
typedef enum {
    FW_IF_IMAGE_TYPE_UNKNOWN = 0,
    FW_IF_IMAGE_TYPE_NONE = 1,
    FW_IF_IMAGE_TYPE_APP = 2,
    FW_IF_IMAGE_TYPE_BOOT = 3,           /**< Custom boot image (reserved) */
    FW_IF_IMAGE_TYPE_BOOTUPDATER = 4,    /**< Temporary boot updater (reserved) */
    /* 0x05–0x7F reserved */
    FW_IF_IMAGE_TYPE_RESERVED_BASE = 0x80u,
    FW_IF_IMAGE_TYPE_RESERVED_END = 0xFEu,
    FW_IF_IMAGE_TYPE_INVALID = 0xFFu
} fw_if_image_type_t;

/** Logical update destination on the MCU. */
typedef enum {
    FW_IF_UPDATE_TARGET_UNKNOWN = 0,
    FW_IF_UPDATE_TARGET_NONE = 1,
    FW_IF_UPDATE_TARGET_MCU_APP = 2,
    FW_IF_UPDATE_TARGET_MCU_BOOT = 3,    /**< Boot region (reserved) */
    FW_IF_UPDATE_TARGET_METADATA = 4,  /**< Metadata / boot request region */
    /* 0x05–0x7F reserved */
    FW_IF_UPDATE_TARGET_RESERVED_BASE = 0x80u,
    FW_IF_UPDATE_TARGET_RESERVED_END = 0xFEu,
    FW_IF_UPDATE_TARGET_INVALID = 0xFFu
} fw_if_update_target_t;

/**
 * Upgrade transport/backend selected by SoC upgrade entry (not image type).
 */
typedef enum {
    FW_IF_UPGRADE_BACKEND_UNKNOWN = 0,
    FW_IF_UPGRADE_BACKEND_NONE = 1,
    FW_IF_UPGRADE_BACKEND_ROM_ISP = 2,
    FW_IF_UPGRADE_BACKEND_CUSTOM_BOOT = 3,
    FW_IF_UPGRADE_BACKEND_BOOTUPDATER = 4, /**< Via boot-updater slot (reserved) */
    /* 0x05–0x7F reserved */
    FW_IF_UPGRADE_BACKEND_RESERVED_BASE = 0x80u,
    FW_IF_UPGRADE_BACKEND_RESERVED_END = 0xFEu,
    FW_IF_UPGRADE_BACKEND_INVALID = 0xFFu
} fw_if_upgrade_backend_t;

#ifdef __cplusplus
}
#endif

#endif /* FW_IF_IMAGE_H */
