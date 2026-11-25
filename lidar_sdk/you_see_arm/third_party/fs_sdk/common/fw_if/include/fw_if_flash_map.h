/**
 * Firmware interface — on-chip flash region constants (layout vocabulary only).
 *
 * Constants only: no linker sections, no HAL, no placement.
 * Board-specific bases remain unspecified until boot/update board integration.
 */

#ifndef FW_IF_FLASH_MAP_H
#define FW_IF_FLASH_MAP_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/** Address not fixed by this ABI revision; resolve via board map / linker script. */
#define FW_IF_FLASH_ADDR_UNSPECIFIED 0xFFFFFFFFu

/**
 * Flash layout mode.
 * STANDALONE_APP: application linked and executed from flash base (no boot region).
 * CUSTOM_BOOT: optional boot region at base, app slot, metadata at tail.
 */
typedef enum {
    FW_IF_FLASH_LAYOUT_UNKNOWN = 0,
    FW_IF_FLASH_LAYOUT_STANDALONE_APP = 1,
    FW_IF_FLASH_LAYOUT_CUSTOM_BOOT = 2,
    FW_IF_FLASH_LAYOUT_INVALID = 0xFFu
} fw_if_flash_layout_t;

/* -------------------------------------------------------------------------- */
/* Standalone application (reviewed: App from 0x00000000, no boot region)      */
/* -------------------------------------------------------------------------- */

#define FW_IF_FLASH_STANDALONE_APP_BASE_ADDR 0x00000000u

/* -------------------------------------------------------------------------- */
/* Custom boot layout (reviewed topology; sizes are defaults, bases TBD)       */
/* -------------------------------------------------------------------------- */

/** Custom boot region at flash base; reviewed size target 4–8 KiB. */
#define FW_IF_FLASH_BOOT_REGION_BASE_ADDR_DEFAULT 0x00000000u
#define FW_IF_FLASH_BOOT_REGION_SIZE_MIN_BYTES 4096u
#define FW_IF_FLASH_BOOT_REGION_SIZE_MAX_BYTES 8192u
#define FW_IF_FLASH_BOOT_REGION_SIZE_DEFAULT_BYTES 8192u

/**
 * Application slot base after boot region.
 * Default is unspecified; set to (boot_base + boot_size) per product map.
 */
#define FW_IF_FLASH_APP_SLOT_BASE_ADDR_DEFAULT FW_IF_FLASH_ADDR_UNSPECIFIED

/** Metadata / boot-request region at end of on-chip flash (base TBD per map). */
#define FW_IF_FLASH_METADATA_REGION_BASE_ADDR_DEFAULT FW_IF_FLASH_ADDR_UNSPECIFIED
#define FW_IF_FLASH_METADATA_REGION_SIZE_DEFAULT_BYTES 4096u

/** Alias for metadata region holding boot request and app-valid markers. */
#define FW_IF_FLASH_BOOT_REQUEST_REGION_BASE_ADDR_DEFAULT \
    FW_IF_FLASH_METADATA_REGION_BASE_ADDR_DEFAULT
#define FW_IF_FLASH_BOOT_REQUEST_REGION_SIZE_DEFAULT_BYTES \
    FW_IF_FLASH_METADATA_REGION_SIZE_DEFAULT_BYTES

#ifdef __cplusplus
}
#endif

#endif /* FW_IF_FLASH_MAP_H */
