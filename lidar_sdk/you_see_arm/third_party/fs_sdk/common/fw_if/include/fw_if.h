/**
 * Firmware interface — common cross-firmware ABI umbrella header.
 *
 * Stable vocabulary shared by SoC SDK, MCU App, and future Custom Boot.
 * Platform-neutral: stdint/stdbool only; no HAL, OS, or transport codec.
 */

#ifndef FW_IF_H
#define FW_IF_H

#include "fw_if_boot_request.h"
#include "fw_if_flash_map.h"
#include "fw_if_image.h"
#include "fw_if_mcu_protocol.h"
#include "fw_if_protocol.h"
#include "fw_if_system_state.h"

#endif /* FW_IF_H */
