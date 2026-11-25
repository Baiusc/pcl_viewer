/* mcu_uart_session.c — all functionality moved to common/fw_link/fw_link_frame.c.
 * This file retained as a compilation unit to satisfy CMake GLOB_RECURSE.
 * Callers use inline wrappers in mcu_uart_session.h which delegate to fw_link_frame_*.
 */

#include "mcu_uart_session.h"

/* Intentionally empty — all logic is in fw_link_frame.c via inline wrappers in the header. */