#ifndef FS_TOP_MODULE_CFG_H
#define FS_TOP_MODULE_CFG_H

/*
 * Project-wide top module configuration.
 *
 * This header intentionally carries only compile-time policy:
 * feature switches, logical backend selection, boot mode, and
 * derived layout values. It must not pull in platform-private
 * driver or OS-specific details.
 */

/* ============================================================
 * 1. Feature compile switches
 * ============================================================ */

/*
 * Default product topology:
 * - SoC-owned time domain.
 * - SoC-owned IMU service.
 *
 * MCU TimeSync is an optional variant for MCU-originated events or sensor data
 * that must be mapped into the SoC time domain. MCU IMU Push is a separate
 * optional backend and is not the same thing as the system IMU service.
 */
#ifndef FS_CFG_FEATURE_MCU_TIMESYNC
#ifdef FS_CFG_FEATURE_TIMESYNC
#define FS_CFG_FEATURE_MCU_TIMESYNC    FS_CFG_FEATURE_TIMESYNC
#else
#define FS_CFG_FEATURE_MCU_TIMESYNC    0
#endif
#endif

#ifndef FS_CFG_FEATURE_MCU_IMU_PUSH
#ifdef FS_CFG_FEATURE_IMU_PUSH
#define FS_CFG_FEATURE_MCU_IMU_PUSH    FS_CFG_FEATURE_IMU_PUSH
#else
#define FS_CFG_FEATURE_MCU_IMU_PUSH    0
#endif
#endif

#ifndef FS_CFG_FEATURE_IMU_SERVICE
#define FS_CFG_FEATURE_IMU_SERVICE     1
#endif

/*
 * Legacy compatibility aliases. These names now mean the optional MCU
 * TimeSync path and MCU-to-SoC IMU Push backend; they no longer represent
 * global time sync or the system IMU service.
 */
#ifndef FS_CFG_FEATURE_TIMESYNC
#define FS_CFG_FEATURE_TIMESYNC        FS_CFG_FEATURE_MCU_TIMESYNC
#elif (FS_CFG_FEATURE_TIMESYNC != FS_CFG_FEATURE_MCU_TIMESYNC)
#error "FS_CFG_FEATURE_TIMESYNC conflicts with FS_CFG_FEATURE_MCU_TIMESYNC"
#endif

#ifndef FS_CFG_FEATURE_IMU_PUSH
#define FS_CFG_FEATURE_IMU_PUSH        FS_CFG_FEATURE_MCU_IMU_PUSH
#elif (FS_CFG_FEATURE_IMU_PUSH != FS_CFG_FEATURE_MCU_IMU_PUSH)
#error "FS_CFG_FEATURE_IMU_PUSH conflicts with FS_CFG_FEATURE_MCU_IMU_PUSH"
#endif

#ifndef FS_CFG_FEATURE_CFG_STORAGE
#define FS_CFG_FEATURE_CFG_STORAGE     1
#endif

#ifndef FS_CFG_FEATURE_MCU_UPGRADE
#define FS_CFG_FEATURE_MCU_UPGRADE     1
#endif

#ifndef FS_CFG_FEATURE_DIAGNOSTIC
#define FS_CFG_FEATURE_DIAGNOSTIC      1
#endif

#ifndef FS_CFG_FEATURE_LOG
#define FS_CFG_FEATURE_LOG             1
#endif

/* ============================================================
 * 1.x PointCloud TimeSync feature switch
 * ============================================================ */
#ifndef FS_CFG_FEATURE_POINTCLOUD_TIMESYNC
#define FS_CFG_FEATURE_POINTCLOUD_TIMESYNC  FS_CFG_FEATURE_MCU_TIMESYNC
#endif

#if FS_CFG_FEATURE_POINTCLOUD_TIMESYNC && !FS_CFG_FEATURE_MCU_TIMESYNC
#error "FS_CFG_FEATURE_POINTCLOUD_TIMESYNC requires FS_CFG_FEATURE_MCU_TIMESYNC"
#endif

#ifndef FS_CFG_POINTCLOUD_TRIGGER_TO_SPAD_FSYNC_DELAY_NS
#define FS_CFG_POINTCLOUD_TRIGGER_TO_SPAD_FSYNC_DELAY_NS  0
#endif

#define FS_CFG_PCL_TRIGGER_MODE_EXTERNAL 0
#define FS_CFG_PCL_TRIGGER_MODE_INTERNAL 1

#ifndef FS_CFG_PCL_TRIGGER_MODE
#define FS_CFG_PCL_TRIGGER_MODE FS_CFG_PCL_TRIGGER_MODE_INTERNAL
#endif

#if (FS_CFG_PCL_TRIGGER_MODE != FS_CFG_PCL_TRIGGER_MODE_EXTERNAL) && \
    (FS_CFG_PCL_TRIGGER_MODE != FS_CFG_PCL_TRIGGER_MODE_INTERNAL)
#error "FS_CFG_PCL_TRIGGER_MODE must be EXTERNAL or INTERNAL"
#endif

/* ============================================================
 * 1.y Board sync topology selection
 * ============================================================ */
#define FS_BOARD_SYNC_TOPOLOGY_L20_SHARED_FSYNC           0
#define FS_BOARD_SYNC_TOPOLOGY_LPF_SOC_FSYNC_TO_I2C_SCL   1

#ifndef FS_CFG_BOARD_SYNC_TOPOLOGY
#define FS_CFG_BOARD_SYNC_TOPOLOGY  FS_BOARD_SYNC_TOPOLOGY_L20_SHARED_FSYNC
#endif

/* ============================================================
 * 2. Boot mode selection (was §3, renumbered after removing scan sync profile)
 * ============================================================ */

#define FS_BOOT_MODE_ROM_ISP           0
#define FS_BOOT_MODE_CUSTOM_BOOT       1

#ifndef FS_CFG_BOOT_MODE
#define FS_CFG_BOOT_MODE               FS_BOOT_MODE_ROM_ISP
#endif

/* ============================================================
 * 4. SoC-MCU interconnect backend
 * ============================================================ */

#define FS_SOC_MCU_LINK_DISABLED          0
#define FS_SOC_MCU_LINK_UART              1
#define FS_SOC_MCU_LINK_I2C               2
#define FS_SOC_MCU_LINK_ROM_ISP_UART      3
#define FS_SOC_MCU_LINK_CUSTOM_BOOT_UART  4

/*
 * Phase 1 keeps fw_link backends transport-shaped: DISABLED / UART / I2C.
 * ROM_ISP_UART and CUSTOM_BOOT_UART describe upgrade data-link selection only;
 * they still collapse to the UART backend and do not imply that fw_link owns
 * BOOT/MODE/RST GPIO sidebands.
 */
#define FS_LINK_BACKEND_DISABLED          FS_SOC_MCU_LINK_DISABLED
#define FS_LINK_BACKEND_UART              FS_SOC_MCU_LINK_UART
#define FS_LINK_BACKEND_I2C               FS_SOC_MCU_LINK_I2C

#define FS_CFG_ROUTE_TO_FW_LINK_BACKEND(route_) \
    (((route_) == FS_SOC_MCU_LINK_DISABLED) ? FS_LINK_BACKEND_DISABLED : \
     (((route_) == FS_SOC_MCU_LINK_I2C) ? FS_LINK_BACKEND_I2C : FS_LINK_BACKEND_UART))

/* ============================================================
 * 5. Protocol route matrix
 * ============================================================ */

#ifndef FS_CFG_ROUTE_CONTROL_CORE
#define FS_CFG_ROUTE_CONTROL_CORE      FS_SOC_MCU_LINK_UART
#endif

#ifndef FS_CFG_ROUTE_SENSOR_REG
#define FS_CFG_ROUTE_SENSOR_REG        FS_SOC_MCU_LINK_UART
#endif

#ifndef FS_CFG_ROUTE_CFG_STORAGE
#define FS_CFG_ROUTE_CFG_STORAGE       FS_SOC_MCU_LINK_UART
#endif

#ifndef FS_CFG_ROUTE_TIME_SYNC
#if FS_CFG_FEATURE_MCU_TIMESYNC
#define FS_CFG_ROUTE_TIME_SYNC         FS_SOC_MCU_LINK_UART
#else
#define FS_CFG_ROUTE_TIME_SYNC         FS_SOC_MCU_LINK_DISABLED
#endif
#endif

#ifndef FS_CFG_ROUTE_IMU_PUSH
#if FS_CFG_FEATURE_MCU_IMU_PUSH
#define FS_CFG_ROUTE_IMU_PUSH          FS_SOC_MCU_LINK_UART
#else
#define FS_CFG_ROUTE_IMU_PUSH          FS_SOC_MCU_LINK_DISABLED
#endif
#endif

#ifndef FS_CFG_ROUTE_DIAGNOSTIC
#define FS_CFG_ROUTE_DIAGNOSTIC        FS_SOC_MCU_LINK_UART
#endif

#ifndef FS_CFG_ROUTE_LOG
#define FS_CFG_ROUTE_LOG               FS_SOC_MCU_LINK_UART
#endif

#ifndef FS_CFG_ROUTE_UPGRADE
#if FS_CFG_BOOT_MODE == FS_BOOT_MODE_CUSTOM_BOOT
/* Custom Boot still uses a UART data path plus external boot/reset sideband. */
#define FS_CFG_ROUTE_UPGRADE           FS_SOC_MCU_LINK_CUSTOM_BOOT_UART
#else
/* ROM ISP uses a UART data path; mode switching stays outside fw_link. */
#define FS_CFG_ROUTE_UPGRADE           FS_SOC_MCU_LINK_ROM_ISP_UART
#endif
#endif

/* ============================================================
 * 6. Compatibility aliases for legacy link selectors
 * ============================================================ */

#if defined(FS_CFG_LINK_CONTROL) && (FS_CFG_LINK_CONTROL != FS_CFG_ROUTE_CONTROL_CORE)
#error "FS_CFG_LINK_CONTROL conflicts with FS_CFG_ROUTE_CONTROL_CORE"
#endif
#if defined(FS_CFG_LINK_STORAGE) && (FS_CFG_LINK_STORAGE != FS_CFG_ROUTE_CFG_STORAGE)
#error "FS_CFG_LINK_STORAGE conflicts with FS_CFG_ROUTE_CFG_STORAGE"
#endif
#if defined(FS_CFG_LINK_DIAGNOSTIC) && (FS_CFG_LINK_DIAGNOSTIC != FS_CFG_ROUTE_DIAGNOSTIC)
#error "FS_CFG_LINK_DIAGNOSTIC conflicts with FS_CFG_ROUTE_DIAGNOSTIC"
#endif
#if defined(FS_CFG_LINK_TIME_SYNC) && (FS_CFG_LINK_TIME_SYNC != FS_CFG_ROUTE_TIME_SYNC)
#error "FS_CFG_LINK_TIME_SYNC conflicts with FS_CFG_ROUTE_TIME_SYNC"
#endif
#if defined(FS_CFG_LINK_IMU_PUSH) && (FS_CFG_LINK_IMU_PUSH != FS_CFG_ROUTE_IMU_PUSH)
#error "FS_CFG_LINK_IMU_PUSH conflicts with FS_CFG_ROUTE_IMU_PUSH"
#endif
#if defined(FS_CFG_LINK_LOG) && (FS_CFG_LINK_LOG != FS_CFG_ROUTE_LOG)
#error "FS_CFG_LINK_LOG conflicts with FS_CFG_ROUTE_LOG"
#endif

#ifdef FS_CFG_LINK_UPDATE_BOOT
#define FS_CFG_LINK_UPDATE_BOOT_LEGACY  FS_CFG_LINK_UPDATE_BOOT
#endif

#ifndef FS_CFG_LINK_UPDATE_BOOT
#define FS_CFG_LINK_UPDATE_BOOT        FS_CFG_ROUTE_TO_FW_LINK_BACKEND(FS_CFG_ROUTE_UPGRADE)
#endif

#ifdef FS_CFG_LINK_UPDATE_BOOT_LEGACY
#if (FS_CFG_LINK_UPDATE_BOOT_LEGACY != FS_CFG_ROUTE_TO_FW_LINK_BACKEND(FS_CFG_ROUTE_UPGRADE))
#error "FS_CFG_LINK_UPDATE_BOOT conflicts with FS_CFG_ROUTE_UPGRADE"
#endif
#endif

#ifndef FS_CFG_LINK_CONTROL
#define FS_CFG_LINK_CONTROL            FS_CFG_ROUTE_TO_FW_LINK_BACKEND(FS_CFG_ROUTE_CONTROL_CORE)
#endif

#ifndef FS_CFG_LINK_STORAGE
#define FS_CFG_LINK_STORAGE            FS_CFG_ROUTE_TO_FW_LINK_BACKEND(FS_CFG_ROUTE_CFG_STORAGE)
#endif

#ifndef FS_CFG_LINK_DIAGNOSTIC
#define FS_CFG_LINK_DIAGNOSTIC         FS_CFG_ROUTE_TO_FW_LINK_BACKEND(FS_CFG_ROUTE_DIAGNOSTIC)
#endif

#ifndef FS_CFG_LINK_TIME_SYNC
#define FS_CFG_LINK_TIME_SYNC          FS_CFG_ROUTE_TO_FW_LINK_BACKEND(FS_CFG_ROUTE_TIME_SYNC)
#endif

#ifndef FS_CFG_LINK_IMU_PUSH
#define FS_CFG_LINK_IMU_PUSH           FS_CFG_ROUTE_TO_FW_LINK_BACKEND(FS_CFG_ROUTE_IMU_PUSH)
#endif

#ifndef FS_CFG_LINK_LOG
#define FS_CFG_LINK_LOG                FS_CFG_ROUTE_TO_FW_LINK_BACKEND(FS_CFG_ROUTE_LOG)
#endif

#ifndef FS_CFG_I2C_TIME_SYNC_SLAVE_ADDR
#define FS_CFG_I2C_TIME_SYNC_SLAVE_ADDR 0x66u
#endif

#ifndef FS_CFG_I2C_TIME_SYNC_REG_PUSH
#define FS_CFG_I2C_TIME_SYNC_REG_PUSH  0x02u
#endif

/* ============================================================
 * 7. Flash layout derivation
 * ============================================================ */

#ifndef FS_CFG_MCU_FLASH_BASE
#define FS_CFG_MCU_FLASH_BASE          0x00000000UL
#endif

#ifndef FS_CFG_MCU_FLASH_SIZE
#define FS_CFG_MCU_FLASH_SIZE          (128UL * 1024UL)
#endif

#ifndef FS_CFG_MCU_FLASH_RESERVED_SIZE
#define FS_CFG_MCU_FLASH_RESERVED_SIZE 1024UL
#endif

#ifndef FS_CFG_MCU_CUSTOM_BOOT_SIZE
#define FS_CFG_MCU_CUSTOM_BOOT_SIZE    (4UL * 1024UL)
#endif

#ifndef FS_CFG_MCU_BOOT_CTRL_SIZE
#define FS_CFG_MCU_BOOT_CTRL_SIZE      512UL
#endif

#ifndef FS_CFG_MCU_META_OFFSET
#define FS_CFG_MCU_META_OFFSET         0x00000100UL
#endif

#ifndef FS_CFG_MCU_META_SIZE
#define FS_CFG_MCU_META_SIZE           0x00000100UL
#endif

#ifndef FS_CFG_MCU_SRAM_BASE
#define FS_CFG_MCU_SRAM_BASE           0x20000000UL
#endif

#ifndef FS_CFG_MCU_SRAM_SIZE
#define FS_CFG_MCU_SRAM_SIZE           (16UL * 1024UL)
#endif

#ifndef FS_CFG_MCU_SHARED_RAM_SIZE
#define FS_CFG_MCU_SHARED_RAM_SIZE     128UL
#endif

#if FS_CFG_BOOT_MODE == FS_BOOT_MODE_CUSTOM_BOOT
#define FS_CFG_MCU_BOOT_SIZE           FS_CFG_MCU_CUSTOM_BOOT_SIZE
#define FS_CFG_MCU_APP_START           (FS_CFG_MCU_FLASH_BASE + FS_CFG_MCU_CUSTOM_BOOT_SIZE)
#else
#define FS_CFG_MCU_BOOT_SIZE           0UL
#undef FS_CFG_MCU_META_OFFSET
#define FS_CFG_MCU_META_OFFSET         0UL
#undef FS_CFG_MCU_META_SIZE
#define FS_CFG_MCU_META_SIZE           0UL
#define FS_CFG_MCU_APP_START           FS_CFG_MCU_FLASH_BASE
#endif

#define FS_CFG_MCU_APP_LIMIT           \
    (FS_CFG_MCU_FLASH_BASE + FS_CFG_MCU_FLASH_SIZE - FS_CFG_MCU_FLASH_RESERVED_SIZE)
#define FS_CFG_MCU_APP_SLOT_SIZE       (FS_CFG_MCU_APP_LIMIT - FS_CFG_MCU_APP_START)
#define FS_CFG_MCU_BOOT_CTRL_ADDR      FS_CFG_MCU_APP_LIMIT
#define FS_CFG_MCU_SHARED_RAM_BASE     \
    (FS_CFG_MCU_SRAM_BASE + FS_CFG_MCU_SRAM_SIZE - FS_CFG_MCU_SHARED_RAM_SIZE)
#define FS_CFG_MCU_RUNTIME_RAM_SIZE    (FS_CFG_MCU_SRAM_SIZE - FS_CFG_MCU_SHARED_RAM_SIZE)

/* ============================================================
 * 8. Consistency checks
 * ============================================================ */

#if (FS_CFG_BOOT_MODE != FS_BOOT_MODE_ROM_ISP) && \
    (FS_CFG_BOOT_MODE != FS_BOOT_MODE_CUSTOM_BOOT)
#error "FS_CFG_BOOT_MODE must be FS_BOOT_MODE_ROM_ISP or FS_BOOT_MODE_CUSTOM_BOOT"
#endif

#if (FS_CFG_ROUTE_CONTROL_CORE != FS_SOC_MCU_LINK_UART) && \
    (FS_CFG_ROUTE_CONTROL_CORE != FS_SOC_MCU_LINK_I2C)
#error "FS_CFG_ROUTE_CONTROL_CORE must use UART or I2C"
#endif

#if (FS_CFG_ROUTE_SENSOR_REG != FS_SOC_MCU_LINK_UART) && \
    (FS_CFG_ROUTE_SENSOR_REG != FS_SOC_MCU_LINK_I2C)
#error "FS_CFG_ROUTE_SENSOR_REG must use UART or I2C"
#endif

#if (FS_CFG_ROUTE_CFG_STORAGE != FS_SOC_MCU_LINK_DISABLED) && \
    (FS_CFG_ROUTE_CFG_STORAGE != FS_SOC_MCU_LINK_UART) && \
    (FS_CFG_ROUTE_CFG_STORAGE != FS_SOC_MCU_LINK_I2C)
#error "FS_CFG_ROUTE_CFG_STORAGE must use DISABLED, UART, or I2C"
#endif

#if (FS_CFG_ROUTE_TIME_SYNC != FS_SOC_MCU_LINK_DISABLED) && \
    (FS_CFG_ROUTE_TIME_SYNC != FS_SOC_MCU_LINK_UART) && \
    (FS_CFG_ROUTE_TIME_SYNC != FS_SOC_MCU_LINK_I2C)
#error "FS_CFG_ROUTE_TIME_SYNC must use DISABLED, UART, or I2C"
#endif

#if (FS_CFG_ROUTE_IMU_PUSH != FS_SOC_MCU_LINK_DISABLED) && \
    (FS_CFG_ROUTE_IMU_PUSH != FS_SOC_MCU_LINK_UART) && \
    (FS_CFG_ROUTE_IMU_PUSH != FS_SOC_MCU_LINK_I2C)
#error "FS_CFG_ROUTE_IMU_PUSH must use DISABLED, UART, or I2C"
#endif

#if (FS_CFG_ROUTE_DIAGNOSTIC != FS_SOC_MCU_LINK_DISABLED) && \
    (FS_CFG_ROUTE_DIAGNOSTIC != FS_SOC_MCU_LINK_UART) && \
    (FS_CFG_ROUTE_DIAGNOSTIC != FS_SOC_MCU_LINK_I2C)
#error "FS_CFG_ROUTE_DIAGNOSTIC must use DISABLED, UART, or I2C"
#endif

#if (FS_CFG_ROUTE_LOG != FS_SOC_MCU_LINK_DISABLED) && \
    (FS_CFG_ROUTE_LOG != FS_SOC_MCU_LINK_UART) && \
    (FS_CFG_ROUTE_LOG != FS_SOC_MCU_LINK_I2C)
#error "FS_CFG_ROUTE_LOG must use DISABLED, UART, or I2C"
#endif

#if (FS_CFG_ROUTE_UPGRADE != FS_SOC_MCU_LINK_DISABLED) && \
    (FS_CFG_ROUTE_UPGRADE != FS_SOC_MCU_LINK_ROM_ISP_UART) && \
    (FS_CFG_ROUTE_UPGRADE != FS_SOC_MCU_LINK_CUSTOM_BOOT_UART)
#error "FS_CFG_ROUTE_UPGRADE must use DISABLED, ROM_ISP_UART, or CUSTOM_BOOT_UART"
#endif

#if (FS_CFG_ROUTE_CONTROL_CORE == FS_SOC_MCU_LINK_DISABLED)
#error "CONTROL_CORE route cannot be disabled"
#endif

#if (FS_CFG_ROUTE_SENSOR_REG != FS_CFG_ROUTE_CONTROL_CORE)
#error "SENSOR_REG route must match CONTROL_CORE in phase 1"
#endif

#if !FS_CFG_FEATURE_MCU_TIMESYNC && (FS_CFG_ROUTE_TIME_SYNC != FS_SOC_MCU_LINK_DISABLED)
#error "FS_CFG_ROUTE_TIME_SYNC must be DISABLED when FS_CFG_FEATURE_MCU_TIMESYNC=0"
#endif

#if FS_CFG_FEATURE_POINTCLOUD_TIMESYNC && \
    (FS_CFG_ROUTE_TIME_SYNC == FS_SOC_MCU_LINK_DISABLED)
#error "FS_CFG_FEATURE_POINTCLOUD_TIMESYNC requires FS_CFG_ROUTE_TIME_SYNC enabled"
#endif

#if !FS_CFG_FEATURE_MCU_IMU_PUSH && (FS_CFG_ROUTE_IMU_PUSH != FS_SOC_MCU_LINK_DISABLED)
#error "FS_CFG_ROUTE_IMU_PUSH must be DISABLED when FS_CFG_FEATURE_MCU_IMU_PUSH=0"
#endif

/*
 * The current MCU IMU Push backend timestamps samples through fs_time_map.
 * Keep the features independently named, but reject builds that enable MCU
 * IMU Push without the optional MCU TimeSync path it currently depends on.
 */
#if FS_CFG_FEATURE_MCU_IMU_PUSH && !FS_CFG_FEATURE_MCU_TIMESYNC
#error "FS_CFG_FEATURE_MCU_IMU_PUSH requires FS_CFG_FEATURE_MCU_TIMESYNC"
#endif

#if !FS_CFG_FEATURE_CFG_STORAGE && (FS_CFG_ROUTE_CFG_STORAGE != FS_SOC_MCU_LINK_DISABLED)
#error "FS_CFG_ROUTE_CFG_STORAGE must be DISABLED when FS_CFG_FEATURE_CFG_STORAGE=0"
#endif

#if !FS_CFG_FEATURE_MCU_UPGRADE && (FS_CFG_ROUTE_UPGRADE != FS_SOC_MCU_LINK_DISABLED)
#error "FS_CFG_ROUTE_UPGRADE must be DISABLED when FS_CFG_FEATURE_MCU_UPGRADE=0"
#endif

#if !FS_CFG_FEATURE_DIAGNOSTIC && (FS_CFG_ROUTE_DIAGNOSTIC != FS_SOC_MCU_LINK_DISABLED)
#error "FS_CFG_ROUTE_DIAGNOSTIC must be DISABLED when FS_CFG_FEATURE_DIAGNOSTIC=0"
#endif

#if !FS_CFG_FEATURE_LOG && (FS_CFG_ROUTE_LOG != FS_SOC_MCU_LINK_DISABLED)
#error "FS_CFG_ROUTE_LOG must be DISABLED when FS_CFG_FEATURE_LOG=0"
#endif

#if FS_CFG_FEATURE_MCU_UPGRADE && \
    (FS_CFG_BOOT_MODE == FS_BOOT_MODE_ROM_ISP) && \
    (FS_CFG_ROUTE_UPGRADE != FS_SOC_MCU_LINK_ROM_ISP_UART)
#error "FS_CFG_ROUTE_UPGRADE must be ROM_ISP_UART in ROM ISP mode"
#endif

#if FS_CFG_FEATURE_MCU_UPGRADE && \
    (FS_CFG_BOOT_MODE == FS_BOOT_MODE_CUSTOM_BOOT) && \
    (FS_CFG_ROUTE_UPGRADE != FS_SOC_MCU_LINK_CUSTOM_BOOT_UART)
#error "FS_CFG_ROUTE_UPGRADE must be CUSTOM_BOOT_UART in custom boot mode"
#endif

#if (FS_CFG_ROUTE_TIME_SYNC == FS_SOC_MCU_LINK_I2C)
#error "I2C TIME_SYNC route is reserved but not implemented in phase 1"
#endif

#if (FS_CFG_ROUTE_CONTROL_CORE == FS_SOC_MCU_LINK_I2C)
#error "I2C CONTROL_CORE route is reserved but not implemented in phase 1"
#endif

#if (FS_CFG_ROUTE_IMU_PUSH == FS_SOC_MCU_LINK_I2C)
#error "I2C IMU_PUSH route is not supported"
#endif

#if (FS_CFG_ROUTE_DIAGNOSTIC == FS_SOC_MCU_LINK_I2C)
#error "I2C DIAGNOSTIC route is not supported"
#endif

#if (FS_CFG_ROUTE_LOG == FS_SOC_MCU_LINK_I2C)
#error "I2C LOG route is not supported"
#endif

#if (FS_CFG_BOOT_MODE == FS_BOOT_MODE_CUSTOM_BOOT) && (FS_CFG_MCU_CUSTOM_BOOT_SIZE == 0UL)
#error "FS_CFG_MCU_CUSTOM_BOOT_SIZE must be non-zero in custom boot mode"
#endif

#if (FS_CFG_MCU_APP_LIMIT <= FS_CFG_MCU_APP_START)
#error "FS_CFG_MCU_APP_LIMIT must be greater than FS_CFG_MCU_APP_START"
#endif

#if (FS_CFG_MCU_SRAM_SIZE <= FS_CFG_MCU_SHARED_RAM_SIZE)
#error "FS_CFG_MCU_SHARED_RAM_SIZE must be smaller than FS_CFG_MCU_SRAM_SIZE"
#endif

#if (FS_CFG_LINK_CONTROL > FS_LINK_BACKEND_I2C) || \
    (FS_CFG_LINK_STORAGE > FS_LINK_BACKEND_I2C) || \
    (FS_CFG_LINK_UPDATE_BOOT > FS_LINK_BACKEND_I2C) || \
    (FS_CFG_LINK_DIAGNOSTIC > FS_LINK_BACKEND_I2C) || \
    (FS_CFG_LINK_TIME_SYNC > FS_LINK_BACKEND_I2C) || \
    (FS_CFG_LINK_IMU_PUSH > FS_LINK_BACKEND_I2C) || \
    (FS_CFG_LINK_LOG > FS_LINK_BACKEND_I2C)
#error "FS_CFG_LINK_* must use DISABLED, UART, or I2C"
#endif

#if (FS_CFG_I2C_TIME_SYNC_REG_PUSH > 0xffu)
#error "FS_CFG_I2C_TIME_SYNC_REG_PUSH must fit in one byte"
#endif

#endif /* FS_TOP_MODULE_CFG_H */
