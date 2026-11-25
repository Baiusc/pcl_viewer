#include <stdint.h>
#include <stdio.h>

#include "fs_err.h"
#include "fs_util.h"
#include "fs_log.h"
#include "fs_version.h"
#include "fs_sdk_api.h"
#include "fs_top_module_cfg.h"

#undef  LOG_TAG
#define LOG_TAG "soc.main"

#if FS_CFG_BOOT_MODE == FS_BOOT_MODE_CUSTOM_BOOT
#define SOC_BOOT_MODE_NAME "custom_boot"
#else
#define SOC_BOOT_MODE_NAME "rom_isp"
#endif

// 雷达工作模式与 RX 配置工作模式的映射表
fs_work_mode_map_t mode_map[FS_SDK_WORK_MODE_CNT] = {
    { .work_mode = FS_SDK_WORK_MODE_IDLE        , .lidar_mode = RX_LIDAR_MODE_IDLE        , },
    { .work_mode = FS_SDK_WORK_MODE_GRAY        , .lidar_mode = RX_LIDAR_MODE_GRAY        , },
    { .work_mode = FS_SDK_WORK_MODE_RANGE       , .lidar_mode = RX_LIDAR_MODE_RANGE_01    , },
    { .work_mode = FS_SDK_WORK_MODE_PCL         , .lidar_mode = RX_LIDAR_MODE_RANGE_01    , },
    { .work_mode = FS_SDK_WORK_MODE_CUSTOM      , .lidar_mode = RX_LIDAR_MODE_RANGE_1     , },
    { .work_mode = FS_SDK_WORK_MODE_GRAY_CUSTOM , .lidar_mode = RX_LIDAR_MODE_GRAY_CUSTOM , },
};

int examples_init(void)
{
    int err = FS_SUCCESS;

    FS_LOGD("Fortsense Lidar SDK Version: %s", fs_sdk_get_version());
    FS_LOGI("boot_mode=%s", SOC_BOOT_MODE_NAME);

    do {
        BREAK_ON_ERR(err = fs_sdk_hw_init());
        BREAK_ON_ERR(err = fs_sdk_mcu_upgrade()); // 可选: MCU 固件升级, 首次开机时会自动烧录 MCU 固件
        BREAK_ON_ERR(err = fs_sdk_mcu_init()); // TODO: 与 MCU 通信协议应当由 example 实现, 而非在 SDK 实现, 因为 SDK 对 MCU 并无太多要求, MCU 只要提供 API 即可
        // BREAK_ON_ERR(err = fs_sdk_pc_support(get_pc_callbacks()));  // 可选: 添加阜时 PC 工具支持, 以使用阜时 AA 调焦工具/校准工具/点云显示工具
        BREAK_ON_ERR(err = fs_sdk_core_init(mode_map, NULL));
        BREAK_ON_ERR(err = fs_sdk_setup_mode(FS_SDK_WORK_MODE_PCL)); // 配置工作模式: 默认进入点云模式
    }
    while(false);

    // mode_ctrl_server_stop();
    // fs_sdk_cleanup();
    return err;
}

int examples_exit(void)
{
    fs_sdk_cleanup();
    return 0;
}
