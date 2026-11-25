#include "soc_time_sync_module.h"

#include "fs_top_module_cfg.h"
#include "mcu_time_sync_pub.h"
#include "utils/fs_err.h"

/* PointCloud TimeSync: three-level gate */
#if FS_CFG_FEATURE_POINTCLOUD_TIMESYNC && \
    FS_CFG_FEATURE_MCU_TIMESYNC && \
    (FS_CFG_ROUTE_TIME_SYNC != FS_SOC_MCU_LINK_DISABLED)
#define SOC_TIME_SYNC_MODULE_ACTIVE 1
#else
#define SOC_TIME_SYNC_MODULE_ACTIVE 0
#endif

bool soc_time_sync_module_enabled(void)
{
#if SOC_TIME_SYNC_MODULE_ACTIVE
    return true;
#else
    return false;
#endif
}

int32_t soc_time_sync_module_pub_prepare(const fs_pointcloud_timesync_caps_t *caps)
{
#if SOC_TIME_SYNC_MODULE_ACTIVE
    return mcu_time_sync_pub_prepare(caps);
#else
    (void)caps;
    return FS_SUCCESS;
#endif
}

int32_t soc_time_sync_module_pub_start_anchor(void)
{
#if SOC_TIME_SYNC_MODULE_ACTIVE
    return mcu_time_sync_pub_start_anchor();
#else
    return FS_SUCCESS;
#endif
}

int32_t soc_time_sync_module_pub_start_trigger(void)
{
#if SOC_TIME_SYNC_MODULE_ACTIVE
    return mcu_time_sync_pub_start_trigger();
#else
    return FS_SUCCESS;
#endif
}

int32_t soc_time_sync_module_pub_stop(void)
{
#if SOC_TIME_SYNC_MODULE_ACTIVE
    return mcu_time_sync_pub_stop();
#else
    return FS_SUCCESS;
#endif
}

int32_t soc_time_sync_module_pub_poll_once(void)
{
#if SOC_TIME_SYNC_MODULE_ACTIVE
    return mcu_time_sync_pub_poll_once();
#else
    return SOC_TIME_SYNC_MODULE_PUB_STOPPED;
#endif
}

void soc_time_sync_module_pub_deinit(void)
{
#if SOC_TIME_SYNC_MODULE_ACTIVE
    mcu_time_sync_pub_deinit();
#endif
}
