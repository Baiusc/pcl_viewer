#include "mcu_log_route.h"

#include "fs_top_module_cfg.h"

bool mcu_log_route_enabled(void)
{
#if !SOC_MCU_LEGACY_LOG_READER_EN || !FS_CFG_FEATURE_LOG
    return false;
#else
    return (FS_CFG_ROUTE_LOG != FS_SOC_MCU_LINK_DISABLED);
#endif
}
