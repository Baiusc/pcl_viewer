#ifndef SOC_TIME_SYNC_MODULE_H
#define SOC_TIME_SYNC_MODULE_H

#include <stdbool.h>
#include <stdint.h>
#include "fs_pointcloud_timesync_caps.h"

#define SOC_TIME_SYNC_MODULE_PUB_STOPPED  1

bool    soc_time_sync_module_enabled(void);
int32_t soc_time_sync_module_pub_prepare(const fs_pointcloud_timesync_caps_t *caps);
int32_t soc_time_sync_module_pub_start_anchor(void);
int32_t soc_time_sync_module_pub_start_trigger(void);
int32_t soc_time_sync_module_pub_stop(void);
int32_t soc_time_sync_module_pub_poll_once(void);
void    soc_time_sync_module_pub_deinit(void);

#endif /* SOC_TIME_SYNC_MODULE_H */
