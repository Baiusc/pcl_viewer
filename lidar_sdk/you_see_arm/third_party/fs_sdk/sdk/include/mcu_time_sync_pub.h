#ifndef __MCU_TIME_SYNC_PUB_H__
#define __MCU_TIME_SYNC_PUB_H__

#include <stdint.h>
#include "fs_pointcloud_timesync_caps.h"

#define MCU_TIME_SYNC_PUB_STOPPED  1

typedef enum {
    MCU_TIME_SYNC_PUB_STATE_STOPPED = 0,
    MCU_TIME_SYNC_PUB_STATE_POINTCLOUD_RUNNING = 1,
} mcu_time_sync_pub_state_t;

int  mcu_time_sync_pub_prepare(const fs_pointcloud_timesync_caps_t *caps);
int  mcu_time_sync_pub_start_anchor(void);
int  mcu_time_sync_pub_start_trigger(void);
int  mcu_time_sync_pub_stop(void);
int  mcu_time_sync_pub_poll_once(void);
void mcu_time_sync_pub_set_soc_state(uint8_t soc_sync_state);
void mcu_time_sync_pub_deinit(void);

#endif
