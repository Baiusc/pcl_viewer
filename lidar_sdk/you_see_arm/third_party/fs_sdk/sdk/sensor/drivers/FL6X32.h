#ifndef __FL6X32_H__
#define __FL6X32_H__

#include <stdint.h>
#include <stdbool.h>
#include "rx_cfg_loader.h"
#include "fs_sensor_hub.h"
// #include "fs_config.h"

#ifdef __cplusplus
extern "C" {
#endif

int32_t FL6X32_deinit(fs_rx_t *rx); // 反初始化设备
int32_t FL6X32_init(fs_rx_t *rx, const int16_t *dev_id, uint8_t dev_cnt); // 初始化设备

int32_t FL6X32_read_register(fs_sensor_tx_rx_e idx, uint32_t addr, uint32_t *data, uint32_t count);
int32_t FL6X32_write_register(fs_sensor_tx_rx_e idx, uint32_t addr, const uint32_t *data, uint32_t count);
#ifdef __cplusplus
}
#endif

#endif /* __FL6X32_H__ */
