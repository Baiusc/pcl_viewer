#ifndef __FS_CLIENT_DATA_PROCESS_H__
#define __FS_CLIENT_DATA_PROCESS_H__

#include <stdint.h>
#include "drivers/imu/ImuSensor.h"

#ifdef __cplusplus
extern "C" {
#endif

int fs_udp_server_init(void);
int fs_udp_server_uninit(void);
// int fs_send_pcl_data_result(unsigned char *algo_result);
int fs_send_imu_data(imu_sensor_data_t *imu_data, uint32_t count);

#ifdef __cplusplus
}
#endif

#endif
