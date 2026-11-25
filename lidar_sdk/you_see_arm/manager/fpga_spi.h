#ifndef LIDAR_FPGA_SPI_H
#define LIDAR_FPGA_SPI_H

#include <stdint.h>

#define LIDAR_BETA_SPOT_MODE 4
#define LIDAR_BETA_SPOT_VERSION 0x11020000U

typedef struct {
	uint32_t version_data;
	uint32_t version_time;
	uint32_t version_num;
	uint32_t hash_value;
	uint32_t mod_state;
} lidar_fpga_version_info_t;

int fpga_print_version(void);
int fpga_read_version_info(lidar_fpga_version_info_t *info);

#endif
