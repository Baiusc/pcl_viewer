/*
 * @Author       : kaihong.yang kaihong.yang@vitalchem.com
 * @Date         : 2026-06-30
 * @FilePath     : /you_see_arm/ys_business/decode/ys_decode_mf150.c
 * @Description  : MF150 专有解码实现
 * Copyright (c) 2026 by Vitalchem, All Rights Reserved.
 */

#include "ys_decode.h"
#include "ys_decode_mf150_fl6031.h"

#include "../algo/algo_parse_cloud.h"

#include "../../common/common.h"
#include "../../core/ys_base.h"
#include "../../core/ys_conf.h"
#include "../../core/ys_core_def.h"

#include <math.h>

/******************************************************************************/
/*                         MF150 专有解析函数                                  */
/******************************************************************************/

int mf150_fl6031_parse_group_to_points(mf150_fl6031_raw_group_packet_t *raw_group,
				       mf150_fl6031_point_cloud_packet_t *dest_packet)
{
	if (raw_group == NULL || dest_packet == NULL) {
		log_error("input data is NULL");
		return 0;
	}

	SslCalibParam calib;
	calib.tof_lsb_to_meter = 0.000149896f;
	calib.start_theta_rad = -0.436332f;
	calib.step_theta_rad = 0.005585f;
	calib.start_omega_rad = -1.047198f;
	calib.step_omega_rad = 0.005760f;

	dest_packet->head.sof = 0xAA;
	dest_packet->head.frame_id = raw_group->frame_id;
	dest_packet->head.group_id = raw_group->group_id;
	dest_packet->head.timestamp = raw_group->timestamp;

	int len = sizeof(mf150_fl6031_point_cloud_head_t);
	uint16_t point_count = 0;

	for (int p = 0; p < MF150_FL6031_RAW_GROUP_PIXELS; p++) {
		const uint8_t *pixel = raw_group->raw_pixels + p * SSL_RAW_PIXEL_BYTES;

		uint32_t coord = pixel[0] | ((uint32_t)pixel[1] << 8) | ((uint32_t)pixel[2] << 16);
		uint16_t row = coord & 0x1FF;
		uint16_t col = (coord >> 9) & 0x1FF;

		uint32_t echo0_front = pixel[3] | ((uint32_t)pixel[4] << 8) | ((uint32_t)pixel[5] << 16);
		uint32_t echo0_rear = pixel[6] | ((uint32_t)pixel[7] << 8) | ((uint32_t)pixel[8] << 16);
		uint16_t echo0_peak = pixel[9] | ((uint16_t)pixel[10] << 8);

		uint32_t echo1_front = pixel[11] | ((uint32_t)pixel[12] << 8) | ((uint32_t)pixel[13] << 16);
		uint16_t echo1_peak = pixel[17] | ((uint16_t)pixel[18] << 8);

		if (echo0_front == 0 && echo0_peak == 0) {
			continue;
		}

		float theta = calib.start_theta_rad + row * calib.step_theta_rad;
		float omega = calib.start_omega_rad + col * calib.step_omega_rad;
		float r_m = (float)echo0_front * calib.tof_lsb_to_meter;
		float cos_theta = cosf(theta);

		one_mf150_fl6031_point_cloud_point *pt =
			(one_mf150_fl6031_point_cloud_point *)(dest_packet->value + len -
							       sizeof(mf150_fl6031_point_cloud_head_t));

		pt->row_position = row;
		pt->column_position = col;
		pt->distance = r_m * 100.0f;
		pt->reflectivity = echo0_peak;
		pt->position_3d.x = r_m * cos_theta * cosf(omega);
		pt->position_3d.y = r_m * cos_theta * sinf(omega);
		pt->position_3d.z = r_m * sinf(theta);
		pt->peak0_position = echo0_front;
		pt->peak0_value = echo0_peak;
		pt->peak1_position = echo1_front;
		pt->peak1_value = echo1_peak;
		pt->rising_edge_distance = echo0_front;
		pt->falling_edge_distance = echo0_rear;
		pt->pulse_width = echo0_rear - echo0_front;
		pt->timestamp = raw_group->timestamp;

		len += sizeof(one_mf150_fl6031_point_cloud_point);
		point_count++;
	}

	dest_packet->head.point_count = point_count;
	return len;
}

bool mf150_fl6031_orgin_point_cloud_data_is_valid(mf150_fl6031_raw_group_packet_t *raw_data)
{
	(void)raw_data;
	return (cal_param.side_light >> 0) & 1;
}
