/*
 * @Author       : kaihong.yang kaihong.yang@vitalchem.com
 * @Date         : 2025-06-24
 * @FilePath     : /you_see_arm/ys_business/consumer/models/ys_consumer_model_cs96_beta.c
 * @Description  : CS96 Beta 消费者模型实现
 * Copyright (c) 2025 by Vitalchem, All Rights Reserved.
 */

#include "ys_consumer_model.h"

#include "../../decode/ys_decode.h"
#include "../../ys_def.h"

int consumer_model_get_processed_len(int lines_v, int echo_mode)
{
	if (echo_mode == SINGLE_ECHO_MODE) {
		return sizeof(point_cloud_head_t) + lines_v * sizeof(cartesian_point_data_t);
	}
	return sizeof(point_cloud_head_t) + lines_v * sizeof(cartesian_point_data_t) * 2;
}

bool consumer_model_point_cloud_is_valid(const void *data)
{
	return orgin_point_cloud_data_is_valid((const raw_protocal_data_packet_t *)data);
}

int consumer_model_transform_point_cloud(const void *raw, void *dest)
{
	return inverted_orgin_point_cloud_data((raw_protocal_data_packet_t *)raw, (uint8_t *)dest);
}

int consumer_model_parse_point_cloud(const void *raw, void *dest)
{
	return deal_orgin_point_cloud_data((raw_protocal_data_packet_t *)raw, (point_cloud_packet_t *)dest);
}
