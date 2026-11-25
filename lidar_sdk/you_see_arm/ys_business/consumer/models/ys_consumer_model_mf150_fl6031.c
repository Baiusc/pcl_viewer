/*
 * @Author       : kaihong.yang kaihong.yang@vitalchem.com
 * @Date         : 2025-06-24
 * @FilePath     : /you_see_arm/ys_business/consumer/models/ys_consumer_model_mf150.c
 * @Description  : MF150 消费者模型实现
 * Copyright (c) 2025 by Vitalchem, All Rights Reserved.
 */

#include "ys_consumer_model.h"

#include "../../decode/ys_decode.h"
#include "../../decode/ys_decode_mf150_fl6031.h"
#include "../../ys_def.h"

int consumer_model_get_processed_len(int lines_v, int echo_mode)
{
	(void)lines_v;
	(void)echo_mode;
	return sizeof(mf150_fl6031_point_cloud_packet_t) - 1 +
	       MF150_FL6031_RAW_GROUP_PIXELS * 3 * sizeof(one_mf150_fl6031_point_cloud_point);
}

bool consumer_model_point_cloud_is_valid(const void *data)
{
	return mf150_fl6031_orgin_point_cloud_data_is_valid((const mf150_fl6031_raw_group_packet_t *)data);
}

int consumer_model_transform_point_cloud(const void *raw, void *dest)
{
	(void)raw;
	(void)dest;
	return 0;
}

int consumer_model_parse_point_cloud(const void *raw, void *dest)
{
	return mf150_fl6031_parse_group_to_points((mf150_fl6031_raw_group_packet_t *)raw,
						  (mf150_fl6031_point_cloud_packet_t *)dest);
}
