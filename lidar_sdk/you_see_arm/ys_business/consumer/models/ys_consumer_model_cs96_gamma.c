/*
 * @Author       : kaihong.yang kaihong.yang@vitalchem.com
 * @Date         : 2026-10-09
 * @FilePath     : /you_see_arm/ys_business/consumer/models/ys_consumer_model_cs96_gamma.c
 * @Description  : CS96 Gamma 消费者模型实现
 * Copyright (c) 2026 by Vitalchem, All Rights Reserved.
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
	// 原始模式用于核对 FPGA 数据，不按标定参数中的面使能过滤 SLOT。
	(void)data;
	return true;
}

int consumer_model_transform_point_cloud(const void *raw, void *dest)
{
	// 返回 0 让流水线直接发送输入，保留 FPGA 包头、像素顺序及所有保留字节。
	(void)raw;
	(void)dest;
	return 0;
}

int consumer_model_parse_point_cloud(const void *raw, void *dest)
{
	return deal_orgin_point_cloud_data((raw_protocal_data_packet_t *)raw, (point_cloud_packet_t *)dest);
}
