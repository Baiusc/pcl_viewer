/*
 * @Author       : kaihong.yang kaihong.yang@vitalchem.com
 * @Date         : 2026-06-30
 * @FilePath     : /you_see_arm/ys_business/decode/ys_decode_mf150.h
 * @Description  : MF150 专有解码接口（仅 MF150 机型编译，独立于 ys_decode.h 避免类型污染）
 * Copyright (c) 2026 by Vitalchem, All Rights Reserved.
 */

#ifndef YS_DECODE_MF150_FL6031_H
#define YS_DECODE_MF150_FL6031_H

#include "../ys_def.h"

#include <stdint.h>
#include <stdbool.h>

int mf150_fl6031_parse_group_to_points(mf150_fl6031_raw_group_packet_t *raw_group,
				       mf150_fl6031_point_cloud_packet_t *dest_packet);
bool mf150_fl6031_orgin_point_cloud_data_is_valid(mf150_fl6031_raw_group_packet_t *raw_data);

#endif // YS_DECODE_MF150_FL6031_H
