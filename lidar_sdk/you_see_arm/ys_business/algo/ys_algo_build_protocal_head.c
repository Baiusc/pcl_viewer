/*
 * @Author       : kaihong.yang kaihong.yang@vitalchem.com
 * @Date         : 2026-03-10 16:54:52
 * @LastEditors  : kaihong.yang kaihong.yang@vitalchem.com
 * @LastEditTime : 2026-03-10 16:54:52
 * @FilePath     : /you_see_arm/ys_business/algo/ys_algo_build_protocal_head.c
 * @Description  :
 * Copyright (c) 2026 by Vitalchem, All Rights Reserved.
 */

#include "ys_algo_build_protocal_head.h"
#include "ys_algo_frame_id.h"

#include "../../core/ys_conf.h"

#if CS96_ALPHA == 1
int build_protocal_head(raw_protocal_data_packet_t *raw_data, point_cloud_head_t *p_head)
{
	if (p_head == NULL) {
		log_error("p head is NULL");
		return RET_FAIL;
	}

	org_point_cloud_packet_t *org_data = (org_point_cloud_packet_t *)(raw_data->value);
	if (org_data == NULL) {
		log_error("org data is NULL");
		return RET_FAIL;
	}

	memcpy(p_head->delay, org_data->delay, sizeof(p_head->delay)); // 时间戳，精确到 us
	p_head->total_slots = total_slots; // 总的 slot 数
	p_head->slot_number = org_data->slot_number; // 当前 slot 号
	p_head->echo_mode = (uint8_t)(config.echo_mode); // 回波模式
	p_head->coordinate_mode = (uint8_t)(config.coordinate_mode); // 坐标模式
	p_head->frame_index = get_frame_id(org_data->slot_number, org_data->code_disc_angle); // 面 ID

	return sizeof(point_cloud_head_t);
}
#elif CS96_BETA == 1
int build_protocal_head(raw_protocal_data_packet_t *raw_data, point_cloud_head_t *p_head)
{
	if (p_head == NULL) {
		log_error("p head is NULL");
		return RET_FAIL;
	}

	org_point_cloud_packet_t *org_data = (org_point_cloud_packet_t *)(raw_data->value);
	if (org_data == NULL) {
		log_error("org data is NULL");
		return RET_FAIL;
	}

	p_head->total_slots = total_slots;
	p_head->slot_number = cs96_beta_get_slot_number(org_data);
	p_head->echo_mode = (uint8_t)(config.echo_mode);
	p_head->coordinate_mode = (uint8_t)(config.coordinate_mode);
	// 时间戳: 从header offset 拷贝4~13字节
	memcpy(p_head->delay, cs96_beta_get_delay(org_data), sizeof(p_head->delay));
	// frame_index 从 FPGA 传来的 fpga_frame_id 取模得到，替代原来通过 scale 判断的方式
	p_head->frame_index = cs96_beta_get_fpga_frame_id(org_data) % 4;

	return sizeof(point_cloud_head_t);
}
#elif CS96_GAMMA == 1
int build_protocal_head(raw_protocal_data_packet_t *raw_data, point_cloud_head_t *p_head)
{
	if (p_head == NULL) {
		log_error("p head is NULL");
		return RET_FAIL;
	}

	org_point_cloud_packet_t *org_data = (org_point_cloud_packet_t *)(raw_data->value);
	if (org_data == NULL) {
		log_error("org data is NULL");
		return RET_FAIL;
	}

	p_head->total_slots = total_slots;
	p_head->slot_number = cs96_gamma_get_slot_number(org_data);
	p_head->echo_mode = (uint8_t)(config.echo_mode);
	p_head->coordinate_mode = (uint8_t)(config.coordinate_mode);
	// 时间戳: SL100 只提供 24 位相对时间(单位 us), 按小端填入 delay 低 3 字节, 高位补 0
	uint32_t relative_time = cs96_gamma_get_relative_time(org_data);
	memset(p_head->delay, 0, sizeof(p_head->delay));
	p_head->delay[0] = (uint8_t)(relative_time & 0xFF);
	p_head->delay[1] = (uint8_t)((relative_time >> 8) & 0xFF);
	p_head->delay[2] = (uint8_t)((relative_time >> 16) & 0xFF);
	// frame_index 从 FPGA 传来的 fpga_frame_id 取模得到
	p_head->frame_index = cs96_gamma_get_fpga_frame_id(org_data) % 4;

	return sizeof(point_cloud_head_t);
}
#endif
