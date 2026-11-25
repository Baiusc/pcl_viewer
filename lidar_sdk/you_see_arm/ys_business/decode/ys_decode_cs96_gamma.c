/*
 * @Author       : kaihong.yang kaihong.yang@vitalchem.com
 * @Date         : 2026-10-09
 * @FilePath     : /you_see_arm/ys_business/decode/ys_decode_cs96_gamma.c
 * @Description  : CS96 Gamma 专有解码实现（SL100 SPAD）
 * Copyright (c) 2026 by Vitalchem, All Rights Reserved.
 */

#include "ys_decode.h"
#include "ys_decode_internal.h"

#include "../algo/ys_algo_build_protocal_head.h"
#include "../algo/ys_algo_def.h"
#include "../algo/ys_algo_frame_id.h"
#include "../algo/ys_algo_horizontal_corners.h"

#include "../../common/common.h"
#include "../../core/ys_base.h"
#include "../../core/ys_conf.h"
#include "../../core/ys_core_def.h"

void decode_calculate_theta(void)
{
	float theta = 0.0f;
	float step = (cal_param.theta_max.f - cal_param.theta_min.f) / cal_param.lines_v;
	for (int i = cal_param.lines_v - 1, j = 0; i >= 0; i--, j++) {
		theta = (i - cal_param.u) * step + cal_param.theta_tilt.f;
		g_theta[j] = theta * k_radian;
	}
}

void decode_init_model_encoder(void)
{
	log_info("[CS96_Gamma] standard_encoder_cnt=%d, encoder_angle_cnt=%d", cal_param.standard_encoder_cnt,
		 g_encoder_angle_cnt);

	for (int i = 0; i < cal_param.standard_encoder_cnt; i++) {
		g_staring_angle[i] =
			(float)(cal_param.angle_face[i] * 256 + cal_param.side_angular_offset[i]) / 256 * 360 / 240;
		log_info("[CS96_Gamma] face[%d] angle_face=%d side_offset=%d → starting_angle=%.4f", i,
			 cal_param.angle_face[i], cal_param.side_angular_offset[i], g_staring_angle[i]);
	}
}

int decode_build_protocal_head(raw_protocal_data_packet_t *raw_data, point_cloud_head_t *p_head)
{
	return build_protocal_head(raw_data, p_head);
}

float decode_get_horizontal_corners(point_cloud_head_t *p_head, org_point_cloud_packet_t *org_data)
{
	return get_horizontal_corners(p_head->slot_number, p_head->frame_index, cs96_gamma_get_mirror_angle(org_data),
				      0);
}

bool orgin_point_cloud_data_is_valid(raw_protocal_data_packet_t *raw_data)
{
	org_point_cloud_packet_t *org_data = (org_point_cloud_packet_t *)(raw_data->value);
	// frame_index 从 FPGA 传来的 fpga_frame_id 取模得到
	int now_frame_index = cs96_gamma_get_fpga_frame_id(org_data) % 4;
	return (cal_param.side_light >> now_frame_index) & 1;
}
