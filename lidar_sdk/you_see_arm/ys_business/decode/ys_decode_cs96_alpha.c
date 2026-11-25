/*
 * @Author       : kaihong.yang kaihong.yang@vitalchem.com
 * @Date         : 2026-06-30
 * @FilePath     : /you_see_arm/ys_business/decode/ys_decode_cs96_alpha.c
 * @Description  : CS96 Alpha 专有解码实现
 * Copyright (c) 2026 by Vitalchem, All Rights Reserved.
 */

#include "ys_decode.h"
#include "ys_decode_internal.h"

#include "../algo/ys_algo_build_protocal_head.h"
#include "../algo/ys_algo_def.h"
#include "../algo/ys_algo_frame_id.h"
#include "../algo/ys_algo_horizontal_corners.h"
#include "../algo/ys_algo_spot_slope.h"

#include "../../common/common.h"
#include "../../core/ys_base.h"
#include "../../core/ys_conf.h"
#include "../../core/ys_core_def.h"

void decode_calculate_theta(void)
{
	float theta = 0.0f;
	for (int i = 0, j = 0; i < cal_param.lines_v; i++, j++) {
		theta = ((i - cal_param.u) * (cal_param.theta_max.f - cal_param.theta_min.f) / cal_param.lines_v);
		theta += cal_param.theta_tilt.f;
		g_theta[j] = theta * k_radian;
	}
}

void decode_init_model_encoder(void)
{
	if (cal_param.side_angular_offset_cnt < sizeof(g_standard_encoder) / sizeof(g_standard_encoder[0])) {
		log_fatal("missing calibration parameters:[side_angular_offset]");
		return;
	} else if (cal_param.side_angular_offset == NULL) {
		log_fatal("missing calibration parameters:[side_angular_offset]");
		return;
	}

	if (cal_param.standard_encoder_cnt < sizeof(g_standard_encoder) / sizeof(g_standard_encoder[0])) {
		log_fatal("missing calibration parameters:[standard_encoder]");
		return;
	} else if (cal_param.angle_face == NULL) {
		log_fatal("missing calibration parameters:[standard_encoder]");
		return;
	}

	for (int i = 0; i < cal_param.standard_encoder_cnt; i++) {
		if (cal_param.angle_face[i] != 0) {
			g_standard_encoder[i] = cal_param.angle_face[i];
		}
		g_standard_encoder[i] += cal_param.side_angular_offset[i];
		log_debug("standard_encoder[%d] = %d", i, g_standard_encoder[i]);

		g_omega0[i] = get_mid_slot_code(cal_param.angle_face[i], cal_param.side_angular_offset[i]);
		log_debug("omega0[%d] = %d", i, g_omega0[i]);
	}
}

int decode_build_protocal_head(raw_protocal_data_packet_t *raw_data, point_cloud_head_t *p_head)
{
	return build_protocal_head(raw_data, p_head);
}

float decode_get_horizontal_corners(point_cloud_head_t *p_head, org_point_cloud_packet_t *org_data)
{
	return get_horizontal_corners(p_head->slot_number, p_head->frame_index, org_data->code_disc_angle);
}

bool orgin_point_cloud_data_is_valid(raw_protocal_data_packet_t *raw_data)
{
	org_point_cloud_packet_t *org_data = (org_point_cloud_packet_t *)(raw_data->value);
	int now_frame_index = get_frame_id(org_data->slot_number, org_data->code_disc_angle);
	return (cal_param.side_light >> now_frame_index) & 1;
}
