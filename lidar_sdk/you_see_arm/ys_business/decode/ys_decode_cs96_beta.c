/*
 * @Author       : kaihong.yang kaihong.yang@vitalchem.com
 * @Date         : 2026-06-30
 * @FilePath     : /you_see_arm/ys_business/decode/ys_decode_cs96_beta.c
 * @Description  : CS96 Beta 专有解码实现
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

#include <stdatomic.h>
#include <time.h>

#define CS96_BETA_FACE_COUNT 4
#define CS96_BETA_PA_COUNTS_PER_REVOLUTION 240.0f
#define CS96_BETA_PA_RANGE_COUNTS 40.0f
#define CS96_BETA_PA_TOLERANCE_COUNTS 5.0f
#define CS96_BETA_FRAME_ID_STATS_INTERVAL 32768U

static bool g_beta_pa_frame_id_ready;

static atomic_uint_fast32_t g_beta_frame_id_samples = ATOMIC_VAR_INIT(0);
static atomic_uint_fast32_t g_beta_frame_id_mismatches = ATOMIC_VAR_INIT(0);
static atomic_uint_fast32_t g_beta_frame_id_unresolved = ATOMIC_VAR_INIT(0);
static atomic_flag g_beta_frame_id_log_lock = ATOMIC_FLAG_INIT;
static time_t g_beta_frame_id_last_log_time;

/**
 * @description  : 按 PA 所在区间确定 beta 面号
 *                 20 位 mirror_angle 的 scale 是编码器刻度，real_angle 为刻度内角度；
 *                 转为角度后，与配置的各面起始角比较。每面区间约 40 个刻度，
 *                 前后各留 5 个刻度容差，以容纳电机转速和边界抖动。
 * @param         {org_point_cloud_packet_t} *org_data 原始点云 SLOT
 * @return        {int} 面号 0~3；无法可靠判断时返回 -1
 */
static int beta_get_pa_frame_index(org_point_cloud_packet_t *org_data)
{
	if (!g_beta_pa_frame_id_ready || org_data == NULL || g_encoder_angle == NULL || g_encoder_angle_cnt <= 0) {
		return -1;
	}

	uint32_t mirror_angle = cs96_beta_get_mirror_angle(org_data);
	uint32_t scale = (mirror_angle >> 12) & 0xFF;
	uint32_t real_angle = mirror_angle & 0xFFF;
	if (scale >= (uint32_t)g_encoder_angle_cnt) {
		return -1;
	}

	float pa_angle = g_encoder_angle[scale] + 0.0009f * (float)real_angle;
	while (pa_angle < 0.0f) {
		pa_angle += 360.0f;
	}
	while (pa_angle >= 360.0f) {
		pa_angle -= 360.0f;
	}

	const float degree_per_pa_count = 360.0f / CS96_BETA_PA_COUNTS_PER_REVOLUTION;
	const float min_delta = -CS96_BETA_PA_TOLERANCE_COUNTS * degree_per_pa_count;
	const float max_delta = (CS96_BETA_PA_RANGE_COUNTS + CS96_BETA_PA_TOLERANCE_COUNTS) * degree_per_pa_count;
	int matched_frame_index = -1;

	for (int i = 0; i < CS96_BETA_FACE_COUNT; i++) {
		float delta = pa_angle - g_staring_angle[i];
		while (delta > 180.0f) {
			delta -= 360.0f;
		}
		while (delta < -180.0f) {
			delta += 360.0f;
		}

		if (delta >= min_delta && delta <= max_delta) {
			// 配置异常导致区间重叠时不猜测面号，交由调用处保留 FPGA ID。
			if (matched_frame_index >= 0) {
				return -1;
			}
			matched_frame_index = i;
		}
	}

	return matched_frame_index;
}

/**
 * @description  : 使用 PA 面号；PA 不在唯一有效区间时回退到 FPGA 面号
 */
static int beta_resolve_frame_index(org_point_cloud_packet_t *org_data, bool *pa_valid)
{
	int fpga_frame_index = cs96_beta_get_fpga_frame_id(org_data) % CS96_BETA_FACE_COUNT;
	int pa_frame_index = beta_get_pa_frame_index(org_data);

	if (pa_valid != NULL) {
		*pa_valid = pa_frame_index >= 0;
	}

	return pa_frame_index >= 0 ? pa_frame_index : fpga_frame_index;
}

int cs96_beta_resolve_scan_face(org_point_cloud_packet_t *slot)
{
	return beta_resolve_frame_index(slot, NULL);
}

static void beta_record_frame_id_stats(uint8_t fpga_frame_id, int fpga_frame_index, int resolved_frame_index,
				       bool pa_valid)
{
	uint_fast32_t sample_count = atomic_fetch_add_explicit(&g_beta_frame_id_samples, 1, memory_order_relaxed) + 1;
	if (pa_valid && fpga_frame_index != resolved_frame_index) {
		atomic_fetch_add_explicit(&g_beta_frame_id_mismatches, 1, memory_order_relaxed);

		time_t now = time(NULL);
		if (!atomic_flag_test_and_set_explicit(&g_beta_frame_id_log_lock, memory_order_acquire)) {
			if (now != (time_t)-1 && now - g_beta_frame_id_last_log_time >= 5) {
				log_warn("[CS96_Beta] PA 纠正面号: fpga_frame_id=%u raw_face=%d pa_face=%d",
					 fpga_frame_id, fpga_frame_index, resolved_frame_index);
				g_beta_frame_id_last_log_time = now;
			}
			atomic_flag_clear_explicit(&g_beta_frame_id_log_lock, memory_order_release);
		}
	} else if (!pa_valid) {
		atomic_fetch_add_explicit(&g_beta_frame_id_unresolved, 1, memory_order_relaxed);
	}

	if (sample_count % CS96_BETA_FRAME_ID_STATS_INTERVAL == 0) {
		uint_fast32_t mismatches =
			atomic_exchange_explicit(&g_beta_frame_id_mismatches, 0, memory_order_relaxed);
		uint_fast32_t unresolved =
			atomic_exchange_explicit(&g_beta_frame_id_unresolved, 0, memory_order_relaxed);
		log_info("[CS96_Beta] 最近 %u 个 SLOT 的 PA/frame ID 校验: 纠正=%u, PA无法判定=%u",
			 CS96_BETA_FRAME_ID_STATS_INTERVAL, (unsigned int)mismatches, (unsigned int)unresolved);
	}
}

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
	log_info("[CS96_Beta] standard_encoder_cnt=%d, encoder_angle_cnt=%d", cal_param.standard_encoder_cnt,
		 g_encoder_angle_cnt);

	g_beta_pa_frame_id_ready = false;
	if (cal_param.standard_encoder_cnt < CS96_BETA_FACE_COUNT || cal_param.angle_face == NULL ||
	    cal_param.side_angular_offset_cnt < CS96_BETA_FACE_COUNT || cal_param.side_angular_offset == NULL) {
		log_warn("[CS96_Beta] 面号 PA 纠偏未启用: 缺少四面的起始编码器值或水平角偏移");
		return;
	}

	for (int i = 0; i < CS96_BETA_FACE_COUNT; i++) {
		g_staring_angle[i] = (float)(cal_param.angle_face[i] * 256 + cal_param.side_angular_offset[i]) / 256 *
				     360 / CS96_BETA_PA_COUNTS_PER_REVOLUTION;
		log_info("[CS96_Beta] face[%d] angle_face=%d side_offset=%d → starting_angle=%.4f", i,
			 cal_param.angle_face[i], cal_param.side_angular_offset[i], g_staring_angle[i]);
	}
	g_beta_pa_frame_id_ready = true;
}

int decode_build_protocal_head(raw_protocal_data_packet_t *raw_data, point_cloud_head_t *p_head)
{
	if (raw_data == NULL) {
		return RET_FAIL;
	}

	int len = build_protocal_head(raw_data, p_head);
	if (len > 0) {
		if (raw_data->seq_num & CS96_BETA_SCAN_FACE_VALID) {
			p_head->frame_index = (raw_data->seq_num >> CS96_BETA_SCAN_FACE_SHIFT) &
					      CS96_BETA_SCAN_FACE_MASK;
		} else {
			org_point_cloud_packet_t *org_data = (org_point_cloud_packet_t *)(raw_data->value);
			p_head->frame_index = cs96_beta_resolve_scan_face(org_data);
		}
	}
	return len;
}

float decode_get_horizontal_corners(point_cloud_head_t *p_head, org_point_cloud_packet_t *org_data)
{
	return get_horizontal_corners(p_head->slot_number, p_head->frame_index, cs96_beta_get_mirror_angle(org_data),
				      0);
}

bool orgin_point_cloud_data_is_valid(raw_protocal_data_packet_t *raw_data)
{
	if (raw_data == NULL) {
		return false;
	}

	org_point_cloud_packet_t *org_data = (org_point_cloud_packet_t *)(raw_data->value);
	uint8_t fpga_frame_id = cs96_beta_get_fpga_frame_id(org_data);
	int fpga_frame_index = fpga_frame_id % CS96_BETA_FACE_COUNT;
	bool pa_valid = false;
	int now_frame_index = beta_resolve_frame_index(org_data, &pa_valid);
	beta_record_frame_id_stats(fpga_frame_id, fpga_frame_index, now_frame_index, pa_valid);

	return (cal_param.side_light >> now_frame_index) & 1;
}
