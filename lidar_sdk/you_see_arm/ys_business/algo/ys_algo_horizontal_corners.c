/*
 * @Author       : kaihong.yang kaihong.yang@vitalchem.com
 * @Date         : 2026-03-10 16:54:52
 * @LastEditors  : kaihong.yang kaihong.yang@vitalchem.com
 * @LastEditTime : 2026-03-10 16:54:52
 * @FilePath     : /you_see_arm/ys_business/algo/ys_algo_horizontal_corners.c
 * @Description  :
 * Copyright (c) 2026 by Vitalchem, All Rights Reserved.
 */

#include "ys_algo_def.h"
#include "ys_algo_horizontal_corners.h"
#include "ys_algo_spot_slope.h"

#include "../../core/ys_conf.h"

#if CS96_ALPHA == 1
static const double k_mirror_portion = (360.0 / MAX_ENCODER); // 转镜角度在两个字节中，每一个度数所占的份额

float get_horizontal_corners(uint16_t slot_number, uint16_t now_frame_index, uint16_t code_disc_angle)
{
	float horizontal_corner = 0.0f; // 转镜相对于零点的角度偏移量，取值范围 [-60,60]
#if 0
    static int g_init_frame = 0;
    static float g_slot0_omega = 0.0; // slot 号为 0 的转镜角度

	float slot_angel = code_disc_angle * k_mirror_portion; // 当前 slot 的角度，取值范围 [0, 360]

	/*
		注意点一：
		slot0 的角度是不准的，我们认为它是 0 度 也就是 cos(-60) 也就是 (slot_angel - slot0_omega) * 2 = 0
		所以 horizontal_corner 直接等于 0 就行
		slot1 我们认为是准的，所以以他为基准，cos(-59.85) 也就是 (slot_angel - slot0_omega) * 2 = 0.15
		所以 horizontal_corner = 0.15
		后面的角度就是 slot_angel - slot1 的角度
	*/

	switch (slot_number) {
	case 2 ... 799:
		/*
			注意点二：如果跨过了 360 度，这样计算就会得出负值，例如 slot0 = 355，sloti = 5，中间相差了 10 度
			这样算出来却是 5 - 355 = -350
			所以说：如果结果是负数，就要 + 360
		*/
		if (slot_angel > 0) {
			horizontal_corner = slot_angel - g_slot0_omega;
		} else {
			horizontal_corner = slot_number * 0.075;
		}

		if (horizontal_corner < 0) {
			horizontal_corner += 360.0f;
#if 1
			static uint16_t g_frame_index = 0; // 帧标识符
			if (g_init_frame == 0) {
				if (g_frame_index % 4 != 0) {
					g_frame_index = 0;
				}
				g_init_frame = 1;
			}
		} else {
			g_init_frame = 0;
#endif
		}
		horizontal_corner *= 2;
		break;
	case 0:
		horizontal_corner = 0.0f;
		break;
	case 1:
		g_slot0_omega = slot_angel; // 保存当前帧的起始转镜角度
		horizontal_corner = 0.15;
		break;
	default:
		horizontal_corner = slot_number * 0.15;
		break;
	}

	horizontal_corner += cal_param.omega0.f;
#else
	float slot_angel; // 当前 slot 的角度

	// 计算差值，带符号
	int16_t raw_diff = (int16_t)(code_disc_angle - g_omega0[now_frame_index]);

	// 解决零点跨越问题
	if (raw_diff > (MAX_ENCODER / 2)) {
		slot_angel = (float)(raw_diff - MAX_ENCODER) * k_mirror_portion * 2; // 正向跨越零点，减一个圆周
	} else if (raw_diff < -(MAX_ENCODER / 2)) {
		slot_angel = (float)(raw_diff + MAX_ENCODER) * k_mirror_portion * 2; // 负向跨越零点，加一个圆周
	} else {
		slot_angel = (float)raw_diff * k_mirror_portion * 2; // 一般情况，不存在零点跨越
	}

	// 调用水平角度修正算法
	if (slot_number != 0) {
		horizontal_corner = get_spot_slope(slot_angel, code_disc_angle, now_frame_index);
	} else {
		horizontal_corner = cal_param.omega0.f;
	}
#endif

#if 0
	// 单线
	horizontal_corner = slot_number * 0.15 + cal_param.omega0.f;
#endif

	return horizontal_corner;
}
#elif CS96_BETA == 1 || CS96_GAMMA == 1
float g_staring_angle[4] = { 0.0f, 0.0f, 0.0f, 0.0f }; // 每一面的起始角度，相差 90°

float get_horizontal_corners(uint16_t slot_number, uint16_t frame_id, uint32_t mirror_angle, uint32_t relative_time)
{
	(void)relative_time;
	float starting_angle = 0.0f;
	float horizontal_corner = 0.0f;
	float angle = 0.0f;

	// 按面聚合统计（slot0 为帧同步标记，此时打印上一面）
	static int s_stat_frame = -1;
	static int s_scale_min = 0, s_scale_max = 0;
	static float s_hc_min = 0.0f, s_hc_max = 0.0f;

	if (frame_id < sizeof(g_staring_angle) / sizeof(g_staring_angle[0])) {
		starting_angle = g_staring_angle[frame_id];
	}

	if (slot_number != 0) {
		int scale = (int)((mirror_angle >> 12) & 0xFF);
		int real_angle = (int)(mirror_angle & 0xFFF);
		angle = g_encoder_angle[scale] + 0.0009f * (float)real_angle;
		horizontal_corner = angle - starting_angle;
		if (horizontal_corner > 180.0f)
			horizontal_corner -= 360.0f;
		else if (horizontal_corner < -180.0f)
			horizontal_corner += 360.0f;

		if (s_stat_frame != (int)frame_id) {
			s_stat_frame = (int)frame_id;
			s_scale_min = s_scale_max = scale;
			s_hc_min = s_hc_max = horizontal_corner;
		} else {
			if (scale < s_scale_min)
				s_scale_min = scale;
			if (scale > s_scale_max)
				s_scale_max = scale;
			if (horizontal_corner < s_hc_min)
				s_hc_min = horizontal_corner;
			if (horizontal_corner > s_hc_max)
				s_hc_max = horizontal_corner;
		}
	} else {
		horizontal_corner = cal_param.omega0.f;

		if (s_stat_frame >= 0) {
			log_info("[FACE] frame_id=%d scale=[%d~%d] hc=[%.2f~%.2f] start=%.2f", s_stat_frame,
				 s_scale_min, s_scale_max, s_hc_min, s_hc_max, g_staring_angle[s_stat_frame]);
			s_stat_frame = -1;
		}
	}
	horizontal_corner = horizontal_corner * 2 + cal_param.omega0.f;

#if CS96_BETA == 1 && BETA_SPOT_SLOPE == 1
	// 输入为已经乘二、加零点偏移的水平角；同步 SLOT 保持原行为。
	if (slot_number != 0) {
		horizontal_corner = get_beta_spot_slope(horizontal_corner, frame_id);
	}
#endif

	return horizontal_corner;
}
#endif
