/*
 * @Author       : kaihong.yang kaihong.yang@vitalchem.com
 * @Date         : 2026-03-10 16:54:52
 * @LastEditors  : kaihong.yang kaihong.yang@vitalchem.com
 * @LastEditTime : 2026-03-10 16:54:52
 * @FilePath     : /you_see_arm/ys_business/algo/ys_algo_frame_id.c
 * @Description  :
 * Copyright (c) 2026 by Vitalchem, All Rights Reserved.
 */

#include "ys_algo_def.h"
#include "ys_algo_frame_id.h"

static uint16_t g_frame_index = 0; // 帧标识符

#if CS96_ALPHA == 1
static uint16_t g_pre_code_disc_angle = 0; // 上一个编码值
int get_frame_id(uint16_t slot_number, uint16_t code_disc_angle)
{
	int now_frame_index = -1;
#if 0
	// 单线
	if (slot_number == 0) {
		g_frame_index++;
	}
	now_frame_index = g_frame_index;
#else
	if (code_disc_angle >= g_standard_encoder[0] && code_disc_angle < g_standard_encoder[1]) {
		now_frame_index = 0;
	} else if (code_disc_angle >= g_standard_encoder[1] && code_disc_angle < g_standard_encoder[2]) {
		now_frame_index = 1;
	} else if (code_disc_angle >= g_standard_encoder[2] && code_disc_angle < g_standard_encoder[3]) {
		now_frame_index = 2;
	} else if (code_disc_angle >= g_standard_encoder[3] || code_disc_angle < g_standard_encoder[0]) {
		now_frame_index = 3;
	}

	uint16_t diff = 0;
	if (code_disc_angle > g_pre_code_disc_angle) {
		diff = code_disc_angle - g_pre_code_disc_angle;
	} else {
		diff = g_pre_code_disc_angle - code_disc_angle;
	}

	// 如果角度突变，认为是错误的值，延续上一个 slot 的帧 ID
	if (diff > 6000) {
		now_frame_index = g_frame_index;
	}

	// 由于 FPGA 提供的 slot0 编码值不准，slot0 的编码值采用自增的方式得到
	if (slot_number == 0) {
		now_frame_index = (g_frame_index + 1) % 4;
	}

	g_frame_index = now_frame_index;
	g_pre_code_disc_angle = code_disc_angle;

#endif
	return now_frame_index;
}
#elif CS96_BETA == 1 || CS96_GAMMA == 1
int get_frame_id(uint16_t slot_number, uint32_t mirror_angle)
{
	uint8_t scale = (uint8_t)((mirror_angle >> 12) & 0xFF);
	int now_frame_index = -1;

	// 4 个面起始发光刻度序号是 8，68，128，188
	if (9 <= scale && scale < 69) {
		now_frame_index = 0;
	} else if (69 <= scale && scale < 129) {
		now_frame_index = 1;
	} else if (129 <= scale && scale < 189) {
		now_frame_index = 2;
	} else if ((189 <= scale) || (scale < 9)) {
		now_frame_index = 3;
	} else {
		now_frame_index = g_frame_index;
	}

	if (slot_number == 0) {
		now_frame_index = (g_frame_index + 1) % 4;
	}

	g_frame_index = now_frame_index;
	return now_frame_index;
}

#endif
