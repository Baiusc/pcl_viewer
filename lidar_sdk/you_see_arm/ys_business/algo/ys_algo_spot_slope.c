/*
 * @Author       : kaihong.yang kaihong.yang@vitalchem.com
 * @Date         : 2025-05-16 02:11:43
 * @LastEditors  : kaihong.yang kaihong.yang@vitalchem.com
 * @LastEditTime : 2025-05-19 02:36:29
 * @FilePath     : /you_see_arm/ys_business/ys_algo_spot_slope.c
 * @Description  :
 * Copyright (c) 2025 by kaihong.yang, email: kaihong.yang@vitalchem.com, All Rights Reserved.
 */

#include "ys_algo_spot_slope.h"

#if CS96_BETA == 1 && BETA_SPOT_SLOPE == 1
#include "../../algo/spot_slope_alog/include/algo_calib_rotate_angle_interface.h"

#include <stdlib.h>
#include "../../core/ys_conf.h"

static CalibRotateAngleHandle handle = NULL;

static const uint16_t code_delay = 7; // 延后编码值，一般固定为 7（反应在光斑上就是其短半轴对应的旋转角）

int init_algo_spot_slope()
{
	if (!config.enable_spot_slope)
		return RET_OK;
#if CS96_BETA == 1
	log_info("Beta 光斑斜率修正编译开关: %d", BETA_SPOT_SLOPE);
#endif
	if (access(SPOT_SLOPE_MAP_PATH, F_OK)) {
		log_debug("No find %s, Use the default spot slope algorithm", SPOT_SLOPE_MAP_PATH);
		return RET_OK;
	} else {
		log_debug("Find %s, Use the spot slope algorithm library", SPOT_SLOPE_MAP_PATH);
	}

	if (handle == NULL) {
		handle = calibRotateAngle_Create();
	} else {
		return RET_OK;
	}

	if (handle == NULL) {
		return RET_FAIL;
	}

#if CS96_BETA == 1 && BETA_SPOT_SLOPE == 1
	int ret = calibRotateAngle_LoadMapByFace(handle, SPOT_SLOPE_MAP_PATH);
#else
	int ret = calibRotateAngle_LoadMap(handle, SPOT_SLOPE_MAP_PATH);
#endif

	if (ret) {
		return RET_OK;
	} else {
		calibRotateAngle_Destroy(handle);
		handle = NULL;
		return RET_FAIL;
	}
}

int uninit_algo_spot_slope()
{
	if (handle != NULL) {
		calibRotateAngle_Destroy(handle);
		handle = NULL;
	}

	return RET_OK;
}

uint16_t get_mid_slot_code(uint16_t code_start, uint16_t code_offset)
{
	// 当 handle 为空，说明没有 spot_slope_map.bin 文件
	if (handle == NULL) {
		// 此时创建一个临时的 handle，只是用来获取 mid_slot 的角度
		CalibRotateAngleHandle tmp_handle = calibRotateAngle_Create();

		if (tmp_handle == NULL) {
			return code_start + code_offset + code_delay + 136 * 40;
		}

		uint16_t code_mid = calibRotateAngle_GetMidSlotCode(tmp_handle, code_start, code_offset, code_delay);

		if (tmp_handle != NULL) {
			calibRotateAngle_Destroy(tmp_handle);
		}

		return code_mid;
	}

	return calibRotateAngle_GetMidSlotCode(handle, code_start, code_offset, code_delay);
}

float get_spot_slope(float angle_src, uint16_t code_src, uint8_t face_idx)
{
	if (handle == NULL) {
		return angle_src;
	}

	return calibRotateAngle_DoCalib(handle, angle_src, code_src, face_idx);
}

float get_beta_spot_slope(float angle_src, uint16_t face_idx)
{
	if (handle == NULL || face_idx >= 4) {
		return angle_src;
	}
	return calibRotateAngle_DoCalibByFace(handle, angle_src, (uint8_t)face_idx);
}

#else
/* 关闭可选库时保留基础几何换算，不读取光斑标定表。 */
int init_algo_spot_slope(void)
{
	return RET_OK;
}
int uninit_algo_spot_slope(void)
{
	return RET_OK;
}
uint16_t get_mid_slot_code(uint16_t code_start, uint16_t code_offset)
{
	return code_start + code_offset + 7 + 136 * 40;
}
float get_spot_slope(float angle_src, uint16_t code_src, uint8_t face_idx)
{
	(void)code_src;
	(void)face_idx;
	return angle_src;
}
float get_beta_spot_slope(float angle_src, uint16_t face_idx)
{
	(void)face_idx;
	return angle_src;
}
#endif
