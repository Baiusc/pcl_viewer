/*
 * @Author       : kaihong.yang kaihong.yang@vitalchem.com
 * @Date         : 2024-12-16 06:55:58
 * @LastEditors  : kaihong.yang kaihong.yang@vitalchem.com
 * @LastEditTime : 2025-09-08 01:24:18
 * @FilePath     : /you_see_arm/ys_business/algo/ys_algo_calib_reflectivity.c
 * @Description  :
 * Copyright (c) 2024 by Vitalchem, All Rights Reserved.
 */

#include "ys_algo_calib_reflectivity.h"

#if YS_REFLECTIVITY
#include "../../algo/calib_reflectivity/include/alog_reflect_interface.h"
#endif

#include <stdlib.h>
#include "../../core/ys_conf.h"

// 定义峰值的最小和最大有效值
#define MIN_PEAK 51
static uint16_t max_peak;

#if YS_REFLECTIVITY
static CalibReflectHandle handle = NULL;
static ReflectPt point = { 0 };
#endif

int init_algo_reflectivity(uint16_t cal_max_peak)
{
	max_peak = cal_max_peak;
	if (!config.enable_reflectivity)
		return RET_OK;
#if YS_REFLECTIVITY
	if (access(REFLECTIVITY_MAP_PATH, F_OK)) {
		log_debug("No find %s, Use the built-in reflectance algorithm", REFLECTIVITY_MAP_PATH);
		return RET_OK;
	} else {
		log_debug("Find %s, Use the reflectivity algorithm library", REFLECTIVITY_MAP_PATH);
	}

	if (handle == NULL) {
		handle = CalibReflect_Create();
	} else {
		return RET_OK;
	}

	if (handle == NULL) {
		return RET_FAIL;
	}

	int ret = CalibReflect_LoadMapFromBinary(handle, REFLECTIVITY_MAP_PATH);

	return ret;
#else
	return RET_OK;
#endif
}

int uninit_algo_reflectivity()
{
#if YS_REFLECTIVITY
	if (handle != NULL) {
		CalibReflect_Destroy(handle);
		handle = NULL;
	}

#endif
	return RET_OK;
}

/**
 * @description  : 计算反射率 0~1.0
 * @param         {uint16_t} peak
 * @return        {float} reflect
 */
static float get_reflect_by_peak(uint16_t peak)
{
	// 若峰值小于最小有效值，返回 0.0
	if (peak <= MIN_PEAK) {
		return 0.f;
	}
	// 若峰值大于最大有效值，返回 1.0
	if (peak >= max_peak) {
		return 1.f;
	}
	// 若峰值在有效范围内，进行归一化，求得反射率
	return (float)(peak - MIN_PEAK) / (float)(max_peak - MIN_PEAK);
}

/**
 * @description  : 计算强度值 0~255
 * @param         {uint16_t} peak
 * @return        {uint8_t} peak
 */
static uint8_t get_intensity_by_peak(uint16_t peak)
{
	// 求出反射率
	float reflect = get_reflect_by_peak(peak);
	// 将反射率映射到 0-255 范围，并转换为 uint8_t
	return (uint8_t)(reflect * 255);
}

uint8_t get_reflect(uint16_t pixel_idx, uint16_t slot_idx, float r, uint16_t peak, float intensity_coefficient)
{
	(void)intensity_coefficient;
#if YS_REFLECTIVITY
	if (handle != NULL) {
		point.pixel_idx = pixel_idx;
		point.slot_idx = slot_idx;
		point.r = r;
		point.peak = peak;
		CalibReflect_CalcIntensity(handle, &point);
		return point.intensity;
	}
#else
	(void)pixel_idx;
	(void)slot_idx;
	(void)r;
#endif
	return get_intensity_by_peak(peak);
}
