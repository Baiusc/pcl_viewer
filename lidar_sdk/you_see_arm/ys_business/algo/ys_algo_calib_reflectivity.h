/*
 * @Author       : kaihong.yang kaihong.yang@vitalchem.com
 * @Date         : 2024-12-16 06:54:52
 * @LastEditors  : kaihong.yang kaihong.yang@vitalchem.com
 * @LastEditTime : 2025-09-08 01:25:22
 * @FilePath     : /you_see_arm/ys_business/algo/ys_algo_calib_reflectivity.h
 * @Description  :
 * Copyright (c) 2024 by Vitalchem, All Rights Reserved.
 */

#ifndef YS_ALGO_CALIB_REFLECTIVITY_H
#define YS_ALGO_CALIB_REFLECTIVITY_H

#include "../ys_def.h"

#include <stdint.h>

#if LOCAL_TEST == 1
#define REFLECTIVITY_MAP_PATH "./file/reflectivity_map.bin"
#else
#define REFLECTIVITY_MAP_PATH "/etc/reflectivity_map.bin"
#endif

/**
 * @description  : 初始化反射率算法接口
 * @param         {uint16_t} cal_max_peak
 * @return        {int} result
 *      0 : success
 *      other : fail
 */
int init_algo_reflectivity(uint16_t cal_max_peak);

/**
 * @description  : 反初始化反射率算法接口
 * @return        {int} result
 *      0 : success
 *      other : fail
 */
int uninit_algo_reflectivity();

/**
 * @description  : 计算单点反射率
 * @param         {uint16_t} pixel_idx
 * @param         {uint16_t} slot_idx
 * @param         {float} r
 * @param         {uint16_t} peak
 * @param         {float} intensity_coefficient
 * @return        {uint8_t} intensity
 */
uint8_t get_reflect(uint16_t pixel_idx, uint16_t slot_idx, float r, uint16_t peak, float intensity_coefficient);

#endif // YS_ALGO_CALIB_REFLECTIVITY_H
