/*
 * @Author       : kaihong.yang kaihong.yang@vitalchem.com
 * @Date         : 2025-05-16 02:11:35
 * @LastEditors  : kaihong.yang kaihong.yang@vitalchem.com
 * @LastEditTime : 2025-05-18 23:51:31
 * @FilePath     : /you_see_arm/ys_business/ys_algo_spot_slope.h
 * @Description  :
 * Copyright (c) 2025 by kaihong.yang, email: kaihong.yang@vitalchem.com, All Rights Reserved.
 */

#ifndef YS_ALGO_SPOT_SLOPE_H
#define YS_ALGO_SPOT_SLOPE_H

#include "../ys_def.h"

#ifndef BETA_SPOT_SLOPE
#define BETA_SPOT_SLOPE 0
#endif

#if LOCAL_TEST == 1
#define SPOT_SLOPE_MAP_PATH "./file/spot_slope_map.bin"
#else
#define SPOT_SLOPE_MAP_PATH "/etc/spot_slope_map.bin"
#endif

/**
 * @description  : 初始化 水平转角修正算法接口
 * @return        {int} result
 *      0 : success
 *      other : fail
 */
int init_algo_spot_slope();

/**
 * @description  : 反初始化 水平转角修正算法接口
 * @return        {int} result
 *      0 : success
 *      other : fail
 */
int uninit_algo_spot_slope();

/**
 * @description  : 获取中间slot的编码值
 * @param         {uint16_t} code_start 第n面转镜的起始发光编码器值，每个面不同
 * @param         {uint16_t} code_offset 转镜零位角偏移量，每个面可能不同
 * @return        {uint16_t} code_mid
 */
uint16_t get_mid_slot_code(uint16_t code_start, uint16_t code_offset);

/**
 * @description  : 计算 修正后的水平转角
 * @param         {float} angle_src 原始角度
 * @param         {uint16_t} code_src 原始编码值
 * @param         {uint8_t} face_idx 面序号
 * @return        {float} angle_new 修正后的角度
 */
float get_spot_slope(float angle_src, uint16_t code_src, uint8_t face_idx);

// Beta 仅按面修正，不使用 Alpha 编码值回退。
float get_beta_spot_slope(float angle_src, uint16_t face_idx);

#endif // YS_ALGO_SPOT_SLOPE_H
