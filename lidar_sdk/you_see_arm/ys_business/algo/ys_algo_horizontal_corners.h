/*
 * @Author       : kaihong.yang kaihong.yang@vitalchem.com
 * @Date         : 2026-03-10 16:54:52
 * @LastEditors  : kaihong.yang kaihong.yang@vitalchem.com
 * @LastEditTime : 2026-03-10 16:54:52
 * @FilePath     : /you_see_arm/ys_business/algo/ys_algo_horizontal_corners.h
 * @Description  :
 * Copyright (c) 2026 by Vitalchem, All Rights Reserved.
 */

#ifndef YS_ALGO_HORIZONTAL_CORNERS_H
#define YS_ALGO_HORIZONTAL_CORNERS_H

#include "../ys_def.h"

#if CS96_ALPHA == 1
/**
 * @description  : 水平角度计算
 * @param         {uint16_t} slot_number slot 号
 * @param         {uint16_t} now_frame_index 帧序号
 * @param         {uint16_t} code_disc_angle 编码值
 * @return        {float} 水平角
 */
float get_horizontal_corners(uint16_t slot_number, uint16_t now_frame_index, uint16_t code_disc_angle);
#elif CS96_BETA == 1 || CS96_GAMMA == 1
/**
 * @description  : 水平角度计算
 * @param         {uint16_t} slot_number slot 号
 * @param         {uint16_t} frame_id 帧序号，用来表示当前是第几面
 * @param         {uint32_t} mirror_angle 转镜角度（20 位有效）
 * @param         {uint32_t} relative_time 相对时间（24 位有效）
 * @return        {float} 水平角
 */
float get_horizontal_corners(uint16_t slot_number, uint16_t frame_id, uint32_t mirror_angle, uint32_t relative_time);
#endif

#endif // YS_ALGO_HORIZONTAL_CORNERS_H
