/*
 * @Author       : kaihong.yang kaihong.yang@vitalchem.com
 * @Date         : 2026-03-10 16:54:52
 * @LastEditors  : kaihong.yang kaihong.yang@vitalchem.com
 * @LastEditTime : 2026-03-10 16:54:52
 * @FilePath     : /you_see_arm/ys_business/algo/ys_algo_frame_id.h
 * @Description  :
 * Copyright (c) 2026 by Vitalchem, All Rights Reserved.
 */

#ifndef YS_ALGO_FRAME_ID_H
#define YS_ALGO_FRAME_ID_H

#include "../ys_def.h"

#if CS96_ALPHA == 1
/**
 * @description  : 帧 ID 计算
 * @param         {uint16_t} slot_number slot 号
 * @param         {uint16_t} code_disc_angle 编码值
 * @return        {int} 帧序号
 */
int get_frame_id(uint16_t slot_number, uint16_t code_disc_angle);
#elif CS96_BETA == 1 || CS96_GAMMA == 1
/**
 * @description  : 帧 ID 计算
 * @param         {uint16_t} slot_number slot 号
 * @param         {uint32_t} mirror_angle 转镜角度（20 位有效），高 8 位为 scale
 * @return        {int} 帧序号
 */
int get_frame_id(uint16_t slot_number, uint32_t mirror_angle);
#endif

#endif // YS_ALGO_FRAME_ID_H
