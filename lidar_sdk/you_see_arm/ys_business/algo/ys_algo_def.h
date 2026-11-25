/*
 * @Author       : kaihong.yang kaihong.yang@vitalchem.com
 * @Date         : 2026-03-10 16:54:52
 * @LastEditors  : kaihong.yang kaihong.yang@vitalchem.com
 * @LastEditTime : 2026-03-10 16:54:52
 * @FilePath     : /you_see_arm/ys_business/algo/ys_algo_def.h
 * @Description  : 用于定义一些多处算法都要使用的变量
 * Copyright (c) 2026 by Vitalchem, All Rights Reserved.
 */

#ifndef YS_ALGO_DEF_H
#define YS_ALGO_DEF_H

#include "../ys_def.h"

// 最大编码值
#if CS96_ALPHA == 1
#define MAX_ENCODER 65536
#elif CS96_BETA == 1 || CS96_GAMMA == 1
#define MAX_ENCODER 61409
#endif

#if CS96_ALPHA == 1
static uint16_t g_omega0[4];
static uint16_t g_standard_encoder[4] = { 0xBB8, 0x4BB8, 0x8BB8, 0xCBB8 }; // 四面镜子对应的标准编码器值
#endif

#if CS96_BETA == 1 || CS96_GAMMA == 1
extern float g_staring_angle[4]; // 每一面的起始角度，相差 90°
#endif

#endif // YS_ALGO_DEF_H
