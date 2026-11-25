/*
 * @Author       : kaihong.yang kaihong.yang@vitalchem.com
 * @Date         : 2026-06-30
 * @FilePath     : /you_see_arm/ys_business/decode/ys_decode_internal.h
 * @Description  : 解码层内部调度接口 —— ys_decode_common.c 通过此接口调用各机型专有实现
 * Copyright (c) 2026 by Vitalchem, All Rights Reserved.
 */

#ifndef YS_DECODE_INTERNAL_H
#define YS_DECODE_INTERNAL_H

#include "../ys_def.h"

extern float *g_theta;
extern float *g_omega_delta;
extern float *g_distance_factor;
extern float *g_delta_distance_index;
extern float *g_ld_intensity_coefficient;
extern const double k_radian;
extern const uint32_t k_error_ft;

void decode_calculate_theta(void);
void decode_init_model_encoder(void);
int decode_build_protocal_head(raw_protocal_data_packet_t *raw_data, point_cloud_head_t *p_head);
float decode_get_horizontal_corners(point_cloud_head_t *p_head, org_point_cloud_packet_t *org_data);

#endif // YS_DECODE_INTERNAL_H
