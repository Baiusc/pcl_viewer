/*
 * @Author       : kaihong.yang kaihong.yang@vitalchem.com
 * @Date         : 2026-06-30
 * @FilePath     : /you_see_arm/ys_business/decode/ys_decode.h
 * @Description  : 点云解码层统一接口 —— 向上对 consumer / config 层暴露，向下编排各 algo 子模块
 * Copyright (c) 2026 by Vitalchem, All Rights Reserved.
 */

#ifndef YS_DECODE_H
#define YS_DECODE_H

#include "../ys_def.h"

#include <stdint.h>
#include <stdbool.h>

int init_decode_paramter(void);
int uninit_decode_paramter(void);

int deal_orgin_point_cloud_data(raw_protocal_data_packet_t *raw_data, point_cloud_packet_t *dest_data);
int inverted_orgin_point_cloud_data(raw_protocal_data_packet_t *raw_data, uint8_t *dest_data);
bool orgin_point_cloud_data_is_valid(raw_protocal_data_packet_t *raw_data);

#if CS96_BETA == 1
/* 仅用于解算点云内部队列，低字节仍为批内序号，原始点云透传不设置。 */
#define CS96_BETA_SCAN_FACE_VALID 0x8000u
#define CS96_BETA_SCAN_FACE_SHIFT 8u
#define CS96_BETA_SCAN_FACE_MASK 0x3u
int cs96_beta_resolve_scan_face(org_point_cloud_packet_t *slot);
#endif

#endif // YS_DECODE_H
