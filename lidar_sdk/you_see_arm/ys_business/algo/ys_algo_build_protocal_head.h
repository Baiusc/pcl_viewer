/*
 * @Author       : kaihong.yang kaihong.yang@vitalchem.com
 * @Date         : 2026-03-10 16:54:52
 * @LastEditors  : kaihong.yang kaihong.yang@vitalchem.com
 * @LastEditTime : 2026-03-10 16:54:52
 * @FilePath     : /you_see_arm/ys_business/algo/ys_algo_build_protocal_head.h
 * @Description  :
 * Copyright (c) 2026 by Vitalchem, All Rights Reserved.
 */

#ifndef YS_ALGO_BUILD_PROTOCAL_HEAD_H
#define YS_ALGO_BUILD_PROTOCAL_HEAD_H

#include "../ys_def.h"

#if CS96_ALPHA == 1
/**
 * @description  : 创建点云协议头
 * @param         {raw_protocal_data_packet_t} raw_data 原始数据包
 * @param         {point_cloud_head_t} p_head 当前点云协议头的存储结构体
 * @return        {int} 创建的协议头长度
 *      -1 : fail
 *      other : success
 */
int build_protocal_head(raw_protocal_data_packet_t *raw_data, point_cloud_head_t *p_head);
#elif CS96_BETA == 1 || CS96_GAMMA == 1
/**
 * @description  : 创建点云协议头
 * @param         {raw_protocal_data_packet_t} raw_data 原始数据包
 * @param         {point_cloud_head_t} p_head 当前点云协议头的存储结构体
 * @return        {int} 创建的协议头长度
 *      -1 : fail
 *      other : success
 */
int build_protocal_head(raw_protocal_data_packet_t *raw_data, point_cloud_head_t *p_head);
#endif

#endif // YS_ALGO_BUILD_PROTOCAL_HEAD_H
