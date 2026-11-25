/*
 * @Author       : kaihong.yang kaihong.yang@vitalchem.com
 * @Date         : 2026-04-08 23:08:47
 * @LastEditors  : kaihong.yang kaihong.yang@vitalchem.com
 * @LastEditTime : 2026-04-08 23:57:39
 * @FilePath     : /you_see_arm/ys_business/producer/point_cloud/ys_producer_point_cloud_cs96_beta.h
 * @Description  :
 * Copyright (c) 2026 by kaihong.yang, email: kaihong.yang@vitalchem.com, All Rights Reserved.
 */
#ifndef YS_PRODUCER_POINT_CLOUD_CS96_BETA_H
#define YS_PRODUCER_POINT_CLOUD_CS96_BETA_H

#include "../../ys_def.h"

/**
 * @description  : 退出点云生产者线程
 * @return        {}
 */
void producer_exit_point_cloud();

/**
 * @description  : 启动点云生产者线程，从驱动或者文件中读取原始点云数据
 * @param         {udp_t} *udp_instance
 * @param         {uint8_t} *data_buf
 * @return        {}
 */
void producer_get_point_cloud(udp_t *udp_instance, int driver_slot_num);

#endif // YS_PRODUCER_POINT_CLOUD_CS96_BETA_H
