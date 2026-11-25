/*
 * @Author       : kaihong.yang kaihong.yang@vitalchem.com
 * @Date         : 2026-04-08 22:59:50
 * @LastEditors  : kaihong.yang kaihong.yang@vitalchem.com
 * @LastEditTime : 2026-04-08 23:58:37
 * @FilePath     : /you_see_arm/ys_business/producer/histogram/ys_producer_histogram_mf150.h
 * @Description  : 直方图生产者
 * Copyright (c) 2025 by kaihong.yang, email: kaihong.yang@vitalchem.com, All Rights Reserved.
 */
#ifndef YS_PRODUCER_HISTOGRAM_MF150_FL6031_H
#define YS_PRODUCER_HISTOGRAM_MF150_FL6031_H

#include "../../ys_def.h"

/**
 * @description  : 退出直方图生产者线程
 * @return        {}
 */
void producer_exit_histogram();

/**
 * @description  : 启动直方图生产者线程，从驱动或者文件中读取原始直方图数据
 * @param         {udp_t} *udp_instance
 * @param         {uint8_t} *data_buf
 * @return        {}
 */
void producer_get_histogram(udp_t *udp_instance);

#endif // YS_PRODUCER_HISTOGRAM_MF150_FL6031_H
