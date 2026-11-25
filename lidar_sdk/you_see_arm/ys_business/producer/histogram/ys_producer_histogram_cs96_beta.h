/*
 * @Author       : kaihong.yang kaihong.yang@vitalchem.com
 * @Date         : 2026-04-08 22:51:10
 * @LastEditors  : kaihong.yang kaihong.yang@vitalchem.com
 * @LastEditTime : 2026-04-08 23:59:00
 * @FilePath     : /you_see_arm/ys_business/producer/histogram/ys_producer_histogram_cs96_beta.h
 * @Description  :
 * Copyright (c) 2026 by kaihong.yang, email: kaihong.yang@vitalchem.com, All Rights Reserved.
 */
#ifndef YS_PRODUCER_HISTOGRAM_CS96_BETA_H
#define YS_PRODUCER_HISTOGRAM_CS96_BETA_H

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

#endif // YS_PRODUCER_HISTOGRAM_CS96_BETA_H
