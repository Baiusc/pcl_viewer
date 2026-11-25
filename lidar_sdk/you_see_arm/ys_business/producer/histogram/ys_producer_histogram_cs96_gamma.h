/*
 * @Author       : kaihong.yang kaihong.yang@vitalchem.com
 * @Date         : 2026-10-09
 * @FilePath     : /you_see_arm/ys_business/producer/histogram/ys_producer_histogram_cs96_gamma.h
 * @Description  : CS96 Gamma 原始直方图生产者（SL100 SPAD，每 SLOT 一帧）
 * Copyright (c) 2026 by Vitalchem, All Rights Reserved.
 */
#ifndef YS_PRODUCER_HISTOGRAM_CS96_GAMMA_H
#define YS_PRODUCER_HISTOGRAM_CS96_GAMMA_H

#include "../../ys_def.h"

/**
 * @description  : 退出直方图生产者线程
 * @return        {}
 */
void producer_exit_histogram();

/**
 * @description  : 启动直方图生产者线程，从驱动或者文件中读取原始直方图数据
 * @param         {udp_t} *udp_instance
 * @return        {}
 */
void producer_get_histogram(udp_t *udp_instance);

#endif // YS_PRODUCER_HISTOGRAM_CS96_GAMMA_H
