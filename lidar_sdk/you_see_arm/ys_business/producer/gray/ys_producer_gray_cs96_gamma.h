/*
 * @Author       : kaihong.yang kaihong.yang@vitalchem.com
 * @Date         : 2026-10-12
 * @FilePath     : /you_see_arm/ys_business/producer/gray/ys_producer_gray_cs96_gamma.h
 * @Description  : CS96 Gamma 灰度图生产者（SL100 SPAD, 每 SLOT 120 像素, 968 字节）
 * Copyright (c) 2026 by Vitalchem, All Rights Reserved.
 */
#ifndef YS_PRODUCER_GRAY_CS96_GAMMA_H
#define YS_PRODUCER_GRAY_CS96_GAMMA_H

#include "../../ys_def.h"

/**
 * @description  : 退出灰度图生产者线程
 * @return        {}
 */
void producer_exit_gray();

/**
 * @description  : 启动灰度图生产者线程，从驱动或者文件中读取原始灰度图数据
 * @param         {udp_t} *udp_instance
 * @param         {int} driver_slot_num 从驱动中一次读取的 slot 个数
 * @return        {}
 */
void producer_get_gray(udp_t *udp_instance, int driver_slot_num);

#endif // YS_PRODUCER_GRAY_CS96_GAMMA_H
