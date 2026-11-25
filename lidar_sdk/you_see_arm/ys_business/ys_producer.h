/*
 * @Author       : kaihong.yang kaihong.yang@vitalchem.com
 * @Date         : 2024-11-13 01:59:57
 * @LastEditors  : kaihong.yang kaihong.yang@vitalchem.com
 * @LastEditTime : 2025-02-27 01:08:06
 * @FilePath     : /you_see_arm/ys_business/ys_producer.h
 * @Description  :
 * Copyright (c) 2024 by Vitalchem, All Rights Reserved.
 */

#ifndef YS_PRODUCER_H
#define YS_PRODUCER_H

/**
 * @description  : 初始化参数
 * @param         {int} lines_v 垂直分辨率
 * @return        {int} result
 * 0: success
 * other: fail
 */
int init_producer_parm(int lines_v);

/**
 * @description  : 生产者线程，从驱动中读取点云或直方图数据后缓存
 * @param         {void} *arg
 * @return        {*}
 */
void *phtread_producer(void *arg);

#endif // YS_PRODUCER_H
