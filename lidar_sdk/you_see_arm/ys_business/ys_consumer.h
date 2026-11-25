/*
 * @Author       : kaihong.yang kaihong.yang@vitalchem.com
 * @Date         : 2025-02-18 08:02:52
 * @LastEditors  : kaihong.yang kaihong.yang@vitalchem.com
 * @LastEditTime : 2025-02-19 02:04:35
 * @FilePath     : /you_see_arm/ys_business/ys_consumer.h
 * @Description  :
 * Copyright (c) 2025 by Vitalchem, All Rights Reserved.
 */

#ifndef YS_CONSUMER_H
#define YS_CONSUMER_H

#include "ys_def.h"

/**
 * @description  : 初始化参数
 * @param         {int} lines_v 垂直分辨率
 * @param         {int} echo_mode 回波模式：单回波/双回波
 * @return        {int} result
 * 0: success
 * other: fail
 */
int init_consumer_parm(int lines_v, int echo_mode);

/**
 * @description  : 消费者线程，从缓存中读取数据，经过处理(可选)后，发送到上位机
 * @param         {void} *arg
 * @return        {*}
 */
void *phtread_consumer(void *arg);

#endif // YS_CONSUMER_H
