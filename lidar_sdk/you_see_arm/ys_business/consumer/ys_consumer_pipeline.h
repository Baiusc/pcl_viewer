/*
 * @Author       : kaihong.yang kaihong.yang@vitalchem.com
 * @Date         : 2025-06-24
 * @FilePath     : /you_see_arm/ys_business/consumer/ys_consumer_pipeline.h
 * @Description  : 消费者公共流水线接口
 * Copyright (c) 2025 by Vitalchem, All Rights Reserved.
 */

#ifndef YS_CONSUMER_PIPELINE_H
#define YS_CONSUMER_PIPELINE_H

#include "../ys_def.h"

#include <signal.h>

/**
 * @description  : 初始化消费者流水线参数（分配输出缓冲区）
 * @param         {int} lines_v 垂直分辨率
 * @param         {int} echo_mode 回波模式
 * @return        {int} RET_OK / RET_FAIL
 */
int init_consumer_pipeline_parm(int lines_v, int echo_mode);

/**
 * @description  : 消费者信号处理函数
 * @param         {int} signum
 */
void consumer_pipeline_sighandler(int signum);

/**
 * @description  : 重置运行标志位（每次消费者线程启动时调用）
 */
void consumer_pipeline_reset_run_flag(void);

/**
 * @description  : 原始直方图数据消费流水线
 * @param         {udp_t} *udp
 */
void consumer_pipeline_raw_histogram(udp_t *udp);

/**
 * @description  : 原始点云数据消费流水线
 * @param         {udp_t} *udp
 */
void consumer_pipeline_raw_point_cloud(udp_t *udp);

/**
 * @description  : 灰度图数据消费流水线（原样透传，不做解码）
 * @param         {udp_t} *udp
 */
void consumer_pipeline_raw_gray(udp_t *udp);

/**
 * @description  : 处理后点云数据消费流水线
 * @param         {udp_t} *udp
 */
void consumer_pipeline_real_point_cloud(udp_t *udp);

/**
 * @description  : 清理消费者流水线资源
 */
void consumer_pipeline_cleanup(void);

#endif // YS_CONSUMER_PIPELINE_H
