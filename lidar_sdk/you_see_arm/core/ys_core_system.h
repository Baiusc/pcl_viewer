/*
 * @Author       : yangkh kaihong.yang@vitalchem.com
 * @Date         : 2024-05-28 23:52:57
 * @LastEditors  : kaihong.yang kaihong.yang@vitalchem.com
 * @LastEditTime : 2025-07-08 18:51:11
 * @FilePath     : /you_see_arm/core/ys_core_system.h
 * @Description  :
 * Copyright (c) 2024 by Vitalchem, All Rights Reserved.
 */

#ifndef YS_CORE_SYSTEM_H
#define YS_CORE_SYSTEM_H

#include "../common/common.h"

#include "../third_party/kfifo/kfifo.h"

/**
 * @description  : Check if the program is already running,and if so,exit the old one
 * @return        {int} result
 *      0 : success
 *      other : fail
 */
int check_pid();

/**
 * @description  : system initialzation(api init)
 * @return        {int} result
 *      0 : success
 *      other : fail
 */
int sys_init();

/**
 * @description  : system uninitialzation(api exit)
 * @return        {int} result
 *      0 : success
 *      other : fail
 */
int sys_uninit();

/**
 * @description  : system spad initialzation
 * @param         {int} work_mode
 * @return        {int} result
 *      0 : success
 *      other : fail
 */
int sys_spad_init(int work_mode);

/**
 * @description  : 驱动(lidar_api)是否已成功初始化
 *                 运行期访问驱动的接口(如版本查询)必须先确认此标志: driver 的
 *                 lidar_version_get() 会解引用共享内存参数区, 未初始化时调用会段错误
 * @return        {int} 1 = 已初始化; 0 = 未初始化
 */
int sys_api_ready();

#if USE_KFIFO
extern struct kfifo g_fifobuf; // 全局的 kfifo 队列，用于存储点云数据
#endif

extern const int org_histogram_pixel_size; // 原始直方图数据中，一个像素点的大小
extern int org_histogram_slot_size; // 原始直方图数据中，一个 slot 的大小
extern int raw_histogram_full_size; // 协议中的原始直方图报文长度

extern int org_point_cloud_slot_size; // 原始点云数据中，一个 slot 的大小
extern int raw_point_cloud_full_size; // 协议中的原始点云报文长度

// 灰度图数据格式常量
#define GRAY_SLOT_HEADER_SIZE 8 // 每个 SLOT 的包头字节数
#if CS96_BETA == 1
// CS96-beta 的 MODE_GRY 布局使用 8 字节 SLOT 头，槽号 [15:0] 有效，取值 0~167。
// 每像素 4B: 灰度值[21:0] + 保留[31:22]
#define GRAY_PIXEL_SIZE 4 // 每个像素的字节数: 灰度值[21:0] + 保留[31:22]
#define GRAY_SLOT_PIXELS 576 // 每个 SLOT 的垂直像素数(与驱动 LIDAR_TRANSIZE=2312 一致, 非点云 lines_v)
#define GRAY_SLOT_SLICES 2 // 一个 SLOT 拆成几个 UDP 包发送(2312B 超过 MTU, 对半拆两片)
#else
// CS96-gamma 的 MODE_GRY_SL100 布局使用 8 字节 SLOT 头，槽号 [4:0] 有效。
// 每像素 8B: ECHO1 灰度[19:0] + ECHO2 灰度[39:20] + 保留[63:40]
#define GRAY_PIXEL_SIZE 8 // 每个像素的字节数: ECHO1 灰度[19:0] + ECHO2 灰度[39:20] + 保留[63:40]
#endif
extern int org_gray_slot_size; // 原始灰度图数据中，一个 slot 的大小
extern int raw_gray_full_size; // 协议中一个灰度图 UDP 包(内存池节点)的长度

#if CS96_ALPHA == 1
#define SLICES_HISTOGRAM_LEN (org_histogram_pixel_size * 2) // 通信协议中，一次发送两个大点

#elif CS96_BETA == 1 || CS96_GAMMA == 1
#define SLICES_HISTOGRAM_LEN (org_histogram_pixel_size * 4) // 通信协议中，一次发送四个大点

#else
#define SLICES_HISTOGRAM_LEN (org_histogram_pixel_size * 2) // 通信协议中，一次发送两个大点

#endif

#endif // YS_CORE_SYSTEM_H
