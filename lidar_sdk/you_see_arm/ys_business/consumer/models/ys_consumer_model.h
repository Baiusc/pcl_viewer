/*
 * @Author       : kaihong.yang kaihong.yang@vitalchem.com
 * @Date         : 2025-06-24
 * @FilePath     : /you_see_arm/ys_business/consumer/models/ys_consumer_model.h
 * @Description  : 消费者模块机型抽象接口 - 消除消费者流水线中的机型宏依赖
 * Copyright (c) 2025 by Vitalchem, All Rights Reserved.
 */

#ifndef YS_CONSUMER_MODEL_H
#define YS_CONSUMER_MODEL_H

#include <stdbool.h>
#include <stdint.h>

/**
 * @description  : 计算处理后的点云输出缓冲区长度
 * @param         {int} lines_v 垂直分辨率
 * @param         {int} echo_mode 回波模式
 * @return        {int} 输出缓冲区所需字节数
 */
int consumer_model_get_processed_len(int lines_v, int echo_mode);

/**
 * @description  : 校验原始点云数据是否有效（无效直接丢弃）
 * @param         {const void} *data 原始数据指针
 * @return        {bool} true: 有效, false: 丢弃
 */
bool consumer_model_point_cloud_is_valid(const void *data);

/**
 * @description  : 变换原始点云数据（CS96_BETA 需反转 pixel 顺序）
 * @param         {const void} *raw 原始数据
 * @param         {void} *dest 目标缓冲区
 * @return        {int} 写入 dest 的字节数；返回 0 表示无需变换，直接用原始数据发送
 */
int consumer_model_transform_point_cloud(const void *raw, void *dest);

/**
 * @description  : 解析原始点云为处理后的点云（FORMAL 模式）
 * @param         {const void} *raw 原始数据
 * @param         {void} *dest 目标缓冲区
 * @return        {int} 写入 dest 的字节数
 */
int consumer_model_parse_point_cloud(const void *raw, void *dest);

#endif // YS_CONSUMER_MODEL_H
