/*
 * @Author       : yangkh kaihong.yang@vitalchem.com
 * @Date         : 2024-05-28 03:05:43
 * @LastEditors  : kaihong.yang kaihong.yang@vitalchem.com
 * @LastEditTime : 2024-08-01 09:29:43
 * @FilePath     : /you_see_arm/core/ys_udp.h
 * @Description  :
 * Copyright (c) 2024 by Vitalchem, All Rights Reserved.
 */

#ifndef YS_UDP_H
#define YS_UDP_H

#include "ys_core_def.h"

/**
 * @description  : 发送数据到中心
 * @param         {udp_center_t} *center
 * @param         {uint8_t} *data
 * @param         {int} len
 * @return        {int} statuss
 */
int send_buf_to_center(udp_center_t *center, uint8_t *data, int len);

/**
 * @description  : 开启广播模式
 * @param         {udp_center_t} *center
 * @return        {int} result
 *      0 : success
 *      other : fail
 */
int set_broadcast_mode_on(udp_center_t *center);

/**
 * @description  : 关闭广播模式
 * @param         {udp_center_t} *center
 * @return        {int} result
 *      0 : success
 *      other : fail
 */
int set_broadcast_mode_off(udp_center_t *center);

/**
 * @description  : update udp instance config
 * @param         {udp_t} *udp_instance
 * @return        {*}
 */
void update_udp_config(udp_t *udp_instance);

/**
 * @description  : pthread of udp instance
 * @param         {void} *argv
 * @return        {*}
 */
void *pthread_cent_udp_client(void *argv);

#endif // YS_UDP_H
