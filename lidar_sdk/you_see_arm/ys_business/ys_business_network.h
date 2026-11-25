/*
 * @Author       : yangkh kaihong.yang@vitalchem.com
 * @Date         : 2024-05-28 23:44:25
 * @LastEditors  : kaihong.yang kaihong.yang@vitalchem.com
 * @LastEditTime : 2025-01-13 02:55:00
 * @FilePath     : /you_see_arm/ys_business/ys_business_network.h
 * @Description  :
 * Copyright (c) 2024 by Vitalchem, All Rights Reserved.
 */

#ifndef YS_BUSINESS_NETWORK_H
#define YS_BUSINESS_NETWORK_H

#include "ys_def.h"

#define PHY_IFACE "eth0"

/**
 * @description  : 响应指令数据的接口，通过UDP发送
 * @param         {udp_center_t} *center
 * @param         {uint8_t} *data
 * @param         {int} len
 * @return        {int} statuss
 */
int respond_command(uint8_t *data, int len);

/**
 * @description  : 发送数据的接口，通过UDP发送
 * @param         {udp_t} *udp
 * @param         {uint8_t} *data
 * @param         {int} len
 * @return        {int} result
 */
int send_data(udp_t *udp, uint8_t *data, int len);

/**
 * @description  : message interface - set ip address info
 * @param         {void} *arg
 *      arg's type must be management_id_t
 * @return        {*}
 */
void set_ip_info(void *arg);

/**
 * @description  : message interface - get ip address info
 * @param         {void} *arg
 *      arg's type must be management_id_t
 * @return        {*}
 */
void get_ip_info(void *arg);

/**
 * @description  : message interface - get mac address info
 * @param         {void} *arg
 *      arg's type must be management_id_t
 * @return        {*}
 */
void get_mac_address(void *arg);

/**
 * @description  : message interface - network handshake
 * @param         {void} *arg
 *      arg's type must be management_id_t
 * @return        {*}
 */
void network_handshake(void *arg);

/**
 * @description  : message interface - network heartbeat
 * @param         {void} *arg
 *      arg's type must be management_id_t
 * @return        {*}
 */
void network_heartbeat(void *arg);

/**
 * @description  : message interface - network break handshake
 * @param         {void} *arg
 *      arg's type must be management_id_t
 * @return        {*}
 */
void network_break_handshake(void *arg);

/**
 * @description  : check system network
 */
void check_network();

/**
 * @description  : initialization udp
 * @param         {calibration_parm_t} *cal_param
 * @param         {configuration_t} *config
 * @return        {int} result
 *      0 : success
 *      other : fail
 */
int init_udp(calibration_parm_t *cal_param, configuration_t *config);

/**
 * @description  : uninitialization udp
 * @return        {*}
 */
void uninit_udp();

extern udp_t udp_cmd_instance;
extern udp_t udp_data_instance;

#endif // YS_BUSINESS_NETWORK_H
