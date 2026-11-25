/*
 * @Author       : yangkh kaihong.yang@vitalchem.com
 * @Date         : 2024-05-28 23:34:39
 * @LastEditors  : kaihong.yang kaihong.yang@vitalchem.com
 * @LastEditTime : 2024-10-23 08:57:37
 * @FilePath     : /you_see_arm/core/ys_core_network.h
 * @Description  :
 * Copyright (c) 2024 by Vitalchem, All Rights Reserved.
 */

#ifndef YS_CORE_NETWORK_H
#define YS_CORE_NETWORK_H

#include <stdint.h>

/* 采集层只查询网络状态；配置由独立网络服务统一执行。 */
int get_ip_via_interface(const char *iface, uint8_t *dest);
int get_mask_via_interface(const char *iface, uint8_t *dest);
int get_gateway_via_interface(const char *iface, uint8_t *dest);
int get_mac_via_interface(const char *iface, uint8_t *dest);

#endif
