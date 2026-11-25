/*
 * @Author       : yangkh kaihong.yang@vitalchem.com
 * @Date         : 2024-05-28 23:36:19
 * @LastEditors  : kaihong.yang kaihong.yang@vitalchem.com
 * @LastEditTime : 2025-09-08 20:08:34
 * @FilePath     : /you_see_arm/ys_business/ys_business_sys.h
 * @Description  :
 * Copyright (c) 2024 by Vitalchem, All Rights Reserved.
 */

#ifndef YS_BUSINESS_SYS_H
#define YS_BUSINESS_SYS_H

/**
 * @description  : message interface - get last error code
 * @param         {void} *arg
 *      arg's type must be management_id_t
 * @return        {*}
 */
void get_last_error_code(void *arg);

/**
 * @description  : message interface - get system model
 * @param         {void} *arg
 *      arg's type must be management_id_t
 * @return        {*}
 */
void get_sys_model(void *arg);

/**
 * @description  : message interface - upgrade fpga
 * @param         {void} *arg
 *      arg's type must be management_id_t
 * @return        {*}
 */
void upgrade_fpga(void *arg);

/**
 * @description  : message interface - get system version
 *      应答固件总包信息 + 各组件版本列表(you_see / ARM 驱动 / 算法 / FPGA / Web 服务)
 * @param         {void} *arg
 *      arg's type must be management_id_t
 * @return        {*}
 */
void get_sys_version(void *arg);

/**
 * @description  : message interface - upgrade system
 * @param         {void} *arg
 *      arg's type must be management_id_t
 * @return        {*}
 */
void upgrade_sys(void *arg);

/**
 * @description  : message interface - reboot system
 * @param         {void} *arg
 *      arg's type must be management_id_t
 * @return        {*}
 */
void reboot_sys(void *arg);

/**
 * @description  : 获取系统 SN。应答负载为 1 字节状态码和 19 字节原始 SN；失败时 SN 字段为零
 * @param         {void} *arg
 *      arg's type must be management_id_t
 * @return        {*}
 */
void get_sys_sn(void *arg);

/**
 * @description  : message interface - clean log
 * @param         {void} *arg
 *      arg's type must be management_id_t
 *      0x01 : clean system log
 *      0x03 : clean kernel log
 *      0x05 : clean application log
 * @return        {*}
 */
void clean_log(void *arg);

/**
 * @description  : check system
 */
void check_system();

/**
 * @description  : message interface - get single histogram
 * @param         {void} *arg
 *      arg's type must be management_id_t
 * @return        {*}
 */
void get_single_histogram(void *arg);

#endif // YS_BUSINESS_SYS_H
