/*
 * @Author       : yangkh kaihong.yang@vitalchem.com
 * @Date         : 2024-05-27 05:19:58
 * @LastEditors  : kaihong.yang kaihong.yang@vitalchem.com
 * @LastEditTime : 2025-05-22 20:24:10
 * @FilePath     : /you_see_arm/ys_business/ys_ps_config.h
 * @Description  :
 * Copyright (c) 2024 by Vitalchem, All Rights Reserved.
 */

#ifndef YS_LOAD_PS_CONFIG_H
#define YS_LOAD_PS_CONFIG_H

#include "ys_def.h"
#include "../core/ys_conf.h"

#if LOCAL_TEST == 1
#define DEFAULT_CONF_PATH "./conf/ys_conf.ini"
#else
#define DEFAULT_CONF_PATH "/etc/ys_conf.ini"
#endif

/**
 * @description  : load config from ys_conf.ini
 * @return        {int} result
 *      0 : success
 *      other : fail
 */
int load_config();

/* 返回本次启动实际加载的配置文件路径。 */
const char *ys_runtime_config_path(void);

/**
 * @description  : destroy config
 * @return        {*}
 */
void destroy_config();

/**
 * @description  : load calibration config from ys_calibration.ini
 * @return        {int} result
 *      0 : success
 *      other : fail
 */
int load_cal_config();

/**
 * @description  : destroy calibration config
 * @return        {*}
 */
void destroy_cal_config();

/**
 * @description  : update calibration parameter to config file
 * @param         {calibration_parm_head_t} *data
 * @return        {int} result
 */
int update_calibration_config(calibration_parm_head_t *data);

/**
 * @description  : message interface - set echo parameter
 * @param         {void} *arg
 *      arg's type must be management_id_t
 * @return        {*}
 */
void set_echo_parm(void *arg);

/**
 * @description  : message interface - get echo parameter
 * @param         {void} *arg
 *      arg's type must be management_id_t
 * @return        {*}
 */
void get_echo_parm(void *arg);

/**
 * @description  : message interface - set coordinate mode
 * @param         {void} *arg
 *      arg's type must be management_id_t
 * @return        {*}
 */
void set_coordinate_mode(void *arg);

/**
 * @description  : message interface - get coordinate mode
 * @param         {void} *arg
 *      arg's type must be management_id_t
 * @return        {*}
 */
void get_coordinate_mode(void *arg);

/**
 * @description  : message interface - get calibration parm
 * @param         {void} *arg
 *      arg's type must be management_id_t
 * @return        {*}
 */
void get_cal_parm(void *arg);

/**
 * @description  : message interface - set calibration parm
 * @param         {void} *arg
 *      arg's type must be management_id_t
 * @return        {*}
 */
void set_cal_parm(void *arg);

#endif // YS_LOAD_PS_CONFIG_H
