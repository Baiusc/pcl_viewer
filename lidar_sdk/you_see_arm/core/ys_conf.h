/*
 * @Author       : kaihong.yang kaihong.yang@vitalchem.com
 * @Date         : 2024-07-13 03:06:00
 * @LastEditors  : kaihong.yang kaihong.yang@vitalchem.com
 * @LastEditTime : 2025-01-02 01:54:33
 * @FilePath     : /you_see_arm/core/ys_conf.h
 * @Description  :
 * Copyright (c) 2024 by Vitalchem, All Rights Reserved.
 */

#ifndef YS_CONF_H
#define YS_CONF_H

#include "ys_core_def.h"

#include "../third_party/iniparser/iniparser.h"

#if LOCAL_TEST == 1
#if CS96_GAMMA == 1
#define DEFAULT_CAL_CONF_PATH "./conf/ys_calibration_gamma.ini"
#else
#define DEFAULT_CAL_CONF_PATH "./conf/ys_calibration.ini"
#endif
#else
/* 实机按机型刷入对应参数, 路径同名不冲突 */
#define DEFAULT_CAL_CONF_PATH "/etc/ys_calibration.ini"
#endif

#if CS96_BETA == 1 || CS96_GAMMA == 1
#if LOCAL_TEST == 1
#define DEFAULT_ENCODER_ANGLE_FILE_PATH "./conf/ys_encoder_angle.csv"
#else
#define DEFAULT_ENCODER_ANGLE_FILE_PATH "/etc/ys_encoder_angle.csv"
#endif
#endif

#define MATCH(s, n) strcmp(section, s) == 0 && strcmp(key, n) == 0

extern dictionary *conf_ini; // 配置参数
extern configuration_t config; // 全局存储 - 配置参数
extern pthread_mutex_t config_lock __attribute__((aligned(8)));

extern dictionary *cal_param_ini; // 标定参数
extern calibration_parm_t cal_param; // 全局存储 - 标定参数
extern pthread_mutex_t cal_lock __attribute__((aligned(8)));

extern int cal_param_sizeof; // 标定参数的数据大小

extern int vcsel_cnt; // VCCSEL 的条数
extern uint16_t total_slots; // 转镜扫描一面中的 slot 数量

#if CS96_BETA == 1 || CS96_GAMMA == 1
extern float *g_encoder_angle; // 电机整数刻度角度表
extern int g_encoder_angle_cnt; // 电机整数刻度角度表的条数
#endif

/**
 * @description  : update config file
 * @param         {dictionary} *ini
 * @param         {const char} *path
 * @return        {int} result
 *      0 : success
 *      other : fail
 */
int core_update_ini_file(dictionary *ini, const char *path);

/**
 * @description  : load calibration config
 * @return        {int} result
 *      0 : success
 *      other : fail
 */
int core_load_cal_ini();

/**
 * @description  : load config file
 * @param         {const char} *conf_path
 * @return        {int} result
 *      0 : success
 *      other : fail
 */
int core_load_conf_ini(const char *conf_path);

/**
 * @description  : unload calibration config
 * @return        {*}
 */
void core_unload_cal_ini();

/**
 * @description  : unload config file
 * @return        {*}
 */
void core_unload_conf_ini();

#endif // YS_CONF_H
