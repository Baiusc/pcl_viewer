/*
 * @Author       : kaihong.yang kaihong.yang@vitalchem.com
 * @Date         : 2024-06-25 03:17:26
 * @LastEditors  : kaihong.yang kaihong.yang@vitalchem.com
 * @LastEditTime : 2025-06-24 01:06:05
 * @FilePath     : /you_see_arm/ys_business/ys_pl_config.h
 * @Description  :
 * Copyright (c) 2024 by Vitalchem, All Rights Reserved.
 */

#ifndef YS_PL_CONFIG_H
#define YS_PL_CONFIG_H

#include "ys_def.h"

#if CS96_ALPHA == 1 || CS96_BETA == 1 || CS96_GAMMA == 1
/***********************************设置VCSEL参数******************************** */
#define SET_VCSEL_CHANNEL 0x00 // 设置发光通道
#define SET_VCSEL_FREQ 0x01 // 设置发光重频
#define SET_VCSEL_LIGHTED_DELAY 0x02 // 设置 VCSEL 次弱发光延时

/***********************************获取VCSEL参数******************************** */
#define GET_VCSEL_CHAN 0x00 // 获取发光通道
#define GET_VCSEL_REFREQ 0x01 // 获取发光重频
#define GET_LD_TEMP 0x02 // 激光发射板温度获取
#define GET_VCSEL_LIGHTED_DELAY 0x03 // 获取 VCSEL 次弱发光延时

/***********************************设置SPAD参数******************************** */
#define SET_VOP_PWM_FREQ 0x00 // imx459 VOP 电压 PWM 频率
#define SET_VOP_PWM_DUTY 0x01 // imx459 VOP 电压 PWM 占空
#define SET_SPAD_SPI 0x02 // imx459 spi 写寄存器

/***********************************获取SPAD参数******************************** */
#define GET_SPAD_TEMP 0x00 // SPAD 温度获取
#define GET_VOP_SAMPLE 0x01 // VOP 电压采样值获取
#define GET_SPAD_PWM_FREQ 0x02 // VOP 电压 PWM 频率
#define GET_SPAD_PWM_DUTY 0x03 // VOP 电压 PWM 占空比
#define GET_SPAD_SPI 0x04 // imx459 spi 读寄存器

/***********************************设置电机参数******************************** */
#define SET_MOTOR_CTRL 0x00 // 电机控制设置
#define SET_MOTOR_PID 0x01 // PID 闭环控制
#define SET_MOTOR_PWM_FREQ 0x02 // 开环控制 PWM 频率
#define SET_MOTOR_PWM_DUTY 0x03 // 开环控制 PWM 占空比
#define SET_CALIB_ZERO_OFFSET 0x04 // 转镜零点位角度偏移量
#define SET_MOTOR_STOP_ANGLE 0x05 // 电机停止角度设置
#define SET_FLAT_OFFSET 0x06 // 转镜水平 0°偏移角设置

/***********************************获取电机参数******************************** */
#define GET_MOTOR_AGLE 0x00 // 转镜实时角度
#define GET_MOTOR_ZERO_TIME 0x01 // 转镜从检测到零点后的时间
#define GET_CALIB_ZERO_OFFSE 0x02 // 转镜零点位角度偏移量
#define GET_MOTOR_CTRL 0x03 // 获取电机控制参数
#define GET_MOTOR_PID 0x04 // 获取 PID 闭环参数
#define GET_MOTOR_PWM_FREP 0x05 // 获取开环控制 PWM 频率参数
#define GET_MOTOR_PWM_DUTY 0x06 // 获取开环控制 PWM 占空比参数
#define GET_MOTOR_STOP_ANGLE 0x07 // 电机停止角度获取
#define GET_MOTOR_PERIOD 0x08 // 检测到相邻两次零位的周期获取
#define GET_FLAT_OFFSET 0x09 // 获取转镜水平 0°偏移角

/***********************************设置PL标定参数******************************** */
#define SET_CALIB_THRESHOLD_TIME 0x00 // 设置阈值标定时间点
#define SET_CALIB_THRESHOLD_BACK 0x01 // 设置背景阈值

/***********************************获取PL标定参数******************************** */
#define GET_CALIB_THRESHOLD_TIME 0x00 // 获取阈值标定时间点
#define GET_CALIB_THRESHOLD_BACK 0x01 // 获取背景阈值

/**
 * @description  : message interface - set refresh rate
 * @param         {void} *arg
 *      arg's type must be management_id_t
 * @return        {*}
 */
void set_refresh_rate(void *arg);

/**
 * @description  : message interface - get refresh rate
 * @param         {void} *arg
 *      arg's type must be management_id_t
 * @return        {*}
 */
void get_refresh_rate(void *arg);

/**
 * @description  : message interface - set vcsel parameter
 * @param         {void} *arg
 *      arg's type must be management_id_t
 * @return        {*}
 */
void set_vcsel_parameter(void *arg);

/**
 * @description  : message interface - get vcsel parameter
 * @param         {void} *arg
 *      arg's type must be management_id_t
 * @return        {*}
 */
void get_vcsel_parameter(void *arg);

/**
 * @description  : message interface - set spad parameter
 * @param         {void} *arg
 *      arg's type must be management_id_t
 * @return        {*}
 */
void set_spad_parameter(void *arg);

/**
 * @description  : message interface - get spad parameter
 * @param         {void} *arg
 *      arg's type must be management_id_t
 * @return        {*}
 */
void get_spad_parameter(void *arg);

/**
 * @description  : message interface - set motor parameter
 * @param         {void} *arg
 *      arg's type must be management_id_t
 * @return        {*}
 */
void set_motor_parameter(void *arg);

/**
 * @description  : message interface - get motor parameter
 * @param         {void} *arg
 *      arg's type must be management_id_t
 * @return        {*}
 */
void get_motor_parameter(void *arg);

#if CS96_ALPHA == 1
/**
 * @description  : get motor encoder
 * @return        {uint16_t} encoder value
 */
uint16_t get_encoder();
#elif CS96_BETA == 1 || CS96_GAMMA == 1
/**
 * @description  : get motor encoder
 * @param         {encoder_info_t} *info
 * @return        {int} result
 *      0 : success
 *      other : fail
 */
uint16_t get_encoder(encoder_info_t *info);
#endif

/**
 * @description  : report camera trigger
 * @return        {*}
 */
void report_camera_trigger();

/**
 * @description  : message interface - set calibration parameter
 * @param         {void} *arg
 *      arg's type must be management_id_t
 * @return        {*}
 */
void set_calib_parameter(void *arg);

/**
 * @description  : message interface - get calibration parameter
 * @param         {void} *arg
 *      arg's type must be management_id_t
 * @return        {*}
 */
void get_calib_parameter(void *arg);

/**
 * @description  : message interface - set side angular offset
 * @param         {void} *arg
 *      arg's type must be management_id_t
 * @return        {*}
 */
void set_side_angular_offset(void *arg);

/**
 * @description  : message interface - set side light en
 * @param         {void} *arg
 *      arg's type must be management_id_t
 * @return        {*}
 */
void set_side_light_en(void *arg);

/**
 * @description  : message interface - set vcsel horizontal angular offset(horizontal_angular_offset)
 * @param         {void} *arg
 *      arg's type must be management_id_t
 * @return        {*}
 */
void set_vcsel_h_offset(void *arg);

/**
 * @description  : message interface - set distance offset(set_zero_distance_offset)
 * @param         {void} *arg
 *      arg's type must be management_id_t
 * @return        {*}
 */
void set_distance_offset(void *arg);

/**
 * @description  : initailized pl calibration parameter
 * @param         {calibration_parm_t *} param
 * @return        {int} result
 */
int pl_calib_parm_init(calibration_parm_t *param);
#elif MF150_FL6031 == 1
/**
 * @description  : initailized pl calibration parameter
 * @param         {calibration_parm_t *} param
 * @return        {int} result
 */
int pl_calib_parm_init(calibration_parm_t *param);
#elif MF150_FL6532 == 1
/**
 * @description  : initailized pl calibration parameter
 * @param         {calibration_parm_t *} param
 * @return        {int} result
 */
int pl_calib_parm_init(calibration_parm_t *param);
#endif

#endif // YS_PL_CONFIG_H
