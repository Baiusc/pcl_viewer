/*
 * @Author       : kaihong.yang kaihong.yang@vitalchem.com
 * @Date         : 2024-10-22 02:17:38
 * @LastEditors  : kaihong.yang kaihong.yang@vitalchem.com
 * @LastEditTime : 2024-10-23 08:33:40
 * @FilePath     : /you_see_arm/ys_business/ys_errno.h
 * @Description  :
 * Copyright (c) 2024 by Vitalchem, All Rights Reserved.
 */

#ifndef YS_ERRNO_H
#define YS_ERRNO_H

#include "ys_def.h"

enum common_error_code {
	run_succ = 0x00,
	illegal_cmd,
	unavailable_cmd,
	run_fail,
};

enum switch_state_error_code { pwd_error = 0x04 }; /* 切换状态指令的错误码 */

/**
 * @description  : save error code
 * @param         {void} *arg
 * @param         {uint8_t} error_code
 * @return        {int} result
 */
int save_error_code(void *arg, uint8_t error_code);

/**
 * @description  : get saved error code
 * @param         {uint8_t} *data
 * @return        {int} result
 */
int get_error_code(uint8_t *data);

#endif // YS_ERRNO_H
