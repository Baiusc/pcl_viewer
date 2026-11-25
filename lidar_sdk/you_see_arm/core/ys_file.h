/*
 * @Author       : yangkh kaihong.yang@vitalchem.com
 * @Date         : 2024-05-27 04:50:41
 * @LastEditors  : kaihong.yang kaihong.yang@vitalchem.com
 * @LastEditTime : 2025-07-09 02:01:30
 * @FilePath     : /you_see_arm/core/ys_file.h
 * @Description  :
 * Copyright (c) 2024 by Vitalchem, All Rights Reserved.
 */

#ifndef YS_FILE_H
#define YS_FILE_H

#include "ys_core_def.h"

#include <stdio.h>

#define SYS_LOG_PATH "/var/log/messages" // 系统日志路径

/**
 * @description  : clean system log
 * @return        {int} result
 *      0 : success
 *      other : fail
 */
int clean_sys_log();

/**
 * @description  : clean kernel log
 * @return        {int} result
 *      0 : success
 *      other : fail
 */
int clean_kernel_log();

/**
 * @description  : clean you_see log file
 * @param         {char} *file_path
 * @return        {int} result
 *      0 : success
 *      other : fail
 */
int clean_app_log(const char *file_path);

/**
 * @description  : update file to flash
 * @param         {const char *} flash_file_path
 * @param         {const char *} file_path
 * @return        {int} result
 *      0 : success
 *      other : fail
 */
int update_flash_file(const char *flash_file_path, const char *file_path);

#endif // YS_FILE_H
