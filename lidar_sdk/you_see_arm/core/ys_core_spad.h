/*
 * @Author       : kaihong.yang kaihong.yang@vitalchem.com
 * @Date         : 2025-01-07 02:53:16
 * @LastEditors  : kaihong.yang kaihong.yang@vitalchem.com
 * @LastEditTime : 2025-01-17 07:50:37
 * @FilePath     : /you_see_arm/core/ys_core_spad.h
 * @Description  :
 * Copyright (c) 2025 by Vitalchem, All Rights Reserved.
 */
#ifndef YS_CORE_SPAD_H
#define YS_CORE_SPAD_H

#define COM_READY 1
#define COM_NOT_READY 0

#if CS96_BETA == 1 || CS96_GAMMA == 1
/**
 * @description  : SPAD init
 * @param         {int} work_mode
 * @return        {int} result
 * 0: success
 * other: failed
 * 注: CS96_GAMMA(SL100) 由 FPGA 自动完成初始化, ARM 端不配置寄存器, 直接返回成功
 */
int spad_init(int work_mode);
#elif MF150_FL6031 == 1
/**
 * @description  : SPAD init
 * @return        {int} result
 * 0: success
 * other: failed
 */
int spad_init();
#endif

#endif // YS_CORE_SPAD_H
