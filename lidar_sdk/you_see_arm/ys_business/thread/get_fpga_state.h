/*
 * @Author       : kaihong.yang kaihong.yang@vitalchem.com
 * @Date         : 2025-09-08 20:07:17
 * @LastEditors  : kaihong.yang kaihong.yang@vitalchem.com
 * @LastEditTime : 2025-09-08 20:07:19
 * @FilePath     : /you_see_arm/ys_business/thread/get_fpga_state.h
 * @Description  :
 * Copyright (c) 2025 by kaihong.yang, email: kaihong.yang@vitalchem.com, All Rights Reserved.
 */

#ifndef GET_FPGA_STATE_H
#define GET_FPGA_STATE_H

#if CS96_ALPHA == 1 || CS96_BETA == 1 || CS96_GAMMA == 1
/**
 * @description  : 获取FPGA的运行状态
 * @param         {void} *arg
 * @return        {*}
 */
void *pthread_get_fpga_state(void *arg);
#endif

#endif // GET_FPGA_STATE_H
