/*
 * @Author       : kaihong.yang kaihong.yang@vitalchem.com
 * @Date         : 2025-09-08 19:36:18
 * @LastEditors  : kaihong.yang kaihong.yang@vitalchem.com
 * @LastEditTime : 2025-09-08 23:12:09
 * @FilePath     : /you_see_arm/ys_business/thread/report_encoder.h
 * @Description  :
 * Copyright (c) 2025 by kaihong.yang, email: kaihong.yang@vitalchem.com, All Rights Reserved.
 */

#ifndef REPORT_ENCODER_H
#define REPORT_ENCODER_H

#if CS96_ALPHA == 1 || CS96_BETA == 1 || CS96_GAMMA == 1
/**
 * @description  : 定时上报电机编码值
 * @param         {void} *arg
 * @return        {*}
 */
void *pthread_report_encoder(void *arg);
#endif

#endif // REPORT_ENCODER_H
