/*
 * @Author       : kaihong.yang kaihong.yang@vitalchem.com
 * @Date         : 2025-09-08 19:15:58
 * @LastEditors  : kaihong.yang kaihong.yang@vitalchem.com
 * @LastEditTime : 2025-09-08 19:57:21
 * @FilePath     : /you_see_arm/ys_business/thread/catch_camera_trigger.h
 * @Description  :
 * Copyright (c) 2025 by kaihong.yang, email: kaihong.yang@vitalchem.com, All Rights Reserved.
 */

#ifndef CATCH_CAMERA_TRIGGER_H
#define CATCH_CAMERA_TRIGGER_H

#if CS96_ALPHA == 1 || CS96_BETA == 1 || CS96_GAMMA == 1
/**
 * @description  : 相机 trigger 信号处理函数
 * @param         {void} *arg
 * @return        {*}
 */
void *pthread_get_trigger(void *arg);
#endif

#endif // CATCH_CAMERA_TRIGGER_H
