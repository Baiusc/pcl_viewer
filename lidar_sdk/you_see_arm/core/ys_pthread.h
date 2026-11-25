/*
 * @Author       : yangkh kaihong.yang@vitalchem.com
 * @Date         : 2024-05-27 21:09:23
 * @LastEditors  : kaihong.yang kaihong.yang@vitalchem.com
 * @LastEditTime : 2024-08-23 01:32:22
 * @FilePath     : /you_see_arm/core/ys_pthread.h
 * @Description  :
 * Copyright (c) 2024 by Vitalchem, All Rights Reserved.
 */

#ifndef YS_PTHREAD_H
#define YS_PTHREAD_H

#include "ys_core_def.h"

#include <signal.h>
#include <pthread.h>

/**
 * @description  : create thread
 * @param         {pthread_t} pid
 * @param         {void} *arg
 * @param         {pthread_attr_t} *attr
 * @param         {int} core_id
 * @param         {int} priority
 * @return        {*}
 */
pthread_t create_thread(void *(*routine)(void *), void *arg, const pthread_attr_t *attr, int core_id, int priority);

/**
 * @description  : exit thread pool all thread
 * @param         {thpool_t} *thpool
 * @return        {*}
 */
void thpool_exit(thpool_t *thpool);

#endif // YS_PTHREAD_H
