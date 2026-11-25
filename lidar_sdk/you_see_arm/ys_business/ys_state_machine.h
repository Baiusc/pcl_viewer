/*
 * @Author       : yangkh kaihong.yang@vitalchem.com
 * @Date         : 2024-05-27 22:05:37
 * @LastEditors  : kaihong.yang kaihong.yang@vitalchem.com
 * @LastEditTime : 2025-05-26 02:03:49
 * @FilePath     : /you_see_arm/ys_business/ys_state_machine.h
 * @Description  :
 * Copyright (c) 2024 by Vitalchem, All Rights Reserved.
 */

#ifndef YS_STATE_MACHINE_H
#define YS_STATE_MACHINE_H

#include "ys_def.h"

#define DEBUG_PASSWORD 0xaabbccdd

#define NO_TASK_TO_RUN 0

/**
 * @description  : exit thread to get in idle state
 * @param         {*}
 * @return        {int} result
 *      0 : success
 *      other : fail
 */
int exit_thread_to_get_in_idle_state();
int stop_capture_for_shutdown(void);

/**
 * @description  : get now state machine
 * @param         {*}
 * @return        {int} state
 */
int get_now_run_state();

/**
 * @description  : message interface - get state machine
 * @param         {void} *arg
 *      arg's type must be management_id_t
 * @return        {*}
 */
void get_run_state(void *arg);

/**
 * @description  : message interface - set state machine
 * @param         {void} *arg
 *      arg's type must be management_id_t
 * @return        {*}
 */
void set_run_state(void *arg);

/**
 * @description  : initialization state machine
 * @param         {int} init_state
 * @return        {*}
 */
void init_state_machine(int init_state);

/**
 * @description  : uninitialization state machine
 * @return        {*}
 */
void uninit_state_machine();

/**
 * @description  : message interface - get self state
 * @param         {void} *arg
 *      arg's type must be management_id_t
 * @return        {*}
 */
void get_self_state(void *arg);

extern thpool_t data_thpool;
extern sys_state_t sys_state;

#endif // YS_STATE_MACHINE_H
