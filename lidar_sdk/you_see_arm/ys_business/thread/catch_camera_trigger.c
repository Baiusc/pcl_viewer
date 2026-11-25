/*
 * @Author       : kaihong.yang kaihong.yang@vitalchem.com
 * @Date         : 2025-09-08 19:17:03
 * @LastEditors  : kaihong.yang kaihong.yang@vitalchem.com
 * @LastEditTime : 2025-09-08 20:50:39
 * @FilePath     : /you_see_arm/ys_business/thread/catch_camera_trigger.c
 * @Description  :
 * Copyright (c) 2025 by kaihong.yang, email: kaihong.yang@vitalchem.com, All Rights Reserved.
 */
#if CS96_ALPHA == 1 || CS96_BETA == 1 || CS96_GAMMA == 1

#define _GNU_SOURCE

#include "catch_camera_trigger.h"

#include "../ys_def.h"
#include "../ys_pl_config.h"

#include <sched.h>

static atomic_uint_fast8_t g_run_flag;

/**
 * @description  : exit signal handler
 * @return        {int} signum
 */
static void exit_sighandler(int signum)
{
	log_debug("get trigger sighandler capture the signal %d", signum);
	atomic_store(&g_run_flag, STOPPED);
}

void *pthread_get_trigger(void *arg)
{
	log_debug("Thread is running on CPU %d", sched_getcpu());

	atomic_init(&g_run_flag, RUNNING);

	if (arg == NULL) {
		log_error("parameter is NULL"); /* err_quit */
		goto end;
	}
	thread_t *th_arg = (thread_t *)arg;

	if (th_arg->signal_enable == ON) {
		log_debug("listen signal no is %d", th_arg->signum);
		signal(th_arg->signum, exit_sighandler);
	}

	while (atomic_load(&g_run_flag) == RUNNING) {
		report_camera_trigger();
		usleep(100);
	}

end:
	pthread_exit(0);
}
#endif
