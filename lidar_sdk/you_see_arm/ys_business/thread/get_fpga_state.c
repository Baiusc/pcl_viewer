/*
 * @Author       : kaihong.yang kaihong.yang@vitalchem.com
 * @Date         : 2025-09-08 20:07:22
 * @LastEditors  : kaihong.yang kaihong.yang@vitalchem.com
 * @LastEditTime : 2025-09-08 20:52:09
 * @FilePath     : /you_see_arm/ys_business/thread/get_fpga_state.c
 * @Description  :
 * Copyright (c) 2025 by kaihong.yang, email: kaihong.yang@vitalchem.com, All Rights Reserved.
 */
#if CS96_ALPHA == 1 || CS96_BETA == 1 || CS96_GAMMA == 1

#define _GNU_SOURCE

#include "get_fpga_state.h"

#include "../ys_def.h"
#include "../ys_msg_distribute.h"
#include "../ys_ps_config.h"
#include "../ys_state_machine.h"

#include "../../core/ys_base.h"

#include <sched.h>

static atomic_uint_fast8_t g_run_flag;

static struct lidar_fpga_selftest *fpga_pre_state = NULL;
static struct lidar_fpga_selftest *fpga_now_state = NULL;

static void exit_sighandler(int signum)
{
	log_debug("get fpga state sighandler capture the signal %d", signum);
	atomic_store(&g_run_flag, STOPPED);
}

void *pthread_get_fpga_state(void *arg)
{
	log_debug("Thread is running on CPU %d", sched_getcpu());

	atomic_init(&g_run_flag, RUNNING);

	if (arg == NULL) {
		log_error("parameter is NULL");
		goto end;
	}
	thread_t *th_arg = (thread_t *)arg;

	if (th_arg->thread_arg == NULL) {
		log_error("thread arg is NULL");
		goto end;
	}
	sys_state_t *system_state = (sys_state_t *)(th_arg->thread_arg);

	if (th_arg->signal_enable == ON) {
		log_debug("listen signal no is %d", th_arg->signum);
		signal(th_arg->signum, exit_sighandler);
	}

	if (system_state->fpga_state == NULL) {
		log_error("fpga_state is NULL");
		goto end;
	}

	fpga_pre_state = (struct lidar_fpga_selftest *)Malloc(sizeof(struct lidar_fpga_selftest));
	fpga_now_state = (struct lidar_fpga_selftest *)Malloc(sizeof(struct lidar_fpga_selftest));

#if LOCAL_TEST == 1
#else
	int deal = config.deal_pl_system_state;
	int ret = RET_FAIL;
	uint8_t buf[] = { CMD_SET_STATE, CMD_ID_SET_STATE, FAULT };
	management_id_t msg;
	msg.code_set = CMD_SET_STATE;
	msg.code_id = CMD_ID_SET_STATE;
	msg.silent = SILENT;
	msg.buf = buf;
#endif

	while (atomic_load(&g_run_flag) == RUNNING) {
#if LOCAL_TEST == 1
		fpga_now_state->done = FINISH;
		fpga_now_state->workmode = FPGA_SCAN;
		fpga_now_state->motor_status = NORMAL;
		fpga_now_state->mirror_angle = NORMAL;
		fpga_now_state->spi_status = NORMAL;
#else
		ret = fpga_status_get(fpga_now_state);
		if (ret != RET_OK) {
			log_error("fpga_status_get error");
			sleep(1);
			continue;
		}

		if (fpga_now_state->done == UN_FINISH) {
			sleep(1);
			continue;
		}

		if (fpga_now_state->motor_status == ABNORMAL && fpga_pre_state->motor_status == NORMAL) {
			ret++;
			log_error("FPGA: motor status error");
		}

		if (fpga_now_state->mirror_angle == ABNORMAL && fpga_pre_state->mirror_angle == NORMAL) {
			ret++;
			log_error("FPGA: mirror status error");
		}

		if (fpga_now_state->spi_status == ABNORMAL && fpga_pre_state->spi_status == NORMAL) {
			ret++;
			log_error("FPGA: spi status error");
		}

		if (ret && deal == ON) {
			set_run_state((void *)&msg);
		}

#endif
		memcpy(fpga_pre_state, fpga_now_state, sizeof(struct lidar_fpga_selftest));
		pthread_mutex_lock(&(system_state->lock));
		memcpy(system_state->fpga_state, fpga_now_state, sizeof(struct lidar_fpga_selftest));
		pthread_mutex_unlock(&(system_state->lock));
		sleep(1);
	}

end:
	Free((void **)&fpga_pre_state);
	Free((void **)&fpga_now_state);
	pthread_exit(0);
}
#endif
