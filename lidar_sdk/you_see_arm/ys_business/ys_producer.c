/*
 * @Author       : kaihong.yang kaihong.yang@vitalchem.com
 * @Date         : 2024-11-13 01:59:50
 * @LastEditors  : kaihong.yang kaihong.yang@vitalchem.com
 * @LastEditTime : 2026-04-08 23:59:47
 * @FilePath     : /you_see_arm/ys_business/ys_producer.c
 * @Description  :
 * Copyright (c) 2024 by Vitalchem, All Rights Reserved.
 */

#define _GNU_SOURCE

// 直方图生产者
#include "producer/histogram/ys_producer_histogram_cs96_alpha.h"
#include "producer/histogram/ys_producer_histogram_cs96_beta.h"
#include "producer/histogram/ys_producer_histogram_cs96_gamma.h"
#include "producer/histogram/ys_producer_histogram_mf150_fl6031.h"

// 点云生产者
#include "producer/point_cloud/ys_producer_point_cloud_cs96_alpha.h"
#include "producer/point_cloud/ys_producer_point_cloud_cs96_beta.h"
#include "producer/point_cloud/ys_producer_point_cloud_cs96_gamma.h"
#include "producer/point_cloud/ys_producer_point_cloud_mf150_fl6031.h"
#include "producer/point_cloud/ys_producer_point_cloud_mf150_fl6532.h"

#if MF150_FL6532 == 1
// Range(原始测距)数据生产者 (FL6532: work_mode=1 原始点云模式)
#include "producer/range/ys_producer_range_mf150_fl6532.h"
#endif

#if CS96_GAMMA == 1
// 灰度图数据生产者 (CS96-gamma 灰度构建: work_mode=3 灰度图模式)
#include "producer/gray/ys_producer_gray_cs96_gamma.h"
#elif CS96_BETA == 1
// 灰度图数据生产者 (CS96-beta 灰度构建: work_mode=3 灰度图模式)
#include "producer/gray/ys_producer_gray_cs96_beta.h"
#endif

#include "ys_producer.h"

#include "../common/common.h"
#include "yousee_lidar_api_public.h"

#include "../core/ys_base.h"
#include "../core/ys_conf.h"
#include "../core/ys_core_def.h"
#include "../core/ys_core_system.h"
#include "../core/ys_memory_pool.h"

#include <sched.h>
#include <stdatomic.h>

static atomic_uint_fast8_t g_run_flag;
static uint8_t *data_buf = NULL;
#if LOCAL_TEST == 1
static FILE *fp = NULL;
static uint8_t *file_node = NULL;
#endif

static int driver_slot_num = 10; // 从驱动中一次读取的 slot 个数

int init_producer_parm(int lines_v)
{
#if CS96_BETA == 1
	// Beta 灰度的每次 DMA 传输只包含 1 个 SLOT。
	// 与点云模式下按 lines_v 决定批量读取的逻辑不同
	if (config.work_mode == GRAY_MODE) {
		driver_slot_num = 1;
		return RET_OK;
	}
#endif
	switch (lines_v) {
	case 96:
		driver_slot_num = 10;
		break;
	case 192:
		driver_slot_num = 5;
		break;
	default:
		driver_slot_num = 10;
		break;
	}
	return RET_OK;
}

/**
 * @description  : producer signal handler
 * @return        {int} signum
 */
static void producer_sighandler(int signum)
{
	log_debug("producer sighandler capture the signal %d", signum);

	// 退出直方图生产者线程
	producer_exit_histogram();

#if MF150_FL6532 == 1
	// 退出 Range 生产者线程
	producer_exit_range();
#endif

#if CS96_GAMMA == 1 || CS96_BETA == 1
	// 退出灰度图生产者线程
	producer_exit_gray();
#endif

	// 退出点云生产者线程
	producer_exit_point_cloud();

	atomic_store(&g_run_flag, STOPPED);
}

void *phtread_producer(void *arg)
{
	log_debug("Thread is running on CPU %d", sched_getcpu());

	atomic_init(&g_run_flag, RUNNING);

	udp_t *udp_instance __attribute__((unused)) = NULL;

	if (arg == NULL) {
		log_error("parameter is NULL"); /* err_quit */
		goto end;
	}
	thread_t *th_arg = (thread_t *)arg;

	if (th_arg->thread_arg == NULL) {
		log_error("thread arg is NULL"); /* err_quit */
		goto end;
	}
	udp_instance = (udp_t *)(th_arg->thread_arg);

	if (th_arg->signal_enable == ON) {
		log_debug("listen signal no is %d", th_arg->signum);
		signal(th_arg->signum, producer_sighandler);
	}

	int work_mode = FORMAL_POINT_CLOUD_MODE;
	pthread_mutex_lock(&(config_lock));
	work_mode = config.work_mode;
	pthread_mutex_unlock(&(config_lock));

	switch (work_mode) {
	case RAW_MODE:
		log_info("get in raw histogram mode");
		producer_get_histogram(udp_instance);
		break;
	case TEST_POINT_CLOUD_MODE:
#if MF150_FL6532 == 1
		// FL6532: work_mode=1 视为原始数据模式, fs_sdk 处于 RANGE 模式, 消费 0x2E 数据
		log_info("get in raw range mode (fs_sdk RANGE)");
		producer_get_range(udp_instance, driver_slot_num);
#else
		log_info("get in raw point cloud mode");
		producer_get_point_cloud(udp_instance, driver_slot_num);
#endif
		break;
	case FORMAL_POINT_CLOUD_MODE:
		log_info("get in real point cloud mode");
		producer_get_point_cloud(udp_instance, driver_slot_num);
		break;
	case GRAY_MODE:
#if CS96_GAMMA == 1 || CS96_BETA == 1
		log_info("get in gray mode");
		producer_get_gray(udp_instance, driver_slot_num);
#else
		log_error("gray mode is not supported on this model!");
#endif
		break;
	default:
		log_error("work mode error!");
		break;
	}

end:
#if LOCAL_TEST == 1
	if (fp != NULL) {
		fclose(fp);
		fp = NULL;
	}
	Free((void **)&file_node);
#endif
	Free((void **)&data_buf);
	pthread_exit(0);
}
