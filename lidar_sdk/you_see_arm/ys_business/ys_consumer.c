/*
 * @Author       : kaihong.yang kaihong.yang@vitalchem.com
 * @Date         : 2025-02-18 08:02:34
 * @LastEditors  : kaihong.yang kaihong.yang@vitalchem.com
 * @LastEditTime : 2025-06-24
 * @FilePath     : /you_see_arm/ys_business/ys_consumer.c
 * @Description  : 消费者线程入口 - 调度层，具体流水线逻辑见 consumer/ys_consumer_pipeline.c
 * Copyright (c) 2025 by Vitalchem, All Rights Reserved.
 */

#define _GNU_SOURCE

#include "ys_consumer.h"

#include "consumer/ys_consumer_pipeline.h"
#if MF150_FL6532 == 1
#include "producer/range/ys_producer_range_mf150_fl6532.h"
#endif

#include "../core/ys_conf.h"

#include <sched.h>

int init_consumer_parm(int lines_v, int echo_mode)
{
	return init_consumer_pipeline_parm(lines_v, echo_mode);
}

void *phtread_consumer(void *arg)
{
	log_debug("Thread is running on CPU %d", sched_getcpu());

	if (arg == NULL) {
		log_error("parameter is NULL");
		goto end;
	}
	thread_t *th_arg = (thread_t *)arg;

	if (th_arg->thread_arg == NULL) {
		log_error("thread arg is NULL");
		goto end;
	}
	udp_t *udp_instance = (udp_t *)(th_arg->thread_arg);

	consumer_pipeline_reset_run_flag();

	if (th_arg->signal_enable == ON) {
		log_debug("listen signal no is %d", th_arg->signum);
		signal(th_arg->signum, consumer_pipeline_sighandler);
	}

	int work_mode = FORMAL_POINT_CLOUD_MODE;
	pthread_mutex_lock(&(config_lock));
	work_mode = config.work_mode;
	pthread_mutex_unlock(&(config_lock));

	switch (work_mode) {
	case RAW_MODE:
		log_info("get in raw histogram mode");
		consumer_pipeline_raw_histogram(udp_instance);
		break;
	case TEST_POINT_CLOUD_MODE:
#if MF150_FL6532 == 1
		// FL6532: work_mode=1 原始(Range)数据, decode 重组为 fs 协议帧(0x2E)发送
		log_info("get in raw range mode");
#if RANGE_MEASURE_ONLY == 1
		log_info("RANGE_MEASURE_ONLY=1: consumer 已禁用, 仅测量数据速率");
		break; /* 测量模式: 不启动消费者流水线 */
#else
		consumer_pipeline_real_point_cloud(udp_instance);
#endif
#else
		log_info("get in raw point cloud mode");
		consumer_pipeline_raw_point_cloud(udp_instance);
#endif
		break;
	case FORMAL_POINT_CLOUD_MODE:
		log_info("get in real point cloud mode");
		consumer_pipeline_real_point_cloud(udp_instance);
		break;
	case GRAY_MODE:
		log_info("get in gray mode");
		consumer_pipeline_raw_gray(udp_instance);
		break;
	default:
		log_error("work mode error!");
		break;
	}

end:
	consumer_pipeline_cleanup();
	pthread_exit(0);
}
