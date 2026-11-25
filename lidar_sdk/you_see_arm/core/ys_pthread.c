/*
 * @Author       : yangkh kaihong.yang@vitalchem.com
 * @Date         : 2024-05-27 21:09:15
 * @LastEditors  : kaihong.yang kaihong.yang@vitalchem.com
 * @LastEditTime : 2025-05-27 00:28:46
 * @FilePath     : /you_see_arm/core/ys_pthread.c
 * @Description  :
 * Copyright (c) 2024 by Vitalchem, All Rights Reserved.
 */

#define _GNU_SOURCE

#include "ys_pthread.h"

#include "../common/common.h"

#include <sched.h>

pthread_t create_thread(void *(*routine)(void *), void *arg, const pthread_attr_t *attr, int core_id, int priority)
{
	pthread_t tid;
	cpu_set_t cpuset;

	if (pthread_create(&tid, attr, routine, arg) != 0) {
		log_fatal("create thread fail");
		return RET_FAIL;
	}

	// 初始化 CPU 集
	CPU_ZERO(&cpuset);
	CPU_SET(core_id, &cpuset);

	// 绑定线程到指定核心
	int result = pthread_setaffinity_np(tid, sizeof(cpu_set_t), &cpuset);
	if (result != RET_OK) {
		log_error("Error setting thread affinity");
	}

	if (priority != 0) {
		struct sched_param param;
		param.sched_priority = priority;
		pthread_setschedparam(tid, SCHED_FIFO, &param);
	}

	usleep(300000);
	return tid;
}

void thpool_exit(thpool_t *thpool)
{
	// 发送信号通知线程退出
	for (int i = thpool->thpool_count - 1; i >= 0; i--) {
		if (thpool->tid[i] != 0) {
			pthread_kill(thpool->tid[i], thpool->signal_num[i]);
			usleep(100);
		}
	}

	usleep(500 * 1000); // 增加到 500ms

	// 等待所有线程正常退出
	for (int i = thpool->thpool_count - 1; i >= 0; i--) {
		if (thpool->tid[i] != 0) {
			int ret = pthread_join(thpool->tid[i], NULL);
			if (ret == 0) {
				log_debug("Thread %lu exited", thpool->tid[i]);
				thpool->tid[i] = 0;
			} else {
				log_error("pthread_join failed for thread %lu: %s", thpool->tid[i], strerror(ret));
			}
		}
	}

	pthread_mutex_lock(&(thpool->lock));
	thpool->thpool_count = 0;
	pthread_mutex_unlock(&(thpool->lock));
}
