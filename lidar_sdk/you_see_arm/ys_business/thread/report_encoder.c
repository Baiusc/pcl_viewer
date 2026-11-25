/*
 * @Author       : kaihong.yang kaihong.yang@vitalchem.com
 * @Date         : 2025-09-08 19:36:27
 * @LastEditors  : kaihong.yang kaihong.yang@vitalchem.com
 * @LastEditTime : 2025-09-08 23:11:02
 * @FilePath     : /you_see_arm/ys_business/thread/report_encoder.c
 * @Description  :
 * Copyright (c) 2025 by kaihong.yang, email: kaihong.yang@vitalchem.com, All Rights Reserved.
 */
#if CS96_ALPHA == 1 || CS96_BETA == 1 || CS96_GAMMA == 1

#define _GNU_SOURCE

#include "../ys_def.h"
#include "../ys_msg_distribute.h"
#include "../ys_pl_config.h"
#include "../ys_state_machine.h"

#include "../../common/ys_protocal.h"

#include "../../core/ys_base.h"
#include "../../core/ys_version.h"

#include <errno.h>
#include <time.h>
#include <sys/timerfd.h>
#include <sys/epoll.h>
#include <fcntl.h>

#if CS96_BETA == 1 || CS96_GAMMA == 1
#define TIMER_INTERVAL_NS 1 * 1000 * 1000
#define MAX_ENCODER_CNT 240
#else
#define TIMER_INTERVAL_NS 200000
#define MAX_EVENTS 5
#define MAX_ENCODER_CNT 400
#endif

#pragma pack(push, 1)
#if CS96_BETA == 1 || CS96_GAMMA == 1
typedef struct {
	uint16_t cnt;
	encoder_info_t value[MAX_ENCODER_CNT];
} report_encoder_t;
#else
typedef struct {
	uint16_t cnt;
	uint16_t value[MAX_ENCODER_CNT];
} report_encoder_t;
#endif
#pragma pack(pop)

static atomic_uint_fast8_t g_run_flag;

// static int timer_fd;
// static int epoll_fd;
// struct epoll_event events[MAX_EVENTS];
static report_encoder_t report_encoder_data;

/**
 * @description  : exit signal handler
 * @return        {int} signum
 */
static void exit_sighandler(int signum)
{
	log_debug("get encoder sighandler capture the signal %d", signum);
	atomic_store(&g_run_flag, STOPPED);
}

#if 0
// 设置文件描述符为非阻塞模式
static int set_nonblocking(int fd)
{
	int flags = fcntl(fd, F_GETFL, 0);
	if (flags == -1) {
		perror("fcntl F_GETFL");
		return -1;
	}

	if (fcntl(fd, F_SETFL, flags | O_NONBLOCK) == -1) {
		perror("fcntl F_SETFL O_NONBLOCK");
		return -1;
	}

	return 0;
}

static void timerfd_init()
{
	// 创建 timerfd
	timer_fd = timerfd_create(CLOCK_MONOTONIC, 0);
	if (timer_fd == -1) {
		perror("timerfd_create");
		exit(EXIT_FAILURE);
	}

	// 设置 timerfd 为非阻塞
	if (set_nonblocking(timer_fd) == -1) {
		close(timer_fd);
		exit(EXIT_FAILURE);
	}

	// 设置定时器参数
	struct itimerspec timer_spec;
	memset(&timer_spec, 0, sizeof(timer_spec));

	// 初始超时：设为 0 表示立即触发
	timer_spec.it_value.tv_sec = 0;
	timer_spec.it_value.tv_nsec = 1; // 立即开始

	// 周期性超时：每 100 微秒
	timer_spec.it_interval.tv_sec = 0;
	timer_spec.it_interval.tv_nsec = TIMER_INTERVAL_NS;

	// 启动定时器
	if (timerfd_settime(timer_fd, 0, &timer_spec, NULL) == -1) {
		perror("timerfd_settime");
		close(timer_fd);
		exit(EXIT_FAILURE);
	}

	// 创建 epoll 实例
	epoll_fd = epoll_create1(0);
	if (epoll_fd == -1) {
		perror("epoll_create1");
		close(timer_fd);
		exit(EXIT_FAILURE);
	}

	// 向 epoll 添加 timerfd
	struct epoll_event ev;
	ev.events = EPOLLIN | EPOLLET; // 边缘触发
	ev.data.fd = timer_fd;

	if (epoll_ctl(epoll_fd, EPOLL_CTL_ADD, timer_fd, &ev) == -1) {
		perror("epoll_ctl");
		close(timer_fd);
		close(epoll_fd);
		exit(EXIT_FAILURE);
	}

	printf("Starting timer with 100us interval...\n");
}
#endif

void *pthread_report_encoder(void *arg)
{
	log_debug("Thread is running on CPU %d", sched_getcpu());

	atomic_init(&g_run_flag, RUNNING);

	management_id_t msg = { 0 };
	msg.avail_state = AWAYS_AVAILABLE;
	msg.code_set = CMD_SET_DEBUG;
	msg.code_id = CMD_ID_REPORT_ENCODER;
	msg.version = ys_version_protocol_major();
	msg.silent = NO_SILENT;

	if (arg == NULL) {
		log_error("parameter is NULL"); /* err_quit */
		goto end;
	}
	thread_t *th_arg = (thread_t *)arg;

	if (th_arg->signal_enable == ON) {
		log_debug("listen signal no is %d", th_arg->signum);
		signal(th_arg->signum, exit_sighandler);
	}

#if 0
	timerfd_init();

	while (atomic_load(&g_run_flag) == RUNNING) {
		int nfds = epoll_wait(epoll_fd, events, MAX_EVENTS, -1);
		if (nfds == -1) {
			if (errno == EINTR) {
				log_error("epoll_wait");
				continue; // 被信号中断，继续
			}
			break;
		}

		// 读取 timer_fd 以重置定时器
		uint64_t exp;
		ssize_t s = read(timer_fd, &exp, sizeof(uint64_t));
		if (s != sizeof(uint64_t)) {
			log_error("read timer_fd");
			break;
		}

		report_encoder_data.value[report_encoder_data.cnt++] = get_encoder(&msg);
		if (report_encoder_data.cnt >= MAX_ENCODER_CNT) {
			pack(&msg, (uint8_t *)&report_encoder_data, sizeof(report_encoder_t), MSG);
			memset(&report_encoder_data, 0, sizeof(report_encoder_data));
		}
	}
#else
	struct timespec next_time;
	clock_gettime(CLOCK_MONOTONIC, &next_time);

	while (atomic_load(&g_run_flag) == RUNNING) {
#if CS96_BETA == 1 || CS96_GAMMA == 1
		if (get_encoder(&report_encoder_data.value[report_encoder_data.cnt]) == RET_OK) {
			report_encoder_data.cnt++;
		}
#else
		report_encoder_data.value[report_encoder_data.cnt++] = get_encoder(&msg);
#endif
		if (report_encoder_data.cnt >= MAX_ENCODER_CNT) {
			pack(&msg, (uint8_t *)&report_encoder_data, sizeof(report_encoder_t), MSG);
			memset(&report_encoder_data, 0, sizeof(report_encoder_data));
		}

		next_time.tv_nsec += TIMER_INTERVAL_NS;
		if (next_time.tv_nsec >= 1000000000) {
			next_time.tv_sec++;
			next_time.tv_nsec -= 1000000000;
		}

		clock_nanosleep(CLOCK_MONOTONIC, TIMER_ABSTIME, &next_time, NULL);
	}
#endif

end:
	// if (timer_fd) {
	// 	close(timer_fd);
	// }

	// if (epoll_fd) {
	// 	close(epoll_fd);
	// }
	pthread_exit(0);
}
#endif
