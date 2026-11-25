/*
 * @Author       : yangkh kaihong.yang@vitalchem.com
 * @Date         : 2024-05-28 23:52:52
 * @LastEditors: kaihong.yang kaihong.yang@vitalchem.com
 * @LastEditTime: 2024-07-24 10:27:14
 * @FilePath     : /you_see_arm/core/ys_core_system.c
 * @Description  :
 * Copyright (c) 2024 by Vitalchem, All Rights Reserved.
 */

#include "ys_base.h"
#include "ys_conf.h"
#include "ys_core_spad.h"
#include "ys_core_system.h"

#include "../common/common.h"
#if MF150_FL6532 == 1
#include "ys_fs_sdk_boot.h"
#else
#include "yousee_lidar_api_public.h"
#endif

#include <sys/wait.h>
#if USE_KFIFO
struct kfifo g_fifobuf; // 全局的 kfifo 队列，用于存储点云数据
uint8_t *kfifo_buffer = NULL;
#endif

#if CS96_ALPHA == 1
const int org_histogram_pixel_size = 1000 * sizeof(uint16_t); // 原始直方图数据中，一个像素点的大小
#elif CS96_BETA == 1
const int org_histogram_pixel_size = 512 * sizeof(uint16_t); // 原始直方图数据中，一个像素点的大小
#elif CS96_GAMMA == 1
const int org_histogram_pixel_size = 512 * sizeof(uint16_t); // SL100 仅上传前 512 个 bin
#else
const int org_histogram_pixel_size = 1000 * sizeof(uint16_t); // 原始直方图数据中，一个像素点的大小
#endif

int org_histogram_slot_size; // 原始直方图数据中，一个 slot 的大小
int raw_histogram_full_size; // 协议中的原始直方图报文长度

int org_point_cloud_slot_size; // 原始点云数据中，一个 slot 的大小
int raw_point_cloud_full_size; // 协议中的原始点云报文长度

int org_gray_slot_size; // 原始灰度图数据中，一个 slot 的大小
int raw_gray_full_size; // 协议中的原始灰度图报文长度

static int init_api_flag = 0;

/**
 * @description  : init histogram data size and point cloud data size
 * @param         {int} lines_v
 * @return        {*}
 */
static void init_data_size(int lines_v)
{
#if CS96_GAMMA == 1
	// SL100 原始直方图固定 120 个像素，包头 32 字节，不依赖点云标定线数。
	org_histogram_slot_size = 32 + 120 * org_histogram_pixel_size;
#else
	org_histogram_slot_size = (8 + lines_v * org_histogram_pixel_size);
#endif
	raw_histogram_full_size = SLICES_HISTOGRAM_LEN + (sizeof(raw_protocal_data_packet_t) - 1);
	log_info("org histogram slot size = %d", org_histogram_slot_size);

#if MF150_FL6031 == 1
	org_point_cloud_slot_size = sizeof(mf150_fl6031_raw_group_packet_t);
	raw_point_cloud_full_size = sizeof(mf150_fl6031_raw_group_packet_t);
#elif MF150_FL6532 == 1
	org_point_cloud_slot_size = sizeof(mf150_fl6532_raw_group_packet_t);
	raw_point_cloud_full_size = sizeof(mf150_fl6532_raw_group_packet_t);
#else
	org_point_cloud_slot_size = (sizeof(org_point_cloud_packet_t) - 1) + lines_v * sizeof(point_cloud_pixel_data_t);
	raw_point_cloud_full_size = org_point_cloud_slot_size + (sizeof(raw_protocal_data_packet_t) - 1);
#endif
	log_info("org point cloud slot size = %d", org_point_cloud_slot_size);

	// 灰度图尺寸: 结构体只在机型分支定义, 故用宏表达, 保证所有机型都能算出该值
#if CS96_BETA == 1
	// Beta 灰度每 SLOT 固定 576 像素，对应驱动的 MODE_GRY。
	// 2312B 超过以太网 MTU, 拆成 GRAY_SLOT_SLICES 片, 一个 UDP 包承载一片
	org_gray_slot_size = GRAY_SLOT_HEADER_SIZE + GRAY_SLOT_PIXELS * GRAY_PIXEL_SIZE;
	raw_gray_full_size = org_gray_slot_size / GRAY_SLOT_SLICES + (sizeof(raw_protocal_data_packet_t) - 1);
#else
	org_gray_slot_size = GRAY_SLOT_HEADER_SIZE + lines_v * GRAY_PIXEL_SIZE;
	raw_gray_full_size = org_gray_slot_size + (sizeof(raw_protocal_data_packet_t) - 1);
#endif
	log_info("org gray slot size = %d, raw gray full size = %d", org_gray_slot_size, raw_gray_full_size);
}

#if USE_KFIFO
/**
 * @description  : initialize system kfifo
 * @return        {*}
 */
static int sys_kfifo_init()
{
	int kfifo_full_size = 0;
	if (config.work_mode == RAW_MODE) {
		kfifo_full_size = (raw_histogram_full_size * 56 * 10); // 初始化一个可以存储 10 帧原始直方图的 kfifo
	} else {
		kfifo_full_size = (raw_point_cloud_full_size * 800 * 10); // 初始化一个可以存储 10 帧点云的 kfifo
	}

	if (is_power_of_two(kfifo_full_size) == false) {
		kfifo_full_size = roundup_pow_of_two(kfifo_full_size);
	}
	kfifo_buffer = (uint8_t *)Malloc(kfifo_full_size);

	if (kfifo_buffer == NULL) {
		return RET_FAIL;
	}

	kfifo_init(&g_fifobuf, kfifo_buffer, kfifo_full_size);

	int n = kfifo_avail(&g_fifobuf);
	log_info("hello kfifo, avail: %d\n", n);

	if (n != kfifo_full_size) {
		log_error("kfifo init error");
		Free((void **)&kfifo_buffer);
		return RET_FAIL;
	}
	return RET_OK;
}
#endif

int check_pid()
{
	FILE *fd = NULL;
#if LOCAL_TEST == 1
	char cmd[128] = "ps -aux | grep you_see | grep -v grep | awk '{print $2}'";
#else
	char cmd[128] = "ps | grep you_see | grep -v grep | awk '$4==\"R\" || $4==\"S\" || $4==\"\" {print $1}'";
#endif
	char read_buf[16] = { '\0' };
	int ret = RET_OK;
	int pid = 0;
	int self_pid = getpid();

	log_info("self pid is %d", self_pid);

	fd = popen(cmd, "r");
	if (fd == NULL) {
		log_error("get ps error");
		return RET_FAIL;
	}

	while (fgets(read_buf, sizeof(read_buf), fd) != NULL) {
		pid = atoi(read_buf);
		if (pid == 0 || pid == self_pid) {
			continue;
		}

		// 发送第一个 SIGINT 信号
		if (kill(pid, SIGINT) == -1) {
			if (errno == ESRCH) {
				log_info("process %d does not exist", pid);
				continue;
			}
			log_error("first kill failed for pid %d, errno=%d", pid, errno);
			ret = RET_FAIL;
			continue;
		}

		// 等待一小段时间后发送第二个 SIGINT 信号
		usleep(50000); // 50ms

		// 发送第二个 SIGINT 信号
		if (kill(pid, SIGINT) == -1) {
			if (errno == ESRCH) {
				log_info("process %d already exited after first SIGINT", pid);
				continue;
			}
			log_error("second kill failed for pid %d, errno=%d", pid, errno);
			ret = RET_FAIL;
			continue;
		}

		// 等待进程退出，最多等待 5 秒
		int wait_count = 0;
		while (wait_count < 50) { // 5 秒 = 50 * 100ms
			if (kill(pid, 0) == -1) {
				if (errno == ESRCH) { // 进程已经不存在
					log_info("process %d exited successfully", pid);
					break;
				}
			}
			usleep(100 * 1000); // 睡眠 100ms
			wait_count++;
		}

		if (wait_count >= 50) {
			log_error("timeout waiting for process %d to exit", pid);
			ret = RET_FAIL;
		}
	}

	pclose(fd);
	fd = NULL;
	return ret;
}

int sys_init()
{
	init_data_size(cal_param.lines_v);

#if LOCAL_TEST == 1
	int ret = RET_OK;
#else
#if MF150_FL6031 == 1
	// lidar_api_init: mode=0是点云模式，mode=1是直方图模式
	int api_mode = config.work_mode == RAW_MODE ? 1 : 0;
	int ret = lidar_api_init(api_mode);
#elif MF150_FL6532 == 1
	// work_mode=1(原始点云模式) -> fs_sdk RANGE, 其它 -> fs_sdk PCL
	int ret = fs_sdk_boot_init(config.work_mode);
#elif CS96_BETA == 1 || CS96_GAMMA == 1
	unsigned int format;
	switch (config.work_mode) {
	case RAW_MODE:
		format = YS_FORMAT_HISTOGRAM;
		break;
	case TEST_POINT_CLOUD_MODE:
	case FORMAL_POINT_CLOUD_MODE:
		format = YS_FORMAT_POINTCLOUD;
		break;
	case GRAY_MODE:
		format = YS_FORMAT_GRAY;
		break;
	default:
		return RET_FAIL;
	}
	log_info("配置 AM6204 接收模式，work_mode=%u，驱动模式=%u", config.work_mode, format);
	int ret = lidar_api_init_mode(format);
#else
	int ret = lidar_api_init();
#endif
#endif

	if (ret) {
		log_fatal("driver init failed");
		return RET_FAIL;
	} else {
		init_api_flag = 1;
	}

#if USE_KFIFO
	ret = sys_kfifo_init();
	if (ret != RET_OK) {
		return RET_FAIL;
	}
#endif

	return RET_OK;
}

int sys_uninit()
{
#if USE_KFIFO
	Free((void **)&kfifo_buffer);
#endif

	if (init_api_flag == 1) {
		int ret = RET_OK;
#if (LOCAL_TEST == 0) && (MF150_FL6532 == 1)
		fs_sdk_boot_exit();
#elif (LOCAL_TEST == 0) && (MF150_FL6532 == 0)
		ret = lidar_api_exit();
#endif

		if (ret != RET_OK) {
			log_fatal("driver uninit failed");
			return RET_FAIL;
		}
	}

	init_api_flag = 0;
	return RET_OK;
}

int sys_api_ready()
{
	return init_api_flag;
}

int sys_spad_init(int work_mode)
{
#if LOCAL_TEST == 1
	log_info("skip spad init in LOCAL_TEST mode");
	return RET_OK;
#else
	long long pre_time, now_time;
	long long spad_init_timeout = 4 * 1000; // spad 初始化的超时时间，单位毫秒
	now_time = pre_time = get_milliseconds_timestamp();

	log_info("start init spad");
#if CS96_BETA == 1 || CS96_GAMMA == 1
	while (spad_init(work_mode) != RET_OK) {
#elif MF150_FL6031 == 1
	while (spad_init() != RET_OK) {
#else
	while (0) {
#endif
		now_time = get_milliseconds_timestamp();
		if (now_time - pre_time >= spad_init_timeout) {
			log_error("spad init timeout");
			return RET_FAIL;
		} else {
			usleep(10);
		}
	}
	log_info("finish init spad");

	return RET_OK;
#endif
}
