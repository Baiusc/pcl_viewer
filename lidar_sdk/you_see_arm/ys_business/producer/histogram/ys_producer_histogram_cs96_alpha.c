/*
 * @Author       : kaihong.yang kaihong.yang@vitalchem.com
 * @Date         : 2026-04-08 22:46:22
 * @LastEditors  : kaihong.yang kaihong.yang@vitalchem.com
 * @LastEditTime : 2026-04-08 23:59:26
 * @FilePath     : /you_see_arm/ys_business/producer/histogram/ys_producer_histogram_cs96_alpha.c
 * @Description  : 直方图生产者
 * Copyright (c) 2025 by kaihong.yang, email: kaihong.yang@vitalchem.com, All Rights Reserved.
 */
#include "ys_producer_histogram_cs96_alpha.h"

#include "../../../core/ys_base.h"
#include "../../../core/ys_core_system.h"

#define ONE_FRAME_HISTOGRAM_DATA_SIZE (56 * org_histogram_slot_size)

static atomic_uint_fast8_t g_run_flag; // 线程运行标识

#if LOCAL_TEST == 1
static char *raw_histogram_data_path = "./file/raw_histogram_data";
#endif

void producer_exit_histogram()
{
	atomic_store(&g_run_flag, STOPPED);
	return;
}

#if LOCAL_TEST == 1
/**
 * @description  : 从文件中读取原始直方图数据
 * @param         {udp_t} *udp_instance
 * @return        {}
 */
static void producer_get_histogram_from_file(udp_t *udp_instance)
{
	FILE *fp = NULL;
	uint8_t *file_node = NULL;
	node_t *p_node = NULL;
	uint16_t index = 0; // 帧标识符

	fp = fopen(raw_histogram_data_path, "r");
	if (fp == NULL) {
		log_error("fopen %s error", raw_histogram_data_path);
		goto end;
	} else {
		log_debug("fopen %s success", raw_histogram_data_path);
	}

	file_node = (uint8_t *)Malloc(ONE_FRAME_HISTOGRAM_DATA_SIZE);
	if (file_node == NULL) {
		goto end;
	}

	uint8_t interval_flag = 0;
	int read_raw_size = fread(file_node, 1, ONE_FRAME_HISTOGRAM_DATA_SIZE, fp);
	log_debug("read size:%d", read_raw_size);

	while (atomic_load(&(g_run_flag)) == RUNNING) {
		if (interval_flag == 0) {
			interval_flag = 1;
			usleep(700 * 1000);
			continue;
		} else {
			interval_flag = 0;
		}

		++index; // 0~65535
		for (int i = 0; i < 56; i++) {
			raw_packet_t *raw_head = (raw_packet_t *)(file_node + i * org_histogram_slot_size);
			for (int j = 0; j < 48; j++) {
				do {
					p_node = (node_t *)allocate_memory(udp_instance->pool);
					// 如果此时 g_run_flag 已经被设置为 STOPPED，则退出循环
					// 因为消费者线程先退出，如果不作 g_run_flag 的判断，那么主进程将会一直处于 pthread_join 的状态
					if (atomic_load(&(g_run_flag)) != RUNNING) {
						goto end;
					}
				} while (p_node == NULL || p_node->data == NULL || p_node->data->buf == NULL);

				raw_protocal_data_packet_t *buf = (raw_protocal_data_packet_t *)(p_node->data->buf);
				buf->sof = 0xAA;
				buf->frame_id = index;
				buf->slot_number = raw_head->slot_number;
				buf->seq_num = j;
				buf->data_len = SLICES_HISTOGRAM_LEN;
				Memcpy(buf->value, raw_head->value + (j * SLICES_HISTOGRAM_LEN), SLICES_HISTOGRAM_LEN);
				atomic_store(&(p_node->write_ok), READABLE);
				usleep(50);
			}
		}
	}

end:
	if (fp != NULL) {
		fclose(fp);
	}
	return;
}
#else
/**
 * @description  : 从驱动中读取原始直方图数据
 * @param         {udp_t} *udp_instance
 * @return        {}
 */
static void producer_get_histogram_from_driver(udp_t *udp_instance)
{
	void *node = NULL;
	node_t *p_node = NULL;
	uint16_t index = 0; // 帧标识符

	while (atomic_load(&(g_run_flag)) == RUNNING) {
		node = lidar_get_bufaddr();
		if (!node) {
			usleep(1000 * 100);
			continue;
		}

		++index; // 0~65535
		for (int i = 0; i < 56; i++) {
			raw_packet_t *raw_head = (raw_packet_t *)((uint8_t *)node + i * org_histogram_slot_size);
			for (int j = 0; j < 48; j++) {
				do {
					p_node = (node_t *)allocate_memory(udp_instance->pool);
					// 如果此时 g_run_flag 已经被设置为 STOPPED，则退出循环
					// 因为消费者线程先退出，如果不作 g_run_flag 的判断，那么主进程将会一直处于 pthread_join 的状态
					if (atomic_load(&(g_run_flag)) != RUNNING) {
						return;
					}
				} while (p_node == NULL || p_node->data == NULL || p_node->data->buf == NULL);

				raw_protocal_data_packet_t *buf = (raw_protocal_data_packet_t *)(p_node->data->buf);
				buf->sof = 0xAA;
				buf->frame_id = index;
				buf->slot_number = raw_head->slot_number;
				buf->seq_num = j;
				buf->data_len = SLICES_HISTOGRAM_LEN;
				Memcpy(buf->value, raw_head->value + (j * SLICES_HISTOGRAM_LEN), SLICES_HISTOGRAM_LEN);
				atomic_store(&(p_node->write_ok), READABLE);
				usleep(50);
			}
		}
	}

	return;
}
#endif // LOCAL_TEST

void producer_get_histogram(udp_t *udp_instance)
{
	atomic_init(&g_run_flag, RUNNING);

	if (udp_instance == NULL) {
		return;
	}

#if LOCAL_TEST == 1
	producer_get_histogram_from_file(udp_instance);
#else
	producer_get_histogram_from_driver(udp_instance);
#endif

	return;
}
