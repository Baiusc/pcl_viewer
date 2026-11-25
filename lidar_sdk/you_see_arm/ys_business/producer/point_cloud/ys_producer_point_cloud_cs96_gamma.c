/*
 * @Author       : kaihong.yang kaihong.yang@vitalchem.com
 * @Date         : 2026-10-09
 * @FilePath     : /you_see_arm/ys_business/producer/point_cloud/ys_producer_point_cloud_cs96_gamma.c
 * @Description  : CS96 Gamma 点云生产者（SL100 SPAD, 每 SLOT 120 像素, 1936 字节）
 * Copyright (c) 2026 by Vitalchem, All Rights Reserved.
 */
#include "ys_producer_point_cloud_cs96_gamma.h"

#include "../../../core/ys_base.h"
#include "../../../core/ys_conf.h"
#include "../../../core/ys_core_system.h"

#include "yousee_lidar_api_public.h"

#include <sched.h>

static atomic_uint_fast8_t g_run_flag; // 线程运行标识

#if LOCAL_TEST == 1
static char *raw_point_cloud_data_path = "./file/CS96-gamma/point_cloud_data";
#endif

void producer_exit_point_cloud()
{
	atomic_store(&g_run_flag, STOPPED);
	return;
}

#if LOCAL_TEST == 1
/**
 * @description  : 从文件中读取点云数据
 * @param         {udp_t} *udp_instance
 * @return        {}
 */
static void producer_get_point_cloud_from_file(udp_t *udp_instance, int driver_slot_num)
{
	FILE *fp = NULL;
	uint8_t *file_node = NULL;
	node_t *p_node = NULL;
	uint16_t index = 0; // 帧标识符

	fp = fopen(raw_point_cloud_data_path, "r");
	if (fp == NULL) {
		log_error("fopen %s error", raw_point_cloud_data_path);
		goto end;
	} else {
		log_info("fopen %s success", raw_point_cloud_data_path);
	}

	file_node = (uint8_t *)Malloc(org_point_cloud_slot_size * driver_slot_num);
	if (file_node == NULL) {
		goto end;
	}

	int sleep_flag = 0;
	int sleep_slot = total_slots - driver_slot_num;
	while (atomic_load(&(g_run_flag)) == RUNNING) {
		int read_size = fread(file_node, 1, org_point_cloud_slot_size * driver_slot_num, fp);
		if (read_size == 0 && feof(fp)) {
			rewind(fp);
			continue;
		}

		for (int i = 0; i < driver_slot_num; i++) {
			uint8_t *data = (uint8_t *)(file_node + i * org_point_cloud_slot_size);
			uint16_t slot_number = combine_uint8_to_uint16(data[1], data[0]) & 0x0FFF;

			// 当 slot 号为 0 的时候，帧 id 自增
			if (slot_number == 0) {
				++index; // 0~65535
			} else if (slot_number == sleep_slot) {
				sleep_flag = 1;
			}

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
			buf->slot_number = slot_number;
			buf->seq_num = i;
			buf->data_len = org_point_cloud_slot_size;
			Memcpy(buf->value, data, org_point_cloud_slot_size);
			atomic_store(&(p_node->write_ok), READABLE);
			usleep(60);
		}

		if (sleep_flag) {
			sleep_flag = 0;
			usleep(config.frame_delay);
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
 * @description  : 从驱动中读取点云数据
 * @param         {udp_t} *udp_instance
 * @return        {}
 */
static void producer_get_point_cloud_from_driver(udp_t *udp_instance, int driver_slot_num)
{
	void *node = NULL;
	node_t *p_node = NULL;
	uint16_t index = 0; // 帧标识符
	uint8_t *tmp = (uint8_t *)Malloc(org_point_cloud_slot_size);
	if (tmp == NULL) {
		return;
	}

	int sleep_flag = 0;
	int sleep_slot = total_slots - driver_slot_num;
	int slot_cnt = 0;
	int slot0_seen = 0;
	while (atomic_load(&(g_run_flag)) == RUNNING) {
		node = lidar_get_bufaddr();
		if (!node) {
			usleep(LINK_ADD_INTERVAL);
			continue;
		}

		for (int i = 0; i < driver_slot_num; i++) {
			uint8_t *data = (uint8_t *)((uint8_t *)node + i * org_point_cloud_slot_size);
			uint16_t slot_number = combine_uint8_to_uint16(data[1], data[0]) & 0x0FFF;

			if (slot_number <= 1) {
				static int s_hdr_dump = 0;
				if (s_hdr_dump < 4) {
					s_hdr_dump++;
					log_info("[HDR] slot=%u byte14(帧id)=0x%02X mirror=0x%05X rel_time=%u",
						 slot_number, data[14],
						 combine_uint8_to_uint32(data[3], data[2], data[1], data[0]) >> 12,
						 combine_uint8_to_uint32(0x00, data[6], data[5], data[4]));
				}
			}

			slot_cnt++;
			if (slot_number == 0) {
				if (slot0_seen > 0 && slot_cnt != total_slots) {
					log_warn("[Producer] slot0 错位！预期每面 %d slot，实际 %d，index=%d",
						 total_slots, slot_cnt, index);
				}
				++index;
				slot_cnt = 0;
				slot0_seen++;
			} else if (slot_number == sleep_slot) {
				sleep_flag = 1;
			}

			p_node = (node_t *)try_allocate_memory(udp_instance->pool);
			if (p_node == NULL || p_node->data == NULL || p_node->data->buf == NULL) {
				// 池满：重试 3 次（不 sleep），仍失败则丢弃当前 slot
				for (int retry = 0; retry < 3; retry++) {
					if (atomic_load(&(g_run_flag)) != RUNNING) {
						return;
					}
					p_node = (node_t *)try_allocate_memory(udp_instance->pool);
					if (p_node != NULL && p_node->data != NULL && p_node->data->buf != NULL) {
						break;
					}
				}
				if (p_node == NULL || p_node->data == NULL || p_node->data->buf == NULL) {
					sched_yield(); // 池持续满，让出 CPU 给消费者
					continue; // 放弃当前 slot，避免自旋导致后续数据丢失
				}
			}

			raw_protocal_data_packet_t *buf = (raw_protocal_data_packet_t *)(p_node->data->buf);
			buf->sof = 0xAA;
			buf->frame_id = index;
			buf->slot_number = slot_number;
			buf->seq_num = i;
			buf->data_len = org_point_cloud_slot_size;
			memcpy(tmp, data, org_point_cloud_slot_size);
			memcpy(buf->value, tmp, org_point_cloud_slot_size);
			atomic_store(&(p_node->write_ok), READABLE);
		}

		if (sleep_flag) {
			sleep_flag = 0;
			usleep(config.frame_delay);
		}
	}

	if (tmp) {
		free(tmp);
	}
	return;
}
#endif // LOCAL_TEST

void producer_get_point_cloud(udp_t *udp_instance, int driver_slot_num)
{
	atomic_init(&g_run_flag, RUNNING);

	if (udp_instance == NULL) {
		return;
	}

#if LOCAL_TEST == 1
	producer_get_point_cloud_from_file(udp_instance, driver_slot_num);
#else
	producer_get_point_cloud_from_driver(udp_instance, driver_slot_num);
#endif

	return;
}
