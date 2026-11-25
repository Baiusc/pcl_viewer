/*
 * @Author       : kaihong.yang kaihong.yang@vitalchem.com
 * @Date         : 2025-07-08 19:10:35
 * @LastEditors  : kaihong.yang kaihong.yang@vitalchem.com
 * @LastEditTime : 2026-04-08 23:57:51
 * @FilePath     : /you_see_arm/ys_business/producer/point_cloud/ys_producer_point_cloud_cs96_beta.c
 * @Description  : 点云生产者
 * Copyright (c) 2025 by kaihong.yang, email: kaihong.yang@vitalchem.com, All Rights Reserved.
 */
#include "ys_producer_point_cloud_cs96_beta.h"

#include "../../../core/ys_base.h"
#include "../../../core/ys_conf.h"
#include "../../../core/ys_core_system.h"
#include "../../decode/ys_decode.h"

#include "yousee_lidar_api_public.h"

#include <sched.h>

static atomic_uint_fast8_t g_run_flag; // 线程运行标识

typedef struct {
	int face;
	uint16_t last_slot;
} beta_scan_face_t;

static bool beta_scan_sequence(beta_scan_face_t *scan, uint8_t *batch, int position, int batch_slots, uint16_t slot,
			       uint16_t *sequence)
{
	*sequence = (uint16_t)position;
	if (config.work_mode != FORMAL_POINT_CLOUD_MODE)
		return true;

	/* 同步槽的角度和面号无效；漏掉同步槽时也要识别槽号回绕。 */
	if (slot == 0 || (int)scan->last_slot - slot > total_slots / 2)
		scan->face = -1;
	scan->last_slot = slot;
	if (scan->face < 0) {
		for (int i = position; i < batch_slots; ++i) {
			org_point_cloud_packet_t *candidate = (void *)(batch + i * org_point_cloud_slot_size);
			uint16_t candidate_slot = cs96_beta_get_slot_number(candidate);
			if (candidate_slot == 0 && i != position)
				break;
			if (candidate_slot > 0 && candidate_slot < total_slots) {
				scan->face = cs96_beta_resolve_scan_face(candidate);
				break;
			}
		}
		if (scan->face < 0)
			return false;
	}
	*sequence |= CS96_BETA_SCAN_FACE_VALID | ((uint16_t)scan->face << CS96_BETA_SCAN_FACE_SHIFT);
	return true;
}

#if LOCAL_TEST == 1
static char *raw_point_cloud_data_path = "./file/point_cloud_data";
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
	beta_scan_face_t scan = { .face = -1 };

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

		int batch_slots = read_size / org_point_cloud_slot_size;
		for (int i = 0; i < batch_slots; i++) {
			uint8_t *data = (uint8_t *)(file_node + i * org_point_cloud_slot_size);
			uint16_t slot_number = combine_uint8_to_uint16(data[1], data[0]) & 0x0FFF;

			// 当 slot 号为 0 的时候，帧 id 自增
			if (slot_number == 0) {
				++index; // 0~65535
			} else if (slot_number == sleep_slot) {
				sleep_flag = 1;
			}
			uint16_t sequence;
			if (!beta_scan_sequence(&scan, file_node, i, batch_slots, slot_number, &sequence))
				continue;

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
			buf->seq_num = sequence;
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
	beta_scan_face_t scan = { .face = -1 };
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
					log_info("[HDR] slot=%u byte14(帧id)=0x%02X mirror=0x%05X", slot_number,
						 data[14],
						 combine_uint8_to_uint32(data[3], data[2], data[1], data[0]) >> 12);
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
			uint16_t sequence;
			if (!beta_scan_sequence(&scan, node, i, driver_slot_num, slot_number, &sequence))
				continue;

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
			buf->seq_num = sequence;
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
