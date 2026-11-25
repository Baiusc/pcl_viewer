/*
 * @Author       : kaihong.yang kaihong.yang@vitalchem.com
 * @Date         : 2026-10-12
 * @FilePath     : /you_see_arm/ys_business/producer/gray/ys_producer_gray_cs96_gamma.c
 * @Description  : CS96 Gamma 灰度图生产者
 *                 （SL100 SPAD, 每 SLOT 8B 头 + 120×8B 灰度像素 = 968 字节）
 *
 * 数据链路：驱动（MODE_GRY_SL100）→ 内存池 → 消费者透传 → UDP。
 * 一帧 20 个 SLOT(与驱动 DDR 的 Ping/Pong 各 20 SLOT 一致), slot 号回到 0 时帧号自增。
 * Copyright (c) 2026 by Vitalchem, All Rights Reserved.
 */
#include "ys_producer_gray_cs96_gamma.h"

#include "../../../core/ys_base.h"
#include "../../../core/ys_conf.h"
#include "../../../core/ys_core_system.h"

#include "yousee_lidar_api_public.h"

#include <sched.h>
#include <time.h>

static atomic_uint_fast8_t g_run_flag; // 线程运行标识

// 灰度图一帧的 SLOT 数（与驱动 DDR 缓冲的 20 个 SLOT 一致）
#define GRAY_SLOTS_PER_FRAME 20

#if LOCAL_TEST == 1
static char *raw_gray_data_path = "./file/CS96-gamma/gray_data";
#endif

void producer_exit_gray()
{
	atomic_store(&g_run_flag, STOPPED);
	return;
}

#if LOCAL_TEST == 1
/**
 * @description  : 从文件中读取灰度图数据
 * @param         {udp_t} *udp_instance
 * @param         {int} driver_slot_num
 * @return        {}
 */
static void producer_get_gray_from_file(udp_t *udp_instance, int driver_slot_num)
{
	FILE *fp = NULL;
	uint8_t *file_node = NULL;
	node_t *p_node = NULL;
	uint16_t index = 0; // 帧标识符

	fp = fopen(raw_gray_data_path, "r");
	if (fp == NULL) {
		log_error("fopen %s error", raw_gray_data_path);
		goto end;
	} else {
		log_info("fopen %s success", raw_gray_data_path);
	}

	file_node = (uint8_t *)Malloc(org_gray_slot_size * driver_slot_num);
	if (file_node == NULL) {
		goto end;
	}

	while (atomic_load(&(g_run_flag)) == RUNNING) {
		int read_size = fread(file_node, 1, org_gray_slot_size * driver_slot_num, fp);
		if (read_size == 0 && feof(fp)) {
			rewind(fp);
			continue;
		}

		for (int i = 0; i < driver_slot_num; i++) {
			uint8_t *data = (uint8_t *)(file_node + i * org_gray_slot_size);
			uint16_t slot_number = data[0] & 0x1F; // SLOT 序号 [4:0]

			// 当 slot 号为 0 的时候，帧 id 自增
			if (slot_number == 0) {
				++index; // 0~65535
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
			buf->data_len = org_gray_slot_size;
			Memcpy(buf->value, data, org_gray_slot_size);
			atomic_store(&(p_node->write_ok), READABLE);
			usleep(60);
		}
	}

end:
	if (fp != NULL) {
		fclose(fp);
	}
	Free((void **)&file_node);
	return;
}
#else
/**
 * @description  : 从驱动中读取灰度图数据
 * @param         {udp_t} *udp_instance
 * @param         {int} driver_slot_num
 * @return        {}
 */
static void producer_get_gray_from_driver(udp_t *udp_instance, int driver_slot_num)
{
	void *node = NULL;
	node_t *p_node = NULL;
	uint16_t index = 0; // 帧标识符
	uint8_t *tmp = (uint8_t *)Malloc(org_gray_slot_size);
	if (tmp == NULL) {
		return;
	}

	int slot_cnt = 0;
	int slot0_seen = 0;
	time_t last_warn_time = 0; // 错位告警限流(实测驱动侧偶发块相位漂移, 见协议文档 §8)
	while (atomic_load(&(g_run_flag)) == RUNNING) {
		node = lidar_get_bufaddr();
		if (!node) {
			usleep(LINK_ADD_INTERVAL);
			continue;
		}

		for (int i = 0; i < driver_slot_num; i++) {
			uint8_t *data = (uint8_t *)((uint8_t *)node + i * org_gray_slot_size);
			uint16_t slot_number = data[0] & 0x1F; // SLOT 序号 [4:0]

			slot_cnt++;
			if (slot_number == 0) {
				if (slot0_seen > 0 && slot_cnt != GRAY_SLOTS_PER_FRAME) {
					// 驱动侧偶发"块相位漂移"(读到相邻块的内容), 非应用层可修正;
					// 不影响上位机按 slot_number 重组, 故限流为每 5 秒最多一条
					time_t now = time(NULL);
					if (now - last_warn_time >= 5) {
						log_warn("[Gray] slot0 错位！预期每帧 %d slot，实际 %d，index=%d",
							 GRAY_SLOTS_PER_FRAME, slot_cnt, index);
						last_warn_time = now;
					}
				}
				++index;
				slot_cnt = 0;
				slot0_seen++;
			}

			p_node = (node_t *)try_allocate_memory(udp_instance->pool);
			if (p_node == NULL || p_node->data == NULL || p_node->data->buf == NULL) {
				// 池满：重试 3 次（不 sleep），仍失败则丢弃当前 slot
				for (int retry = 0; retry < 3; retry++) {
					if (atomic_load(&(g_run_flag)) != RUNNING) {
						Free((void **)&tmp);
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
			buf->data_len = org_gray_slot_size;
			Memcpy(tmp, data, org_gray_slot_size);
			Memcpy(buf->value, tmp, org_gray_slot_size);
			atomic_store(&(p_node->write_ok), READABLE);
		}
	}

	Free((void **)&tmp);
	return;
}
#endif // LOCAL_TEST

void producer_get_gray(udp_t *udp_instance, int driver_slot_num)
{
	atomic_init(&g_run_flag, RUNNING);

	if (udp_instance == NULL) {
		return;
	}

#if LOCAL_TEST == 1
	producer_get_gray_from_file(udp_instance, driver_slot_num);
#else
	producer_get_gray_from_driver(udp_instance, driver_slot_num);
#endif

	return;
}
