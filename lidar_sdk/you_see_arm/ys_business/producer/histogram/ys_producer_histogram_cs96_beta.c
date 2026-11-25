/*
 * @Author       : kaihong.yang kaihong.yang@vitalchem.com
 * @Date         : 2025-07-08 19:10:30
 * @LastEditors  : kaihong.yang kaihong.yang@vitalchem.com
 * @LastEditTime : 2026-04-08 23:59:08
 * @FilePath     : /you_see_arm/ys_business/producer/histogram/ys_producer_histogram_cs96_beta.c
 * @Description  : 直方图生产者
 * Copyright (c) 2025 by kaihong.yang, email: kaihong.yang@vitalchem.com, All Rights Reserved.
 */
#include "ys_producer_histogram_cs96_beta.h"

#include "../../../core/ys_base.h"
#include "../../../core/ys_conf.h"
#include "../../../core/ys_core_system.h"

static atomic_uint_fast8_t g_run_flag; // 线程运行标识

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
	while (atomic_load(&(g_run_flag)) == RUNNING) {
		sleep(1);
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

	uint16_t pre_slot_number = 0; // 上一个 slot 的 slot number

	uint32_t pkt_total = 0; // 验证日志：从驱动收到的 slot 包总数
	uint32_t pkt_since_inc = 0; // 验证日志：距离上一次 frame_id 自增收到的 slot 包数

	while (atomic_load(&(g_run_flag)) == RUNNING) {
		node = lidar_get_bufaddr();
		if (!node) {
			usleep(100);
			continue;
		}
		// 一包为 98312 字节 前 8 字节为包头 后面是数据 一包有 96 个大点 一个大点是 512 个小点
		// 直方图一帧分 56 个包，从驱动中取出来是 1 个 slot
		raw_packet_t *raw_head = (raw_packet_t *)((uint8_t *)node);
		uint8_t *raw_hdr = (uint8_t *)node;

		uint16_t old_index = index;
		uint16_t old_pre_slot_number = pre_slot_number;

		if (raw_head->slot_number == 0 && pre_slot_number == 55) {
			// 原始直方图模式：一帧由 56 个 slot 组成，slot 号从 55 回到 0 时帧号自增
			if (++index >= 65535) {
				index = 0;
			}
		}
		pre_slot_number = raw_head->slot_number;

		// ===== 验证日志：观察每个 slot 包与 frame_id(index) 的对应关系 =====
		pkt_total++;
		pkt_since_inc++;
		log_info("[HIST_FRAME] pkt#%u slot=%u pre_slot=%u frame_id=%u hdr=%02X%02X%02X%02X%02X%02X%02X%02X",
			 pkt_total, raw_head->slot_number, old_pre_slot_number, index, raw_hdr[0], raw_hdr[1],
			 raw_hdr[2], raw_hdr[3], raw_hdr[4], raw_hdr[5], raw_hdr[6], raw_hdr[7]);
		if (index != old_index) {
			log_info(
				"[HIST_FRAME] frame_id 自增: %u -> %u (触发包 slot=%u pre_slot=%u), 距上次自增收到 %u 个 slot 包",
				old_index, index, raw_head->slot_number, old_pre_slot_number, pkt_since_inc);
			pkt_since_inc = 0;
		}
		// ===== 验证日志结束 =====

		uint8_t *data = NULL;
		static uint8_t tmp[512 * sizeof(uint16_t) * 4];

		for (int k = 0, seq_num = 0; k < 96; k += 4, seq_num++) {
			do {
				p_node = (node_t *)allocate_memory(udp_instance->pool);
				if (atomic_load(&(g_run_flag)) != RUNNING) {
					return;
				}
			} while (p_node == NULL || p_node->data == NULL || p_node->data->buf == NULL);

			raw_protocal_data_packet_t *buf = (raw_protocal_data_packet_t *)(p_node->data->buf);
			buf->sof = 0xAA;
			buf->frame_id = index;
			buf->slot_number = raw_head->slot_number;
			// 纵坐标，原始取值范围是 0~95，也就是 96 线，由于每次上传的是四个点，所以取值范围变为 0~23
			seq_num %= 24;
			buf->seq_num = seq_num; // 0 ~ 23
			buf->data_len = SLICES_HISTOGRAM_LEN;
			data = raw_head->value + (k * org_histogram_pixel_size);
			memcpy(tmp, data, SLICES_HISTOGRAM_LEN);
			memcpy(buf->value, tmp, SLICES_HISTOGRAM_LEN);
			atomic_store(&(p_node->write_ok), READABLE);
			usleep(5);
		}
		usleep(10);
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
