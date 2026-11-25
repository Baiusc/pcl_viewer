/*
 * @Author       : pine changbo.hu@votinfrared.com
 * @Date         : 2026-10-13
 * @FilePath     : /you_see_arm/ys_business/producer/gray/ys_producer_gray_cs96_beta.c
 * @Description  : CS96 Beta 灰度图生产者
 *                 （每 SLOT 8B 头 + 576×4B 灰度像素 = 2312 字节）
 *
 * 数据链路：驱动（MODE_GRY）→ 内存池 → 消费者透传 → UDP。
 * 一帧 168 个 SLOT(与驱动 DDR 的 168 块环形缓冲一致), slot 号回到 0 时帧号自增。
 * 单个 SLOT 2312B 超过以太网 MTU, 对半拆成 2 片发送, 每片 = 9B 协议头 + 1156B,
 * seq_num 即片号(0~1), 上位机按 (frame_id, slot_number) 分组、按 seq_num 排序拼接。
 * Copyright (c) 2026 by Vitalchem, All Rights Reserved.
 */
#include "ys_producer_gray_cs96_beta.h"

#include "../../../core/ys_base.h"
#include "../../../core/ys_conf.h"
#include "../../../core/ys_core_system.h"

#include "yousee_lidar_api_public.h"

#include <sched.h>
#include <time.h>

static atomic_uint_fast8_t g_run_flag; // 线程运行标识

// 灰度图一帧的 SLOT 数（与驱动 DDR 缓冲的 168 个 SLOT 一致）
#define GRAY_SLOTS_PER_FRAME 168

// 每片承载的原始数据长度(SLOT 对半拆; 片0 = 8B 头 + 287 像素, 片1 = 289 像素, 边界天然对齐像素)
#define GRAY_SLICE_DATA_SIZE (org_gray_slot_size / GRAY_SLOT_SLICES)

#if LOCAL_TEST == 1
static char *raw_gray_data_path = "./file/CS96-beta/gray_data";
#endif

void producer_exit_gray()
{
	atomic_store(&g_run_flag, STOPPED);
	return;
}

/**
 * @description  : 把一个原始 SLOT 按 GRAY_SLOT_SLICES 拆片写入内存池
 *                 (每片一个节点/一个 UDP 包, seq_num = 片号)
 * @param         {udp_t} *udp_instance
 * @param         {const uint8_t} *slot_data 原始 SLOT 数据(org_gray_slot_size 字节)
 * @param         {uint16_t} frame_id 帧号
 * @param         {uint16_t} slot_number SLOT 序号
 * @return        {}
 */
static void gray_write_slot_to_pool(udp_t *udp_instance, const uint8_t *slot_data, uint16_t frame_id,
				    uint16_t slot_number)
{
	for (int s = 0; s < GRAY_SLOT_SLICES; s++) {
		node_t *p_node = (node_t *)try_allocate_memory(udp_instance->pool);
		if (p_node == NULL || p_node->data == NULL || p_node->data->buf == NULL) {
			// 池满：重试 3 次（不 sleep），仍失败则丢弃当前片
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
				continue; // 放弃当前片，避免自旋导致后续数据丢失
			}
		}

		raw_protocal_data_packet_t *buf = (raw_protocal_data_packet_t *)(p_node->data->buf);
		buf->sof = 0xAA;
		buf->frame_id = frame_id;
		buf->slot_number = slot_number;
		buf->seq_num = (uint16_t)s; // 片号: 0 = 前半片, 1 = 后半片
		buf->data_len = GRAY_SLICE_DATA_SIZE;
		Memcpy(buf->value, slot_data + s * GRAY_SLICE_DATA_SIZE, GRAY_SLICE_DATA_SIZE);
		atomic_store(&(p_node->write_ok), READABLE);
	}
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
	uint8_t *slot_buf = NULL;
	uint16_t index = 0; // 帧标识符

	// 回放数据按 SLOT 顺序读取, 每次处理一个 SLOT, 与驱动路径一致
	(void)driver_slot_num;

	fp = fopen(raw_gray_data_path, "r");
	if (fp == NULL) {
		log_error("fopen %s error", raw_gray_data_path);
		goto end;
	} else {
		log_info("fopen %s success", raw_gray_data_path);
	}

	slot_buf = (uint8_t *)Malloc(org_gray_slot_size);
	if (slot_buf == NULL) {
		goto end;
	}

	while (atomic_load(&(g_run_flag)) == RUNNING) {
		int read_size = fread(slot_buf, 1, org_gray_slot_size, fp);
		if (read_size < org_gray_slot_size) {
			if (feof(fp)) {
				rewind(fp);
				continue;
			}
			log_error("fread error: %d", read_size);
			break;
		}

		uint16_t slot_number = (uint16_t)((uint16_t)slot_buf[1] << 8 | slot_buf[0]); // 槽号 [15:0]

		// 当 slot 号为 0 的时候，帧 id 自增
		if (slot_number == 0) {
			++index; // 0~65535
		}

		gray_write_slot_to_pool(udp_instance, slot_buf, index, slot_number);
		usleep(5000); // 模拟实机低帧率(约 1 秒/帧), 避免回放数据瞬间塞满内存池
	}

end:
	if (fp != NULL) {
		fclose(fp);
	}
	Free((void **)&slot_buf);
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
	uint16_t index = 0; // 帧标识符

	// Beta 灰度的每次 DMA 传输只包含 1 个 SLOT，
	// 因此不按 driver_slot_num 循环(该参数由 init_producer_parm 置 1, 保留以统一接口)
	(void)driver_slot_num;

	int slot_cnt = 0;
	int slot0_seen = 0;
	time_t last_warn_time = 0; // 错位告警限流(驱动侧偶发块相位漂移, 见协议文档)
	while (atomic_load(&(g_run_flag)) == RUNNING) {
		node = lidar_get_bufaddr();
		if (!node) {
			usleep(LINK_ADD_INTERVAL);
			continue;
		}

		const uint8_t *data = (const uint8_t *)node;
		uint16_t slot_number = (uint16_t)((uint16_t)data[1] << 8 | data[0]); // 槽号 [15:0]

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

		gray_write_slot_to_pool(udp_instance, data, index, slot_number);
	}
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
