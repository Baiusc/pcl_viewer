/*
 * @Author       : kaihong.yang kaihong.yang@vitalchem.com
 * @Date         : 2026-10-09
 * @FilePath     : /you_see_arm/ys_business/producer/histogram/ys_producer_histogram_cs96_gamma.c
 * @Description  : CS96 Gamma 直方图生产者（SL100 SPAD）
 * Copyright (c) 2026 by Vitalchem, All Rights Reserved.
 *
 * 驱动模式 2 每次返回 32B 包头 + 120×512×2B 数据。
 * FPGA 每 56 个 SLOT 才报帧中断，应用层仍将每个 SLOT 作为独立帧发送。
 */
#include "ys_producer_histogram_cs96_gamma.h"

#include "../../../core/ys_base.h"
#include "../../../core/ys_core_system.h"
#include "../../../core/ys_memory_pool.h"

#if LOCAL_TEST != 1
#include "yousee_lidar_api_public.h"
#endif

#include <stddef.h>

#define HISTOGRAM_PIXELS 120
#define HISTOGRAM_PIXELS_PER_PACKET 4
#define HISTOGRAM_BINS 512
#define HISTOGRAM_SLOT_SIZE (32 + HISTOGRAM_PIXELS * HISTOGRAM_BINS * sizeof(uint16_t))

#if CS96_GAMMA == 1
_Static_assert(offsetof(raw_packet_t, value) == 32, "SL100 直方图包头必须为 32 字节");
#endif

static atomic_uint_fast8_t g_run_flag; // 线程运行标识

void producer_exit_histogram()
{
	atomic_store(&g_run_flag, STOPPED);
	return;
}

void producer_get_histogram(udp_t *udp_instance)
{
	atomic_init(&g_run_flag, RUNNING);

	if (udp_instance == NULL || udp_instance->pool == NULL) {
		return;
	}

	// 驱动返回共享 DDR 地址，先复制完整 SLOT，避免等待内存池时被后续采集覆盖。
	uint8_t *slot = (uint8_t *)Malloc(HISTOGRAM_SLOT_SIZE);
	if (slot == NULL) {
		return;
	}
	uint16_t frame_id = 0;

#if LOCAL_TEST == 1
	const char *path = "./file/CS96-gamma/raw_histogram_data";
	FILE *fp = fopen(path, "rb");
	if (fp == NULL) {
		log_error("无法打开原始直方图回放文件 %s", path);
		goto end;
	}
#endif

	log_info("CS96-gamma 原始直方图：每 SLOT 一帧，每帧 30 包，每包 4 像素、512 bin");

	while (atomic_load(&(g_run_flag)) == RUNNING) {
#if LOCAL_TEST == 1
		size_t size = fread(slot, 1, HISTOGRAM_SLOT_SIZE, fp);
		if (size == 0 && feof(fp)) {
			rewind(fp);
			size = fread(slot, 1, HISTOGRAM_SLOT_SIZE, fp);
		}
		if (size != HISTOGRAM_SLOT_SIZE) {
			log_error("原始直方图回放数据不完整：读取 %zu 字节，预期 %zu 字节", size,
				  (size_t)HISTOGRAM_SLOT_SIZE);
			break;
		}
#else
		void *data = lidar_get_bufaddr();
		if (data == NULL) {
			usleep(100);
			continue;
		}
		memcpy(slot, data, HISTOGRAM_SLOT_SIZE);
#endif
		// 不依赖 FPGA 的 SLOT 序号或帧中断，0/3 两种直方图工作模式均逐 SLOT 自增。
		++frame_id; // uint16_t 自然回绕，取值 0～65535
		const raw_packet_t *raw = (const raw_packet_t *)slot;
		for (int seq = 0; seq < HISTOGRAM_PIXELS / HISTOGRAM_PIXELS_PER_PACKET; ++seq) {
			node_t *node = NULL;
			while (atomic_load(&g_run_flag) == RUNNING) {
				node = (node_t *)try_allocate_memory(udp_instance->pool);
				if (node != NULL) {
					break;
				}
				usleep(100);
			}
			if (node == NULL) {
				goto end;
			}

			raw_protocal_data_packet_t *packet = (raw_protocal_data_packet_t *)node->data->buf;
			packet->sof = 0xAA;
			packet->frame_id = frame_id;
			packet->slot_number = 0; // 上位机每帧仅一个 SLOT，FPGA 原始槽号不用于组帧
			packet->seq_num = seq;
			packet->data_len = SLICES_HISTOGRAM_LEN;
			memcpy(packet->value, raw->value + seq * SLICES_HISTOGRAM_LEN, SLICES_HISTOGRAM_LEN);
			node->data->len = raw_histogram_full_size;
			atomic_store(&node->write_ok, READABLE);
		}
#if LOCAL_TEST == 1
		// 回放按每帧 800ms 节奏，分段等待以响应退出信号。
		for (int i = 0; i < 80 && atomic_load(&g_run_flag) == RUNNING; ++i) {
			usleep(10000);
		}
#endif
	}

end:
#if LOCAL_TEST == 1
	if (fp != NULL) {
		fclose(fp);
	}
#endif
	Free((void **)&slot);
	return;
}
