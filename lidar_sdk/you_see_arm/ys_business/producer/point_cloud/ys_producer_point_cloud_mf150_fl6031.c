/*
 * @Author       : kaihong.yang kaihong.yang@vitalchem.com
 * @Date         : 2025-07-08 19:10:35
 * @LastEditors  : kaihong.yang kaihong.yang@vitalchem.com
 * @LastEditTime : 2026-04-08 23:57:51
 * @FilePath     : /you_see_arm/ys_business/producer/point_cloud/ys_producer_point_cloud_mf150.c
 * @Description  : 点云生产者
 * Copyright (c) 2025 by kaihong.yang, email: kaihong.yang@vitalchem.com, All Rights Reserved.
 */
#include "ys_producer_point_cloud_mf150_fl6031.h"

#include "../../../core/ys_base.h"
#include "../../../core/ys_conf.h"
#include "../../../core/ys_core_system.h"

#include "yousee_lidar_api_public.h"

static atomic_uint_fast8_t g_run_flag; // 线程运行标识

void producer_exit_point_cloud()
{
	atomic_store(&g_run_flag, STOPPED);
	return;
}

#if LOCAL_TEST == 0
/**
 * @description  : 从驱动中读取点云数据
 * @param         {udp_t} *udp_instance
 * @return        {}
 */
static void producer_get_point_cloud_from_driver(udp_t *udp_instance)
{
	int ret;
	V4l2Buffer node;
	uint16_t cnt = 0;
	uint16_t index = 0; // 帧标识符
	node_t *p_node;
	mf150_fl6031_raw_group_packet_t *buf;

	while (atomic_load(&(g_run_flag)) == RUNNING) {
		ret = v4l2_get_buffer(&node);
		if (ret) {
			continue;
		}

		for (int i = 0; i < MF150_FL6031_ONCE_PACK_N; i++) {
			if (++cnt >= MF150_FL6031_FRAME_PACK_N) {
				cnt = 0;
				++index;
			}

			do {
				p_node = (node_t *)allocate_memory(udp_instance->pool);

				// 如果此时 g_run_flag 已经被设置为 STOPPED，则退出循环
				// 因为消费者线程先退出，如果不作 g_run_flag 的判断，那么主进程将会一直处于 pthread_join 的状态
				if (atomic_load(&(g_run_flag)) != RUNNING) {
					return;
				}
			} while (p_node == NULL || p_node->data == NULL || p_node->data->buf == NULL);

			buf = (mf150_fl6031_raw_group_packet_t *)(p_node->data->buf);
			buf->sof = 0xAA;
			buf->frame_id = index;
			buf->group_id = cnt;
			buf->data_len = MF150_FL6031_PACK_SIZE;
			buf->timestamp = (uint64_t)get_microseconds_timestamp();
			memcpy(buf->raw_pixels, (uint8_t *)node.start + i * MF150_FL6031_PACK_SIZE,
			       MF150_FL6031_PACK_SIZE);
			atomic_store(&(p_node->write_ok), READABLE);
		}

		v4l2_put_buffer(&node);
	}

	return;
}
#endif // LOCAL_TEST

void producer_get_point_cloud(udp_t *udp_instance, int driver_slot_num)
{
	(void)driver_slot_num;
	atomic_init(&g_run_flag, RUNNING);

	if (udp_instance == NULL) {
		return;
	}

#if LOCAL_TEST == 0
	producer_get_point_cloud_from_driver(udp_instance);
#endif

	return;
}
