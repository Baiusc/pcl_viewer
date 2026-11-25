/*
 * @Author       : kaihong.yang kaihong.yang@vitalchem.com
 * @Date         : 2025-07-08 19:10:35
 * @LastEditors  : kaihong.yang kaihong.yang@vitalchem.com
 * @LastEditTime : 2026-04-08 23:57:51
 * @FilePath     : /you_see_arm/ys_business/producer/point_cloud/ys_producer_point_cloud_mf150.c
 * @Description  : 点云生产者
 * Copyright (c) 2025 by kaihong.yang, email: kaihong.yang@vitalchem.com, All Rights Reserved.
 */
#include "../../../core/ys_base.h"
#include "../../../core/ys_conf.h"
#include "../../../core/ys_core_def.h"
#include "../../../core/ys_core_system.h"

#include "../../../third_party/fs_sdk/sdk/net_server/inc/fs_net_protocol.h"
#include "../../../third_party/fs_sdk/sdk/src/rx_cfg/rx_cfg_loader.h"
#include "../../../third_party/fs_sdk/sdk/algo/inc/algo.h"

static atomic_uint_fast8_t g_run_flag; // 线程运行标识

// 跨 group 状态变量:

static uint16_t g_next_start_sequence = 0; // 下一个包的 data_sequence
static uint16_t g_max_sequence = 0; // 已见过的最大 sequence (用于帧 END 标记)
static int32_t g_max_group_num = 0; // 一帧虚拟 group 总数(缓存)
static int32_t g_circl_cnt = 0; // 节流计数: 前4个group不发送

/* ---- 帧级调试统计 ---- */
static uint32_t dbg_frame_pkt_total = 0; // 本帧产生的 UDP 包总数
static uint32_t dbg_frame_short_pkt = 0; // 本帧非满包数量(残差包)
static uint32_t dbg_frame_dot_total = 0; // 本帧总点数

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
	fs_lidar_data_info_t *data_queue = NULL;
	node_t *p_node;
	mf150_fl6532_raw_group_packet_t *buf;

	while (atomic_load(&(g_run_flag)) == RUNNING) {
		algo_pixel_queue_pop_data(&data_queue, 100);
		if (!data_queue) {
			continue;
		}

		/* 缓存一帧的虚拟 group 总数(fs_sdk 原版逻辑) */
		if (g_max_group_num == 0) {
			int32_t all_group_num = 0;
			const rx_parse_cfg_t *parse_cfg;
			rx_cfg_get_rx_parse(RX_LIDAR_MODE_RANGE_01, &parse_cfg);
			uint8_t frame_cfg_cnt = (uint8_t)parse_cfg->is_ab_frame + 1;
			for (int32_t ab_idx = 0; ab_idx < frame_cfg_cnt; ab_idx++) {
				if (!parse_cfg->frm_cfg[ab_idx].is_b_frame && ab_idx == 1) {
					continue;
				} else if (parse_cfg->frm_cfg[ab_idx].is_b_frame && ab_idx == 0) {
					continue;
				}
				all_group_num += parse_cfg->frm_cfg[ab_idx].group_cnt;
			}
			g_max_group_num = all_group_num;
		}

		uint16_t pixel_len = data_queue->group_info.pixel_len;
		uint32_t data_count = data_queue->group_info.pixel_count;
		uint8_t roi_index = data_queue->group_info.roi_index & 0x03;

		/* 新帧第一个group: sequence 从 0 开始, max_sequence 同步清零保证 END 标记正确
         * 注意: g_pcl_merge 模式下 vir_group_idx 恒为 0, 帧首必须用 sub_frame_idx==0 判断 */
		if (data_queue->ROI_info[roi_index].sub_frame_idx == 0) {
			/* 打印上一帧统计 */
			if (dbg_frame_pkt_total > 0) {
				log_info("[FL6532-PCL] frame stat: pkt_total=%u short_pkt=%u dot_total=%u (short=%uB)",
					 dbg_frame_pkt_total, dbg_frame_short_pkt, dbg_frame_dot_total,
					 sizeof(fs_protocol_head_t) + sizeof(fs_lidar_pcl_echo_data_t) + 7 * pixel_len);
			}
			g_next_start_sequence = 0;
			g_max_sequence = 0;
			dbg_frame_pkt_total = 0;
			dbg_frame_short_pkt = 0;
			dbg_frame_dot_total = 0;
		}

		int32_t remain_dot_num = (int32_t)data_count;
		int32_t max_dot_num =
			(FS_PROTOCOL_UDP_MTU_SIZE - sizeof(fs_protocol_head_t) - sizeof(fs_lidar_pcl_echo_data_t)) /
			pixel_len;
		if (max_dot_num == 0) {
			free(data_queue->data);
			free(data_queue);
			continue;
		}
		int32_t pkt_num = (data_count % max_dot_num) ? (data_count / max_dot_num + 1) :
							       (data_count / max_dot_num);

		log_debug("[FL6532-PCL] group: vir=%u phy=%u cnt=%u pixel_len=%u max_dot=%d pkt_num=%d sub=%u/%u",
			  data_queue->group_info.vir_group_idx, data_queue->group_info.phy_group_idx, data_count,
			  pixel_len, max_dot_num, pkt_num, data_queue->ROI_info[roi_index].sub_frame_idx,
			  data_queue->ROI_info[roi_index].sub_frame_cnt);
		for (int32_t j = 0; j < pkt_num; j++) {
			uint16_t seq = g_next_start_sequence + j;
			uint8_t frame_flag = FS_PCL_FRAME_FLAG_NORMAL;
			if ((0 == j) && (data_queue->ROI_info[roi_index].sub_frame_idx == 0)) {
				frame_flag = FS_PCL_FRAME_FLAG_START;
			} else if (j == (pkt_num - 1) && (seq == g_max_sequence)) {
				frame_flag = FS_PCL_FRAME_FLAG_END;
			}

			uint16_t dot_num = (remain_dot_num >= max_dot_num) ? (uint16_t)max_dot_num :
									     (uint16_t)remain_dot_num;
			remain_dot_num -= dot_num;

			/* 节流: circl_cnt > 3 才产生节点(保持 fs_sdk 原行为) */
			if (g_circl_cnt > 3) {
				do {
					p_node = (node_t *)allocate_memory(udp_instance->pool);
					if (atomic_load(&(g_run_flag)) != RUNNING) {
						free(data_queue->data);
						free(data_queue);
						return;
					}
				} while (p_node == NULL || p_node->data == NULL || p_node->data->buf == NULL);
				buf = (mf150_fl6532_raw_group_packet_t *)(p_node->data->buf);
				memset(buf, 0, sizeof(mf150_fl6532_raw_group_packet_t));

				buf->sof = 0xAA;
				buf->frame_id = data_queue->frm_id;
				buf->group_id = data_queue->group_info.vir_group_idx;
				buf->data_len = sizeof(fs_protocol_head_t) + sizeof(fs_lidar_pcl_echo_data_t) +
						dot_num * pixel_len;
				buf->timestamp = data_queue->timestamp_base;
				buf->data_type = data_queue->data_type;

				/* === echo 平铺字段 === */
				buf->echo_protocol_ver_minor = PROTOCOL_PCL_VER_MINOR;
				buf->echo_data_sequence = seq;
				buf->echo_frame_cnt = data_queue->frm_id;
				buf->echo_dot_num = dot_num;

				switch ((uint32_t)data_queue->data_type) {
				case DATA_TYPE_POINTCLOUD:
					buf->echo_type = FS_PCL_DIS_XYZ;
					break;
				case DATA_TYPE_CICL_DATA:
					buf->echo_type = FS_CICL_DATA;
					break;
				case DATA_TYPE_GL_DATA:
					buf->echo_type = FS_GL_DATA;
					break;
				default:
					buf->echo_type = FS_CICL_DATA;
					break;
				}
				buf->echo_peak_type_2_1 = ((data_queue->peak_type[1] & 0x0F) << 4) |
							  (data_queue->peak_type[0] & 0x0F);
				buf->echo_frame_flag = frame_flag;
				buf->echo_width = data_queue->ROI_info[roi_index].width;
				buf->echo_height = data_queue->ROI_info[roi_index].height;
				buf->echo_max_bin_cnt = data_queue->max_bin_cnt;
				buf->echo_ab_frame_idx = ((data_queue->ROI_info[roi_index].pcl_unit & 0x7F) << 1) |
							 (data_queue->ab_frame_idx & 0x01);
				buf->echo_max_sub_frame_cnt = data_queue->max_sub_frame_cnt & 0x0F;
				buf->echo_roi_index = roi_index;
				buf->echo_pixel_len = pixel_len;
				buf->echo_data_offset = 0;
				buf->echo_data_len = 0;
				buf->echo_peak_type_4_3 = ((data_queue->peak_type[3] & 0x0F) << 4) |
							  (data_queue->peak_type[2] & 0x0F);
				buf->echo_timestamp_base = data_queue->timestamp_base;
				memcpy(buf->raw_pixels, data_queue->data + (size_t)(j * max_dot_num) * pixel_len,
				       (size_t)dot_num * pixel_len);
				atomic_store(&(p_node->write_ok), READABLE);

				dbg_frame_pkt_total++;
				dbg_frame_dot_total += dot_num;
				if (buf->data_len < 1462) {
					dbg_frame_short_pkt++;
				}
				if (frame_flag != FS_PCL_FRAME_FLAG_NORMAL) {
					log_info("[FL6532-PCL] pkt: seq=%u flag=%u dot=%u len=%u", seq, frame_flag,
						 dot_num, buf->data_len);
				}
			}
		}
		// 跨 group 状态推进(顺序与原版一致: END 判断用的是更新前的max_sequence)
		g_next_start_sequence = g_next_start_sequence + (uint16_t)pkt_num;
		g_circl_cnt++;
		if (g_circl_cnt > 4) {
			g_circl_cnt = 4;
		}
		if (g_max_sequence < (g_next_start_sequence - 1)) {
			g_max_sequence = g_next_start_sequence - 1;
		}

		free(data_queue->data);
		free(data_queue);
	}
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
