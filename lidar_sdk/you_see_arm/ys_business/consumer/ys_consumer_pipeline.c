/*
 * @Author       : kaihong.yang kaihong.yang@vitalchem.com
 * @Date         : 2025-06-24
 * @FilePath     : /you_see_arm/ys_business/consumer/ys_consumer_pipeline.c
 * @Description  : 消费者公共流水线 - 零机型宏，纯业务逻辑
 * Copyright (c) 2025 by Vitalchem, All Rights Reserved.
 */

#include "ys_consumer_pipeline.h"
#include "ys_pipeline_io.h"

#include "models/ys_consumer_model.h"

#include "../../core/ys_base.h"
#include "../../core/ys_conf.h"
#include "../../core/ys_management.h"

#include <stdatomic.h>

static atomic_uint_fast8_t g_run_flag;
int processed_point_cloud_data_len;
uint8_t *dest_data = NULL;
uint8_t *dest_one_slot_org_data = NULL;
void consumer_pipeline_sighandler(int signum)
{
	log_debug("consumer sighandler capture the signal %d", signum);
	atomic_store(&g_run_flag, STOPPED);
}

void consumer_pipeline_reset_run_flag(void)
{
	atomic_init(&g_run_flag, RUNNING);
}

int init_consumer_pipeline_parm(int lines_v, int echo_mode)
{
	int len = consumer_model_get_processed_len(lines_v, echo_mode);
	processed_point_cloud_data_len = len;

	if (dest_data != NULL) {
		Free((void **)&dest_data);
	}
	dest_data = (uint8_t *)calloc(len, sizeof(uint8_t));

	if (dest_one_slot_org_data != NULL) {
		Free((void **)&dest_one_slot_org_data);
	}
	dest_one_slot_org_data = (uint8_t *)calloc(raw_point_cloud_full_size, sizeof(uint8_t));

	return RET_OK;
}

void consumer_pipeline_cleanup(void)
{
	Free((void **)&dest_data);
	Free((void **)&dest_one_slot_org_data);
}

/* 这里只验证 ARM 流水线包结构。硬件格式仍由经过验证的平台适配器负责确认。 */
static int management_packet_valid(const uint8_t *data, int length)
{
	if (!data || length < (int)(sizeof(raw_protocal_data_packet_t) - 1))
		return 0;
	const raw_protocal_data_packet_t *packet = (const void *)data;
	if (packet->sof != 0xAA || packet->data_len != length - (int)(sizeof(*packet) - 1))
		return 0;
	if (config.work_mode == RAW_MODE)
		return packet->slot_number < 56 && packet->seq_num < 30;
	if (config.work_mode == GRAY_MODE) {
#if CS96_BETA == 1
		return packet->slot_number < 168 && packet->seq_num < 2;
#elif CS96_GAMMA == 1
		return packet->slot_number < 20 && packet->seq_num < 10;
#else
		return 0;
#endif
	}
	return packet->slot_number < 800;
}

/**********************************************************************************************/
/*************************************原始直方图数据消费***************************************/
/**********************************************************************************************/
void consumer_pipeline_raw_histogram(udp_t *udp)
{
	uint8_t data[raw_histogram_full_size];

	pipeline_io_ctx_t io_ctx;
	pipeline_io_init(&io_ctx, udp, data, raw_histogram_full_size, 0, 0);

	uint8_t *in_buf = NULL;

	while (atomic_load(&(g_run_flag)) == RUNNING) {
		if (!pipeline_io_fetch_input(&io_ctx, &in_buf, raw_histogram_full_size)) {
			usleep(UDP_SEND_INTERVAL);
			continue;
		}
		ys_management_progress(config.work_mode, management_packet_valid(in_buf, raw_histogram_full_size));
		pipeline_io_flush_output(&io_ctx, in_buf, raw_histogram_full_size);
		usleep(UDP_SEND_INTERVAL);
	}

	pipeline_io_deinit(&io_ctx);
}

/**********************************************************************************************/
/*************************************灰度图数据消费*******************************************/
/**********************************************************************************************/
void consumer_pipeline_raw_gray(udp_t *udp)
{
	uint8_t data[raw_gray_full_size];

	pipeline_io_ctx_t io_ctx;
	pipeline_io_init(&io_ctx, udp, data, raw_gray_full_size, 0, 0);

	uint8_t *in_buf = NULL;

	while (atomic_load(&(g_run_flag)) == RUNNING) {
		if (!pipeline_io_fetch_input(&io_ctx, &in_buf, raw_gray_full_size)) {
			usleep(UDP_SEND_INTERVAL);
			continue;
		}
		// 灰度图数据不做解码，协议头 + 原始 SLOT 数据原样发送给上位机
		ys_management_progress(config.work_mode, management_packet_valid(in_buf, raw_gray_full_size));
		pipeline_io_flush_output(&io_ctx, in_buf, raw_gray_full_size);
		usleep(UDP_SEND_INTERVAL);
	}

	pipeline_io_deinit(&io_ctx);
}

/**********************************************************************************************/
/*************************************原始点云数据消费*****************************************/
/**********************************************************************************************/
void consumer_pipeline_raw_point_cloud(udp_t *udp)
{
	uint8_t data[raw_point_cloud_full_size];

	pipeline_io_ctx_t io_ctx;
	pipeline_io_init(&io_ctx, udp, data, raw_point_cloud_full_size, 0, 0);

	uint8_t *in_buf = NULL;

	while (atomic_load(&(g_run_flag)) == RUNNING) {
		if (!pipeline_io_fetch_input(&io_ctx, &in_buf, raw_point_cloud_full_size)) {
			usleep(UDP_SEND_INTERVAL);
			continue;
		}

		if (!consumer_model_point_cloud_is_valid(in_buf)) {
			ys_management_progress(config.work_mode, 0);
			pipeline_io_skip_input(&io_ctx);
			usleep(UDP_SEND_INTERVAL);
			continue;
		}

		int transform_len = consumer_model_transform_point_cloud(in_buf, dest_one_slot_org_data);
		uint8_t *send_buf = (transform_len > 0) ? dest_one_slot_org_data : in_buf;
		int send_len = (transform_len > 0) ? transform_len : raw_point_cloud_full_size;

		ys_management_progress(config.work_mode, management_packet_valid(in_buf, raw_point_cloud_full_size));
		pipeline_io_flush_output(&io_ctx, send_buf, send_len);
		usleep(UDP_SEND_INTERVAL);
	}

	pipeline_io_deinit(&io_ctx);
}

/**********************************************************************************************/
/*************************************处理后点云数据消费***************************************/
/**********************************************************************************************/
void consumer_pipeline_real_point_cloud(udp_t *udp)
{
	uint8_t data[raw_point_cloud_full_size];

	pipeline_io_ctx_t io_ctx;
	if (pipeline_io_init(&io_ctx, udp, data, raw_point_cloud_full_size, 1, processed_point_cloud_data_len) !=
	    RET_OK) {
		return;
	}

	uint8_t *in_buf = NULL;

	while (atomic_load(&(g_run_flag)) == RUNNING) {
		if (!pipeline_io_fetch_input(&io_ctx, &in_buf, raw_point_cloud_full_size)) {
			usleep(UDP_SEND_INTERVAL);
			continue;
		}

		uint8_t *out_buf = pipeline_io_get_output_buf(&io_ctx, dest_data);
		int len = consumer_model_parse_point_cloud(in_buf, out_buf);
		ys_management_progress(config.work_mode,
				       len > 0 && management_packet_valid(in_buf, raw_point_cloud_full_size));
		pipeline_io_flush_output(&io_ctx, out_buf, len);
#if !(MF150_FL6532 == 1 && RANGE_TX_LZ4)
		/* Range 压缩流数据量大(每帧约 2000 包), 发送后限速会把帧率压到 ~2fps,
		 * 故该路径全速发送; 其余模式保留原有节流 */
		usleep(UDP_SEND_INTERVAL);
#endif
	}

	pipeline_io_deinit(&io_ctx);
}
