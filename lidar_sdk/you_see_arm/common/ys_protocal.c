/*
 * @Author       : yangkh kaihong.yang@vitalchem.com
 * @Date         : 2024-05-27 04:52:33
 * @LastEditors  : kaihong.yang kaihong.yang@vitalchem.com
 * @LastEditTime : 2025-05-26 18:54:43
 * @FilePath     : /you_see_arm/common/ys_protocal.c
 * @Description  :
 * Copyright (c) 2024 by Vitalchem, All Rights Reserved.
 */

#include "common.h"
#include "ys_protocal.h"

#include "../ys_business/ys_business_network.h"
#include "../ys_business/ys_errno.h"
#include "../ys_business/ys_msg_distribute.h"

#include "../core/ys_base.h"

#define UNKNOW_DATA -2

int recv_data_len = 0;
uint8_t *recv_data = NULL;
static uint8_t ack_buf[ACK_BUF_MAX_LEN] = { 0 };

int cmd_clean()
{
	Free((void **)&recv_data);
	recv_data_len = 0;
	return RET_OK;
}

static int cmd_distribution(uint8_t cmd_type, uint8_t version, uint8_t *data, int data_len)
{
	switch (cmd_type) {
	case CMD:
		trigger_event(version, data, data_len);
		break;
	case ACK:
		/* code */
		break;
	case MSG:
		/* code */
		break;
	default:
		log_error("unknow cmd type");
		return RET_FAIL;
	}

	return RET_OK;
}

/**
 * @description  : parse command data from link via protocal
 * @param         {uint8_t} *data
 * @param         {int} data_len
 * @return        {*}
 */
int parse_cmd_data(void *arg, uint8_t *data, int data_len)
{
	cmd_packet_t *cmd = (cmd_packet_t *)data;

	// 协议 1 报文头
	if (data[1] == 0x01 && data[0] == 0xaa) {
		uint8_t data_head[7];
		memcpy(data_head, data, VERSION_1_HEAD_LEN);

		uint16_t crc_16 = crc16_checkout(data_head, VERSION_1_HEAD_LEN);

		if (crc_16 != cmd->crc_16) {
			log_error("cmd data head crc16 checkout fail,%x,%x", crc_16, cmd->crc_16);
			return RET_FAIL;
		}

		int cmd_len = (int)cmd->length;
		uint8_t data_body[DATA_BODY_MAX_LEN];
		memcpy(data_body, data + 9, cmd_len);

		uint32_t crc_32 = crc32_checkout(data_body, cmd_len);
		uint32_t data_crc_32 = combine_uint8_to_uint32(data[data_len - 1], data[data_len - 2],
							       data[data_len - 3], data[data_len - 4]);

		if (crc_32 != data_crc_32) {
			log_error("cmd data body crc32 checkout fail,%x,%x", crc_32, data_crc_32);
			return RET_FAIL;
		}

		// 多包组合逻辑
		if (cmd->seq_num > 0 || (cmd->seq_num == 0 && cmd->length >= DATA_BODY_MAX_LEN)) {
			// 保存分段数据
			recv_data_len += cmd->length;
			if (recv_data == NULL) {
				recv_data = (uint8_t *)Malloc(recv_data_len);
			} else {
				uint8_t *tmp = (uint8_t *)realloc(recv_data, recv_data_len);
				if (tmp == NULL) {
					log_error("realloc failed");
					cmd_clean();
					return RET_FAIL;
				}

				recv_data = tmp;
			}
			memcpy(recv_data + (recv_data_len - cmd->length), data_body, cmd->length);

			if (cmd->length < DATA_BODY_MAX_LEN) {
				cmd_distribution(cmd->cmd_type, cmd->version, recv_data, recv_data_len);

				cmd_clean();
				return RET_OK;
			} else {
				return RET_OK;
			}
		} else {
			cmd_distribution(cmd->cmd_type, cmd->version, data_body, cmd_len);
		}

		return RET_OK;
	} else {
		log_error("cmd data head error");
		return UNKNOW_DATA;
	}
}

int proto_version_1_build(void *arg, uint8_t *data, uint16_t len, uint8_t *buf, uint8_t type, uint16_t cnt)
{
	management_id_t *msg = (management_id_t *)arg;

	/* Head */
	cmd_head_packet_t head_packet = { 0 };

	head_packet.sof = 0xAA;
	head_packet.version = msg->version;
	head_packet.length = 2 + len; // 加上 cmd_set 和 cmd_id 的长度
	head_packet.cmd_type = type;
	head_packet.seq_num = cnt;
	head_packet.crc_16 = crc16_checkout((uint8_t *)&head_packet, VERSION_1_HEAD_LEN);

	/* Body */
	cmd_body_packet_t body_packet = { 0 };

	if (len > sizeof(body_packet.ack_data)) {
		log_error("data too long!");
		return RET_FAIL;
	}

	body_packet.code_set = msg->code_set;
	body_packet.code_id = msg->code_id;
	memcpy(body_packet.ack_data, data, len);

	/* Tail */
	cmd_tail_packet_t tail_packet = { 0 };
	tail_packet.crc_32 = crc32_checkout((uint8_t *)&body_packet, 2 + len);

	/* Combine */
	int offset = 0;
	memcpy(buf + offset, (uint8_t *)&head_packet, sizeof(cmd_head_packet_t));
	offset += sizeof(cmd_head_packet_t);
	memcpy(buf + offset, (uint8_t *)&body_packet, 2);
	offset += 2;
	memcpy(buf + offset, data, len);
	offset += len;
	memcpy(buf + offset, (uint8_t *)&tail_packet, sizeof(cmd_tail_packet_t));
	offset += sizeof(cmd_tail_packet_t);

	return offset;
}

int pack(void *arg, uint8_t *data, uint16_t data_len, uint8_t type)
{
	if (arg == NULL) {
		return RET_FAIL;
	}

	int len = 0;
	int i = 0;
	int sended_len = 0;
	management_id_t *msg = (management_id_t *)arg;

	switch (msg->version) {
	case 0x01:
		for (i = 0; i < (int)(data_len / ACK_DATA_MAX_LEN); i++) {
			len = proto_version_1_build(msg, data + (i * ACK_DATA_MAX_LEN), ACK_DATA_MAX_LEN, ack_buf, type,
						    (uint16_t)i);

			/* Send */
			respond_command(ack_buf, len);
			sended_len += ACK_DATA_MAX_LEN;
		}
		// 如果还剩下没发完
		if (sended_len < data_len) {
			len = proto_version_1_build(msg, data + (i * ACK_DATA_MAX_LEN), data_len - sended_len, ack_buf,
						    type, (uint16_t)i);

			/* Send */
			respond_command(ack_buf, len);
			sended_len += (data_len - sended_len);
		}
		break;

	default:
		break;
	}

	/* Save error code */
	if (data[0] != RET_OK) {
		save_error_code(msg, data[0]);
	}

	if (sended_len != data_len) {
		return RET_FAIL;
	}
	return RET_OK;
}
