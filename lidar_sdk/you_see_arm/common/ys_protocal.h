/*
 * @Author       : yangkh kaihong.yang@vitalchem.com
 * @Date         : 2024-05-27 04:52:38
 * @LastEditors  : kaihong.yang kaihong.yang@vitalchem.com
 * @LastEditTime : 2025-02-18 08:33:16
 * @FilePath     : /you_see_arm/common/ys_protocal.h
 * @Description  :
 * Copyright (c) 2024 by Vitalchem, All Rights Reserved.
 */

#ifndef YS_PROTOCAL_H
#define YS_PROTOCAL_H

#include <stdint.h>

#define CRC16_LEN 2
#define CRC32_LEN 4

#define VERSION_1_HEAD_LEN 7 // 版本 1 的协议头长度
#define VERSION_1_ACK_LEN (VERSION_1_HEAD_LEN + CRC16_LEN + CRC32_LEN) // 版本 1 的固定 ACK 长度
#define VERSION_1_MSG_LEN (VERSION_1_HEAD_LEN + CRC16_LEN + CRC32_LEN) // 版本 1 的固定 MSG 长度

#define DATA_BODY_MAX_LEN 1400 // 数据段最大长度

#define CMD 0x00 // 指令
#define ACK 0x01 // 响应
#define MSG 0x02 // 消息

#pragma pack(push, 1)
typedef struct cmd_packet {
	uint8_t sof; // 起始字节，固定为 0xAA
	uint8_t version; // 协议版本，当前为 1
	uint16_t length; // 数据段长度，最大值：1400
	uint8_t cmd_type; // 命令类型
	uint16_t seq_num; // 数据段序列号
	uint16_t crc_16; // 包头校验码
	uint8_t *data; // 数据段
	uint32_t crc_32; // 数据段检验码
} cmd_packet_t;
#pragma pack(pop)

/**
 * @description  : 协议指令解析器析构函数
 * @return        {int} result
 * 0: success
 * other: failed
 */
int cmd_clean();

/**
 * @description  : parse command data from link via protocal
 * @param         {uint8_t} *data
 * @param         {int} data_len
 * @return        {*}
 */
int parse_cmd_data(void *arg, uint8_t *data, int data_len);

/**
 * @description  : 打包接口
 * @param         {void} *arg
 * @param         {uint8_t} *data
 * @param         {uint16_t} len
 * @param         {uint8_t} *buf
 * @param         {uint8_t} type
 * @param         {uint16_t} cnt
 * @return        {*}
 */
int proto_version_1_build(void *arg, uint8_t *data, uint16_t len, uint8_t *buf, uint8_t type, uint16_t cnt);

/**
 * @description  : pack data and send to center
 * @param         {void} *arg
 * @param         {uint8_t} *data
 * @param         {uint16_t} data_len
 * @param         {uint8_t} type
 * @return        {int} result
 *      0 : success
 *      other : fail
 */
int pack(void *arg, uint8_t *data, uint16_t data_len, uint8_t type);

#endif // YS_PROTOCAL_H
