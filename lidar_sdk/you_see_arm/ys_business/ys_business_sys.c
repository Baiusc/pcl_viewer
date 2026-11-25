/*
 * @Author       : yangkh kaihong.yang@vitalchem.com
 * @Date         : 2024-05-28 23:34:28
 * @LastEditors: kaihong.yang kaihong.yang@vitalchem.com
 * @LastEditTime: 2024-08-25 09:41:43
 * @FilePath     : /you_see_arm/ys_business/ys_business_sys.c
 * @Description  :
 * Copyright (c) 2024 by Vitalchem, All Rights Reserved.
 */

#define _GNU_SOURCE

#include "ys_business_sys.h"
#include "../core/ys_management.h"
#include "ys_errno.h"
#include "ys_msg_distribute.h"
#include "ys_pl_config.h"
#include "ys_state_machine.h"

#include "../core/ys_base.h"
#include "../core/ys_conf.h"
#include "../core/ys_core_system.h"
#include "../core/ys_file.h"
#include "../core/ys_version.h"
#include "../core/ys_version_driver.h"
#include "../network/network.h"

#include "../common/common.h"
#include "../common/ys_protocal.h"

#include <sched.h>
#include <string.h>

void get_last_error_code(void *arg)
{
	if (arg == NULL) {
		return;
	}

	management_id_t *msg = (management_id_t *)arg;
	error_code_t ack_data = { 0 };
	int ret = RET_FAIL;

	if (get_error_code((uint8_t *)&(ack_data.value))) {
		ret = run_succ;
	} else {
		ret = run_fail;
	}

	ack_data.ret_code = (uint8_t)ret;
	pack(msg, (uint8_t *)&ack_data, sizeof(ack_data), ACK);
	return;
}

void get_sys_model(void *arg)
{
	if (arg == NULL) {
		return;
	}
	// management_id_t *msg = (management_id_t *)arg;

	// uint8_t ack_buf[ACK_BUF_MAX_LEN] = { '\0' };
}

void upgrade_fpga(void *arg)
{
	if (arg == NULL) {
		return;
	}
	// management_id_t *msg = (management_id_t *)arg;

	// uint8_t ack_buf[ACK_BUF_MAX_LEN] = { '\0' };
}

/**
 * @description  : 把采集到的组件版本填入 0x0302 的 32 字节组件记录
 * @param         {version_component_t} *dst 目标组件记录
 * @param         {uint8_t} id 组件 ID, 见 ys_component_id_t
 * @param         {const ys_component_version_t} *src 采集结果
 * @return        {*}
 */
static void fill_component(version_component_t *dst, uint8_t id, const ys_component_version_t *src)
{
	size_t sha_len = (size_t)src->sha_len;

	if (sha_len > sizeof(dst->git_sha)) {
		sha_len = sizeof(dst->git_sha);
	}

	dst->id = id;
	dst->valid = (src->valid != 0) ? 1 : 0;
	dst->major = src->major;
	dst->minor = src->minor;
	dst->patch = src->patch;
	dst->dirty = (src->dirty != 0) ? 1 : 0;
	dst->sha_len = (uint8_t)sha_len;
	memcpy(dst->git_sha, src->sha, sha_len);
	dst->build_time = (uint32_t)(src->build_time & 0xFFFFFFFFULL);
	dst->build_time_hi = (uint32_t)(src->build_time >> 32);
	return;
}

void get_sys_version(void *arg)
{
	if (arg == NULL) {
		log_error("parameter is NULL");
		return;
	}

	management_id_t *msg = (management_id_t *)arg;
	const ys_build_info_t *build = ys_version_build_info();
	version_list_t ack_data = { 0 };
	ys_component_version_t comp_ver = { 0 };
	size_t self_sha_len = 0;
	int i = 0;

	/* ---- 固件总包 ---- */
	ack_data.head.ret_code = (uint8_t)run_succ;
	ack_data.head.component_count = (uint8_t)YS_COMPONENT_MAX;
	ack_data.head.fw_major = build->fw.major;
	ack_data.head.fw_minor = build->fw.minor;
	ack_data.head.fw_patch = build->fw.patch;
	snprintf(ack_data.head.product, sizeof(ack_data.head.product), "%s", build->product);
	snprintf(ack_data.head.release_id, sizeof(ack_data.head.release_id), "%s", build->release_id);

	/* ---- 组件版本列表: 先全部按未接入占位, 再逐个填入已知组件 ---- */
	for (i = 0; i < (int)YS_COMPONENT_MAX; i++) {
		ack_data.component[i].id = (uint8_t)i;
	}

	/* you_see 应用(总成): 全部字段来自构建期固化的版本标识 */
	self_sha_len = (size_t)build->sha_len;
	if (self_sha_len > sizeof(comp_ver.sha) - 1) {
		self_sha_len = sizeof(comp_ver.sha) - 1;
	}
	comp_ver.valid = 1;
	comp_ver.major = build->fw.major;
	comp_ver.minor = build->fw.minor;
	comp_ver.patch = build->fw.patch;
	comp_ver.dirty = (build->dirty != 0) ? 1 : 0;
	memcpy(comp_ver.sha, build->sha, self_sha_len);
	comp_ver.sha_len = (uint8_t)self_sha_len;
	comp_ver.build_time = build->build_time;
	fill_component(&ack_data.component[YS_COMPONENT_YOU_SEE], (uint8_t)YS_COMPONENT_YOU_SEE, &comp_ver);

	/* ARM 驱动: liblidar.so 构建期注入的版本标识; 库无版本接口时保持未接入(valid=0) */
	if (ys_version_arm_driver(&comp_ver) == 0) {
		fill_component(&ack_data.component[YS_COMPONENT_ARM_DRIVER], (uint8_t)YS_COMPONENT_ARM_DRIVER,
			       &comp_ver);
	}

	/* FPGA: 版本寄存器(0x3000~0x3010)按 BCD 解析; 读失败时保持未接入(valid=0) */
	if (ys_version_fpga(&comp_ver) == 0) {
		fill_component(&ack_data.component[YS_COMPONENT_FPGA], (uint8_t)YS_COMPONENT_FPGA, &comp_ver);
	}

	/* 算法 / Web 服务程序两个组件待各自版本方案就绪后填入并置 valid=1。
	 * 报文格式不变, 上位机无需改动。 */

	pack(msg, (uint8_t *)&ack_data, sizeof(ack_data), ACK);
	return;
}

void upgrade_sys(void *arg)
{
	if (arg == NULL) {
		return;
	}
	// management_id_t *msg = (management_id_t *)arg;

	// uint8_t ack_buf[ACK_BUF_MAX_LEN] = { '\0' };
}

void reboot_sys(void *arg)
{
	if (arg == NULL) {
		return;
	}
	management_id_t *msg = (management_id_t *)arg;
	int ret = ys_management_enabled() ? RET_FAIL : RET_OK;

	if (ret == RET_OK) {
		ret = run_succ;
	} else {
		ret = run_fail;
	}

	pack(msg, (uint8_t *)&ret, 1, ACK);
	if (!ys_management_enabled()) {
		int status __attribute__((unused)) = system("reboot");
	}
	return;
}

void get_sys_sn(void *arg)
{
	if (arg == NULL) {
		return;
	}
	management_id_t *msg = (management_id_t *)arg;
	struct {
		uint8_t ret_code;
		uint8_t sn[NET_SN_LENGTH];
	} ack_data = { 0 };
	char sn[NET_SN_CAPACITY];

	if (net_sn_read("/dev/mtd3", sn) == 0) {
		ack_data.ret_code = (uint8_t)run_succ;
		memcpy(ack_data.sn, sn, sizeof(ack_data.sn));
	} else {
		ack_data.ret_code = (uint8_t)run_fail;
	}
	pack(msg, (uint8_t *)&ack_data, sizeof(ack_data), ACK);
}

void clean_log(void *arg)
{
	if (arg == NULL) {
		log_error("parameter is NULL");
		return;
	}

	management_id_t *msg = (management_id_t *)arg;
	int ret = RET_FAIL;
	int type = msg->buf[1];

	switch (type) {
	case 0x01:
		ret = clean_sys_log();
		break;
	case 0x03:
		ret = clean_kernel_log();
		break;
	case 0x05:
		ret = clean_app_log(config.log_file_path);
		break;
	default:
		break;
	}

	if (ret == RET_OK) {
		ret = run_succ;
	} else {
		ret = run_fail;
	}

	pack(msg, (uint8_t *)&ret, 1, ACK);
	return;
}

void check_system()
{
	int ret;
start:
	ret = check_pid();

	if (ret == RET_OK) {
		log_info("System check success");
	} else {
		log_error("System check fail");
		goto start;
	}
	return;
}

void get_single_histogram(void *arg)
{
	if (arg == NULL) {
		return;
	}

	management_id_t *msg = (management_id_t *)arg;
	shistogram_data_t ack_data = { 0 };
	int ret = RET_FAIL;

#if LOCAL_TEST == 1
	ret = RET_OK;
#elif MF150_FL6031 == 1
	ret = RET_FAIL;
#elif MF150_FL6532 == 1
	ret = RET_FAIL;
#else
	uint8_t value_u8[4];
	uint32_t value_u32[4];

	for (int i = 0; i < 4; i++) {
		for (int j = 0; j < 4; j++) {
			value_u8[j] = msg->buf[3 + j + (i * 4)];
		}
		value_u32[i] = combine_uint8_to_uint32(value_u8[0], value_u8[1], value_u8[2], value_u8[3]);
	}

	ret = shistogram_data_get(value_u32[0], value_u32[1], (void *)(ack_data.shistogram_data));
#endif

	if (ret == RET_OK) {
		ret = run_succ;
	} else {
		ret = run_fail;
	}

	ack_data.ret_code = (uint8_t)ret;
	pack(msg, (uint8_t *)&ack_data, sizeof(ack_data), ACK);
	return;
}
