/*
 * @Author       : kaihong.yang kaihong.yang@vitalchem.com
 * @Date         : 2024-06-25 03:17:19
 * @LastEditors  : kaihong.yang kaihong.yang@vitalchem.com
 * @LastEditTime : 2025-09-08 23:41:04
 * @FilePath     : /you_see_arm/ys_business/ys_pl_config.c
 * @Description  :
 * Copyright (c) 2024 by Vitalchem, All Rights Reserved.
 */

#define _GNU_SOURCE

#include "decode/ys_decode.h"
#include "ys_business_network.h"
#include "ys_errno.h"
#include "ys_pl_config.h"

#include "../common/common.h"
#include "yousee_lidar_api_public.h"
#include "../common/ys_protocal.h"

#include "../core/ys_base.h"
#include "../core/ys_conf.h"

#if CS96_ALPHA == 1 || CS96_BETA == 1 || CS96_GAMMA == 1
void set_refresh_rate(void *arg)
{
	if (arg == NULL) {
		return;
	}

	management_id_t *msg = (management_id_t *)arg;
	int ret = RET_FAIL;

	uint8_t value_u8[4];
	int fps = 0;

	for (int i = 0; i < 4; i++) {
		value_u8[i] = msg->buf[3 + i];
	}

	fps = (int)combine_uint8_to_uint32(value_u8[3], value_u8[2], value_u8[1], value_u8[0]);
	if (fps == 10 || fps == 20) {
#if LOCAL_TEST == 1
		ret = RET_OK;
#else
		ret = sys_fps_set(fps);
#endif
	}

	if (ret == RET_OK) {
		ret = run_succ;
	} else {
		ret = run_fail;
	}

	pack(msg, (uint8_t *)&ret, 1, ACK);
	return;
}

void get_refresh_rate(void *arg)
{
	if (arg == NULL) {
		return;
	}

	management_id_t *msg = (management_id_t *)arg;
	frame_rate_t ack_data = { 0 };
	int ret = RET_FAIL;
	int fps = 0;

#if LOCAL_TEST == 1
	ret = RET_OK;
	fps = 10;
#else
	fps = sys_fps_get();
	if (fps > 0) {
		ret = RET_OK;
	} else {
		ret = RET_FAIL;
	}
#endif

	if (ret == RET_OK) {
		ret = run_succ;
	} else {
		ret = run_fail;
	}

	ack_data.ret_code = (uint8_t)ret;
	ack_data.frequency = (uint32_t)fps;
	pack(msg, (uint8_t *)&ack_data, sizeof(ack_data), ACK);
	return;
}

/*********************************************************************************************************************/
/********************************************************VCSEL********************************************************/
/*********************************************************************************************************************/

void set_vcsel_parameter(void *arg)
{
	if (arg == NULL) {
		return;
	}

	management_id_t *msg = (management_id_t *)arg;
	set_pl_respond_t ack_data = { 0 };
	int ret = RET_FAIL;

	uint8_t value_u8[4];
	uint32_t value_u32[4];

	switch (msg->buf[2]) {
	case SET_VCSEL_CHANNEL:
		for (int i = 0; i < 4; i++) {
			value_u8[i] = msg->buf[3 + i];
		}

		value_u32[0] = combine_uint8_to_uint32(value_u8[3], value_u8[2], value_u8[1], value_u8[0]);
		log_debug("Set vscel channel %x", value_u32[0]);
		if (value_u32[0] > 0x3F) {
			ret = RET_FAIL;
			break;
		}
#if LOCAL_TEST == 1
		ret = RET_OK;
#else
		ret = vcsel_chan_set(value_u32[0]);
#endif

		break;
	case SET_VCSEL_FREQ:
		for (int i = 0; i < 4; i++) {
			value_u8[i] = msg->buf[3 + i];
		}

		value_u32[0] = combine_uint8_to_uint32(value_u8[3], value_u8[2], value_u8[1], value_u8[0]);
		log_debug("Set vscel refreq %x", value_u32[0]);
		if (value_u32[0] > 500) {
			ret = RET_FAIL;
			break;
		}
#if LOCAL_TEST == 1
		ret = RET_OK;
#else
		ret = vcsel_refreq_set(value_u32[0]);
#endif

		break;
	case SET_VCSEL_LIGHTED_DELAY:
		value_u8[0] = msg->buf[3];

		log_debug("Set vscel lighted delay %x", value_u8[0]);
#if LOCAL_TEST == 1
		ret = RET_OK;
#else
		ret = calib_lightnd_delay_set(value_u8[0]);
#endif

		break;
	default:
		break;
	}

	if (ret == RET_OK) {
		ret = run_succ;
	} else {
		ret = run_fail;
	}

	// 组包、回复 sdk
	ack_data.cmd_type = msg->buf[2];
	ack_data.ret_code = (uint8_t)ret;
	pack(msg, (uint8_t *)&ack_data, sizeof(ack_data), ACK);
	return;
}

static int get_vcsel_chen(management_id_t *msg, uint8_t *ack_data)
{
	vcsel_chan_t *data = (vcsel_chan_t *)ack_data;
	int ret = run_fail;

#if LOCAL_TEST == 1
	data->value = 1;
#else
	data->value = vcsel_chan_get();
#endif

	if (data->value != ERR_FPGA_REG32) {
		ret = run_succ;
	} else {
		ret = run_fail;
	}
	data->cmd_type = msg->buf[2];
	data->ret_code = (uint8_t)ret;

	return sizeof(vcsel_chan_t);
}

static int get_vcsel_refreq(management_id_t *msg, uint8_t *ack_data)
{
	vcsel_refreq_t *data = (vcsel_refreq_t *)ack_data;
	int ret = run_fail;

#if LOCAL_TEST == 1
	data->value = 1;
#else
	data->value = vcsel_refreq_get();
#endif

	if (data->value != ERR_FPGA_REG32) {
		ret = run_succ;
	} else {
		ret = run_fail;
	}
	data->cmd_type = msg->buf[2];
	data->ret_code = (uint8_t)ret;

	return sizeof(vcsel_refreq_t);
}

static int get_ld_temp(management_id_t *msg, uint8_t *ack_data)
{
	ld_temp_t *data = (ld_temp_t *)ack_data;
	int ret = RET_FAIL;

#if LOCAL_TEST == 1
	ret = RET_OK;
	data->value = 1;
#else
	ret = ld_temp_get(&(data->value));
	data->value = uint16_to_float_positive_number(data->value);
#endif

	if (ret == RET_OK) {
		ret = run_succ;
	} else {
		ret = run_fail;
	}

	data->cmd_type = msg->buf[2];
	data->ret_code = (uint8_t)ret;

	return sizeof(ld_temp_t);
}

static int get_calib_lightnd_delay(management_id_t *msg, uint8_t *ack_data)
{
	calib_lightnd_delay_t *data = (calib_lightnd_delay_t *)ack_data;
	int ret = run_fail;

#if LOCAL_TEST == 1
	data->value = 1;
#else
	data->value = calib_lightnd_delay_get();
#endif

	if (data->value >= 0) {
		ret = run_succ;
	} else {
		ret = run_fail;
	}
	data->cmd_type = msg->buf[2];
	data->ret_code = (uint8_t)ret;

	return sizeof(calib_lightnd_delay_t);
}

void get_vcsel_parameter(void *arg)
{
	if (arg == NULL) {
		return;
	}

	management_id_t *msg = (management_id_t *)arg;
	uint8_t ack_data[ACK_BUF_MAX_LEN] = { 0 };
	int len = 0;

	switch (msg->buf[2]) {
	case GET_VCSEL_CHAN:
		len = get_vcsel_chen(msg, ack_data);
		break;
	case GET_VCSEL_REFREQ:
		len = get_vcsel_refreq(msg, ack_data);
		break;
	case GET_LD_TEMP:
		len = get_ld_temp(msg, ack_data);
		break;
	case GET_VCSEL_LIGHTED_DELAY:
		len = get_calib_lightnd_delay(msg, ack_data);
		break;
	default:
		break;
	}

	pack(msg, ack_data, len, ACK);
	return;
}

/********************************************************************************************************************/
/********************************************************SPAD********************************************************/
/********************************************************************************************************************/
static int get_spad_temp(management_id_t *msg, uint8_t *ack_data)
{
	spad_temp_t *data = (spad_temp_t *)ack_data;
	int ret = RET_FAIL;

#if LOCAL_TEST == 1
	ret = RET_OK;
	data->value = 1;
#else
	ret = spad_temp_get(&(data->value));
	data->value = uint16_to_float_positive_number(data->value);
#endif

	if (ret == RET_OK) {
		ret = run_succ;
	} else {
		ret = run_fail;
	}

	data->cmd_type = msg->buf[2];
	data->ret_code = (uint8_t)ret;

	return sizeof(spad_temp_t);
}

static int get_vop_sample(management_id_t *msg, uint8_t *ack_data)
{
	vop_sample_t *data = (vop_sample_t *)ack_data;
	int ret = run_fail;

#if LOCAL_TEST == 1
	data->value = 1;
#else
	data->value = vop_sample_get();
#endif

	if (data->value != ERR_FPGA_REG32) {
		ret = run_succ;
	} else {
		ret = run_fail;
	}
	data->cmd_type = msg->buf[2];
	data->ret_code = (uint8_t)ret;

	return sizeof(vop_sample_t);
}

static int get_spad_pwm_freq(management_id_t *msg, uint8_t *ack_data)
{
	spad_pwm_freq_t *data = (spad_pwm_freq_t *)ack_data;
	int ret = run_fail;

#if LOCAL_TEST == 1
	data->value = 1;
#else
	data->value = spad_pwm_freq_get();
#endif

	if (data->value != ERR_FPGA_REG32) {
		ret = run_succ;
	} else {
		ret = run_fail;
	}
	data->cmd_type = msg->buf[2];
	data->ret_code = (uint8_t)ret;

	return sizeof(spad_pwm_freq_t);
}

static int get_spad_pwm_duty(management_id_t *msg, uint8_t *ack_data)
{
	spad_pwm_duty_t *data = (spad_pwm_duty_t *)ack_data;
	int ret = run_fail;

#if LOCAL_TEST == 1
	data->value = 1;
#else
	data->value = spad_pwm_duty_get();
#endif

	if (data->value != ERR_FPGA_REG32) {
		ret = run_succ;
	} else {
		ret = run_fail;
	}
	data->cmd_type = msg->buf[2];
	data->ret_code = (uint8_t)ret;

	return sizeof(spad_pwm_duty_t);
}

static int get_spad_spi(management_id_t *msg, uint8_t *ack_data)
{
	spad_spi_read_t *data = (spad_spi_read_t *)ack_data;
	int ret = run_fail;
	uint8_t value_u8[4];
	uint32_t value_u32[2];

	for (int i = 0; i < 2; i++) {
		for (int j = 0; j < 4; j++) {
			value_u8[j] = msg->buf[3 + j + (i * 4)];
		}
		value_u32[i] = combine_uint8_to_uint32(value_u8[3], value_u8[2], value_u8[1], value_u8[0]);
	}

#if LOCAL_TEST == 1
	data->value = 1;
	log_info("read spi reg %08x, size %02x", value_u32[0], value_u32[1]);
#else
	data->value = spad_spi_read(value_u32[0], value_u32[1]);
#endif

	if (data->value != ERR_FPGA_REG32) {
		ret = run_succ;
	} else {
		ret = run_fail;
	}
	data->cmd_type = msg->buf[2];
	data->ret_code = (uint8_t)ret;

	return sizeof(spad_spi_read_t);
}

void set_spad_parameter(void *arg)
{
	if (arg == NULL) {
		return;
	}

	management_id_t *msg = (management_id_t *)arg;
	set_pl_respond_t ack_data = { 0 };
	int ret = RET_FAIL;

	uint8_t value_u8[4];
	uint32_t value_u32[4];

	switch (msg->buf[2]) {
	case SET_VOP_PWM_FREQ:
		for (int i = 0; i < 4; i++) {
			value_u8[i] = msg->buf[3 + i];
		}

		value_u32[0] = combine_uint8_to_uint32(value_u8[3], value_u8[2], value_u8[1], value_u8[0]);
		log_debug("Set spad pwd freq %x", value_u32[0]);
#if LOCAL_TEST == 1
		ret = RET_OK;
#else
		ret = spad_pwm_freq_set(value_u32[0]);
#endif

		break;
	case SET_VOP_PWM_DUTY:
		for (int i = 0; i < 4; i++) {
			value_u8[i] = msg->buf[3 + i];
		}

		value_u32[0] = combine_uint8_to_uint32(value_u8[3], value_u8[2], value_u8[1], value_u8[0]);
		log_debug("Set spad pwd duty %x", value_u32[0]);

		if (value_u32[0] > 100) {
			ret = RET_FAIL;
			break;
		}
#if LOCAL_TEST == 1
		ret = RET_OK;
#else
		ret = spad_pwm_duty_set(value_u32[0]);
#endif

		break;
	case SET_SPAD_SPI:
		for (int i = 0; i < 3; i++) {
			for (int j = 0; j < 4; j++) {
				value_u8[j] = msg->buf[3 + j + (i * 4)];
			}
			value_u32[i] = combine_uint8_to_uint32(value_u8[3], value_u8[2], value_u8[1], value_u8[0]);
		}

		log_debug("Set spad spi reg %x data %x size %x", value_u32[0], value_u32[1], value_u32[2]);
		if (value_u32[2] < 1 || value_u32[2] > 4) {
			ret = RET_FAIL;
			break;
		}
#if LOCAL_TEST == 1
		ret = RET_OK;
#else
		ret = spad_spi_write(value_u32[0], value_u32[1], value_u32[2]);
#endif

		break;
	default:
		break;
	}

	if (ret == RET_OK) {
		ret = run_succ;
	} else {
		ret = run_fail;
	}

	// 组包、回复 sdk
	ack_data.cmd_type = msg->buf[2];
	ack_data.ret_code = (uint8_t)ret;
	pack(msg, (uint8_t *)&ack_data, sizeof(ack_data), ACK);
	return;
}

void get_spad_parameter(void *arg)
{
	if (arg == NULL) {
		return;
	}

	management_id_t *msg = (management_id_t *)arg;
	uint8_t ack_data[ACK_BUF_MAX_LEN] = { '\0' };
	int len = 0;

	switch (msg->buf[2]) {
	case GET_SPAD_TEMP:
		len = get_spad_temp(msg, ack_data);
		break;
	case GET_VOP_SAMPLE:
		len = get_vop_sample(msg, ack_data);
		break;
	case GET_SPAD_PWM_FREQ:
		len = get_spad_pwm_freq(msg, ack_data);
		break;
	case GET_SPAD_PWM_DUTY:
		len = get_spad_pwm_duty(msg, ack_data);
		break;
	case GET_SPAD_SPI:
		len = get_spad_spi(msg, ack_data);
		break;
	default:
		break;
	}

	pack(msg, ack_data, len, ACK);
	return;
}

/*********************************************************************************************************************/
/********************************************************MOTOR********************************************************/
/*********************************************************************************************************************/
static int get_motor_agle(management_id_t *msg, uint8_t *ack_data)
{
	motor_agle_t *data = (motor_agle_t *)ack_data;
	int ret = RET_FAIL;

#if LOCAL_TEST == 1
	ret = RET_OK;
	data->value.f = 12.0;
#else
	ret = motor_agle_get(&(data->value.f));
#endif

	if (ret == RET_OK) {
		ret = run_succ;
	} else {
		ret = run_fail;
	}

	data->cmd_type = msg->buf[2];
	data->ret_code = (uint8_t)ret;

	return sizeof(motor_agle_t);
}

static int get_motor_zero_time(management_id_t *msg, uint8_t *ack_data)
{
	motor_zero_time_t *data = (motor_zero_time_t *)ack_data;
	int ret = run_fail;

#if LOCAL_TEST == 1
	data->value = 500;
#else
	data->value = motor_zero_time_get();
#endif

	if (data->value != ERR_FPGA_REG32) {
		ret = run_succ;
	} else {
		ret = run_fail;
	}
	data->cmd_type = msg->buf[2];
	data->ret_code = (uint8_t)ret;

	return sizeof(motor_zero_time_t);
}

static int get_calib_zero_offset(management_id_t *msg, uint8_t *ack_data)
{
	calib_zero_offset_t *data = (calib_zero_offset_t *)ack_data;
	int ret = RET_FAIL;

#if LOCAL_TEST == 1
	ret = RET_OK;
	data->value = 1;
#else
	ret = calib_zero_offset_get(&(data->value));
	data->value = uint16_to_float_positive_number(data->value);
#endif

	if (ret == RET_OK) {
		ret = run_succ;
	} else {
		ret = run_fail;
	}

	data->cmd_type = msg->buf[2];
	data->ret_code = (uint8_t)ret;

	return sizeof(calib_zero_offset_t);
}

static int get_motor_ctrl(management_id_t *msg, uint8_t *ack_data)
{
	motor_ctrl_t *data = (motor_ctrl_t *)ack_data;
	int ret = run_fail;

#if LOCAL_TEST == 1
	data->value = 1;
#else
	data->value = motor_ctrl_get();
#endif

	if (data->value != ERR_FPGA_REG32) {
		ret = run_succ;
	} else {
		ret = run_fail;
	}
	data->cmd_type = msg->buf[2];
	data->ret_code = (uint8_t)ret;

	return sizeof(motor_ctrl_t);
}

static int get_motor_pid(management_id_t *msg, uint8_t *ack_data)
{
	motor_pid_t *data = (motor_pid_t *)ack_data;
	int ret = run_fail;
	pid_info_t value;

#if LOCAL_TEST == 1
	value.u32 = 1;
#else
	value.u32 = motor_pid_get();
#endif

	if (value.u32 != ERR_FPGA_REG32) {
		ret = run_succ;
	} else {
		ret = run_fail;
	}

#if LOCAL_TEST == 1
	data->value[0] = 1;
	data->value[1] = 5;
	data->value[2] = 10;
	data->value[3] = 15;
#else
	data->value[0] = value.pid.enable;
	data->value[1] = value.pid.p;
	data->value[2] = value.pid.i;
	data->value[3] = value.pid.d;
#endif

	data->cmd_type = msg->buf[2];
	data->ret_code = (uint8_t)ret;

	return sizeof(motor_pid_t);
}

static int get_motor_pwm_freq(management_id_t *msg, uint8_t *ack_data)
{
	motor_pwm_freq_t *data = (motor_pwm_freq_t *)ack_data;
	int ret = run_fail;

#if LOCAL_TEST == 1
	data->value = 1;
#else
	data->value = motor_pwm_freq_get();
#endif

	if (data->value != ERR_FPGA_REG32) {
		ret = run_succ;
	} else {
		ret = run_fail;
	}
	data->cmd_type = msg->buf[2];
	data->ret_code = (uint8_t)ret;

	return sizeof(motor_pwm_freq_t);
}

static int get_motor_pwm_duty(management_id_t *msg, uint8_t *ack_data)
{
	motor_pwm_duty_t *data = (motor_pwm_duty_t *)ack_data;
	int ret = run_fail;

#if LOCAL_TEST == 1
	data->value = 1;
#else
	data->value = motor_pwm_duty_get();
#endif

	if (data->value != ERR_FPGA_REG32) {
		ret = run_succ;
	} else {
		ret = run_fail;
	}
	data->cmd_type = msg->buf[2];
	data->ret_code = (uint8_t)ret;

	return sizeof(motor_pwm_duty_t);
}

static int get_motor_stop_angle(management_id_t *msg, uint8_t *ack_data)
{
	motor_stop_angle_t *data = (motor_stop_angle_t *)ack_data;
	int ret = run_fail;

#if LOCAL_TEST == 1
	data->value = 1;
#else
	data->value = motor_stop_angle_get();
#endif

	if (data->value != ERR_FPGA_REG32) {
		ret = run_succ;
	} else {
		ret = run_fail;
	}
	data->cmd_type = msg->buf[2];
	data->ret_code = (uint8_t)ret;

	return sizeof(motor_stop_angle_t);
}

static int get_motor_period(management_id_t *msg, uint8_t *ack_data)
{
	motor_period_t *data = (motor_period_t *)ack_data;
	int ret = run_fail;

#if LOCAL_TEST == 1
	data->value = 1;
#else
	data->value = motor_period_get();
#endif

	if (data->value != ERR_FPGA_REG32) {
		ret = run_succ;
	} else {
		ret = run_fail;
	}
	data->cmd_type = msg->buf[2];
	data->ret_code = (uint8_t)ret;

	return sizeof(motor_period_t);
}

static int get_calib_flat_offset(management_id_t *msg, uint8_t *ack_data)
{
	calib_flat_offset_t *data = (calib_flat_offset_t *)ack_data;
	int ret = run_fail;

#if LOCAL_TEST == 1
	data->value[0] = 1;
	data->value[1] = 2;
	data->value[2] = 3;
	data->value[3] = 4;
#else
	data->value[0] = calib_flat0_offset_get();
	data->value[1] = calib_flat1_offset_get();
	data->value[2] = calib_flat2_offset_get();
	data->value[3] = calib_flat3_offset_get();
#endif

	log_info("calib flat offset: [0]=%u [1]=%u [2]=%u [3]=%u", data->value[0], data->value[1], data->value[2],
		 data->value[3]);

	for (int i = 0; i < sizeof(data->value) / sizeof(data->value[0]); i++) {
		if (data->value[i] >= 0) {
			ret = run_succ;
		} else {
			ret = run_fail;
			break;
		}
	}
	data->cmd_type = msg->buf[2];
	data->ret_code = (uint8_t)ret;

	return sizeof(calib_flat_offset_t);
}

void set_motor_parameter(void *arg)
{
	if (arg == NULL) {
		return;
	}

	management_id_t *msg = (management_id_t *)arg;
	set_pl_respond_t ack_data = { 0 };
	int ret = RET_FAIL;

	char str_value[16];
	char str_key[64];

	uint8_t value_u8[4];
	uint32_t value_u16[4];
	uint32_t value_u32[4];

	switch (msg->buf[2]) {
	case SET_MOTOR_CTRL:
		for (int i = 0; i < 2; i++) {
			for (int j = 0; j < 4; j++) {
				value_u8[j] = msg->buf[3 + j + (i * 4)];
			}
			value_u32[i] = combine_uint8_to_uint32(value_u8[3], value_u8[2], value_u8[1], value_u8[0]);
		}

		log_debug("Set motor ctrl %x %x", value_u32[0], value_u32[1]);
#if LOCAL_TEST == 1
		ret = RET_OK;
#else
		ret = motor_ctrl_set(value_u32[0], value_u32[1]);
#endif

		break;
	case SET_MOTOR_PID:
		for (int i = 0; i < 4; i++) {
			for (int j = 0; j < 4; j++) {
				value_u8[j] = msg->buf[3 + j + (i * 4)];
			}
			value_u32[i] = combine_uint8_to_uint32(value_u8[3], value_u8[2], value_u8[1], value_u8[0]);
		}

		log_debug("Set motor pid %x %x %x %x", value_u32[0], value_u32[1], value_u32[2], value_u32[3]);
#if LOCAL_TEST == 1
		ret = RET_OK;
#else
		ret = motor_pid_set(value_u32[0], value_u32[1], value_u32[2], value_u32[3]);
#endif

		break;
	case SET_MOTOR_PWM_FREQ:
		for (int i = 0; i < 4; i++) {
			value_u8[i] = msg->buf[3 + i];
		}

		value_u32[0] = combine_uint8_to_uint32(value_u8[3], value_u8[2], value_u8[1], value_u8[0]);
		log_debug("Set motor pwm freq %x", value_u32[0]);
#if LOCAL_TEST == 1
		ret = RET_OK;
#else
		ret = motor_pwm_freq_set(value_u32[0]);
#endif

		break;
	case SET_MOTOR_PWM_DUTY:
		for (int i = 0; i < 4; i++) {
			value_u8[i] = msg->buf[3 + i];
		}

		value_u32[0] = combine_uint8_to_uint32(value_u8[3], value_u8[2], value_u8[1], value_u8[0]);
		log_debug("Set motor pwm duty %x", value_u32[0]);
		if (value_u32[0] > 100) {
			ret = RET_FAIL;
			break;
		}
#if LOCAL_TEST == 1
		ret = RET_OK;
#else
		ret = motor_pwm_duty_set(value_u32[0]);
#endif

		break;
	case SET_CALIB_ZERO_OFFSET:
		for (int i = 0; i < 4; i++) {
			value_u8[i] = msg->buf[3 + i];
		}

		value_u32[0] = combine_uint8_to_uint32(value_u8[3], value_u8[2], value_u8[1], value_u8[0]);
		log_debug("Set calib zero offset %x", value_u32[0]);
#if LOCAL_TEST == 1
		ret = RET_OK;
#else
		ret = calib_zero_offset_set((short)value_u32[0]);
#endif

		break;
	case SET_MOTOR_STOP_ANGLE:
		for (int i = 0; i < 4; i++) {
			value_u8[i] = msg->buf[3 + i];
		}

		value_u32[0] = combine_uint8_to_uint32(value_u8[3], value_u8[2], value_u8[1], value_u8[0]);
		log_debug("Set motor stop angle %x", value_u32[0]);
#if LOCAL_TEST == 1
		ret = RET_OK;
#else
		ret = motor_stop_angle_set(value_u32[0]);
#endif

		break;
	case SET_FLAT_OFFSET:
		for (int i = 0; i < 2; i++) {
			for (int j = 0; j < 2; j++) {
				value_u8[j] = msg->buf[3 + j + (i * 2)];
			}
			value_u16[i] = combine_uint8_to_uint16(value_u8[1], value_u8[0]);
		}

		log_debug("Set horizontal mirror %x = %d", value_u16[0], value_u16[1]);

#if LOCAL_TEST == 1
		ret = RET_OK;
#else
		switch (value_u16[0]) {
		case 0:
			ret = calib_flat0_offset_set(value_u16[1]);
			break;
		case 1:
			ret = calib_flat1_offset_set(value_u16[1]);
			break;
		case 2:
			ret = calib_flat2_offset_set(value_u16[1]);
			break;
		case 3:
			ret = calib_flat3_offset_set(value_u16[1]);
			break;
		default:
			ret = RET_FAIL;
			break;
		}
#endif

		if (cal_param.side_angular_offset_cnt < value_u16[0] + 1) {
			log_fatal("missing calibration parameters:[side_angular_offset]");
			ret = RET_FAIL;
			break;
		}

		cal_param.side_angular_offset[value_u16[0]] = value_u16[1];
		snprintf(str_value, sizeof(str_value), "%d", cal_param.side_angular_offset[value_u16[0]]);
		snprintf(str_key, sizeof(str_key), "side_angular_offset:side%d_angular_offset", value_u16[0]);
		log_debug("set key %s value %s", str_key, str_value);
		iniparser_set(cal_param_ini, str_key, str_value);
		init_decode_paramter();

		break;
	default:
		break;
	}

	if (ret == RET_OK) {
		ret = run_succ;
	} else {
		ret = run_fail;
	}

	// 更新到文件中
	if (msg->buf[2] == SET_FLAT_OFFSET && ret == run_succ &&
	    core_update_ini_file(cal_param_ini, DEFAULT_CAL_CONF_PATH))
		ret = run_fail;

	// 组包、回复 sdk
	ack_data.cmd_type = msg->buf[2];
	ack_data.ret_code = (uint8_t)ret;
	pack(msg, (uint8_t *)&ack_data, sizeof(ack_data), ACK);
	return;
}

void get_motor_parameter(void *arg)
{
	if (arg == NULL) {
		return;
	}

	management_id_t *msg = (management_id_t *)arg;
	uint8_t ack_data[ACK_BUF_MAX_LEN] = { '\0' };
	int len = 0;

	switch (msg->buf[2]) {
	case GET_MOTOR_AGLE:
		len = get_motor_agle(msg, ack_data);
		break;
	case GET_MOTOR_ZERO_TIME:
		len = get_motor_zero_time(msg, ack_data);
		break;
	case GET_CALIB_ZERO_OFFSE:
		len = get_calib_zero_offset(msg, ack_data);
		break;
	case GET_MOTOR_CTRL:
		len = get_motor_ctrl(msg, ack_data);
		break;
	case GET_MOTOR_PID:
		len = get_motor_pid(msg, ack_data);
		break;
	case GET_MOTOR_PWM_FREP:
		len = get_motor_pwm_freq(msg, ack_data);
		break;
	case GET_MOTOR_PWM_DUTY:
		len = get_motor_pwm_duty(msg, ack_data);
		break;
	case GET_MOTOR_STOP_ANGLE:
		len = get_motor_stop_angle(msg, ack_data);
		break;
	case GET_MOTOR_PERIOD:
		len = get_motor_period(msg, ack_data);
		break;
	case GET_FLAT_OFFSET:
		len = get_calib_flat_offset(msg, ack_data);
		break;
	default:
		break;
	}

	pack(msg, ack_data, len, ACK);
	return;
}

#if CS96_ALPHA == 1
uint16_t get_encoder()
{
	// uint16_t encoder_value;
	uint32_t abz_value;

#if LOCAL_TEST == 1
	// encoder_value = 0;
	abz_value = 0;
#elif CS96_ALPHA == 1
	// encoder_value = (uint16_t)regr32(0x80004010);
	abz_value = regr32(0x80004014);
#else
	abz_value = 0;
#endif

	return (uint16_t)(abz_value >> 16);
}
#elif CS96_BETA == 1 || CS96_GAMMA == 1
uint16_t get_encoder(encoder_info_t *info)
{
	if (info == NULL) {
		return RET_FAIL;
	}

	static uint32_t pre_time = 0;
#if LOCAL_TEST == 1
	uint16_t val1 = 0; // 刻度时间低 16 位
	uint16_t val2 = 0; // 低 8 位：刻度时间高 8 位；高 8 位：刻度计数
#else
	uint16_t val1 = spiRegRead(0x2048); // 刻度时间低 16 位
	uint16_t val2 = spiRegRead(0x204C); // 低 8 位：刻度时间高 8 位；高 8 位：刻度计数
#endif
	uint16_t count = (val2 & 0xFF00) >> 8;

	// 先拼接成完整的 32 位整数
	uint32_t raw_time_int = ((uint32_t)(val2 & 0x00FF) << 16) | val1;

	// 过滤相同刻度的数
	if (raw_time_int == pre_time) {
		return RET_FAIL;
	} else {
		pre_time = raw_time_int;
	}

	// 将整数转换为浮点数
	float time_f = (float)raw_time_int;

	// 计算时间 (8ns 单位转 us)
	float time_val = time_f * 8.0f / 1000.0f; // us

	info->cnt = count;
	info->value.f = time_val;
	return RET_OK;
}
#endif

void report_camera_trigger()
{
#if LOCAL_TEST == 1
	return;
#endif
#if CS96_ALPHA == 1
	static uint32_t camera_trigger;
	static uint8_t pre_trigger, now_trigger;

	camera_trigger = regr32(0x80000010);
	now_trigger = camera_trigger & 0x01;

	if (pre_trigger == 0 && now_trigger == 1) {
		struct timespec te;
		clock_gettime(CLOCK_REALTIME, &te);

		trigger_time_t now_time;
		now_time.time[0] = te.tv_nsec * 1e-3; // usec
		now_time.time[1] = te.tv_sec; // sec

		uint8_t data[11] = { 0xAA, 0xBB, 0x01 };
		memcpy(data + 3, now_time.delay, 8);
		respond_command(data, sizeof(data));
	}
	pre_trigger = now_trigger;
#endif
	return;
}

/********************************************************************************************************************/
/********************************************************CONF********************************************************/
/********************************************************************************************************************/
static int get_calib_threshold_time(management_id_t *msg, uint8_t *ack_data)
{
	calib_threshold_time_t *data = (calib_threshold_time_t *)ack_data;
	int ret = RET_FAIL;

#if LOCAL_TEST == 1
	data->value = 1;
#else
	data->value = calib_threshold_time_get();
#endif

	if (data->value >= 0) {
		ret = run_succ;
	} else {
		ret = run_fail;
	}

	data->cmd_type = msg->buf[2];
	data->ret_code = (uint8_t)ret;

	return sizeof(calib_threshold_time_t);
}

static int get_calib_threshold_back(management_id_t *msg, uint8_t *ack_data)
{
	calib_threshold_back_t *data = (calib_threshold_back_t *)ack_data;
	int ret = RET_FAIL;

#if LOCAL_TEST == 1
	data->value = 1;
#elif MF150_FL6031 == 1
	data->value = 0;
#else
	data->value = calib_threshold_back_get();
#endif

	if (data->value >= 0) {
		ret = run_succ;
	} else {
		ret = run_fail;
	}

	data->cmd_type = msg->buf[2];
	data->ret_code = (uint8_t)ret;

	return sizeof(calib_threshold_back_t);
}

void set_calib_parameter(void *arg)
{
	if (arg == NULL) {
		return;
	}

	management_id_t *msg = (management_id_t *)arg;
	set_pl_respond_t ack_data = { 0 };
	int ret = RET_FAIL;

	uint8_t value_u8[4];

	switch (msg->buf[2]) {
	case SET_CALIB_THRESHOLD_TIME:
		value_u8[0] = msg->buf[3];

		log_debug("Set calibration threshold time %x", value_u8[0]);
#if LOCAL_TEST == 1
		ret = RET_OK;
#elif MF150_FL6031 == 1
		ret = RET_OK;
#else
		ret = calib_threshold_time_set(value_u8[0]);
#endif

		break;
	case SET_CALIB_THRESHOLD_BACK:
		value_u8[0] = msg->buf[3];

		log_debug("Set calibration threshold background %x", value_u8[0]);
#if LOCAL_TEST == 1
		ret = RET_OK;
#elif MF150_FL6031 == 1
		ret = RET_OK;
#else
		ret = calib_threshold_back_set(value_u8[0]);
#endif

		break;
	default:
		break;
	}

	if (ret == RET_OK) {
		ret = run_succ;
	} else {
		ret = run_fail;
	}

	// 组包、回复 sdk
	ack_data.cmd_type = msg->buf[2];
	ack_data.ret_code = (uint8_t)ret;
	pack(msg, (uint8_t *)&ack_data, sizeof(ack_data), ACK);
	return;
}

void get_calib_parameter(void *arg)
{
	if (arg == NULL) {
		return;
	}

	management_id_t *msg = (management_id_t *)arg;
	uint8_t ack_data[ACK_BUF_MAX_LEN] = { '\0' };
	int len = 0;

	switch (msg->buf[2]) {
	case GET_CALIB_THRESHOLD_TIME:
		len = get_calib_threshold_time(msg, ack_data);
		break;
	case GET_CALIB_THRESHOLD_BACK:
		len = get_calib_threshold_back(msg, ack_data);
		break;
	default:
		break;
	}

	pack(msg, ack_data, len, ACK);
	return;
}

void set_side_angular_offset(void *arg)
{
	if (arg == NULL) {
		return;
	}

	management_id_t *msg = (management_id_t *)arg;
	int ret = RET_FAIL;

	uint16_t value_u8[2];
	uint16_t value_u16;

	char str_value[16];
	char str_key[64];

	// 如果配置项不存在，则认为是非法指令
	if (cal_param.side_angular_offset_cnt <= 0 || cal_param.side_angular_offset == NULL) {
		ret = illegal_cmd;
		log_fatal("missing calibration parameters:[side_angular_offset]");
		goto report;
	}

	for (int i = 0; i < cal_param.side_angular_offset_cnt; i++) {
		for (int j = 0; j < 2; j++) {
			value_u8[j] = msg->buf[3 + j + (i * 2)];
		}
		value_u16 = combine_uint8_to_uint16(value_u8[1], value_u8[0]);

#if LOCAL_TEST == 1
		ret = RET_OK;
#else
		switch (i) {
		case 0:
			ret = calib_flat0_offset_set(value_u16);
			break;
		case 1:
			ret = calib_flat1_offset_set(value_u16);
			break;
		case 2:
			ret = calib_flat2_offset_set(value_u16);
			break;
		case 3:
			ret = calib_flat3_offset_set(value_u16);
			break;
		default:
			break;
		}
#endif

		if (ret != RET_OK) {
			ret = RET_FAIL;
			log_fatal("set FPGA side %d angular offset fail", i);
			break;
		} else {
			cal_param.side_angular_offset[i] = value_u16;
			snprintf(str_value, sizeof(str_value), "%d", cal_param.side_angular_offset[i]);
			snprintf(str_key, sizeof(str_key), "side_angular_offset:side%d_angular_offset", i);
			log_debug("set key %s value %s", str_key, str_value);
			iniparser_set(cal_param_ini, str_key, str_value);
		}
	}

	if (ret == RET_OK) {
		ret = run_succ;
	} else {
		ret = run_fail;
	}

	// 更新到文件中
	if (ret == run_succ && core_update_ini_file(cal_param_ini, DEFAULT_CAL_CONF_PATH))
		ret = run_fail;
	init_decode_paramter();

report:
	// 组包、回复 sdk
	pack(msg, (uint8_t *)&ret, 1, ACK);
	return;
}

void set_side_light_en(void *arg)
{
	if (arg == NULL) {
		return;
	}

	management_id_t *msg = (management_id_t *)arg;
	int ret = RET_FAIL;

	uint16_t value_u8;

	char str_value[16];
	char str_key[64];

	value_u8 = msg->buf[2];

#if LOCAL_TEST == 1
	ret = RET_OK;
#else
	ret = calib_light_enb_all(value_u8);
#endif

	if (ret != RET_OK) {
		ret = RET_FAIL;
		log_fatal("每面发光使能设置失败");
	} else {
		cal_param.side_light = value_u8;
		snprintf(str_value, sizeof(str_value), "%d", cal_param.side_light);
		snprintf(str_key, sizeof(str_key), "side_light:side_light");
		log_debug("set key %s value %s", str_key, str_value);
		iniparser_set(cal_param_ini, str_key, str_value);
	}

	if (ret == RET_OK) {
		ret = run_succ;
	} else {
		ret = run_fail;
	}

	// 更新到文件中
	if (ret == run_succ && core_update_ini_file(cal_param_ini, DEFAULT_CAL_CONF_PATH))
		ret = run_fail;

	// 组包、回复 sdk
	pack(msg, (uint8_t *)&ret, 1, ACK);
	return;
}

void set_vcsel_h_offset(void *arg)
{
	if (arg == NULL) {
		return;
	}

	management_id_t *msg = (management_id_t *)arg;
	int ret = RET_FAIL;

	uint16_t count;
	uint8_t *data = NULL;

	char str_value[16];
	char str_key[64];

	count = combine_uint8_to_uint16(msg->buf[3], msg->buf[2]);

	if (count > cal_param.omega_delta_cnt || count <= 0) {
		ret = illegal_cmd;
		log_error("omega_delta_cnt = %d, but new count = %d", cal_param.omega_delta_cnt, count);
		goto ack;
	}

	data = msg->buf + 4;

	for (int i = 0; i < count; i++) {
		cal_param.omega_delta[i].u32 =
			combine_uint8_to_uint32(data[i * 4 + 3], data[i * 4 + 2], data[i * 4 + 1], data[i * 4]);
		snprintf(str_value, sizeof(str_value), "%g", cal_param.omega_delta[i].f);
		snprintf(str_key, sizeof(str_key), "horizontal_angular_offset:omega_delta%d", i);
		log_debug("set key %s value %s", str_key, str_value);
		iniparser_set(cal_param_ini, str_key, str_value);
	}

	// 更新到文件中
	ret = core_update_ini_file(cal_param_ini, DEFAULT_CAL_CONF_PATH);
	ret += init_decode_paramter();

	if (ret == RET_OK) {
		ret = run_succ;
	} else {
		ret = run_fail;
	}
ack:
	// 组包、回复 sdk
	pack(msg, (uint8_t *)&ret, 1, ACK);
	return;
}

void set_distance_offset(void *arg)
{
	if (arg == NULL) {
		return;
	}

	management_id_t *msg = (management_id_t *)arg;
	int ret = RET_FAIL;

	uint16_t count;
	uint8_t *data = NULL;

	char str_value[16];
	char str_key[64];

	count = combine_uint8_to_uint16(msg->buf[3], msg->buf[2]);

	if (count > cal_param.d_delta_cnt || count <= 0) {
		ret = illegal_cmd;
		log_error("d_delta_cnt = %d, but new count = %d", cal_param.d_delta_cnt, count);
		goto ack;
	}

	data = msg->buf + 4;

	for (int i = 0; i < count; i++) {
		cal_param.d_delta[i].u32 =
			combine_uint8_to_uint32(data[i * 4 + 3], data[i * 4 + 2], data[i * 4 + 1], data[i * 4]);
		snprintf(str_value, sizeof(str_value), "%g", cal_param.d_delta[i].f);
		snprintf(str_key, sizeof(str_key), "zero_distance_offset:d%d_delta", i);
		log_debug("set key %s value %s", str_key, str_value);
		iniparser_set(cal_param_ini, str_key, str_value);
	}

	// 更新到文件中
	ret = core_update_ini_file(cal_param_ini, DEFAULT_CAL_CONF_PATH);
	ret += init_decode_paramter();

	if (ret == RET_OK) {
		ret = run_succ;
	} else {
		ret = run_fail;
	}
ack:
	// 组包、回复 sdk
	pack(msg, (uint8_t *)&ret, 1, ACK);
	return;
}

int pl_calib_parm_init(calibration_parm_t *param)
{
#if LOCAL_TEST == 1
	return RET_OK;
#else
	int ret = RET_FAIL;

	// 设置 SPAD 偏压
	if (config.set_vop_bias == ON) {
		// 如果占空比大于频率的 66%，保护
		if (param->vop_bias_duty > param->vop_bias_freq * 0.66) {
			log_error("非法参数，占空比不能大于频率的百分之 66");
			goto end;
		}
		ret = spad_pwm_freq_set(param->vop_bias_freq);
		log_info("设置 spad vop 频率= %d", param->vop_bias_freq);
		if (ret != RET_OK) {
			ret = RET_FAIL;
			log_fatal("set spad vop bias freq fail");
			goto end;
		}

		ret = spad_pwm_duty_set(param->vop_bias_duty);
		log_info("设置 spad vop 占空比 = %d", param->vop_bias_duty);
		if (ret != RET_OK) {
			ret = RET_FAIL;
			log_fatal("set spad vop bias duty fail");
			goto end;
		}
	}

	// 阈值
	if (config.set_pl_threshold == ON) {
		ret = calib_threshold_time_set(param->threshold_time);
		if (ret != RET_OK) {
			ret = RET_FAIL;
			log_fatal("set FPGA threshold time fail");
			goto end;
		}

		ret = calib_threshold_back_set(param->threshold_back);
		if (ret != RET_OK) {
			ret = RET_FAIL;
			log_fatal("set FPGA threshold background fail");
			goto end;
		}
	}

	// 零位角
	if (config.set_pl_zero_angle == ON) {
		if (param->side_angular_offset_cnt <= 0 || param->side_angular_offset == NULL) {
			ret = RET_FAIL;
			log_fatal("missing calibration parameters:[side_angular_offset]");
			goto end;
		}

		ret = calib_flat0_offset_set(param->side_angular_offset[0]);
		if (ret != RET_OK) {
			ret = RET_FAIL;
			log_fatal("set FPGA side0 angular offset fail");
			goto end;
		}

		ret = calib_flat1_offset_set(param->side_angular_offset[1]);
		if (ret != RET_OK) {
			ret = RET_FAIL;
			log_fatal("set FPGA side1 angular offset fail");
			goto end;
		}

		ret = calib_flat2_offset_set(param->side_angular_offset[2]);
		if (ret != RET_OK) {
			ret = RET_FAIL;
			log_fatal("set FPGA side2 angular offset fail");
			goto end;
		}

		ret = calib_flat3_offset_set(param->side_angular_offset[3]);
		if (ret != RET_OK) {
			ret = RET_FAIL;
			log_fatal("set FPGA side3 angular offset fail");
			goto end;
		}
	}

	// 面使能
	if (config.set_pl_side_en == ON) {
		log_info("设置每面发光使能为：%d", cal_param.side_light);
		ret = calib_light_enb_all(cal_param.side_light);
		if (ret != RET_OK) {
			ret = RET_FAIL;
			log_fatal("每面发光使能设置失败");
			goto end;
		}
	}

	// 起始发光编码值
	if (config.set_pl_standard_encoder == ON) {
		if (param->standard_encoder_cnt <= 1 || param->angle_face == NULL) {
			ret = RET_FAIL;
			log_fatal("missing calibration parameters:[standard_encoder]");
			goto end;
		}

		log_info("设置第 1 面起始发光编码器值为：%d", cal_param.angle_face[0]);
		ret = calib_light_enb1(cal_param.angle_face[0]);
		if (ret != RET_OK) {
			ret = RET_FAIL;
			log_fatal("第 1 面起始发光编码器值设置失败");
			goto end;
		}

		log_info("设置第 2 面起始发光编码器值为：%d", cal_param.angle_face[1]);
		ret = calib_light_enb2(cal_param.angle_face[1]);
		if (ret != RET_OK) {
			ret = RET_FAIL;
			log_fatal("第 2 面起始发光编码器值设置失败");
			goto end;
		}

		log_info("设置第 3 面起始发光编码器值为：%d", cal_param.angle_face[2]);
		ret = calib_light_enb3(cal_param.angle_face[2]);
		if (ret != RET_OK) {
			ret = RET_FAIL;
			log_fatal("第 3 面起始发光编码器值设置失败");
			goto end;
		}

		log_info("设置第 4 面起始发光编码器值为：%d", cal_param.angle_face[3]);
		ret = calib_light_enb4(cal_param.angle_face[3]);
		if (ret != RET_OK) {
			ret = RET_FAIL;
			log_fatal("第 4 面起始发光编码器值设置失败");
			goto end;
		}
	}

end:
	return ret;
#endif
}
#elif MF150_FL6031 == 1
int pl_calib_parm_init(calibration_parm_t *param)
{
	return RET_OK;
}
#elif MF150_FL6532 == 1
int pl_calib_parm_init(calibration_parm_t *param)
{
	return RET_OK;
}
#endif
