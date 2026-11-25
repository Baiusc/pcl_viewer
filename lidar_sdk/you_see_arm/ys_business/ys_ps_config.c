/*
 * @Author       : yangkh kaihong.yang@vitalchem.com
 * @Date         : 2024-05-27 05:19:48
 * @LastEditors  : kaihong.yang kaihong.yang@vitalchem.com
 * @LastEditTime : 2025-05-26 18:27:53
 * @FilePath     : /you_see_arm/ys_business/ys_ps_config.c
 * @Description  :
 * Copyright (c) 2024 by Vitalchem, All Rights Reserved.
 */

#include "decode/ys_decode.h"
#include "ys_errno.h"
#include "ys_pl_config.h"
#include "ys_ps_config.h"

#include "../common/common.h"
#include "../common/ys_protocal.h"

#include "../core/ys_base.h"
#include "../core/ys_user_settings.h"
#include <limits.h>

/* 固定本次启动实际加载的路径，后续握手和参数修改必须写回同一文件。 */
static char runtime_config_path[PATH_MAX] = DEFAULT_CONF_PATH;

const char *ys_runtime_config_path(void)
{
	return runtime_config_path;
}

int load_config()
{
	const char *path = getenv("YS_CONFIG_PATH");
	if (path && (path[0] != '/' || strlen(path) >= sizeof(runtime_config_path)))
		return RET_FAIL;
	bzero(&config, sizeof(configuration_t));
	config.fp = NULL;
	snprintf(runtime_config_path, sizeof(runtime_config_path), "%s", path ? path : DEFAULT_CONF_PATH);
	return core_load_conf_ini(runtime_config_path);
}

void destroy_config()
{
	if (config.fp != NULL) {
		fclose(config.fp);
	}
	return core_unload_conf_ini();
}

int load_cal_config()
{
	bzero(&cal_param, sizeof(cal_param));

	return core_load_cal_ini();
}

void destroy_cal_config()
{
	return core_unload_cal_ini();
}

static int save_user_settings(int echo_mode, int echo_type, int coordinate_mode)
{
	const char *path = getenv("YS_USER_SETTINGS_PATH");
	if (!path)
		return RET_OK;
	ys_user_settings_t settings = { echo_mode, echo_type, coordinate_mode };
	return ys_user_settings_store(path, &settings);
}

static int update_runtime_settings(void)
{
	int ret = core_update_ini_file(conf_ini, ys_runtime_config_path());
	/* 托管设置已经原子落盘，运行配置仅为可再生成的副本。 */
	if (ret && getenv("YS_USER_SETTINGS_PATH")) {
		log_warn("用户参数已保存，临时运行配置写回失败");
		return RET_OK;
	}
	return ret;
}

void set_echo_parm(void *arg)
{
	char str_value[8];

	if (arg == NULL) {
		return;
	}

	management_id_t *msg = (management_id_t *)arg;
	int ret = RET_FAIL;
	uint8_t echo_mode = 0;
	uint8_t echo_type = 0;

	echo_mode = msg->buf[2];
	echo_type = msg->buf[3];

	if (echo_mode != SINGLE_ECHO_MODE && echo_mode != DOUBLE_ECHO_MODE) {
		ret = RET_FAIL;
		goto ack;
	}

	if (echo_type != STRONGEST_ECHO_TYPE && echo_type != LAST_ECHO_TYPE) {
		ret = RET_FAIL;
		goto ack;
	}

	pthread_mutex_lock(&(config_lock));
	if (save_user_settings(echo_mode, echo_type, config.coordinate_mode)) {
		pthread_mutex_unlock(&(config_lock));
		goto ack;
	}
	config.echo_mode = echo_mode;
	snprintf(str_value, sizeof(str_value), "%d", echo_mode);
	iniparser_set(conf_ini, "system:echo_mode", str_value);

	config.echo_type = echo_type;
	snprintf(str_value, sizeof(str_value), "%d", echo_type);
	iniparser_set(conf_ini, "system:echo_type", str_value);
	ret = update_runtime_settings();
	pthread_mutex_unlock(&(config_lock));

ack:
	if (ret == RET_OK) {
		ret = run_succ;
	} else {
		ret = run_fail;
	}

	pack(msg, (uint8_t *)&ret, 1, ACK);
	return;
}

void get_echo_parm(void *arg)
{
	if (arg == NULL) {
		return;
	}

	management_id_t *msg = (management_id_t *)arg;
	point_cloud_echo_info_t ack_data = { 0 };
	int ret = RET_OK;

	if (ret == RET_OK) {
		ret = run_succ;
	} else {
		ret = run_fail;
	}

	ack_data.ret_code = (uint8_t)ret;
	pthread_mutex_lock(&(config_lock));
	ack_data.echo_mode = config.echo_mode;
	ack_data.echo_type = config.echo_type;
	pthread_mutex_unlock(&(config_lock));
	pack(msg, (uint8_t *)&ack_data, sizeof(ack_data), ACK);
	return;
}

void set_coordinate_mode(void *arg)
{
	char str_value[8];

	if (arg == NULL) {
		return;
	}

	management_id_t *msg = (management_id_t *)arg;
	int ret = RET_FAIL;
	uint8_t coordinate_mode = 0;

	coordinate_mode = msg->buf[2];

	if (coordinate_mode != CARTESIAN_COORDINATE_SYSTEM && coordinate_mode != SPHERICAL_COORDINATE_SYSTEM) {
		ret = RET_FAIL;
		goto ack;
	}

	pthread_mutex_lock(&(config_lock));
	if (save_user_settings(config.echo_mode, config.echo_type, coordinate_mode)) {
		pthread_mutex_unlock(&(config_lock));
		goto ack;
	}
	config.coordinate_mode = coordinate_mode;
	snprintf(str_value, sizeof(str_value), "%d", coordinate_mode);
	iniparser_set(conf_ini, "system:coordinate_mode", str_value);

	ret = update_runtime_settings();
	pthread_mutex_unlock(&(config_lock));

ack:
	if (ret == RET_OK) {
		ret = run_succ;
	} else {
		ret = run_fail;
	}

	pack(msg, (uint8_t *)&ret, 1, ACK);
	return;
}

void get_coordinate_mode(void *arg)
{
	if (arg == NULL) {
		return;
	}

	management_id_t *msg = (management_id_t *)arg;
	coordinate_info_t ack_data = { 0 };
	int ret = RET_FAIL;
	uint8_t coordinate_mode = 0;

	pthread_mutex_lock(&(config_lock));
	coordinate_mode = config.coordinate_mode;
	pthread_mutex_unlock(&(config_lock));
	ret = RET_OK;

	if (ret == RET_OK) {
		ret = run_succ;
	} else {
		ret = run_fail;
	}

	ack_data.ret_code = (uint8_t)ret;
	ack_data.mode = coordinate_mode;
	pack(msg, (uint8_t *)&ack_data, sizeof(ack_data), ACK);
	return;
}

int update_calibration_config(calibration_parm_head_t *data)
{
	char key[64], value[64];
	int offset = 0;
	uint8_t *parm_data = (uint8_t *)data->value;

	if (cal_param.pixel_in_vcsel_id_cnt != data->pixel_in_vcsel_id_cnt) {
		log_error("pixel_in_vcsel_id_cnt no match");
		return RET_FAIL;
	} else if (cal_param.int_ld_cnt != data->int_ld_cnt) {
		log_error("int_ld_cnt no match");
		return RET_FAIL;
	} else if (cal_param.omega_delta_cnt != data->omega_delta_cnt) {
		log_error("omega_delta_cnt no match");
		return RET_FAIL;
	} else if (cal_param.d_delta_cnt != data->d_delta_cnt) {
		log_error("d_delta_cnt no match");
		return RET_FAIL;
	} else if (cal_param.a_cnt != data->a_cnt) {
		log_error("a_cnt no match");
		return RET_FAIL;
	} else if (cal_param.side_angular_offset_cnt != data->side_angular_offset_cnt) {
		log_error("side_angular_offset_cnt no match");
		return RET_FAIL;
	} else if (cal_param.standard_encoder_cnt != data->standard_encoder_cnt) {
		log_error("standard_encoder_cnt no match");
		return RET_FAIL;
	}

	/* parameter */
	cal_param.lines_v = data->lines_v;
	snprintf(value, sizeof(value), "%d", cal_param.lines_v);
	iniparser_set(cal_param_ini, "parameter:lines_v", value);

	cal_param.lines_h = data->lines_h;
	snprintf(value, sizeof(value), "%d", cal_param.lines_h);
	iniparser_set(cal_param_ini, "parameter:lines_h", value);

	if (data->omega0.f > 0.0f && data->omega0.f < 360.0f) {
		cal_param.omega0 = data->omega0;
		snprintf(value, sizeof(value), "%g", cal_param.omega0.f);
		iniparser_set(cal_param_ini, "parameter:omega0", value);
	}

	cal_param.theta_min = data->theta_min;
	snprintf(value, sizeof(value), "%g", cal_param.theta_min.f);
	iniparser_set(cal_param_ini, "parameter:theta_min", value);

	cal_param.theta_max = data->theta_max;
	snprintf(value, sizeof(value), "%g", cal_param.theta_max.f);
	iniparser_set(cal_param_ini, "parameter:theta_max", value);

	cal_param.theta_tilt = data->theta_tilt;
	snprintf(value, sizeof(value), "%g", cal_param.theta_tilt.f);
	iniparser_set(cal_param_ini, "parameter:theta_tilt", value);

	cal_param.u = data->u;
	snprintf(value, sizeof(value), "%d", cal_param.u);
	iniparser_set(cal_param_ini, "parameter:u", value);

	cal_param.sat_peak_max = data->sat_peak_max;
	snprintf(value, sizeof(value), "%d", cal_param.sat_peak_max);
	iniparser_set(cal_param_ini, "parameter:sat_peak_max", value);

	cal_param.sat_peak_min = data->sat_peak_min;
	snprintf(value, sizeof(value), "%d", cal_param.sat_peak_min);
	iniparser_set(cal_param_ini, "parameter:sat_peak_min", value);

	cal_param.dist_max = data->dist_max;
	snprintf(value, sizeof(value), "%d", cal_param.dist_max);
	iniparser_set(cal_param_ini, "parameter:dist_max", value);

	cal_param.fov_max = data->fov_max;
	snprintf(value, sizeof(value), "%g", cal_param.fov_max.f);
	iniparser_set(cal_param_ini, "parameter:fov_max", value);

	cal_param.k = data->k;
	snprintf(value, sizeof(value), "%g", cal_param.k.f);
	iniparser_set(cal_param_ini, "parameter:k", value);

	/* side_angular_offset */
	for (int i = 0; i < cal_param.side_angular_offset_cnt; i++) {
		cal_param.side_angular_offset[i] = combine_uint8_to_uint16(parm_data[offset + 1], parm_data[offset]);
		snprintf(key, sizeof(key), "side_angular_offset:side%d_angular_offset", i);
		snprintf(value, sizeof(value), "%d", cal_param.side_angular_offset[i]);
		iniparser_set(cal_param_ini, key, value);
		offset += 2;
	}

	/* standard_encoder */
	for (int i = 0; i < cal_param.standard_encoder_cnt; i++) {
		cal_param.angle_face[i] = combine_uint8_to_uint16(parm_data[offset + 1], parm_data[offset]);
		snprintf(key, sizeof(key), "standard_encoder:angle_face_%d", i);
		snprintf(value, sizeof(value), "%d", cal_param.angle_face[i]);
		iniparser_set(cal_param_ini, key, value);
		offset += 2;
	}

	/* vcsel_index */
	for (int i = 0; i < cal_param.pixel_in_vcsel_id_cnt; i++) {
		cal_param.pixel_in_vcsel_id[i] = combine_uint8_to_uint16(parm_data[offset + 1], parm_data[offset]);
		snprintf(key, sizeof(key), "vcsel_index:pixel_in_vcsel_id%d", i);
		snprintf(value, sizeof(value), "%d", cal_param.pixel_in_vcsel_id[i]);
		iniparser_set(cal_param_ini, key, value);
		offset += 2;
	}

	/* light_intensity */
	for (int i = 0; i < cal_param.int_ld_cnt; i++) {
		cal_param.int_ld[i].f = transform_uint8_to_float(parm_data + offset);
		snprintf(key, sizeof(key), "light_intensity:int_ld%d", i);
		snprintf(value, sizeof(value), "%g", cal_param.int_ld[i].f);
		iniparser_set(cal_param_ini, key, value);
		offset += 4;
	}

	/* horizontal_angular_offset */
	for (int i = 0; i < cal_param.omega_delta_cnt; i++) {
		cal_param.omega_delta[i].f = transform_uint8_to_float(parm_data + offset);
		snprintf(key, sizeof(key), "horizontal_angular_offset:omega_delta%d", i);
		snprintf(value, sizeof(value), "%g", cal_param.omega_delta[i].f);
		iniparser_set(cal_param_ini, key, value);
		offset += 4;
	}

	/* zero_distance_offset */
	for (int i = 0; i < cal_param.d_delta_cnt; i++) {
		cal_param.d_delta[i].f = transform_uint8_to_float(parm_data + offset);
		snprintf(key, sizeof(key), "zero_distance_offset:d%d_delta", i);
		snprintf(value, sizeof(value), "%g", cal_param.d_delta[i].f);
		iniparser_set(cal_param_ini, key, value);
		offset += 4;
	}

	/* distance_factor */
	for (int i = 0; i < cal_param.a_cnt; i++) {
		cal_param.a[i].f = transform_uint8_to_float(parm_data + offset);
		snprintf(key, sizeof(key), "distance_factor:a%d", i);
		snprintf(value, sizeof(value), "%g", cal_param.a[i].f);
		iniparser_set(cal_param_ini, key, value);
		offset += 4;
	}

	// 更新到文件中
	int ret = core_update_ini_file(cal_param_ini, DEFAULT_CAL_CONF_PATH);
	init_decode_paramter();
	pl_calib_parm_init(&cal_param);

	return ret;
}

void get_cal_parm(void *arg)
{
	if (arg == NULL) {
		return;
	}

	management_id_t *msg = (management_id_t *)arg;
	uint8_t *ack_data = (uint8_t *)Malloc(cal_param_sizeof + 1);
	int ret = run_fail;
	int len = 0;
	int cpy_len = 0;

	if (ack_data == NULL) {
		ret = run_fail;
		len += 1;
		pack(msg, (uint8_t *)&ret, len, ACK);
		return;
	}

	ret = run_succ;
	ack_data[0] = (uint8_t)ret;
	len += 1;

	/* parameter */
	memcpy(ack_data + len, &(cal_param.lines_v), sizeof(cal_param.lines_v));
	len += sizeof(cal_param.lines_v);

	memcpy(ack_data + len, &(cal_param.lines_h), sizeof(cal_param.lines_h));
	len += sizeof(cal_param.lines_h);

	memcpy(ack_data + len, &(cal_param.omega0.u32), sizeof(cal_param.omega0));
	len += sizeof(cal_param.omega0);

	memcpy(ack_data + len, &(cal_param.theta_min.u32), sizeof(cal_param.theta_min));
	len += sizeof(cal_param.theta_min);

	memcpy(ack_data + len, &(cal_param.theta_max.u32), sizeof(cal_param.theta_max));
	len += sizeof(cal_param.theta_max);

	memcpy(ack_data + len, &(cal_param.theta_tilt.u32), sizeof(cal_param.theta_tilt));
	len += sizeof(cal_param.theta_tilt);

	memcpy(ack_data + len, &(cal_param.u), sizeof(cal_param.u));
	len += sizeof(cal_param.u);

	memcpy(ack_data + len, &(cal_param.sat_peak_max), sizeof(cal_param.sat_peak_max));
	len += sizeof(cal_param.sat_peak_max);

	memcpy(ack_data + len, &(cal_param.sat_peak_min), sizeof(cal_param.sat_peak_min));
	len += sizeof(cal_param.sat_peak_min);

	memcpy(ack_data + len, &(cal_param.dist_max), sizeof(cal_param.dist_max));
	len += sizeof(cal_param.dist_max);

	memcpy(ack_data + len, &(cal_param.fov_max.u32), sizeof(cal_param.fov_max));
	len += sizeof(cal_param.fov_max);

	memcpy(ack_data + len, &(cal_param.k.u32), sizeof(cal_param.k));
	len += sizeof(cal_param.k);

	/* cnt */
	memcpy(ack_data + len, &(cal_param.side_angular_offset_cnt), sizeof(cal_param.side_angular_offset_cnt));
	len += sizeof(cal_param.side_angular_offset_cnt);

	memcpy(ack_data + len, &(cal_param.standard_encoder_cnt), sizeof(cal_param.standard_encoder_cnt));
	len += sizeof(cal_param.standard_encoder_cnt);

	memcpy(ack_data + len, &(cal_param.pixel_in_vcsel_id_cnt), sizeof(cal_param.pixel_in_vcsel_id_cnt));
	len += sizeof(cal_param.pixel_in_vcsel_id_cnt);

	memcpy(ack_data + len, &(cal_param.int_ld_cnt), sizeof(cal_param.int_ld_cnt));
	len += sizeof(cal_param.int_ld_cnt);

	memcpy(ack_data + len, &(cal_param.omega_delta_cnt), sizeof(cal_param.omega_delta_cnt));
	len += sizeof(cal_param.omega_delta_cnt);

	memcpy(ack_data + len, &(cal_param.d_delta_cnt), sizeof(cal_param.d_delta_cnt));
	len += sizeof(cal_param.d_delta_cnt);

	memcpy(ack_data + len, &(cal_param.a_cnt), sizeof(cal_param.a_cnt));
	len += sizeof(cal_param.a_cnt);

	/* side_angular_offset */
	cpy_len = cal_param.side_angular_offset_cnt * sizeof(uint16_t);
	memcpy(ack_data + len, cal_param.side_angular_offset, cpy_len);
	len += cpy_len;

	/* standard_encoder */
	cpy_len = cal_param.standard_encoder_cnt * sizeof(uint16_t);
	memcpy(ack_data + len, cal_param.angle_face, cpy_len);
	len += cpy_len;

	/* vcsel_index */
	cpy_len = cal_param.pixel_in_vcsel_id_cnt * sizeof(uint16_t);
	memcpy(ack_data + len, cal_param.pixel_in_vcsel_id, cpy_len);
	len += cpy_len;

	/* light_intensity */
	cpy_len = cal_param.int_ld_cnt * sizeof(float_union_t);
	memcpy(ack_data + len, cal_param.int_ld, cpy_len);
	len += cpy_len;

	/* horizontal_angular_offset */
	cpy_len = cal_param.omega_delta_cnt * sizeof(float_union_t);
	memcpy(ack_data + len, cal_param.omega_delta, cpy_len);
	len += cpy_len;

	/* zero_distance_offset */
	cpy_len = cal_param.d_delta_cnt * sizeof(float_union_t);
	memcpy(ack_data + len, cal_param.d_delta, cpy_len);
	len += cpy_len;

	/* distance_factor */
	cpy_len = cal_param.a_cnt * sizeof(float_union_t);
	memcpy(ack_data + len, cal_param.a, cpy_len);
	len += cpy_len;

	pack(msg, ack_data, len, ACK);
	Free((void **)&ack_data);
	return;
}

void set_cal_parm(void *arg)
{
	if (arg == NULL) {
		return;
	}

	management_id_t *msg = (management_id_t *)arg;
	int ret = RET_FAIL;

	calibration_parm_head_t *calibration_info = (calibration_parm_head_t *)(msg->buf + 2);

	ret = update_calibration_config(calibration_info);

	if (ret == RET_OK) {
		ret = run_succ;
	} else {
		ret = run_fail;
	}

	pack(msg, (uint8_t *)&ret, 1, ACK);
	return;
}
