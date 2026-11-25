/*
 * @Author       : kaihong.yang kaihong.yang@vitalchem.com
 * @Date         : 2026-06-30
 * @FilePath     : /you_see_arm/ys_business/decode/ys_decode_common.c
 * @Description  : 解码层共享实现 —— 所有机型共同的算法逻辑
 * Copyright (c) 2026 by Vitalchem, All Rights Reserved.
 */

#include "ys_decode.h"
#include "ys_decode_internal.h"

#include "../algo/ys_algo_calib_reflectivity.h"
#include "../algo/ys_algo_spot_slope.h"

#include "../../common/common.h"
#include "../../core/ys_base.h"
#include "../../core/ys_conf.h"
#include "../../core/ys_core_def.h"

#include <math.h>
#include <string.h>

#define PC_HEAD_SIZE (sizeof(point_cloud_head_t))

static int g_piexl_max_cnt = 0;
float *g_theta;
static char *g_vcsel_index;
float *g_omega_delta;
float *g_distance_factor;
float *g_delta_distance_index;
float *g_ld_intensity_coefficient;

const uint32_t k_error_ft = 0x10;
const double k_radian = (PI / 180.0);

/******************************************************************************/
/*                            初始化 / 析构                                    */
/******************************************************************************/

static int algo_free(void)
{
	Free((void **)&g_theta);
	Free((void **)&g_vcsel_index);
	Free((void **)&g_omega_delta);
	Free((void **)&g_distance_factor);
	Free((void **)&g_delta_distance_index);
	Free((void **)&g_ld_intensity_coefficient);
	return RET_OK;
}

static int algo_malloc(void)
{
	algo_free();
	g_theta = (float *)Malloc(sizeof(float) * g_piexl_max_cnt);
	g_vcsel_index = (char *)Malloc(sizeof(char) * g_piexl_max_cnt);
	g_omega_delta = (float *)Malloc(sizeof(float) * g_piexl_max_cnt);
	g_distance_factor = (float *)Malloc(sizeof(float) * g_piexl_max_cnt);
	g_delta_distance_index = (float *)Malloc(sizeof(float) * g_piexl_max_cnt);
	g_ld_intensity_coefficient = (float *)Malloc(sizeof(float) * g_piexl_max_cnt);

	return RET_OK;
}

static int calculate_vcsel_index(void)
{
	int mod_result;
	int ret = RET_FAIL;

	if (vcsel_cnt == 0) {
		vcsel_cnt = 4;
	}
	const int one_vcsel_cnt = cal_param.lines_v / vcsel_cnt;

	pthread_mutex_lock(&(cal_lock));
	if (cal_param.pixel_in_vcsel_id_cnt < vcsel_cnt) {
		log_fatal("missing calibration parameters:[vcsel_index]");
		ret = RET_FAIL;
		goto end;
	}

	if (cal_param.int_ld_cnt < vcsel_cnt) {
		log_fatal("missing calibration parameters:[light_intensity]");
		ret = RET_FAIL;
		goto end;
	}

	for (int i = 0; i < cal_param.lines_v; i++) {
		mod_result = i % (one_vcsel_cnt * 2);

		if (mod_result >= 0 && mod_result < one_vcsel_cnt) {
			g_vcsel_index[i] = config.vcsel_start_location;
		} else if (mod_result >= one_vcsel_cnt && mod_result < (one_vcsel_cnt * 2)) {
			g_vcsel_index[i] = 1 - config.vcsel_start_location;
		} else {
			g_vcsel_index[i] = -1;
		}

		for (int j = 0; j < vcsel_cnt; j++) {
			if (j >= 1) {
				if (cal_param.pixel_in_vcsel_id[j - 1] < i && i <= cal_param.pixel_in_vcsel_id[j]) {
					g_ld_intensity_coefficient[i] = 702.0 * cal_param.int_ld[j].f;
				} else {
					g_ld_intensity_coefficient[i] = 702.0 * 1.0;
				}
			} else {
				if (0 <= i && i <= cal_param.pixel_in_vcsel_id[j]) {
					g_ld_intensity_coefficient[i] = 702.0 * cal_param.int_ld[j].f;
				} else {
					g_ld_intensity_coefficient[i] = 702.0 * 1.0;
				}
			}
		}
	}
	ret = RET_OK;
end:
	pthread_mutex_unlock(&(cal_lock));
	return ret;
}

static int parse_ini_file(void)
{
	if (g_piexl_max_cnt != cal_param.omega_delta_cnt || cal_param.omega_delta == NULL) {
		log_fatal("missing calibration parameters:[horizontal_angular_offset]");
		return RET_FAIL;
	}

	if (g_piexl_max_cnt != cal_param.d_delta_cnt || cal_param.d_delta == NULL) {
		log_fatal("missing calibration parameters:[zero_distance_offset]");
		return RET_FAIL;
	}

	if (g_piexl_max_cnt > cal_param.a_cnt || cal_param.a == NULL) {
		log_fatal("missing calibration parameters:[distance_factor]");
		return RET_FAIL;
	}

	for (int i = 0; i < g_piexl_max_cnt; i++) {
		g_delta_distance_index[i] = cal_param.d_delta[i].f;
		g_distance_factor[i] = cal_param.a[i].f;
		g_omega_delta[i] = cal_param.omega_delta[i].f;
	}

	return RET_OK;
}

int init_decode_paramter(void)
{
	int ret = RET_FAIL;
	ret = init_algo_reflectivity(cal_param.sat_peak_max);
	if (ret != RET_OK) {
		return RET_FAIL;
	}

	ret = init_algo_spot_slope();
	if (ret != RET_OK) {
		log_error("init algo spot slope failed!");
		return RET_FAIL;
	}

	pthread_mutex_lock(&(cal_lock));
	if (g_piexl_max_cnt != cal_param.lines_v) {
		g_piexl_max_cnt = cal_param.lines_v;
		algo_malloc();
	}
	pthread_mutex_unlock(&(cal_lock));

	decode_calculate_theta();

	ret = calculate_vcsel_index();
	if (ret != RET_OK) {
		return RET_FAIL;
	}

	ret = parse_ini_file();
	if (ret != RET_OK) {
		return RET_FAIL;
	}

	decode_init_model_encoder();

	return RET_OK;
}

int uninit_decode_paramter(void)
{
	int ret = RET_FAIL;
	ret = uninit_algo_reflectivity();
	if (ret != RET_OK) {
		return RET_FAIL;
	}

	ret = uninit_algo_spot_slope();
	if (ret != RET_OK) {
		return RET_FAIL;
	}

	ret = algo_free();
	if (ret != RET_OK) {
		return RET_FAIL;
	}

	return RET_OK;
}

/******************************************************************************/
/*                           距离 / 反射率 / 坐标转换                           */
/******************************************************************************/

static uint32_t decode_tof(const uint8_t tof[3])
{
	uint32_t value = combine_uint8_to_uint32(0x00, tof[2], tof[1], tof[0]);
#if CS96_GAMMA == 1
	// Gamma：高 4 位保留，低 20 位为 14 位整数、6 位小数，单位 ns。
	value &= 0xFFFFF;
#endif
	return value;
}

static float calculate_distance(int pixel_id, uint32_t ft, uint16_t wd, uint16_t pk)
{
#if CS96_GAMMA == 1
	if (ft <= k_error_ft) {
		return 0.0f;
	}
	// 原始值 / 64 得到 ns，再转为 ps；往返飞行时间除以 2，保留小数精度。
	float distance = velocitylight * (ft * (1000.0 / 128.0)) * g_distance_factor[pixel_id];
#else
	static float distance = 0;
	if (ft <= k_error_ft) {
		return distance;
	}

	distance = velocitylight * (ft >> 1) * g_distance_factor[pixel_id];
#endif
	distance += g_delta_distance_index[pixel_id];
	return distance;
}

static uint8_t calculate_reflect(int pixel_id, uint16_t slot_number, float r, uint16_t pk)
{
	if (r <= 0.0f) {
		return 0;
	}
	return get_reflect((uint16_t)pixel_id, slot_number, r, pk, g_ld_intensity_coefficient[pixel_id]);
}

static int algo_coordinate_system(uint8_t *dest, float r, float theta, float omega, uint8_t albedo)
{
	switch (config.coordinate_mode) {
	case CARTESIAN_COORDINATE_SYSTEM: {
		static cartesian_point_data_t point_data;
		point_data.x.f = r * cos(theta) * cos(omega);
#if CS96_BETA == 1 || CS96_GAMMA == 1
		point_data.y.f = r * cos(theta) * sin(omega) * -1.0f;
#else
		point_data.y.f = r * cos(theta) * sin(omega);
#endif
		point_data.z.f = r * sin(theta);
		point_data.albedo = albedo;
		memcpy(dest, (uint8_t *)&point_data, sizeof(point_data));
		return sizeof(point_data);
	}
	case SPHERICAL_COORDINATE_SYSTEM: {
		static spherical_point_data_t point_data;
		point_data.r.f = r;
		point_data.theta.f = theta;
		point_data.omega.f = omega;
		point_data.albedo = albedo;
		memcpy(dest, (uint8_t *)&point_data, sizeof(point_data));
		return sizeof(point_data);
	}
	default:
		return 0;
		break;
	}
}

/******************************************************************************/
/*                             公开函数                                        */
/******************************************************************************/

int deal_orgin_point_cloud_data(raw_protocal_data_packet_t *raw_data, point_cloud_packet_t *dest_data)
{
	if (raw_data == NULL || dest_data == NULL) {
		log_error("input data is NULL");
		return 0;
	}

	org_point_cloud_packet_t *org_data = (org_point_cloud_packet_t *)(raw_data->value);
	if (org_data == NULL) {
		log_error("org data is NULL");
		return 0;
	}

	point_cloud_head_t *p_head = &(dest_data->p_head);
	if (p_head == NULL) {
		log_error("p head is NULL");
		return 0;
	}

	int len = decode_build_protocal_head(raw_data, p_head);
	if (len == RET_FAIL) {
		log_error("build protocal head failed");
		return 0;
	}

	float r = 0.0f;
	uint32_t ft[2] = { 0 };
	uint16_t wd[2] = { 0 };
	uint16_t pk[2] = { 0 };
	uint8_t albedo = 0;

	float horizontal_corner = decode_get_horizontal_corners(p_head, org_data);

	for (int i = 0; i < cal_param.lines_v; i++) {
		memset(ft, 0, sizeof(ft));
		memset(wd, 0, sizeof(wd));
		memset(pk, 0, sizeof(pk));
		uint8_t *data = org_data->value + (i * sizeof(point_cloud_pixel_data_t));

		float pitch_angle_theta = g_theta[i];

		float horizontal_corners_omega = (horizontal_corner + g_omega_delta[i]);
		horizontal_corners_omega *= k_radian;

		if (config.echo_mode == DOUBLE_ECHO_MODE) {
			for (int j = 1, k = 1; j >= 0; j--, k--) {
				echo_data_t *echo_data = (echo_data_t *)(data + j * sizeof(echo_data_t));
				ft[k] = decode_tof(echo_data->ft);

				if (ft[k] == k_error_ft) {
					continue;
				}

				{
					uint32_t et_val = decode_tof(echo_data->et);
					wd[k] = (uint16_t)(et_val - ft[k]);
					pk[k] = echo_data->pk;
				}
			}
		} else {
			for (int j = 1; j >= 0; j--) {
				if (config.echo_type == FIRST_ECHO_TYPE) {
					j = 0;
				}

				echo_data_t *echo_data = (echo_data_t *)(data + j * sizeof(echo_data_t));
				uint32_t tmp_ft = decode_tof(echo_data->ft);

				if (tmp_ft == k_error_ft) {
					continue;
				}

				ft[0] = pk[0] > echo_data->pk ? ft[0] : tmp_ft;
				{
					uint32_t et_val = decode_tof(echo_data->et);
					wd[0] = pk[0] > echo_data->pk ? wd[0] : (uint16_t)(et_val - tmp_ft);
				}
				pk[0] = pk[0] > echo_data->pk ? pk[0] : echo_data->pk;

				if (config.echo_type == LAST_ECHO_TYPE) {
					break;
				}
			}
		}

		for (int k = 0; k < config.echo_mode + 1; k++) {
			r = calculate_distance(i, ft[k], wd[k], pk[k]);
			albedo = calculate_reflect(i, p_head->slot_number, r, pk[k]);

			uint8_t *dest = dest_data->value + len - PC_HEAD_SIZE;
			len += algo_coordinate_system(dest, r, pitch_angle_theta, horizontal_corners_omega, albedo);
		}
	}

	return len;
}

int inverted_orgin_point_cloud_data(raw_protocal_data_packet_t *raw_data, uint8_t *dest_data)
{
	int len = 0;

	memcpy(dest_data + len, raw_data, sizeof(raw_protocal_data_packet_t) - 1);
	len += sizeof(raw_protocal_data_packet_t) - 1;

	org_point_cloud_packet_t *org_point_cloud_packet = (org_point_cloud_packet_t *)(raw_data->value);
	memcpy(dest_data + len, org_point_cloud_packet, sizeof(org_point_cloud_packet_t) - 1);
	len += sizeof(org_point_cloud_packet_t) - 1;

	point_cloud_pixel_data_t *pixel_data = NULL;
	for (int i = cal_param.lines_v - 1; i >= 0; i--) {
		pixel_data = (point_cloud_pixel_data_t *)(org_point_cloud_packet->value +
							  (i * sizeof(point_cloud_pixel_data_t)));
		memcpy(dest_data + len, pixel_data, sizeof(point_cloud_pixel_data_t));
		len += sizeof(point_cloud_pixel_data_t);
	}

	return len;
}
