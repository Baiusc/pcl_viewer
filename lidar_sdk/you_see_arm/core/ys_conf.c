/*
 * @Author       : kaihong.yang kaihong.yang@vitalchem.com
 * @Date         : 2024-07-13 03:05:54
 * @LastEditors  : kaihong.yang kaihong.yang@vitalchem.com
 * @LastEditTime : 2025-10-15 04:25:31
 * @FilePath     : /you_see_arm/core/ys_conf.c
 * @Description  :
 * Copyright (c) 2024 by Vitalchem, All Rights Reserved.
 */

#include "ys_base.h"
#include "ys_conf.h"
#include "ys_flash_commit.h"

#include "../common/common.h"
#include <fcntl.h>
#include <limits.h>
#include <sys/stat.h>
#include <unistd.h>

dictionary *conf_ini = NULL; // 全局存储 - 配置参数
configuration_t config; // 全局存储 - 配置参数
pthread_mutex_t config_lock __attribute__((aligned(8))); // 确保 8 字节对齐，否则在 Arm 架构会 Bus error

dictionary *cal_param_ini = NULL; // 全局存储 - 标定参数
calibration_parm_t cal_param; // 全局存储 - 标定参数
pthread_mutex_t cal_lock __attribute__((aligned(8))); // 确保 8 字节对齐，否则在 Arm 架构会 Bus error

int cal_param_sizeof = 0; // 标定参数的数据大小

int vcsel_cnt = 0; // VCCSEL 的条数
uint16_t total_slots = 800; // 转镜扫描一面中的 slot 数量

#if CS96_BETA == 1 || CS96_GAMMA == 1
#include "../third_party/minicsv/minicsv.h"

float *g_encoder_angle; // 电机整数刻度角度表
int g_encoder_angle_cnt; // 电机整数刻度角度表的条数

static int init_encoder_angle_table()
{
	FILE *fp = fopen(DEFAULT_ENCODER_ANGLE_FILE_PATH, "r");
	if (!fp) {
		log_error("无法打开文件 %s", DEFAULT_ENCODER_ANGLE_FILE_PATH);
		return RET_FAIL;
	}

	char buf[32]; // 每行缓冲区
	char *cols[2]; // 最多 2 列
	size_t cols_count;
	size_t line = 0;

	while (fgets(buf, sizeof(buf), fp)) {
		line++;
		char *r = buf;
		while (*r != '\0') {
			r = minicsv_parse_line(r, cols, &cols_count, 2);
			minicsv_trim_cols(cols, cols_count);

			if (cols_count < 2) {
				log_error("第 %03d 行解析失败，列数不足：%zu", line, cols_count);
				continue;
			}

			if (cols[0][0] < '0' || cols[0][0] > '9') {
				log_debug("第 %03d 行跳过非数据行：[%s],[%s]", line, cols[0], cols[1]);
				continue;
			}

			g_encoder_angle_cnt++;
			float *p = (float *)realloc(g_encoder_angle, g_encoder_angle_cnt * sizeof(float));
			g_encoder_angle = p;
			g_encoder_angle[g_encoder_angle_cnt - 1] = atof(cols[1]);

			log_debug("第 %03d 行解析结果：[%03d],[%.06f]", line, g_encoder_angle_cnt - 1,
				  g_encoder_angle[g_encoder_angle_cnt - 1]);

			if (*r == '\0')
				break;
		}
	}

	fclose(fp);
	return RET_OK;
}

static void uninit_encoder_angle_table()
{
	if (g_encoder_angle) {
		free(g_encoder_angle);
		g_encoder_angle = NULL;
		g_encoder_angle_cnt = 0;
	}
}
#endif

int core_update_ini_file(dictionary *ini, const char *path)
{
	char target[PATH_MAX], temporary[PATH_MAX];
	struct stat st;
	if (!realpath(path, target)) {
		if (errno != ENOENT || !lstat(path, &st) ||
		    snprintf(target, sizeof(target), "%s", path) >= (int)sizeof(target))
			return RET_FAIL;
	}
	mode_t mode = 0600;
	if (!stat(target, &st)) {
		if (!S_ISREG(st.st_mode))
			return RET_FAIL;
		mode = st.st_mode & 0777;
	} else if (errno != ENOENT) {
		return RET_FAIL;
	}
	if (snprintf(temporary, sizeof(temporary), "%s.tmp.XXXXXX", target) >= (int)sizeof(temporary))
		return RET_FAIL;

	int fd = mkstemp(temporary);
	if (fd < 0)
		return RET_FAIL;
	FILE *file = fdopen(fd, "w");
	int bad = 1;
	if (file) {
		iniparser_dump_ini(ini, file);
		bad = fflush(file) || ferror(file) || fchmod(fd, mode) || fsync(fd);
		if (fclose(file))
			bad = 1;
	} else {
		close(fd);
	}
	const char *kind = !strcmp(path, DEFAULT_CAL_CONF_PATH) ? "calibration" : NULL;
	if (!bad && (ys_flash_commit(kind, temporary) || rename(temporary, target)))
		bad = 1;
	if (!bad) {
		char *slash = strrchr(target, '/');
		const char *parent = ".";
		if (slash) {
			*slash = '\0';
			parent = *target ? target : "/";
		}
		int dirfd = open(parent, O_RDONLY | O_DIRECTORY | O_CLOEXEC);
		bad = dirfd < 0 || fsync(dirfd);
		if (dirfd >= 0)
			close(dirfd);
	}
	if (bad) {
		unlink(temporary);
		log_error("保存配置失败：%s", path);
	}
	return bad ? RET_FAIL : RET_OK;
}

int core_load_cal_ini()
{
	char key[64];
	int count = 0;
	int len = 0;
	int ret = RET_FAIL;

#if CS96_BETA == 1 || CS96_GAMMA == 1
	init_encoder_angle_table();
#endif

	ret = pthread_mutex_init(&(cal_lock), NULL);
	if (ret != 0) {
		log_error("pthread_mutex_init failed: %s\n", strerror(ret));
		return RET_FAIL;
	}

	log_info("Read conf %s", DEFAULT_CAL_CONF_PATH);

	cal_param_ini = iniparser_load(DEFAULT_CAL_CONF_PATH);
	if (cal_param_ini == NULL) {
		log_error("Can't load '%s'\n", DEFAULT_CAL_CONF_PATH);
		return RET_FAIL;
	}

	/* parameter */
	log_debug("[parameter]");
	snprintf(key, sizeof(key), "parameter:lines_v");
	cal_param.lines_v = iniparser_getint(cal_param_ini, key, 96);
	len += sizeof(cal_param.lines_v);
	log_debug("lines_v = %d", cal_param.lines_v);

	snprintf(key, sizeof(key), "parameter:lines_h");
	cal_param.lines_h = iniparser_getint(cal_param_ini, key, 800);
	len += sizeof(cal_param.lines_h);
	log_debug("lines_h = %d", cal_param.lines_h);
	switch (config.fps) {
	case 10:
		total_slots = cal_param.lines_h;
		break;
	case 20:
		total_slots = cal_param.lines_h / 2;
		break;
	default:
		total_slots = cal_param.lines_h;
		break;
	}

	snprintf(key, sizeof(key), "parameter:omega0");
	cal_param.omega0.f = (float)iniparser_getdouble(cal_param_ini, key, -60.0f);
	len += sizeof(cal_param.omega0);
	log_debug("omega0 = %g", cal_param.omega0.f);

	snprintf(key, sizeof(key), "parameter:theta_min");
	cal_param.theta_min.f = (float)iniparser_getdouble(cal_param_ini, key, -10.0f);
	len += sizeof(cal_param.theta_min);
	log_debug("theta_min = %g", cal_param.theta_min.f);

	snprintf(key, sizeof(key), "parameter:theta_max");
	cal_param.theta_max.f = (float)iniparser_getdouble(cal_param_ini, key, 10.0f);
	len += sizeof(cal_param.theta_max);
	log_debug("theta_max = %g", cal_param.theta_max.f);

	snprintf(key, sizeof(key), "parameter:theta_tilt");
	cal_param.theta_tilt.f = (float)iniparser_getdouble(cal_param_ini, key, -2.5f);
	len += sizeof(cal_param.theta_tilt);
	log_debug("theta_tilt = %g", cal_param.theta_tilt.f);

	snprintf(key, sizeof(key), "parameter:u");
	cal_param.u = iniparser_getint(cal_param_ini, key, 47);
	len += sizeof(cal_param.u);
	log_debug("u = %d", cal_param.u);

	snprintf(key, sizeof(key), "parameter:sat_peak_max");
	cal_param.sat_peak_max = iniparser_getint(cal_param_ini, key, 1612);
	len += sizeof(cal_param.sat_peak_max);
	log_debug("sat_peak_max = %d", cal_param.sat_peak_max);

	snprintf(key, sizeof(key), "parameter:sat_peak_min");
	cal_param.sat_peak_min = iniparser_getint(cal_param_ini, key, 0);
	len += sizeof(cal_param.sat_peak_min);
	log_debug("sat_peak_min = %d", cal_param.sat_peak_min);

	snprintf(key, sizeof(key), "parameter:dist_max");
	cal_param.dist_max = iniparser_getint(cal_param_ini, key, 200);
	len += sizeof(cal_param.dist_max);
	log_debug("dist_max = %d", cal_param.dist_max);

	snprintf(key, sizeof(key), "parameter:fov_max");
	cal_param.fov_max.f = (float)iniparser_getdouble(cal_param_ini, key, 120.0f);
	len += sizeof(cal_param.fov_max);
	log_debug("fov_max = %g", cal_param.fov_max.f);

	snprintf(key, sizeof(key), "parameter:k");
	cal_param.k.f = (float)iniparser_getdouble(cal_param_ini, key, 1.0f);
	len += sizeof(cal_param.k);
	log_debug("k = %g", cal_param.k.f);
	log_debug("");

	/* fpga */
	log_debug("[fpga]");
	snprintf(key, sizeof(key), "fpga:threshold_time");
	cal_param.threshold_time = (uint8_t)iniparser_getint(cal_param_ini, key, 60);
	len += sizeof(cal_param.threshold_time);
	log_debug("threshold_time = %d", cal_param.threshold_time);

	snprintf(key, sizeof(key), "fpga:threshold_back");
	cal_param.threshold_back = (uint8_t)iniparser_getint(cal_param_ini, key, 200);
	len += sizeof(cal_param.threshold_back);
	log_debug("threshold_back = %d", cal_param.threshold_back);

	snprintf(key, sizeof(key), "fpga:vop_bias_freq");
	cal_param.vop_bias_freq = iniparser_getint(cal_param_ini, key, 10000);
	len += sizeof(cal_param.vop_bias_freq);
	log_debug("vop_bias_freq = %d", cal_param.vop_bias_freq);

	snprintf(key, sizeof(key), "fpga:vop_bias_duty");
	cal_param.vop_bias_duty = iniparser_getint(cal_param_ini, key, 4000);
	len += sizeof(cal_param.vop_bias_duty);
	log_debug("vop_bias_duty = %d", cal_param.vop_bias_duty);
	log_debug("");

	/* side_angular_offset */
	log_debug("[side_angular_offset]");
	len += sizeof(cal_param.side_angular_offset_cnt);
	count = iniparser_getsecnkeys(cal_param_ini, "side_angular_offset");
	cal_param.side_angular_offset_cnt = count;
	cal_param.side_angular_offset = (uint16_t *)calloc(count, sizeof(uint16_t));
	len += (count * sizeof(uint16_t));
	for (int i = 0; i < count; i++) {
		snprintf(key, sizeof(key), "side_angular_offset:side%d_angular_offset", i);
		cal_param.side_angular_offset[i] = (uint16_t)iniparser_getuint64(cal_param_ini, key, 0.0f);
		log_debug("side%d_angular_offset = %d", i, cal_param.side_angular_offset[i]);
	}
	log_debug("");

	/* standard_encoder */
	log_debug("[standard_encoder]");
	len += sizeof(cal_param.standard_encoder_cnt);
	count = iniparser_getsecnkeys(cal_param_ini, "standard_encoder");
	cal_param.standard_encoder_cnt = count;
	cal_param.angle_face = (uint16_t *)calloc(count, sizeof(uint16_t));
	len += (count * sizeof(uint16_t));
	for (int i = 0; i < count; i++) {
		snprintf(key, sizeof(key), "standard_encoder:angle_face_%d", i + 1);
		cal_param.angle_face[i] = (uint16_t)iniparser_getuint64(cal_param_ini, key, 0);
		log_debug("angle_face_%d = %d", i + 1, cal_param.angle_face[i]);
	}
	log_debug("");

	/* side_light */
	log_debug("[side_light]");
	len += sizeof(cal_param.side_light);
	snprintf(key, sizeof(key), "side_light:side_light");
	cal_param.side_light = (uint8_t)iniparser_getuint64(cal_param_ini, key, 0xF);
	log_debug("side_light = %d", cal_param.side_light);
	log_debug("");

	/* vcsel_index */
	log_debug("[vcsel_index]");
	len += sizeof(cal_param.pixel_in_vcsel_id_cnt);
	count = iniparser_getsecnkeys(cal_param_ini, "vcsel_index");
	vcsel_cnt = count;
	cal_param.pixel_in_vcsel_id_cnt = count;
	cal_param.pixel_in_vcsel_id = (uint16_t *)calloc(count, sizeof(uint16_t));
	len += (count * sizeof(uint16_t));
	for (int i = 0; i < count; i++) {
		snprintf(key, sizeof(key), "vcsel_index:pixel_in_vcsel_id%d", i);
		cal_param.pixel_in_vcsel_id[i] = iniparser_getint(cal_param_ini, key, 0);
		log_debug("pixel_in_vcsel_id%d = %d", i, cal_param.pixel_in_vcsel_id[i]);
	}
	log_debug("");

	/* light_intensity */
	log_debug("[light_intensity]");
	len += sizeof(cal_param.int_ld_cnt);
	count = iniparser_getsecnkeys(cal_param_ini, "light_intensity");
	cal_param.int_ld_cnt = count;
	cal_param.int_ld = (float_union_t *)calloc(count, sizeof(float_union_t));
	len += (count * sizeof(float_union_t));
	for (int i = 0; i < count; i++) {
		snprintf(key, sizeof(key), "light_intensity:int_ld%d", i);
		cal_param.int_ld[i].f = (float)iniparser_getdouble(cal_param_ini, key, 1.0f);
		log_debug("int_ld%d = %g", i, cal_param.int_ld[i].f);
	}
	log_debug("");

	/* horizontal_angular_offset */
	log_debug("[horizontal_angular_offset]");
	len += sizeof(cal_param.omega_delta_cnt);
	count = iniparser_getsecnkeys(cal_param_ini, "horizontal_angular_offset");
	cal_param.omega_delta_cnt = cal_param.lines_v;
	cal_param.omega_delta = (float_union_t *)calloc(cal_param.omega_delta_cnt, sizeof(float_union_t));
	len += (cal_param.omega_delta_cnt * sizeof(float_union_t));
	for (int i = 0; i < cal_param.omega_delta_cnt; i++) {
		snprintf(key, sizeof(key), "horizontal_angular_offset:omega_delta%d", i);
		cal_param.omega_delta[i].f = (float)iniparser_getdouble(cal_param_ini, key, -1.3f);
		log_debug("omega_delta%d = %g", i, cal_param.omega_delta[i].f);
	}
	log_debug("");

	/* zero_distance_offset */
	log_debug("[zero_distance_offset]");
	len += sizeof(cal_param.d_delta_cnt);
	count = iniparser_getsecnkeys(cal_param_ini, "zero_distance_offset");
	cal_param.d_delta_cnt = cal_param.lines_v;
	cal_param.d_delta = (float_union_t *)calloc(cal_param.d_delta_cnt, sizeof(float_union_t));
	len += (cal_param.d_delta_cnt * sizeof(float_union_t));
	for (int i = 0; i < cal_param.d_delta_cnt; i++) {
		snprintf(key, sizeof(key), "zero_distance_offset:d%d_delta", i);
		cal_param.d_delta[i].f = (float)iniparser_getdouble(cal_param_ini, key, 0.0f);
		log_debug("d%d_delta = %g", i, cal_param.d_delta[i].f);
	}
	log_debug("");

	/* distance_factor */
	log_debug("[distance_factor]");
	len += sizeof(cal_param.a_cnt);
	count = iniparser_getsecnkeys(cal_param_ini, "distance_factor");
	cal_param.a_cnt = cal_param.lines_v;
	cal_param.a = (float_union_t *)calloc(cal_param.a_cnt, sizeof(float_union_t));
	len += (cal_param.a_cnt * sizeof(float_union_t));
	for (int i = 0; i < cal_param.a_cnt; i++) {
		snprintf(key, sizeof(key), "distance_factor:a%d", i);
		cal_param.a[i].f = (float)iniparser_getdouble(cal_param_ini, key, 1.0f);
		log_debug("a%d = %g", i, cal_param.a[i].f);
	}

	cal_param_sizeof = len;
	log_info("Read conf %s success", DEFAULT_CAL_CONF_PATH);

	log_info("Lidar's resolution %d(v) x %d(h)", cal_param.lines_v, cal_param.lines_h);
	return RET_OK;
}

int core_load_conf_ini(const char *conf_path)
{
	char key[64];
	const char *str_value;
	int ret = RET_FAIL;

	ret = pthread_mutex_init(&(config_lock), NULL);
	if (ret != 0) {
		log_error("pthread_mutex_init failed: %s\n", strerror(ret));
		return RET_FAIL;
	}

	log_info("Read conf %s", conf_path);

	conf_ini = iniparser_load(conf_path);
	if (conf_ini == NULL) {
		log_error("Can't load '%s'\n", conf_path);
		return RET_FAIL;
	}

	/* system */
	snprintf(key, sizeof(key), "system:log_level");
	config.log_level = iniparser_getint(conf_ini, key, LOG_INFO);
	if (config.log_level < LOG_TRACE || config.log_level > LOG_FATAL) {
		config.log_level = LOG_INFO;
	}
	log_set_level(config.log_level);
	log_debug("[system]");
	log_debug("log_level = %d", config.log_level);

	snprintf(key, sizeof(key), "system:log_file_path");
	str_value = iniparser_getstring(conf_ini, key, NULL);
	if (str_value != NULL && strlen(str_value) > 0) {
		snprintf(config.log_file_path, sizeof(config.log_file_path), "%s", str_value);
		log_debug("log_file_path = %s", config.log_file_path);
		config.fp = fopen(config.log_file_path, "a");
		if (config.fp != NULL) {
			log_add_fp(config.fp, config.log_level);
		} else {
			log_error("open log file fail reason = %s", strerror(errno));
		}
	} else {
		log_debug("log_file_path not configured or empty");
		config.log_file_path[0] = '\0'; // 清空路径
	}

	snprintf(key, sizeof(key), "system:work_mode");
	int requested_work_mode = iniparser_getint(conf_ini, key, RAW_MODE);
	config.work_mode = requested_work_mode;
	log_debug("work_mode = %d", config.work_mode);
#ifdef YS_WORK_MODE_MASK
	if (requested_work_mode < 0 || requested_work_mode > GRAY_MODE ||
	    !(YS_WORK_MODE_MASK & (1U << requested_work_mode))) {
		log_error("工作模式 %d 不在当前固件支持范围内", requested_work_mode);
		return RET_FAIL;
	}
#endif

	config.enable_reflectivity = iniparser_getboolean(conf_ini, "system:enable_reflectivity", 0);
	config.enable_spot_slope = iniparser_getboolean(conf_ini, "system:enable_spot_slope", 0);
	snprintf(key, sizeof(key), "system:echo_mode");
	config.echo_mode = iniparser_getint(conf_ini, key, SINGLE_ECHO_MODE);
	log_debug("echo_mode = %d", config.echo_mode);

	snprintf(key, sizeof(key), "system:echo_type");
	config.echo_type = iniparser_getint(conf_ini, key, STRONGEST_ECHO_TYPE);
	log_debug("echo_type = %d", config.echo_type);

	snprintf(key, sizeof(key), "system:coordinate_mode");
	config.coordinate_mode = iniparser_getint(conf_ini, key, CARTESIAN_COORDINATE_SYSTEM);
	log_debug("coordinate_mode = %d", config.coordinate_mode);
	log_debug("");

	/* hardware */
	log_debug("[hardware]");
	snprintf(key, sizeof(key), "hardware:vcsel_start_location");
	str_value = iniparser_getstring(conf_ini, key, NULL);
	log_debug("vcsel_start_location = %s", str_value);
	if (str_value != NULL && !strcmp(str_value, "right")) {
		config.vcsel_start_location = RIGHT_VCSEL;
	} else {
		config.vcsel_start_location = LEFT_VCSEL;
	}

	snprintf(key, sizeof(key), "hardware:fps");
	config.fps = iniparser_getint(conf_ini, key, 10);
	log_debug("fps = %d", config.fps);
#if LOCAL_TEST == 0
	// int ret = RET_FAIL;
	// int fps = RET_FAIL;
	// do {
	// 	ret = sys_fps_set(config.fps);
	// 	sleep(1);
	// 	fps = sys_fps_get();
	// } while (ret == RET_OK && fps == config.fps);
#endif
	log_debug("");

	/* network */
	log_debug("[network]");
	snprintf(key, sizeof(key), "network:center_broadcast_port");
	config.center_broadcast_port = iniparser_getint(conf_ini, key, 55000);
	log_debug("center_broadcast_port = %d", config.center_broadcast_port);

	snprintf(key, sizeof(key), "network:center_ip");
	str_value = iniparser_getstring(conf_ini, key, NULL);
	if (str_value != NULL) {
		snprintf(config.center_ip, sizeof(config.center_ip), "%s", str_value);
	}
	log_debug("center_ip = %s", config.center_ip);

	snprintf(key, sizeof(key), "network:local_cmd_port");
	config.local_cmd_port = iniparser_getint(conf_ini, key, 0);
	log_debug("local_cmd_port = %d", config.local_cmd_port);

	snprintf(key, sizeof(key), "network:center_cmd_port");
	config.center_cmd_port = iniparser_getint(conf_ini, key, 0);
	log_debug("center_cmd_port = %d", config.center_cmd_port);

	snprintf(key, sizeof(key), "network:local_data_port");
	config.local_data_port = iniparser_getint(conf_ini, key, 0);
	log_debug("local_data_port = %d", config.local_data_port);

	snprintf(key, sizeof(key), "network:center_data_port");
	config.center_data_port = iniparser_getint(conf_ini, key, 0);
	log_debug("center_data_port = %d", config.center_data_port);

	snprintf(key, sizeof(key), "network:default_state");
	config.default_state = iniparser_getint(conf_ini, key, 0);
	log_debug("default_state = %d", config.default_state);

	snprintf(key, sizeof(key), "network:handshake_timeout_sec");
	config.handshake_timeout_sec = (uint8_t)iniparser_getint(conf_ini, key, 5);
	log_debug("handshake_timeout_sec = %d", config.handshake_timeout_sec);

	snprintf(key, sizeof(key), "network:frame_delay");
	config.frame_delay = iniparser_getint(conf_ini, key, 0);
	config.frame_delay *= 1000; // ms -> us
	log_debug("frame_delay = %dus", config.frame_delay);

	snprintf(key, sizeof(key), "network:protocol");
	str_value = iniparser_getstring(conf_ini, key, NULL);
	log_debug("protocol = %s", str_value);
	if (str_value != NULL && !strcmp(str_value, "tcp")) {
		config.protocol = TCP;
	} else {
		config.protocol = UDP;
	}

	/* config */
	log_debug("[config]");
	snprintf(key, sizeof(key), "config:set_spad");
	config.set_spad = iniparser_getint(conf_ini, key, 0);
	log_debug("set_spad = %d", config.set_spad);

	snprintf(key, sizeof(key), "config:set_pl_threshold");
	config.set_pl_threshold = iniparser_getint(conf_ini, key, 0);
	log_debug("set_pl_threshold = %d", config.set_pl_threshold);

	snprintf(key, sizeof(key), "config:set_pl_zero_angle");
	config.set_pl_zero_angle = iniparser_getint(conf_ini, key, 0);
	log_debug("set_pl_zero_angle = %d", config.set_pl_zero_angle);

	snprintf(key, sizeof(key), "config:set_pl_standard_encoder");
	config.set_pl_standard_encoder = iniparser_getint(conf_ini, key, 0);
	log_debug("set_pl_standard_encoder = %d", config.set_pl_standard_encoder);

	snprintf(key, sizeof(key), "config:set_pl_side_en");
	config.set_pl_side_en = iniparser_getint(conf_ini, key, 0);
	log_debug("set_pl_side_en = %d", config.set_pl_side_en);

	snprintf(key, sizeof(key), "config:deal_pl_system_state");
	config.deal_pl_system_state = iniparser_getint(conf_ini, key, 0);
	log_debug("deal_pl_system_state = %d", config.deal_pl_system_state);

	snprintf(key, sizeof(key), "config:set_vop_bias");
	config.set_vop_bias = iniparser_getint(conf_ini, key, 0);
	log_debug("set_vop_bias = %d", config.set_vop_bias);

	/* thread */
	log_debug("[thread]");
	snprintf(key, sizeof(key), "thread:en_dbus_thread");
	config.en_dbus_thread = (bool)iniparser_getint(conf_ini, key, 0);
	log_debug("en_dbus_thread = %d", config.en_dbus_thread);

	snprintf(key, sizeof(key), "thread:en_camera_trigger_thread");
	config.en_camera_trigger_thread = (bool)iniparser_getint(conf_ini, key, 0);
	log_debug("en_camera_trigger_thread = %d", config.en_camera_trigger_thread);

	snprintf(key, sizeof(key), "thread:en_pl_system_state_thread");
	config.en_pl_system_state_thread = (bool)iniparser_getint(conf_ini, key, 0);
	log_debug("en_pl_system_state_thread = %d", config.en_pl_system_state_thread);

	snprintf(key, sizeof(key), "thread:en_report_encoder_thread");
	config.en_report_encoder_thread = (bool)iniparser_getint(conf_ini, key, 0);
	log_debug("en_report_encoder_thread = %d", config.en_report_encoder_thread);

	log_info("Read conf %s success", conf_path);
	return RET_OK;
}

void core_unload_cal_ini()
{
	if (cal_param_ini != NULL) {
		iniparser_freedict(cal_param_ini);
		cal_param_ini = NULL;
	}

#if CS96_BETA == 1 || CS96_GAMMA == 1
	uninit_encoder_angle_table();
#endif

	Free((void **)&(cal_param.side_angular_offset));
	Free((void **)&(cal_param.angle_face));
	Free((void **)&(cal_param.pixel_in_vcsel_id));
	Free((void **)&(cal_param.int_ld));
	Free((void **)&(cal_param.omega_delta));
	Free((void **)&(cal_param.d_delta));
	Free((void **)&(cal_param.a));

	pthread_mutex_destroy(&(cal_lock));
	return;
}

void core_unload_conf_ini()
{
	if (conf_ini != NULL) {
		iniparser_freedict(conf_ini);
		conf_ini = NULL;
	}

	pthread_mutex_destroy(&(config_lock));
	return;
}
