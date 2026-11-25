/*
 * @Author       : yangkh kaihong.yang@vitalchem.com
 * @Date         : 2024-05-27 23:22:02
 * @LastEditors: kaihong.yang kaihong.yang@vitalchem.com
 * @LastEditTime: 2024-07-12 02:43:26
 * @FilePath     : /you_see_arm/ys_business/ys_msg_distribute.c
 * @Description  :
 * Copyright (c) 2024 by Vitalchem, All Rights Reserved.
 */

#include "decode/ys_decode.h"
#include "ys_business_network.h"
#include "ys_business_sys.h"
#include "ys_consumer.h"
#include "ys_errno.h"
#include "ys_msg_distribute.h"
#include "ys_pl_config.h"
#include "ys_producer.h"
#include "ys_ps_config.h"
#include "ys_state_machine.h"

#include "thread/catch_camera_trigger.h"
#include "thread/get_fpga_state.h"
#include "thread/report_encoder.h"
#include "thread/ys_dbus.h"

#include "../common/common.h"
#include "../common/ys_protocal.h"

#include "../core/ys_base.h"
#include "../core/ys_conf.h"
#include "../core/ys_core_system.h"
#include "../core/ys_pthread.h"
#include "../core/ys_udp.h"
#include "../core/ys_management.h"

thpool_t global_thpool; // 保存存活于整个软件生命周期的线程

// 存储事件处理函数的数组
static event_handler_t event_handlers[TAB_MAX_ROW][TAB_MAX_COL];

/*
 * 与机型无关的通用指令表: 所有机型共用。
 * 版本标识指令不涉及任何硬件差异, 在此注册一次即可, 无需在 3 张机型表里重复维护。
 */
static management_id_t common_code_tab[] = {
	// 系统指令集
	{ CMD_SET_SYSTEM, CMD_ID_GET_SYS_VERSION, AWAYS_AVAILABLE, get_sys_version, NO_SILENT, 0, NULL },
};

#if CS96_ALPHA == 1 || CS96_BETA == 1 || CS96_GAMMA == 1
static management_id_t code_tab[] = {
	// 指令集，指令 ID，可用状态，处理函数，是否响应，协议版本号，响应内容

	// 通用指令集
	{ CMD_SET_COMMON, MSG_ID_BROADCAST, AWAYS_AVAILABLE, NULL, NO_SILENT, 0, NULL }, // 广播信息
	{ CMD_SET_COMMON, CMD_ID_GET_MODEL, AWAYS_AVAILABLE, get_sys_model, NO_SILENT, 0, NULL }, // TODO
	{ CMD_SET_COMMON, CMD_ID_HANDSHAKE, AWAYS_AVAILABLE, network_handshake, NO_SILENT, 0, NULL },
	{ CMD_SET_COMMON, CMD_ID_HEARTBEAT, AWAYS_AVAILABLE, network_heartbeat, NO_SILENT, 0, NULL },
	{ CMD_SET_COMMON, CMD_ID_SET_IP, AWAYS_AVAILABLE, set_ip_info, NO_SILENT, 0, NULL },
	{ CMD_SET_COMMON, CMD_ID_GET_IP, AWAYS_AVAILABLE, get_ip_info, NO_SILENT, 0, NULL },
	{ CMD_SET_COMMON, CMD_ID_GET_MAC, AWAYS_AVAILABLE, get_mac_address, NO_SILENT, 0, NULL },
	{ CMD_SET_COMMON, CMD_ID_BREAK_HANDSHAKE, AWAYS_AVAILABLE, network_break_handshake, NO_SILENT, 0, NULL },

	// 参数指令集
	{ CMD_SET_PARAMETER, CMD_ID_SET_RS_FILTRATION, AWAYS_AVAILABLE, NULL, NO_SILENT, 0, NULL }, // TODO
	{ CMD_SET_PARAMETER, CMD_ID_SET_ANIT_SUNLIGHT, AWAYS_AVAILABLE, NULL, NO_SILENT, 0, NULL }, // TODO
	{ CMD_SET_PARAMETER, CMD_ID_SET_ECHO_PARM, AWAYS_AVAILABLE, set_echo_parm, NO_SILENT, 0, NULL },
	{ CMD_SET_PARAMETER, CMD_ID_GET_ECHO_PARM, AWAYS_AVAILABLE, get_echo_parm, NO_SILENT, 0, NULL },
	{ CMD_SET_PARAMETER, CMD_ID_SET_REFRESH_RATE, AWAYS_AVAILABLE, set_refresh_rate, NO_SILENT, 0, NULL },
	{ CMD_SET_PARAMETER, CMD_ID_GET_REFRESH_RATE, AWAYS_AVAILABLE, get_refresh_rate, NO_SILENT, 0, NULL },
	{ CMD_SET_PARAMETER, CMD_ID_SET_COORDINATE, AWAYS_AVAILABLE, set_coordinate_mode, NO_SILENT, 0, NULL },
	{ CMD_SET_PARAMETER, CMD_ID_GET_COORDINATE, AWAYS_AVAILABLE, get_coordinate_mode, NO_SILENT, 0, NULL },

	// 状态指令集
	{ CMD_SET_STATE, CMD_ID_GET_RUN_STATE, AWAYS_AVAILABLE, get_run_state, NO_SILENT, 0, NULL },
	{ CMD_SET_STATE, CMD_ID_GET_SELF_STATE, AWAYS_AVAILABLE, get_self_state, NO_SILENT, 0, NULL },
	{ CMD_SET_STATE, CMD_ID_SET_STATE, AWAYS_AVAILABLE, set_run_state, NO_SILENT, 0, NULL },
	{ CMD_SET_STATE, CMD_ID_GET_LAST_ERROR_CODE, AWAYS_AVAILABLE, get_last_error_code, NO_SILENT, 0, NULL },

	// 系统指令集(查看系统固件版本见文件头部的通用指令表 common_code_tab)
	{ CMD_SET_SYSTEM, CMD_ID_UPGRADE_FPGA, SET_AVAIL_STATE(IDLE), upgrade_fpga, NO_SILENT, 0, NULL }, // TODO
	{ CMD_SET_SYSTEM, CMD_ID_UPGRADE_SYS, SET_AVAIL_STATE(IDLE), upgrade_sys, NO_SILENT, 0, NULL }, // TODO
	{ CMD_SET_SYSTEM, CMD_ID_REBOOT, SET_AVAIL_STATE(IDLE), reboot_sys, NO_SILENT, 0, NULL },
	{ CMD_SET_SYSTEM, CMD_ID_GET_SYS_SN, AWAYS_AVAILABLE, get_sys_sn, NO_SILENT, 0, NULL },

	// 调试指令集
	{ CMD_SET_DEBUG, CMD_ID_DOWNLOAD_SYS_LOG, SET_AVAIL_STATE(DEBUG), NULL, NO_SILENT, 0, NULL }, // TODO
	{ CMD_SET_DEBUG, CMD_ID_CLEAN_SYS_LOG, SET_AVAIL_STATE(DEBUG), clean_log, NO_SILENT, 0, NULL },
	{ CMD_SET_DEBUG, CMD_ID_DOWNLOAD_KERNEL_LOG, SET_AVAIL_STATE(DEBUG), NULL, NO_SILENT, 0, NULL }, // TODO
	{ CMD_SET_DEBUG, CMD_ID_CLEAN_KERNEL_LOG, SET_AVAIL_STATE(DEBUG), clean_log, NO_SILENT, 0, NULL },
	{ CMD_SET_DEBUG, CMD_ID_DOWNLOAD_APP_LOG, SET_AVAIL_STATE(DEBUG), NULL, NO_SILENT, 0, NULL }, // TODO
	{ CMD_SET_DEBUG, CMD_ID_CLEAN_APP_LOG, SET_AVAIL_STATE(DEBUG), clean_log, NO_SILENT, 0, NULL },
	{ CMD_SET_DEBUG, CMD_ID_GET_CALIBRATION_PARM, AWAYS_AVAILABLE, get_cal_parm, NO_SILENT, 0, NULL },
	{ CMD_SET_DEBUG, CMD_ID_SET_CALIBRATION_PARM, AWAYS_AVAILABLE, set_cal_parm, NO_SILENT, 0, NULL },
	{ CMD_SET_DEBUG, CMD_ID_GET_SHISTOGRAM, SET_AVAIL_STATE(DEBUG), get_single_histogram, NO_SILENT, 0, NULL },
	{ CMD_SET_DEBUG, CMD_ID_SET_VCSEL_PARM, SET_AVAIL_STATE(DEBUG), set_vcsel_parameter, NO_SILENT, 0, NULL },
	{ CMD_SET_DEBUG, CMD_ID_GET_VCSEL_PARM, SET_AVAIL_STATE(DEBUG), get_vcsel_parameter, NO_SILENT, 0, NULL },
	{ CMD_SET_DEBUG, CMD_ID_SET_SPAD_PARM, SET_AVAIL_STATE(DEBUG), set_spad_parameter, NO_SILENT, 0, NULL },
	{ CMD_SET_DEBUG, CMD_ID_GET_SPAD_PARM, SET_AVAIL_STATE(DEBUG), get_spad_parameter, NO_SILENT, 0, NULL },
	{ CMD_SET_DEBUG, CMD_ID_SET_MOTOR_PARM, AWAYS_AVAILABLE, set_motor_parameter, NO_SILENT, 0, NULL },
	{ CMD_SET_DEBUG, CMD_ID_GET_MOTOR_PARM, AWAYS_AVAILABLE, get_motor_parameter, NO_SILENT, 0, NULL },
	{ CMD_SET_DEBUG, CMD_ID_SET_PL_CALIB_PARM, SET_AVAIL_STATE(DEBUG), set_calib_parameter, NO_SILENT, 0, NULL },
	{ CMD_SET_DEBUG, CMD_ID_GET_PL_CALIB_PARM, SET_AVAIL_STATE(DEBUG), get_calib_parameter, NO_SILENT, 0, NULL },
	{ CMD_SET_DEBUG, CMD_ID_SET_SIDE_H_OFFSET, AWAYS_AVAILABLE, set_side_angular_offset, NO_SILENT, 0, NULL },
	{ CMD_SET_DEBUG, CMD_ID_SET_SIDE_LIGHT_EN, AWAYS_AVAILABLE, set_side_light_en, NO_SILENT, 0, NULL },
	{ CMD_SET_DEBUG, CMD_ID_SET_VCSEL_H_OFFSET, AWAYS_AVAILABLE, set_vcsel_h_offset, NO_SILENT, 0, NULL },
	{ CMD_SET_DEBUG, CMD_ID_SET_DISTANCE_OFFSET, AWAYS_AVAILABLE, set_distance_offset, NO_SILENT, 0, NULL },

};
#elif MF150_FL6031 == 1
static management_id_t code_tab[] = {
	// 指令集，指令 ID，可用状态，处理函数，是否响应，协议版本号，响应内容

	// 通用指令集
	{ CMD_SET_COMMON, MSG_ID_BROADCAST, AWAYS_AVAILABLE, NULL, NO_SILENT, 0, NULL }, // 广播信息
	{ CMD_SET_COMMON, CMD_ID_GET_MODEL, AWAYS_AVAILABLE, get_sys_model, NO_SILENT, 0, NULL }, // TODO
	{ CMD_SET_COMMON, CMD_ID_HANDSHAKE, AWAYS_AVAILABLE, network_handshake, NO_SILENT, 0, NULL },
	{ CMD_SET_COMMON, CMD_ID_HEARTBEAT, AWAYS_AVAILABLE, network_heartbeat, NO_SILENT, 0, NULL },
	{ CMD_SET_COMMON, CMD_ID_SET_IP, AWAYS_AVAILABLE, set_ip_info, NO_SILENT, 0, NULL },
	{ CMD_SET_COMMON, CMD_ID_GET_IP, AWAYS_AVAILABLE, get_ip_info, NO_SILENT, 0, NULL },
	{ CMD_SET_COMMON, CMD_ID_GET_MAC, AWAYS_AVAILABLE, get_mac_address, NO_SILENT, 0, NULL },
	{ CMD_SET_COMMON, CMD_ID_BREAK_HANDSHAKE, AWAYS_AVAILABLE, network_break_handshake, NO_SILENT, 0, NULL },

	// 参数指令集

	// 状态指令集

	// 系统指令集

	// 调试指令集
};
#elif MF150_FL6532 == 1
static management_id_t code_tab[] = {
	// 指令集，指令 ID，可用状态，处理函数，是否响应，协议版本号，响应内容

	// 通用指令集
	{ CMD_SET_COMMON, MSG_ID_BROADCAST, AWAYS_AVAILABLE, NULL, NO_SILENT, 0, NULL }, // 广播信息
	{ CMD_SET_COMMON, CMD_ID_GET_MODEL, AWAYS_AVAILABLE, get_sys_model, NO_SILENT, 0, NULL }, // TODO
	{ CMD_SET_COMMON, CMD_ID_HANDSHAKE, AWAYS_AVAILABLE, network_handshake, NO_SILENT, 0, NULL },
	{ CMD_SET_COMMON, CMD_ID_HEARTBEAT, AWAYS_AVAILABLE, network_heartbeat, NO_SILENT, 0, NULL },
	{ CMD_SET_COMMON, CMD_ID_SET_IP, AWAYS_AVAILABLE, set_ip_info, NO_SILENT, 0, NULL },
	{ CMD_SET_COMMON, CMD_ID_GET_IP, AWAYS_AVAILABLE, get_ip_info, NO_SILENT, 0, NULL },
	{ CMD_SET_COMMON, CMD_ID_GET_MAC, AWAYS_AVAILABLE, get_mac_address, NO_SILENT, 0, NULL },
	{ CMD_SET_COMMON, CMD_ID_BREAK_HANDSHAKE, AWAYS_AVAILABLE, network_break_handshake, NO_SILENT, 0, NULL },

	// 参数指令集

	// 状态指令集

	// 系统指令集

	// 调试指令集
};
#endif

/**
 * @description  : register event handler
 * @param         {uint8_t} code_set
 * @param         {uint8_t} code_id
 * @param         {event_handler_t} handler
 * @return        {*}
 */
static void register_event_handler(uint8_t code_set, uint8_t code_id, event_handler_t handler)
{
	event_handlers[code_set][code_id] = handler;
	return;
}

/**
 * @description  : register all event handler
 * @param         {*}
 * @return        {*}
 */
static void register_all_event_handler()
{
	/* 先注册与机型无关的通用指令, 再注册当前机型的指令 */
	for (int i = 0; i < (int)(sizeof(common_code_tab) / sizeof(common_code_tab[0])); i++) {
		register_event_handler(common_code_tab[i].code_set, common_code_tab[i].code_id,
				       common_code_tab[i].handler);
	}

	for (int i = 0; i < (int)(sizeof(code_tab) / sizeof(code_tab[0])); i++) {
		register_event_handler(code_tab[i].code_set, code_tab[i].code_id, code_tab[i].handler);
	}
	log_info("register all event handler success");
	return;
}

/**
 * @description  : 按 (指令集, 指令 ID) 查找指令表项, 先查通用表再查机型表
 * @param         {uint8_t} code_set
 * @param         {uint8_t} code_id
 * @return        {management_id_t *} 命中的表项指针; 未找到返回 NULL
 */
static management_id_t *find_msg_entry(uint8_t code_set, uint8_t code_id)
{
	size_t i = 0;

	/* 先查与机型无关的通用表, 再查当前机型的表 */
	for (i = 0; i < sizeof(common_code_tab) / sizeof(common_code_tab[0]); i++) {
		if (common_code_tab[i].code_set == code_set && common_code_tab[i].code_id == code_id) {
			return &common_code_tab[i];
		}
	}

	for (i = 0; i < sizeof(code_tab) / sizeof(code_tab[0]); i++) {
		if (code_tab[i].code_set == code_set && code_tab[i].code_id == code_id) {
			return &code_tab[i];
		}
	}

	return NULL;
}

void trigger_event(uint8_t version, uint8_t *data, int data_len)
{
	management_id_t *msg = find_msg_entry(data[0], data[1]);
	if (msg == NULL) {
		int ret = illegal_cmd;
		management_id_t tmp_msg = { 0 };
		tmp_msg.avail_state = AWAYS_AVAILABLE;
		tmp_msg.buf = NULL;
		tmp_msg.code_id = data[1];
		tmp_msg.code_set = data[0];
		tmp_msg.handler = NULL;
		tmp_msg.silent = NO_SILENT;
		tmp_msg.version = version;
		log_error("no find message index");
		pack(&tmp_msg, (uint8_t *)&ret, 1, ACK);
		return;
	}

	msg->silent = NO_SILENT;
	msg->version = version;
	/* 在读取地址或握手字段之前校验完整长度，短包不能进入网络配置流程。 */
	if (msg->code_set == CMD_SET_COMMON &&
	    ((msg->code_id == CMD_ID_SET_IP && data_len < 14) ||
	     (msg->code_id == CMD_ID_HANDSHAKE && data_len < (int)sizeof(network_handshake_t)))) {
		uint8_t result = run_fail;
		pack(msg, &result, 1, ACK);
		return;
	}
	msg->buf = (uint8_t *)Malloc(data_len + 1);
	if (msg->buf == NULL) {
		return;
	}
	memcpy(msg->buf, data, data_len);

	// check that the command is available in the current running state
	int is_avail = CHECK_CMD_AVAILABLE(msg->avail_state, get_now_run_state());
	if (is_avail == CMD_NOT_AVAILABLE) {
		int ret = unavailable_cmd;
		log_error("this command is not available in the current state");
		pack(msg, (uint8_t *)&ret, 1, ACK);
		goto end;
	}

	event_handler_t handler = event_handlers[msg->code_set][msg->code_id];
	if (handler) {
		handler((void *)msg);
	}

end:
	Free((void **)&(msg->buf));
	return;
}

/**
 * @description  : event initialization function
 * @return        {int} result
 *      0 : success
 *      other : fail
 */
static int event_init()
{
	int ret = RET_FAIL;
	ret = init_decode_paramter();
	if (ret != RET_OK) {
		log_fatal("init algo paramter fail");
		return RET_FAIL;
	}

	if (config.work_mode != RAW_MODE) {
		// 写入标定参数到 FPGA
		pl_calib_parm_init(&cal_param);
	}

	pthread_mutex_init(&(global_thpool.lock), NULL);

	ret = init_udp(&cal_param, &config);
	if (ret != RET_OK) {
		log_fatal("initialization udp fail");
		return RET_FAIL;
	}

	return RET_OK;
}

/**
 * @description  : event exit function
 * @return        {int} result
 *      0 : success
 *      other : fail
 */
static int event_exit()
{
	pthread_mutex_destroy(&(global_thpool.lock));
	return RET_OK;
}

/**
 * @description  : application exit function
 * @return        {*}
 */
static int app_exit()
{
	/* 先停止控制指令线程，避免退出时再次启动采集。 */
	thpool_exit(&global_thpool);
	int stop_ret = stop_capture_for_shutdown();

	log_debug("uninit udp");
	uninit_udp();
	log_debug("uninit udp success");

	log_debug("uninit cmd");
	cmd_clean();
	log_debug("uninit cmd success");

	log_debug("uninit state machine");
	uninit_state_machine();
	log_debug("uninit state machine success");

	log_debug("exit event loop");
	event_exit();
	log_debug("exit event loop success");

	log_debug("exit algo paramter");
	uninit_decode_paramter();
	log_debug("exit algo paramter success");

	/* 驱动映射和配置由 main 在所有线程退出后统一释放。 */
	return stop_ret;
}

int event_loop()
{
	/* 进行一系列初始化 */
	if (event_init() != RET_OK) {
		return RET_FAIL;
	}

	register_all_event_handler();

	/* 初始化状态机，进入默认状态（扫描） */
	init_state_machine(config.default_state);

	/* 创建全局线程，该类线程存活于整个软件生命周期 */
	thread_t th_arg;

	th_arg.thread_arg = NULL;
	th_arg.signal_enable = ON;

	// 线程 1：D-Bus 交互线程，提供进程间交互接口
	th_arg.thread_arg = (void *)(&sys_state);
	global_thpool.signal_num[0] = th_arg.signum = MY_GLOBAL_SIGNAL - 0;
	if (config.en_dbus_thread == true) {
		global_thpool.tid[0] = create_thread(pthread_dbus_interface, NULL, NULL, 0, NO_SET_PRIORITY);
	}
	global_thpool.thpool_count++;

	// 线程 2：响应指令的 UDP 线程
	th_arg.thread_arg = (void *)(&udp_cmd_instance);
	global_thpool.signal_num[1] = th_arg.signum = MY_GLOBAL_SIGNAL - 1;
	global_thpool.tid[1] = create_thread(pthread_cent_udp_client, &th_arg, NULL, 0, NO_SET_PRIORITY);
	global_thpool.thpool_count++;

#if CS96_ALPHA == 1 || CS96_BETA == 1 || CS96_GAMMA == 1
	// 线程 3：获取 FPGA 的运行状态
	th_arg.thread_arg = (void *)(&sys_state);
	global_thpool.signal_num[2] = th_arg.signum = MY_GLOBAL_SIGNAL - 2;
	if (config.work_mode != RAW_MODE && config.en_pl_system_state_thread == true) {
		global_thpool.tid[2] = create_thread(pthread_get_fpga_state, &th_arg, NULL, 0, NO_SET_PRIORITY);
	}
	global_thpool.thpool_count++;
#endif

#if CS96_ALPHA == 1 || CS96_BETA == 1 || CS96_GAMMA == 1
	// 线程 4：获取 trigger 信号
	th_arg.thread_arg = (void *)(&sys_state);
	global_thpool.signal_num[3] = th_arg.signum = MY_GLOBAL_SIGNAL - 3;
	if (config.en_camera_trigger_thread == true) {
		global_thpool.tid[3] = create_thread(pthread_get_trigger, &th_arg, NULL, 3, SET_HIGHEST_PRIORITY);
	}
	global_thpool.thpool_count++;
#endif

#if CS96_ALPHA == 1 || CS96_BETA == 1 || CS96_GAMMA == 1
	// 线程 5：上报电机实时编码值
	th_arg.thread_arg = NULL;
	global_thpool.signal_num[4] = th_arg.signum = MY_GLOBAL_SIGNAL - 4;
	if (config.en_report_encoder_thread == true) {
		global_thpool.tid[4] = create_thread(pthread_report_encoder, &th_arg, NULL, 3, SET_HIGHEST_PRIORITY);
	}
	global_thpool.thpool_count++;
#endif

	// 运行事件循环
	ys_management_initialized(config.work_mode);
	while (!ys_shutdown_requested())
		usleep(100000);
	return app_exit();
}
