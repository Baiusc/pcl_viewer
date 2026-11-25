/*
 * @Author       : kaihong.yang kaihong.yang@vitalchem.com
 * @Date         : 2025-09-08 19:55:28
 * @LastEditors  : kaihong.yang kaihong.yang@vitalchem.com
 * @LastEditTime : 2025-09-08 23:26:34
 * @FilePath     : /you_see_arm/ys_business/thread/ys_dbus.c
 * @Description  :
 * Copyright (c) 2025 by kaihong.yang, email: kaihong.yang@vitalchem.com, All Rights Reserved.
 */

#define _GNU_SOURCE

#include "ys_dbus.h"

#include "../ys_def.h"
#include "../ys_state_machine.h"

#include "../../core/ys_version.h"

#include "../../third_party/cJSON/cJSON.h"
#include "../../third_party/dbus/include/dbus.h"

#include <sched.h>

static atomic_uint_fast8_t g_run_flag;

static const char *server_name = "lidar.bus";

/**
 * @description  : exit signal handler
 * @return        {int} signum
 */
static void exit_sighandler(int signum)
{
	log_debug("dbus sighandler capture the signal %d", signum);
	atomic_store(&g_run_flag, STOPPED);
}

/**
 * @description  : D-Bus initialization
 * @param         {DBusConnection} **connection
 * @param         {DBusError} *error
 * @return        {int} result
 */
static int dbus_init(DBusConnection **connection, DBusError *error)
{
	dbus_error_init(error);

	*connection = dbus_bus_get(DBUS_BUS_SESSION, error);
	if (dbus_error_is_set(error)) {
		log_error("connect to DBus error: %s", error->message);
		dbus_error_free(error);
		return RET_FAIL;
	}

	dbus_bus_request_name(*connection, server_name, DBUS_NAME_FLAG_REPLACE_EXISTING, error);
	if (dbus_error_is_set(error)) {
		log_error("register DBus service failed: %s", error->message);
		dbus_error_free(error);
		return RET_FAIL;
	}
	return RET_OK;
}

void *pthread_dbus_interface(void *arg)
{
	log_debug("Thread is running on CPU %d", sched_getcpu());

	atomic_init(&g_run_flag, RUNNING);

	int ret = RET_FAIL;
	DBusError error;
	DBusConnection *connection = NULL;
	DBusMessage *message = NULL;
#if CS96_ALPHA == 1 || CS96_BETA == 1 || CS96_GAMMA == 1
	sys_state_t *system_state = NULL;
#endif

	if (arg == NULL) {
		log_error("parameter is NULL"); /* err_quit */
		goto exit;
	}
	thread_t *th_arg = (thread_t *)arg;

	if (th_arg->thread_arg == NULL) {
		log_error("thread arg is NULL"); /* err_quit */
		goto exit;
	}

#if CS96_ALPHA == 1 || CS96_BETA == 1 || CS96_GAMMA == 1
	system_state = (sys_state_t *)(th_arg->thread_arg);
	if (system_state == NULL) {
		log_error("system_state is NULL"); /* err_quit */
		goto exit;
	}

	if (system_state->fpga_state == NULL) {
		log_error("fpga_state is NULL"); /* err_quit */
		goto exit;
	}
#endif

	if (th_arg->signal_enable == ON) {
		log_debug("listen signal no is %d", th_arg->signum);
		signal(th_arg->signum, exit_sighandler);
	}

	log_info("Start D-Bus interface");
	ret = dbus_init(&connection, &error);
	if (ret != RET_OK) {
		log_error("D-Bus initialization error");
		goto exit;
	}

	while (atomic_load(&g_run_flag) == RUNNING) {
		dbus_connection_read_write_dispatch(connection, -1);

		message = dbus_connection_pop_message(connection);
		if (message == NULL) {
			sleep(1);
			continue;
		}

		// 获取版本信息
		if (dbus_message_is_method_call(message, "lidar.get", "version")) {
			char version_str[YS_VERSION_STRING_MAX] = { '\0' };
			cJSON *root = NULL;

			ys_version_string(version_str, sizeof(version_str));

			root = cJSON_CreateObject();
			cJSON_AddNumberToObject(root, "code", 0);
			/* 保持既有字段与格式, 不新增任何字段: 本次不扩展对 D-Bus 的暴露 */
			cJSON_AddStringToObject(root, "version", version_str);

			DBusMessage *reply;
			char *reply_str = cJSON_PrintUnformatted(root);
			reply = dbus_message_new_method_return(message);
			dbus_message_append_args(reply, DBUS_TYPE_STRING, &reply_str, DBUS_TYPE_INVALID);
			dbus_connection_send(connection, reply, NULL);
			dbus_message_unref(reply);
			cJSON_Delete(root);
			cJSON_free(reply_str);
			goto end;
		}

		// 获取工作模式
		if (dbus_message_is_method_call(message, "lidar.get", "work_mode")) {
			cJSON *root = NULL;
			root = cJSON_CreateObject();
			cJSON_AddNumberToObject(root, "code", 0);
			cJSON_AddNumberToObject(root, "work_mode", get_now_run_state());

			DBusMessage *reply;
			char *reply_str = cJSON_PrintUnformatted(root);
			reply = dbus_message_new_method_return(message);
			dbus_message_append_args(reply, DBUS_TYPE_STRING, &reply_str, DBUS_TYPE_INVALID);
			dbus_connection_send(connection, reply, NULL);
			dbus_message_unref(reply);
			cJSON_Delete(root);
			cJSON_free(reply_str);
			goto end;
		}

#if CS96_ALPHA == 1 || CS96_BETA == 1 || CS96_GAMMA == 1
		// 获取自检状态
		if (dbus_message_is_method_call(message, "lidar.get", "selftest_info")) {
			cJSON *root = NULL;
			root = cJSON_CreateObject();
			cJSON_AddNumberToObject(root, "code", 0);
			pthread_mutex_lock(&(system_state->lock));
			cJSON_AddNumberToObject(root, "fpga_selftest_done", system_state->fpga_state->done);
			cJSON_AddNumberToObject(root, "fpga_work_mode", system_state->fpga_state->workmode);
			cJSON_AddNumberToObject(root, "motor_status", system_state->fpga_state->motor_status);
			cJSON_AddNumberToObject(root, "mirror_angle_status", system_state->fpga_state->mirror_angle);
			cJSON_AddNumberToObject(root, "spi_status", system_state->fpga_state->spi_status);
			pthread_mutex_unlock(&(system_state->lock));

			DBusMessage *reply;
			char *reply_str = cJSON_PrintUnformatted(root);
			reply = dbus_message_new_method_return(message);
			dbus_message_append_args(reply, DBUS_TYPE_STRING, &reply_str, DBUS_TYPE_INVALID);
			dbus_connection_send(connection, reply, NULL);
			dbus_message_unref(reply);
			cJSON_Delete(root);
			cJSON_free(reply_str);
			goto end;
		}
#endif

end:
		if (message) {
			dbus_message_unref(message);
			message = NULL;
		}
	}

exit:
	pthread_exit(0);
}
