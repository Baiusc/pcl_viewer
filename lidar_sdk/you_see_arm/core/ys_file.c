/*
 * @Author       : yangkh kaihong.yang@vitalchem.com
 * @Date         : 2024-05-27 03:57:07
 * @LastEditors  : kaihong.yang kaihong.yang@vitalchem.com
 * @LastEditTime : 2025-10-14 23:05:07
 * @FilePath     : /you_see_arm/core/ys_file.c
 * @Description  :
 * Copyright (c) 2024 by Vitalchem, All Rights Reserved.
 */

#include "ys_file.h"

#include "../common/common.h"

#include <sys/mount.h>
#include <sys/klog.h>
#include <unistd.h>

#define NO_FIND -2

const char *g_flash_source = "/dev/mtdblock2"; // flash 路径
const char *g_flash_target = "/mnt/mtdblock2"; // 挂载点
const char *file_system_type = "jffs2";

/**
 * @description  : 检查 flash 是否被挂载
 * @return        {bool} result
 *      false : 已经被挂载
 *      true : 没有被挂载
 */
static bool check_flash_is_mounted()
{
	char buf[128];
	FILE *fd = fopen("/proc/mounts", "r");
	if (fd == NULL) {
		return false;
	}

	while (fgets(buf, sizeof(buf), fd)) {
		if (strstr(buf, g_flash_source) != NULL) {
			fclose(fd);
			return false;
		}
	}

	fclose(fd);
	return true;
}

int clean_sys_log()
{
	if (truncate(SYS_LOG_PATH, 0) != RET_OK) {
		log_error("clean system log error");
		return RET_FAIL;
	}
	return RET_OK;
}

int clean_kernel_log()
{
	// 5 是 SYSLOG_ACTION_CLEAR 操作码
	if (klogctl(5, NULL, 0) < 0) {
		log_error("clean kernel log error");
		return RET_FAIL;
	}
	return RET_OK;
}

int clean_app_log(const char *file_path)
{
	if (truncate(file_path, 0) != RET_OK) {
		log_error("clean app log error");
		return RET_FAIL;
	}
	return RET_OK;
}

int update_flash_file(const char *flash_file_path, const char *file_path)
{
	// 检查文件是否存在
	if (access(file_path, F_OK)) {
		log_error("%s no exist", file_path);
		return RET_FAIL;
	}

	// 检查是否已经挂载
	if (check_flash_is_mounted() != true) {
		return RET_FAIL;
	}

	// 执行挂载
	log_debug("mounting %s to %s with type %s", g_flash_source, g_flash_target, file_system_type);
	if (mount(g_flash_source, g_flash_target, file_system_type, 0, NULL) != 0) {
		log_error("mount failed(%s)", strerror(errno));
		return RET_FAIL;
	}

	char cmd[512];
	snprintf(cmd, sizeof(cmd), "cp -r %s %s && sync", file_path, flash_file_path);
	log_debug("run command [%s]", cmd);
	if (system(cmd) != 0) {
		log_error("copy file to flash failed(%s)", strerror(errno));
		umount(g_flash_target);
		return RET_FAIL;
	}

	umount(g_flash_target);
	return RET_OK;
}
