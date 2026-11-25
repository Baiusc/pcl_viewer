/*
 * @Author       : huyidao
 * @Date         : 2026-09-21 08:55:00
 * @LastEditors  : huyidao
 * @LastEditTime : 2026-09-21 08:55:00
 * @FilePath     : /you_see_arm/core/ys_version.c
 * @Description  : 固件版本标识模块实现, 全部取值来自构建期生成的版本头
 * Copyright (c) 2024 by Vitalchem, All Rights Reserved.
 */

#ifndef _POSIX_C_SOURCE
#define _POSIX_C_SOURCE 200809L // 为 gmtime_r 提供声明
#endif

#include "ys_version.h"

#include "../common/common.h"

#include <stdio.h>
#include <string.h>
#include <time.h>

/*
 * 构建期生成的版本头(scripts/gen_version.sh 产出, 默认位于 build/generated/)。
 * pre-commit 的 clang-tidy / cppcheck 会单独编译本文件, 此时生成头可能尚不存在,
 * 故先用 __has_include 探测: 缺失时退化为"未知", 保证任何编译路径都不会因缺少
 * 生成文件而失败, 同时绝不把"未知"谎报成"干净"。
 *
 * 本文件是全工程唯一包含生成头的源文件, 因此每次构建刷新时间戳只会重编这一处,
 * 不会引发全量重新编译。
 */
#if defined(__has_include)
#if __has_include("ys_version_gen.h")
#include "ys_version_gen.h"
#define YS_VERSION_GEN_PRESENT 1
#endif
#endif

#ifndef YS_VERSION_GEN_PRESENT
#define YS_BUILD_GIT_AVAILABLE 0
#define YS_BUILD_GIT_SHA ""
#define YS_BUILD_DIRTY 0
#define YS_BUILD_TIME 0ULL
#define YS_BUILD_FW_MAJOR 0
#define YS_BUILD_FW_MINOR 0
#define YS_BUILD_FW_PATCH 0
/*
 * 缺生成头时协议主版本仍取 1: 当前线上协议即为 v1, 且广播包构造只认主版本 1,
 * 退化成 0 会让广播静默消失, 上位机再也发现不了设备。
 */
#define YS_BUILD_PROTO_MAJOR 1
#define YS_BUILD_PROTO_MINOR 0
#define YS_BUILD_PROTO_PATCH 0
#define YS_BUILD_RELEASE_ID ""
#endif

static const ys_build_info_t g_build_info = {
	.sha = YS_BUILD_GIT_SHA,
	.sha_len = (uint8_t)(sizeof(YS_BUILD_GIT_SHA) - 1u),
	.git_available = YS_BUILD_GIT_AVAILABLE,
	.dirty = YS_BUILD_DIRTY,
	.build_time = YS_BUILD_TIME,
	.fw = { YS_BUILD_FW_MAJOR, YS_BUILD_FW_MINOR, YS_BUILD_FW_PATCH },
	.proto = { YS_BUILD_PROTO_MAJOR, YS_BUILD_PROTO_MINOR, YS_BUILD_PROTO_PATCH },
	.release_id = YS_BUILD_RELEASE_ID,
	.product = MODEL,
};

const ys_build_info_t *ys_version_build_info(void)
{
	return &g_build_info;
}

uint8_t ys_version_protocol_major(void)
{
	return g_build_info.proto.major;
}

int ys_version_string(char *buf, size_t len)
{
	const ys_build_info_t *info = &g_build_info;
	unsigned long long build_time = (unsigned long long)info->build_time;

	if (buf == NULL || len == 0) {
		return -1;
	}

	/* 无 git 信息时明确标注 nogit: 此时 dirty 不可信, 不可谎报为"干净" */
	if (info->git_available == 0 || info->sha_len == 0) {
		return snprintf(buf, len, "%u.%u.%u-nogit-%llu", (unsigned)info->fw.major, (unsigned)info->fw.minor,
				(unsigned)info->fw.patch, build_time);
	}

	if (info->dirty != 0) {
		return snprintf(buf, len, "%u.%u.%u-%s-dirty-%llu", (unsigned)info->fw.major, (unsigned)info->fw.minor,
				(unsigned)info->fw.patch, info->sha, build_time);
	}

	return snprintf(buf, len, "%u.%u.%u-%s-%llu", (unsigned)info->fw.major, (unsigned)info->fw.minor,
			(unsigned)info->fw.patch, info->sha, build_time);
}

int ys_version_build_time_string(char *buf, size_t len)
{
	struct tm tm_utc;
	time_t t = 0;

	if (buf == NULL || len == 0) {
		return -1;
	}

	if (g_build_info.build_time == 0) {
		return snprintf(buf, len, "unknown");
	}

	t = (time_t)g_build_info.build_time;
	if (gmtime_r(&t, &tm_utc) == NULL) {
		return snprintf(buf, len, "unknown");
	}

	return (int)strftime(buf, len, "%Y-%m-%dT%H:%M:%SZ", &tm_utc);
}

void ys_version_log_banner(void)
{
	const ys_build_info_t *info = &g_build_info;
	char version[YS_VERSION_STRING_MAX] = { '\0' };
	char built_at[32] = { '\0' };

	ys_version_string(version, sizeof(version));
	ys_version_build_time_string(built_at, sizeof(built_at));

	log_info("========================================");
	log_info("固件标识 : %s", version);
	log_info("固件版本 : %u.%u.%u", (unsigned)info->fw.major, (unsigned)info->fw.minor, (unsigned)info->fw.patch);
	log_info("协议版本 : %u.%u.%u", (unsigned)info->proto.major, (unsigned)info->proto.minor,
		 (unsigned)info->proto.patch);
	log_info("产品型号 : %s", info->product);
	log_info("发布标识 : %s", info->release_id);
	log_info("构建时间 : %s (UTC)", built_at);
	log_info("========================================");

	if (info->dirty != 0) {
		log_warn("!!! 本固件由带有未提交改动的源码构建(-dirty), 同一提交无法复现该二进制 !!!");
		log_warn("!!! 请勿用于正式发布或验收, 请提交改动后重新构建 !!!");
	}

	if (info->git_available == 0) {
		log_warn("!!! 构建时未获取到 git 信息, 固件来源不可追溯 !!!");
	}

	return;
}
