/*
 * @Author       : huyidao
 * @Date         : 2026-10-10 08:30:00
 * @LastEditors  : huyidao
 * @LastEditTime : 2026-10-10 08:30:00
 * @FilePath     : /you_see_arm/core/ys_version_driver.c
 * @Description  : 驱动与 FPGA 版本采集适配层实现
 *
 * 数据来源全部在驱动子模块的 lidar_version_get() 内封装, 本模块只做:
 *   1. 机型/库差异的隔离(条件编译 + 弱引用);
 *   2. FPGA 版本寄存器的 BCD 解析与合法性校验。
 * Copyright (c) 2024 by Vitalchem, All Rights Reserved.
 */

#include "ys_version_driver.h"

#include "../common/common.h"
#include "ys_core_system.h"

#include <stdio.h>
#include <string.h>

#if CS96_ALPHA == 1 || CS96_BETA == 1 || CS96_GAMMA == 1
#include "yousee_lidar_api_public.h"

/*
 * 弱引用: CS96-alpha / MF150-fl6031 等机型的预编译 liblidar.so 尚未提供该符号,
 * 弱引用保证链接不失败, 运行期判空降级为"未接入"(valid=0), 待库更新后自动生效。
 */
extern int lidar_version_get(struct lidar_version *version) __attribute__((weak));
#endif

/**
 * @description  : 解析一个字节内的两位 BCD
 * @param         {uint8_t} b 待解析字节
 * @return        {int} 0~99 解析结果; -1 任一半字节大于 9(非法 BCD)
 */
static int bcd_byte(uint8_t b)
{
	uint8_t hi = (uint8_t)(b >> 4);
	uint8_t lo = (uint8_t)(b & 0x0Fu);

	if (hi > 9 || lo > 9) {
		return -1;
	}
	return (int)hi * 10 + (int)lo;
}

/**
 * @description  : 解析版本日期寄存器(小端 BCD, 逐字节依次为 日/月/年低两位/世纪)
 *                 如 0x20250910 -> 2025-09-10
 * @param         {uint32_t} v 寄存器值
 * @param         {int} *year / *month / *day 解析结果
 * @return        {int} 0 = 成功; -1 = 非法(BCD 非法或超出合理范围)
 */
static int bcd_date_decode(uint32_t v, int *year, int *month, int *day)
{
	int cent = bcd_byte((uint8_t)(v >> 24));
	int yy = bcd_byte((uint8_t)(v >> 16));
	int mo = bcd_byte((uint8_t)(v >> 8));
	int da = bcd_byte((uint8_t)v);
	int y = 0;

	if (cent < 0 || yy < 0 || mo < 0 || da < 0) {
		return -1;
	}

	y = cent * 100 + yy;
	if (y < 2000 || y > 2199 || mo < 1 || mo > 12 || da < 1 || da > 31) {
		return -1;
	}

	*year = y;
	*month = mo;
	*day = da;
	return 0;
}

/**
 * @description  : 解析版本时刻寄存器(小端 BCD, 逐字节依次为 秒/分/时)
 *                 如 0x143000 -> 14:30:00
 * @param         {uint32_t} v 寄存器值
 * @param         {int} *hour / *minute / *second 解析结果
 * @return        {int} 0 = 成功; -1 = 非法
 */
static int bcd_clock_decode(uint32_t v, int *hour, int *minute, int *second)
{
	int hh = bcd_byte((uint8_t)(v >> 16));
	int mm = bcd_byte((uint8_t)(v >> 8));
	int ss = bcd_byte((uint8_t)v);

	if (hh < 0 || mm < 0 || ss < 0 || hh > 23 || mm > 59 || ss > 59) {
		return -1;
	}

	*hour = hh;
	*minute = mm;
	*second = ss;
	return 0;
}

/**
 * @description  : 解析版本号寄存器(大端 BCD 三段, 由高到低依次为 主/次/修订)
 *                 如 0x143000 -> 14.30.00
 * @param         {uint32_t} v 寄存器值
 * @param         {int} *major / *minor / *patch 解析结果
 * @return        {int} 0 = 成功; -1 = 非法
 */
static int bcd_semver_decode(uint32_t v, int *major, int *minor, int *patch)
{
	int ma = bcd_byte((uint8_t)(v >> 16));
	int mi = bcd_byte((uint8_t)(v >> 8));
	int pa = bcd_byte((uint8_t)v);

	if (ma < 0 || mi < 0 || pa < 0) {
		return -1;
	}

	*major = ma;
	*minor = mi;
	*patch = pa;
	return 0;
}

/**
 * @description  : 公历日期转 Unix 纪元日数(Howard Hinnant 算法, 零 libc 扩展依赖,
 *                 避免 timegm 的 feature-test macro 要求)
 * @param         {int} y 年; {int} m 月 1~12; {int} d 日 1~31
 * @return        {int64_t} 距 1970-01-01 的天数(可为负)
 */
static int64_t days_from_civil(int y, int m, int d)
{
	int era = 0;
	int64_t doe = 0;
	unsigned int yoe = 0;
	unsigned int doy = 0;

	y -= (m <= 2);
	era = (y >= 0 ? y : y - 399) / 400;
	yoe = (unsigned int)(y - era * 400);
	doy = (unsigned int)((153 * (m + (m > 2 ? -3 : 9)) + 2) / 5 + d - 1);
	doe = (int64_t)(yoe * 365u + yoe / 4u - yoe / 100u + doy);

	return (int64_t)era * 146097 + doe - 719468;
}

int ys_version_arm_driver(ys_component_version_t *out)
{
	if (out == NULL) {
		return -1;
	}
	memset(out, 0, sizeof(*out));

#if CS96_ALPHA == 1 || CS96_BETA == 1 || CS96_GAMMA == 1
	{
		struct lidar_version ver;
		const struct arm_version *api = NULL;

		/*
		 * 双重前置检查:
		 *   sys_api_ready() : driver 的 lidar_version_get() 会解引用共享内存参数区,
		 *                     lidar_api_init 未成功时调用会段错误;
		 *   弱符号判空      : 预编译库未提供该接口时降级为"未接入"。
		 */
		if (sys_api_ready() == 0 || lidar_version_get == NULL) {
			return -1;
		}

		memset(&ver, 0, sizeof(ver));
		if (lidar_version_get(&ver) != 0) {
			return -1;
		}

		/* 库未注入版本标识(构建时无 git 信息)时视为未接入, 不谎报为有效 */
		api = &ver.arm_ver_api;
		if (api->sha_len == 0) {
			return -1;
		}

		out->valid = 1;
		out->major = api->fw_major;
		out->minor = api->fw_minor;
		out->patch = api->fw_patch;
		out->dirty = (api->dirty != 0) ? 1 : 0;
		out->sha_len = (api->sha_len > YS_VERSION_SHA_MAX) ? (uint8_t)YS_VERSION_SHA_MAX : api->sha_len;
		memcpy(out->sha, api->sha, out->sha_len);
		out->build_time = (uint64_t)api->build_time;
		return 0;
	}
#else
	return -1;
#endif
}

int ys_version_fpga(ys_component_version_t *out)
{
	if (out == NULL) {
		return -1;
	}
	memset(out, 0, sizeof(*out));

#if CS96_ALPHA == 1 || CS96_BETA == 1 || CS96_GAMMA == 1
	{
		struct lidar_version ver;
		int year = 0;
		int month = 0;
		int day = 0;
		int hour = 0;
		int minute = 0;
		int second = 0;
		int major = 0;
		int minor = 0;
		int patch = 0;
		int64_t days = 0;

		/* 前置检查同 ys_version_arm_driver: 驱动未初始化或库无接口时降级为"未接入" */
		if (sys_api_ready() == 0 || lidar_version_get == NULL) {
			return -1;
		}

		memset(&ver, 0, sizeof(ver));
		if (lidar_version_get(&ver) != 0) {
			return -1;
		}

		/* 任一寄存器读失败即认为版本不可信 */
		if (ver.fpga_ver_date == ERR_FPGA_REG32 || ver.fpga_ver_time == ERR_FPGA_REG32 ||
		    ver.fpga_ver_num == ERR_FPGA_REG32 || ver.fpga_ver_hash == ERR_FPGA_REG32 ||
		    ver.fpga_modify == ERR_FPGA_REG32) {
			return -1;
		}
		/* 全部为 0 表示 FPGA 侧尚未实现版本寄存器 */
		if (ver.fpga_ver_date == 0 && ver.fpga_ver_time == 0 && ver.fpga_ver_num == 0 &&
		    ver.fpga_ver_hash == 0 && ver.fpga_modify == 0) {
			return -1;
		}

		if (bcd_date_decode(ver.fpga_ver_date, &year, &month, &day) != 0 ||
		    bcd_clock_decode(ver.fpga_ver_time, &hour, &minute, &second) != 0 ||
		    bcd_semver_decode(ver.fpga_ver_num, &major, &minor, &patch) != 0) {
			return -1;
		}

		days = days_from_civil(year, month, day);

		out->valid = 1;
		out->major = (uint8_t)major;
		out->minor = (uint8_t)minor;
		out->patch = (uint8_t)patch;
		out->dirty = (ver.fpga_modify != 0) ? 1 : 0;
		/* FPGA 以 32 位构建 hash 代替 git sha, 统一转为 8 位十六进制 ASCII;
		 * hash 为 0 表示 FPGA 侧未提供哈希, 按协议语义置 sha_len=0(无哈希信息) */
		if (ver.fpga_ver_hash != 0) {
			snprintf(out->sha, sizeof(out->sha), "%08x", ver.fpga_ver_hash);
			out->sha_len = (uint8_t)strlen(out->sha);
		}
		out->build_time =
			(uint64_t)days * 86400u + (uint64_t)hour * 3600u + (uint64_t)minute * 60u + (uint64_t)second;
		return 0;
	}
#else
	return -1;
#endif
}
