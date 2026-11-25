/*
 * @Author       : huyidao
 * @Date         : 2026-10-10 08:30:00
 * @LastEditors  : huyidao
 * @LastEditTime : 2026-10-10 08:30:00
 * @FilePath     : /you_see_arm/core/ys_version_driver.h
 * @Description  : 驱动与 FPGA 版本采集适配层 - 与机型无关的通用组件
 *
 * 0x0302 组件版本列表中的 ARM 驱动与 FPGA 两项在此采集:
 *   ARM 驱动 : 取 liblidar.so 构建期注入的版本标识(arm_ver_api), 即驱动子模块自身的 git 身份
 *   FPGA     : 经驱动 lidar_version_get() 读 0x3000~0x3010 版本寄存器, 按 BCD 解析
 *
 * 机型兼容策略:
 *   - MF150-fl6532 不链接 liblidar.so(走 fs_sdk), 以条件编译隔离, 恒返回失败;
 *   - 其余机型对 lidar_version_get 采用弱引用, 预编译库尚未提供该符号时链接不失败,
 *     运行期判空降级为"未接入"(valid=0), 待各机型库更新后自动生效。
 * Copyright (c) 2024 by Vitalchem, All Rights Reserved.
 */

#ifndef YS_VERSION_DRIVER_H
#define YS_VERSION_DRIVER_H

#include <stdint.h>

#include "ys_version.h"

/* 单个组件的版本信息, 字段语义与 ys_def.h 的 version_component_t 一致, 不含报文特有字段 */
typedef struct {
	uint8_t valid; // 1 = 采集成功
	uint8_t major; // 主版本号
	uint8_t minor; // 次版本号
	uint8_t patch; // 修订版本号
	uint8_t dirty; // 1 = 该组件二进制不可复现(有未提交改动 / FPGA 有修改)
	uint8_t sha_len; // sha 有效长度, 0 表示无
	char sha[YS_VERSION_SHA_MAX + 1]; // git 短 sha 或 FPGA hash 的 ASCII
	uint64_t build_time; // 构建/编译时刻 UTC Unix 秒
} ys_component_version_t;

/**
 * @description  : 采集 ARM 驱动(liblidar.so)的构建版本
 * @param         {ys_component_version_t} *out 采集结果, 失败时 valid 保持 0
 * @return        {int} 0 = 成功; -1 = 不可用(库无版本接口 / 库未注入版本 / 机型不支持)
 */
int ys_version_arm_driver(ys_component_version_t *out);

/**
 * @description  : 采集 FPGA 版本(读 0x3000~0x3010 版本寄存器并解析)
 *                 date/time 为小端 BCD, num 为大端 BCD 三段, hash 转 8 位十六进制 ASCII,
 *                 modify 作为 dirty。任一寄存器读失败或 BCD 非法则整条不可用。
 * @param         {ys_component_version_t} *out 采集结果, 失败时 valid 保持 0
 * @return        {int} 0 = 成功; -1 = 不可用(读失败 / BCD 非法 / 机型不支持)
 */
int ys_version_fpga(ys_component_version_t *out);

#endif // YS_VERSION_DRIVER_H
