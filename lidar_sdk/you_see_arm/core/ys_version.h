/*
 * @Author       : huyidao
 * @Date         : 2026-09-21 08:55:00
 * @LastEditors  : huyidao
 * @LastEditTime : 2026-09-21 08:55:00
 * @FilePath     : /you_see_arm/core/ys_version.h
 * @Description  : 固件版本标识模块 - 与机型无关的通用组件
 *
 * 固件的唯一身份由四个维度共同确定, 缺一不可:
 *   1. git 短 sha    : 定位源码提交, 回答"这是哪个代码节点"
 *   2. dirty 标记    : 回答"这是不是干净的提交", 为 1 时该二进制无法由任何提交复现
 *   3. 构建时间戳    : 回答"这是哪一次构建", 同一提交的多次构建靠它区分
 *   4. 协议版本      : 兼容性契约, 与固件版本解耦, 不随固件发版每次变化
 *
 * 所有值均在构建期由 scripts/gen_version.sh 从 git 自动注入(见生成的 ys_version_gen.h),
 * 本模块不含任何需要人工维护的版本号常量。
 * Copyright (c) 2024 by Vitalchem, All Rights Reserved.
 */

#ifndef YS_VERSION_H
#define YS_VERSION_H

#include <stddef.h>
#include <stdint.h>

#define YS_VERSION_SHA_MAX 12 // git 短 sha 最大长度(git 在存在歧义时会自动加长)
#define YS_VERSION_STRING_MAX 64 // 版本标识串建议缓冲区长度
#define YS_VERSION_FIELD_MAX 16 // release_id / product 字段长度(含结尾 NUL)

/* 三段式版本号 */
typedef struct {
	uint8_t major; // 主版本号
	uint8_t minor; // 次版本号
	uint8_t patch; // 修订版本号
} ys_semver_t;

/* 构建期固化的版本标识, 进程生命周期内只读 */
typedef struct {
	char sha[YS_VERSION_SHA_MAX + 1]; // git 短 sha(ASCII), 空串表示构建时无 git 信息
	uint8_t sha_len; // sha 有效长度, 0 表示未知
	uint8_t git_available; // 构建时 git 是否可用
	uint8_t dirty; // 1 = 构建时存在未提交改动(该二进制不可复现)
	uint64_t build_time; // 构建时刻 UTC Unix 秒
	ys_semver_t fw; // 固件版本(来自 v1.2.3 形式 tag)
	ys_semver_t proto; // 指令通道协议版本(来自 proto-v1.2.3 形式 tag)
	char release_id[YS_VERSION_FIELD_MAX]; // 发布标识, 如 "20260921-000"
	char product[YS_VERSION_FIELD_MAX]; // 产品名(来自 MODEL 宏)
} ys_build_info_t;

/**
 * @description  : 获取构建期固化的版本标识
 * @return        {const ys_build_info_t *} 指向只读静态数据, 永不为 NULL
 */
const ys_build_info_t *ys_version_build_info(void);

/**
 * @description  : 指令通道协议主版本号, 即报文头的 version 字节
 *                 广播包与出站报文均以此为准, 不再读取配置文件
 * @return        {uint8_t} 协议主版本号(当前为 1)
 */
uint8_t ys_version_protocol_major(void);

/**
 * @description  : 组装完整固件标识串, 三种形态:
 *                 有 git 且干净 : "1.2.3-38da7cf-1758400000"
 *                 有 git 但脏   : "1.2.3-38da7cf-dirty-1758400000"
 *                 无 git 信息   : "1.2.3-nogit-1758400000"
 * @param         {char} *buf 调用方提供的缓冲区
 * @param         {size_t} len 缓冲区长度
 * @return        {int} snprintf 语义: 未截断时为写入长度(不含结尾 NUL); 截断时 >= len; 参数非法返回 -1
 */
int ys_version_string(char *buf, size_t len);

/**
 * @description  : 构建时刻的 ISO8601 UTC 文本, 如 "2026-09-21T08:55:00Z"
 *                 时间戳为 0 或无生成头时返回 "unknown"
 * @param         {char} *buf 调用方提供的缓冲区
 * @param         {size_t} len 缓冲区长度
 * @return        {int} snprintf 语义; 参数非法返回 -1
 */
int ys_version_build_time_string(char *buf, size_t len);

/**
 * @description  : 打印启动版本横幅; dirty 或无 git 信息时追加 WARNING
 * @return        {*}
 */
void ys_version_log_banner(void);

#endif // YS_VERSION_H
