/*
 * @Author       : huyidao
 * @Date         : 2026-09-16
 * @FilePath     : /you_see_arm/core/ys_compress.h
 * @Description  : 通用流式压缩组件
 *
 *                 设计目标: 与具体数据源解耦, 支持"边收边压边发"的流式场景:
 *                   ys_compress_begin()  开启一个压缩流(通常对应一帧数据)
 *                   ys_compress_update() 喂入任意长度数据片段, 压缩结果经 sink 回调吐出
 *                   ys_compress_end()    结束压缩流, 输出收尾字节
 *
 *                 底层当前使用 LZ4 Frame 格式(third_party/lz4):
 *                   - 输出为标准 LZ4F 流, 上位机可用任意 lz4 库解压
 *                   - linked block 模式: 跨块引用前块字典, 适合分片到达的连续数据
 *                   - 每个压缩流 = 一个 LZ4F 帧, 自带结束标记与可选内容校验
 *
 * Copyright (c) 2026 by Vitalchem, All Rights Reserved.
 */

#ifndef YS_COMPRESS_H
#define YS_COMPRESS_H

#include "ys_base.h"

/* 压缩算法类型(预留扩展) */
typedef enum {
	YS_COMPRESS_ALGO_LZ4 = 0, // LZ4 Frame 格式
} ys_compress_algo_e;

/**
 * @description  : 压缩输出回调, 每产生一段压缩数据调用一次
 * @param         {const uint8_t} *data 压缩数据
 * @param         {uint32_t} len 数据长度
 * @param         {void} *user 调用者上下文
 * @return        {int32_t} RET_OK 继续; 其他值中止压缩
 */
typedef int32_t (*ys_compress_sink_t)(const uint8_t *data, uint32_t len, void *user);

/* 压缩配置 */
typedef struct {
	ys_compress_algo_e algo; // 压缩算法
	int32_t level; // 压缩级别: 0=最快; LZ4 常规 0~12, >9 走 HC
	uint32_t block_size; // 内部块大小(字节): 64KB/256KB/1MB/4MB
	bool checksum; // 是否启用内容校验(帧尾 4B xxhash)
	uint32_t transpose_stride; // 记录宽度(字节), >0 时对输入做字节转置预处理;
		// 0=不转置。如 Range 像素 278 字节: 把 N 条 278B
		// 记录转成 278 个字段平面, 显著提升压缩率;
		// 解压端需按相同 stride 做逆转置还原
} ys_compress_cfg_t;

typedef struct ys_compress_s ys_compress_t;

/**
 * @description  : 创建压缩实例
 * @param         {const ys_compress_cfg_t} *cfg 配置, 传 NULL 使用默认值
 * @return        {ys_compress_t} *实例, 失败返回 NULL
 */
ys_compress_t *ys_compress_create(const ys_compress_cfg_t *cfg);

/**
 * @description  : 销毁压缩实例
 * @param         {ys_compress_t} *ctx
 * @return        {*}
 */
void ys_compress_destroy(ys_compress_t *ctx);

/**
 * @description  : 开启一个压缩流并注册输出回调
 * @param         {ys_compress_t} *ctx
 * @param         {ys_compress_sink_t} sink 输出回调(流生命周期内有效)
 * @param         {void} *user 回调上下文
 * @return        {int32_t} RET_OK / RET_FAIL
 */
int32_t ys_compress_begin(ys_compress_t *ctx, ys_compress_sink_t sink, void *user);

/**
 * @description  : 喂入一段数据, 压缩结果经 sink 回调吐出(可能为 0 字节)
 * @param         {ys_compress_t} *ctx
 * @param         {const uint8_t} *in 原始数据
 * @param         {uint32_t} in_len 数据长度
 * @return        {int32_t} RET_OK / RET_FAIL
 */
int32_t ys_compress_update(ys_compress_t *ctx, const uint8_t *in, uint32_t in_len);

/**
 * @description  : 结束压缩流, 输出收尾字节
 * @param         {ys_compress_t} *ctx
 * @return        {int32_t} RET_OK / RET_FAIL
 */
int32_t ys_compress_end(ys_compress_t *ctx);

/* 累计统计(创建至今) */
uint64_t ys_compress_total_in(const ys_compress_t *ctx);
uint64_t ys_compress_total_out(const ys_compress_t *ctx);

/* 当前流统计(begin~end) */
uint64_t ys_compress_stream_in(const ys_compress_t *ctx);
uint64_t ys_compress_stream_out(const ys_compress_t *ctx);

#endif // YS_COMPRESS_H
