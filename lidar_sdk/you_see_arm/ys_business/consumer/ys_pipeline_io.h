/*
 * @Author       : kaihong.yang kaihong.yang@vitalchem.com
 * @Date         : 2026-06-30
 * @FilePath     : /you_see_arm/ys_business/consumer/ys_pipeline_io.h
 * @Description  : 消费者流水线 IO 适配器 —— 将 KFIFO/内存池/零拷贝/sendmmsg 的差异
 *                 彻底隔离，通过编译期静态展开实现零运行时开销。
 * Copyright (c) 2026 by Vitalchem, All Rights Reserved.
 */

#ifndef YS_PIPELINE_IO_H
#define YS_PIPELINE_IO_H

#include "../ys_def.h"
#include "../ys_business_network.h"
#include "../../core/ys_core_system.h"

#include <stdbool.h>

typedef struct {
	udp_t *udp;
#if USE_KFIFO
	uint8_t *kfifo_buf;
	int kfifo_buf_size;
#else
	block_t *current_block;
	node_t *current_node;
#if USE_SENDMMSG_FUNC == 1
	block_t *storage_block_head;
	memory_pool_t *dest_data_pool;
	block_t *p_block;
#endif
#endif
} pipeline_io_ctx_t;

// =================================================================
// 路径 A：KFIFO 模式（低频备用路径）
// =================================================================
#if USE_KFIFO

static inline int __attribute__((always_inline)) pipeline_io_init(pipeline_io_ctx_t *ctx, udp_t *udp,
								  uint8_t *kfifo_buf, int kfifo_buf_size,
								  int need_output_pool, int out_block_size)
{
	(void)need_output_pool;
	(void)out_block_size;
	ctx->udp = udp;
	ctx->kfifo_buf = kfifo_buf;
	ctx->kfifo_buf_size = kfifo_buf_size;
	memset(ctx->kfifo_buf, 0, (size_t)kfifo_buf_size);
	return RET_OK;
}

static inline bool __attribute__((always_inline)) pipeline_io_fetch_input(pipeline_io_ctx_t *ctx, uint8_t **out_buf,
									  int size)
{
	if (kfifo_len(&g_fifobuf) < size) {
		return false;
	}
	if (kfifo_out(&g_fifobuf, ctx->kfifo_buf, size) == size) {
		*out_buf = ctx->kfifo_buf;
		return true;
	}
	return false;
}

static inline uint8_t *__attribute__((always_inline)) pipeline_io_get_output_buf(pipeline_io_ctx_t *ctx,
										 uint8_t *default_dest)
{
	(void)ctx;
	return default_dest;
}

static inline void __attribute__((always_inline)) pipeline_io_skip_input(pipeline_io_ctx_t *ctx)
{
	(void)ctx;
}

static inline void __attribute__((always_inline)) pipeline_io_flush_output(pipeline_io_ctx_t *ctx, uint8_t *send_buf,
									   int send_len)
{
	send_data(ctx->udp, send_buf, send_len);
}

static inline void __attribute__((always_inline)) pipeline_io_deinit(pipeline_io_ctx_t *ctx)
{
	(void)ctx;
}

#else /* ================================================================ */
/* 路径 B：内存池模式（主流高频路径）                                */
/* ================================================================ */

static inline int __attribute__((always_inline)) pipeline_io_init(pipeline_io_ctx_t *ctx, udp_t *udp,
								  uint8_t *unused_buf, int unused_size,
								  int need_output_pool, int out_block_size)
{
	(void)unused_buf;
	(void)unused_size;
	ctx->udp = udp;
	ctx->current_block = atomic_load(&(udp->pool->head));
	ctx->current_node = ctx->current_block->node;
#if USE_SENDMMSG_FUNC == 1
	ctx->storage_block_head = NULL;
	if (need_output_pool) {
		set_block_size(out_block_size);
		ctx->dest_data_pool = create_memory_pool(16 + 1);
		if (ctx->dest_data_pool == NULL) {
			return RET_FAIL;
		}
		ctx->p_block = atomic_load(&(ctx->dest_data_pool->head));
	} else {
		ctx->dest_data_pool = NULL;
	}
#else
	(void)need_output_pool;
	(void)out_block_size;
#endif
	return RET_OK;
}

static inline bool __attribute__((always_inline)) pipeline_io_fetch_input(pipeline_io_ctx_t *ctx, uint8_t **out_buf,
									  int size)
{
	(void)size;
	if (atomic_load(&(ctx->current_node->write_ok)) != READABLE) {
		return false;
	}
	*out_buf = ctx->current_node->data->buf;
	return true;
}

static inline uint8_t *__attribute__((always_inline)) pipeline_io_get_output_buf(pipeline_io_ctx_t *ctx,
										 uint8_t *default_dest)
{
#if USE_SENDMMSG_FUNC == 1
	if (ctx->dest_data_pool != NULL) {
		return ctx->p_block->node->data->buf;
	}
#endif
	(void)ctx;
	return default_dest;
}

static inline void __attribute__((always_inline)) pipeline_io_skip_input(pipeline_io_ctx_t *ctx)
{
	atomic_store(&(ctx->current_node->write_ok), WRITABLE);
	ctx->current_block = ctx->current_block->next;
	ctx->current_node = ctx->current_block->node;
}

static inline void __attribute__((always_inline)) pipeline_io_flush_output(pipeline_io_ctx_t *ctx, uint8_t *send_buf,
									   int send_len)
{
#if USE_SENDMMSG_FUNC == 1
	int ret = send_data(ctx->udp, send_buf, send_len);
	switch (ret) {
	case 1:
		ctx->storage_block_head = ctx->current_block;
		/* fall through */
	case RET_OK:
		atomic_store(&(ctx->current_node->write_ok), SENDABLE);
		ctx->current_block = ctx->current_block->next;
		ctx->current_node = ctx->current_block->node;
		if (ctx->dest_data_pool != NULL) {
			ctx->p_block = ctx->p_block->next;
		}
		break;
	default:
		while (ctx->storage_block_head != ctx->current_block) {
			atomic_store(&(ctx->storage_block_head->node->write_ok), WRITABLE);
			ctx->storage_block_head = ctx->storage_block_head->next;
		}
		break;
	}
#else
	if (send_data(ctx->udp, send_buf, send_len) == send_len || ctx->udp->udpc.center.is_handshaked == NO_REG) {
		atomic_store(&(ctx->current_node->write_ok), WRITABLE);
		ctx->current_block = ctx->current_block->next;
		ctx->current_node = ctx->current_block->node;
	}
#endif
}

static inline void __attribute__((always_inline)) pipeline_io_deinit(pipeline_io_ctx_t *ctx)
{
#if USE_SENDMMSG_FUNC == 1
	if (ctx->dest_data_pool != NULL) {
		destroy_memory_pool(ctx->dest_data_pool);
	}
#endif
	(void)ctx;
}

#endif /* USE_KFIFO */

#endif // YS_PIPELINE_IO_H
