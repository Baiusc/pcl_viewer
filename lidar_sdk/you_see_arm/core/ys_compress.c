/*
 * @Author       : huyidao
 * @Date         : 2026-09-16
 * @FilePath     : /you_see_arm/core/ys_compress.c
 * @Description  : 通用流式压缩组件实现(LZ4 Frame)
 * Copyright (c) 2026 by Vitalchem, All Rights Reserved.
 */

#include "ys_base.h"
#include "ys_compress.h"

#include "../common/common.h"

#include "../third_party/log.c/log.h"
#include "../third_party/lz4/lz4frame.h"

#include <stdlib.h>
#include <string.h>

/* 默认配置: 256KB 块, 快速级别, 带内容校验, 不转置 */
#define YS_COMPRESS_DEF_LEVEL 0
#define YS_COMPRESS_DEF_BLOCK_SIZE (256u * 1024u)
#define YS_COMPRESS_DEF_CHECKSUM true

/* 输出缓冲在 bound 基础上额外预留(帧头/帧尾/对齐) */
#define YS_COMPRESS_OUT_MARGIN 64u

/* 分块转置的 tile 尺寸(字节), 兼顾 cache 行与循环开销 */
#define YS_COMPRESS_TRANSPOSE_TILE 32u

struct ys_compress_s {
	ys_compress_cfg_t cfg;
	LZ4F_preferences_t prefs;
	LZ4F_cctx *cctx;

	uint8_t *out_buf; // 压缩输出缓冲
	size_t out_cap; // 输出缓冲容量

	uint8_t *trans_buf; // 转置预处理缓冲(transpose_stride > 0 时使用)
	size_t trans_cap;

	ys_compress_sink_t sink;
	void *sink_user;
	bool stream_open;

	uint64_t total_in;
	uint64_t total_out;
	uint64_t stream_in;
	uint64_t stream_out;
};

/**
 * @description  : 分块字节转置: 把 rows 条 stride 字节的记录转为 stride 个平面
 *                 dst[c * rows + r] = src[r * stride + c]
 *
 *                 实现要点: 直接在 dst/src 间转置总有一侧跨 rows/stride 字节
 *                 跳跃访问(实测每次写都 cache miss, 转置成为全链路瓶颈);
 *                 故先用 tile 局部缓冲承接"读 src(连续) -> 写 tile(连续)",
 *                 再从 tile 转置到 dst(读 tile 在 L1 内跳跃, 写 dst 连续),
 *                 跳跃访问被限制在小缓冲内。
 * @param         {uint8_t} *dst 输出(大小 rows*stride)
 * @param         {const uint8_t} *src 输入(大小 rows*stride)
 * @param         {uint32_t} rows 记录数
 * @param         {uint32_t} stride 记录字节数
 * @return        {*}
 */
static void compress_transpose(uint8_t *dst, const uint8_t *src, uint32_t rows, uint32_t stride)
{
	uint8_t tile[YS_COMPRESS_TRANSPOSE_TILE][YS_COMPRESS_TRANSPOSE_TILE];

	for (uint32_t r0 = 0; r0 < rows; r0 += YS_COMPRESS_TRANSPOSE_TILE) {
		uint32_t tw = rows - r0;

		if (tw > YS_COMPRESS_TRANSPOSE_TILE) {
			tw = YS_COMPRESS_TRANSPOSE_TILE;
		}
		for (uint32_t c0 = 0; c0 < stride; c0 += YS_COMPRESS_TRANSPOSE_TILE) {
			uint32_t th = stride - c0;

			if (th > YS_COMPRESS_TRANSPOSE_TILE) {
				th = YS_COMPRESS_TRANSPOSE_TILE;
			}

			/* 读 src 连续, 写 tile 连续 */
			for (uint32_t r = 0; r < tw; r++) {
				memcpy(&tile[r][0], src + (size_t)(r0 + r) * stride + c0, th);
			}
			/* 读 tile(L1 内跳跃), 写 dst 连续 */
			for (uint32_t c = 0; c < th; c++) {
				uint8_t *d = dst + (size_t)(c0 + c) * rows + r0;

				for (uint32_t r = 0; r < tw; r++) {
					d[r] = tile[r][c];
				}
			}
		}
	}
}

static LZ4F_blockSizeID_t block_size_to_id(uint32_t size)
{
	if (size <= 64u * 1024u) {
		return LZ4F_max64KB;
	}
	if (size <= 256u * 1024u) {
		return LZ4F_max256KB;
	}
	if (size <= 1024u * 1024u) {
		return LZ4F_max1MB;
	}
	return LZ4F_max4MB;
}

static int32_t compress_emit(ys_compress_t *ctx, size_t len)
{
	if (len == 0) {
		return RET_OK;
	}
	if (ctx->sink(ctx->out_buf, (uint32_t)len, ctx->sink_user) != RET_OK) {
		log_error("[COMPRESS] sink rejected %zu bytes", len);
		return RET_FAIL;
	}
	ctx->stream_out += len;
	ctx->total_out += len;
	return RET_OK;
}

ys_compress_t *ys_compress_create(const ys_compress_cfg_t *cfg)
{
	ys_compress_cfg_t def_cfg = {
		.algo = YS_COMPRESS_ALGO_LZ4,
		.level = YS_COMPRESS_DEF_LEVEL,
		.block_size = YS_COMPRESS_DEF_BLOCK_SIZE,
		.checksum = YS_COMPRESS_DEF_CHECKSUM,
	};
	const ys_compress_cfg_t *use = cfg ? cfg : &def_cfg;

	if (use->algo != YS_COMPRESS_ALGO_LZ4) {
		log_error("[COMPRESS] unsupported algo %d", use->algo);
		return NULL;
	}

	ys_compress_t *ctx = (ys_compress_t *)calloc(1, sizeof(ys_compress_t));
	if (!ctx) {
		log_error("[COMPRESS] alloc context failed");
		return NULL;
	}

	ctx->cfg = *use;
	if (ctx->cfg.block_size == 0) {
		ctx->cfg.block_size = YS_COMPRESS_DEF_BLOCK_SIZE;
	}

	ctx->prefs.frameInfo.blockSizeID = block_size_to_id(ctx->cfg.block_size);
	ctx->prefs.frameInfo.blockMode = LZ4F_blockLinked;
	ctx->prefs.frameInfo.contentChecksumFlag = ctx->cfg.checksum ? LZ4F_contentChecksumEnabled :
								       LZ4F_noContentChecksum;
	ctx->prefs.compressionLevel = ctx->cfg.level;
	ctx->prefs.autoFlush = 0; // 攒块输出: 压缩率优先, 每块最多 block_size 延迟

	ctx->out_cap = LZ4F_compressBound(ctx->cfg.block_size, &ctx->prefs) + YS_COMPRESS_OUT_MARGIN;
	ctx->out_buf = (uint8_t *)malloc(ctx->out_cap);
	if (!ctx->out_buf) {
		log_error("[COMPRESS] alloc out buffer(%zu) failed", ctx->out_cap);
		free(ctx);
		return NULL;
	}

	LZ4F_errorCode_t ret = LZ4F_createCompressionContext(&ctx->cctx, LZ4F_VERSION);
	if (LZ4F_isError(ret)) {
		log_error("[COMPRESS] create LZ4F ctx failed: %s", LZ4F_getErrorName(ret));
		free(ctx->out_buf);
		free(ctx);
		return NULL;
	}

	log_info("[COMPRESS] created: algo=LZ4 level=%d block=%uKB checksum=%d transpose_stride=%u", ctx->cfg.level,
		 ctx->cfg.block_size / 1024u, ctx->cfg.checksum, ctx->cfg.transpose_stride);
	return ctx;
}

void ys_compress_destroy(ys_compress_t *ctx)
{
	if (!ctx) {
		return;
	}
	if (ctx->cctx) {
		LZ4F_freeCompressionContext(ctx->cctx);
	}
	free(ctx->out_buf);
	free(ctx->trans_buf);
	free(ctx);
}

int32_t ys_compress_begin(ys_compress_t *ctx, ys_compress_sink_t sink, void *user)
{
	if (!ctx || !sink) {
		return RET_FAIL;
	}
	if (ctx->stream_open) {
		log_error("[COMPRESS] stream already open");
		return RET_FAIL;
	}

	ctx->sink = sink;
	ctx->sink_user = user;
	ctx->stream_in = 0;
	ctx->stream_out = 0;

	size_t n = LZ4F_compressBegin(ctx->cctx, ctx->out_buf, ctx->out_cap, &ctx->prefs);
	if (LZ4F_isError(n)) {
		log_error("[COMPRESS] compressBegin failed: %s", LZ4F_getErrorName(n));
		return RET_FAIL;
	}
	ctx->stream_open = true;

	return compress_emit(ctx, n);
}

int32_t ys_compress_update(ys_compress_t *ctx, const uint8_t *in, uint32_t in_len)
{
	if (!ctx || !ctx->stream_open || !in) {
		return RET_FAIL;
	}
	if (in_len == 0) {
		return RET_OK;
	}

	const uint8_t *feed = in;
	uint32_t feed_len = in_len;

	/* 可选: 字节转置预处理(把逐记录布局转为逐字段平面) */
	if (ctx->cfg.transpose_stride > 0) {
		uint32_t stride = ctx->cfg.transpose_stride;
		uint32_t rows = in_len / stride;
		uint32_t rem = in_len % stride;

		if (rows > 0) {
			size_t need = (size_t)rows * stride + rem;

			if (need > ctx->trans_cap) {
				uint8_t *new_buf = (uint8_t *)realloc(ctx->trans_buf, need);

				if (!new_buf) {
					log_error("[COMPRESS] grow transpose buffer(%zu) failed", need);
					return RET_FAIL;
				}
				ctx->trans_buf = new_buf;
				ctx->trans_cap = need;
			}
			compress_transpose(ctx->trans_buf, in, rows, stride);
			if (rem > 0) {
				/* 尾部不足一条记录的数据原样追加 */
				memcpy(ctx->trans_buf + (size_t)rows * stride, in + (size_t)rows * stride, rem);
			}
			feed = ctx->trans_buf;
		}
	}

	/* 输入可能大于内部块大小, 按需扩容输出缓冲 */
	size_t bound = LZ4F_compressBound(feed_len, &ctx->prefs) + YS_COMPRESS_OUT_MARGIN;
	if (bound > ctx->out_cap) {
		uint8_t *new_buf = (uint8_t *)realloc(ctx->out_buf, bound);
		if (!new_buf) {
			log_error("[COMPRESS] grow out buffer(%zu) failed", bound);
			return RET_FAIL;
		}
		ctx->out_buf = new_buf;
		ctx->out_cap = bound;
	}

	size_t n = LZ4F_compressUpdate(ctx->cctx, ctx->out_buf, ctx->out_cap, feed, feed_len, NULL);
	if (LZ4F_isError(n)) {
		log_error("[COMPRESS] compressUpdate(%u) failed: %s", feed_len, LZ4F_getErrorName(n));
		return RET_FAIL;
	}

	ctx->stream_in += in_len;
	ctx->total_in += in_len;

	return compress_emit(ctx, n);
}

int32_t ys_compress_end(ys_compress_t *ctx)
{
	if (!ctx || !ctx->stream_open) {
		return RET_FAIL;
	}

	size_t n = LZ4F_compressEnd(ctx->cctx, ctx->out_buf, ctx->out_cap, NULL);
	if (LZ4F_isError(n)) {
		log_error("[COMPRESS] compressEnd failed: %s", LZ4F_getErrorName(n));
		ctx->stream_open = false;
		return RET_FAIL;
	}
	ctx->stream_open = false;

	return compress_emit(ctx, n);
}

uint64_t ys_compress_total_in(const ys_compress_t *ctx)
{
	return ctx ? ctx->total_in : 0;
}

uint64_t ys_compress_total_out(const ys_compress_t *ctx)
{
	return ctx ? ctx->total_out : 0;
}

uint64_t ys_compress_stream_in(const ys_compress_t *ctx)
{
	return ctx ? ctx->stream_in : 0;
}

uint64_t ys_compress_stream_out(const ys_compress_t *ctx)
{
	return ctx ? ctx->stream_out : 0;
}
