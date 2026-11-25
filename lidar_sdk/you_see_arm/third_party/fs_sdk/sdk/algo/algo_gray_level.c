#include <stdlib.h>
#include <string.h>
#include <stdint.h>
#include <unistd.h>
#include <stdio.h>
#include <getopt.h>
#include <errno.h>
#include <fcntl.h>
#include <sys/ioctl.h>
#include <assert.h>
#include <sys/time.h>

#include <linux/types.h>
#include <sys/mman.h>

#include "fs_err.h"
#include "fs_util.h"
#include "algo.h"
#include "algo_gray_level.h"

#ifdef LOG_TAG
#undef LOG_TAG
#endif
#define LOG_TAG "AlgoGl"

static lidar_frm_info_t gray_lidar_frm_info = {
    .cur_lidar_frm_idx  = 0,
    .cur_mipi_frm_idx   = 0,
    .lidar_frm_ready    = false,
    .find_start_frm     = false,
    .pixel_count        = 0,
    .pixel_len          = 0,
    .total_len          = 0,
};

static fs_lidar_data_info_t gray_lidar_data_info[2] = { 0 };

#define ALGO_GRAY_MAX_GROUP_CNT 72
#define ALGO_GRAY_MAX_SLOT_CNT   8

#if defined(__GNUC__)
#define FS_HOT_OPTIMIZE __attribute__((optimize("O3"), hot))
#else
#define FS_HOT_OPTIMIZE
#endif

#ifndef ALGO_GRAY_TIME_STAT_EN
#define ALGO_GRAY_TIME_STAT_EN 0
#endif

static const uint8_t gray_spad_array[144] = {
    0,  12,  24,  36,  48,  60,  1, 13,  25,  37,  49,  61,
    2,  14,  26,  38,  50,  62,  3, 15,  27,  39,  51,  63,
    4,  16,  28,  40,  52,  64,  5, 17,  29,  41,  53,  65,
    6,  18,  30,  42,  54,  66,  7, 19,  31,  43,  55,  67,
    8,  20,  32,  44,  56,  68,  9, 21,  33,  45,  57,  69,
    10, 22,  34,  46,  58,  70, 11, 23,  35,  47,  59,  71,
    72, 84,  96, 108, 120, 132, 73, 85,  97, 109, 121, 133,
    74, 86,  98, 110, 122, 134, 75, 87,  99, 111, 123, 135,
    76, 88, 100, 112, 124, 136, 77, 89, 101, 113, 125, 137,
    78, 90, 102, 114, 126, 138, 79, 91, 103, 115, 127, 139,
    80, 92, 104, 116, 128, 140, 81, 93, 105, 117, 129, 141,
    82, 94, 106, 118, 130, 142, 83, 95, 107, 119, 131, 143,
};

typedef struct {
    uint16_t phy_group_id;
    uint8_t slot_cnt_per_group;
    uint8_t piece_cnt_per_block;
    uint8_t channel_num;
    uint8_t ch_cnt_per_piece;
    uint8_t remap_slot_id;
} gray_decode_ctx_t;

static inline int32_t algo_gray_prepare_decode_ctx(const rx_parse_frm_cfg_t* frm_cfg,
                                                   const gray_data_t* gray_data,
                                                   gray_decode_ctx_t* dctx)
{
    uint8_t slot_loop_cnt;

    CHECK_PTR(frm_cfg);
    CHECK_PTR(gray_data);
    CHECK_PTR(dctx);

    if (gray_data->group_index >= ALGO_GRAY_MAX_GROUP_CNT) {
        return FS_ERR_BAD_PARAMS;
    }
    if (gray_data->slot_index >= ALGO_GRAY_MAX_SLOT_CNT) {
        return FS_ERR_BAD_PARAMS;
    }

    dctx->phy_group_id = frm_cfg->group_cfg[gray_data->group_index].group_id - frm_cfg->start_group;
    dctx->slot_cnt_per_group = frm_cfg->slot_cnt_per_group;
    dctx->piece_cnt_per_block = frm_cfg->piece_cnt_per_block;
    dctx->ch_cnt_per_piece = frm_cfg->half_piece_per_ch < 2 ? 2 : 1;
    dctx->channel_num = frm_cfg->channel_cnt_per_slot;

    if (dctx->channel_num == 0 || dctx->piece_cnt_per_block == 0 || dctx->slot_cnt_per_group == 0) {
        return FS_ERR_BAD_PARAMS;
    }

    slot_loop_cnt = dctx->slot_cnt_per_group / 4U;
    if (dctx->slot_cnt_per_group > 4U) {
        if (slot_loop_cnt == 0U) {
            return FS_ERR_BAD_PARAMS;
        }
        dctx->remap_slot_id = (gray_data->slot_index % slot_loop_cnt) * 4U + gray_data->slot_index / slot_loop_cnt;
    } else {
        dctx->remap_slot_id = gray_data->slot_index;
    }

    return FS_SUCCESS;
}

static inline void algo_gray_decode_spad_id_fast(const gray_decode_ctx_t* dctx,
                                                 uint32_t spad_id_in_slot,
                                                 uint16_t* row,
                                                 uint16_t* col)
{
    uint32_t spad_div4 = spad_id_in_slot >> 2;
    uint32_t ch_id_in_slot = spad_div4 % dctx->channel_num;
    uint32_t cap_idx_in_channel = spad_div4 / dctx->channel_num;
    uint32_t piece_id_in_slot = ch_id_in_slot / dctx->ch_cnt_per_piece;
    uint32_t area_id = ch_id_in_slot % dctx->ch_cnt_per_piece;
    uint32_t spad_id_in_piece;
    uint32_t piece_id_in_group;

    spad_id_in_piece = cap_idx_in_channel * 4U + (spad_id_in_slot & 0x3U) + area_id * (144U / dctx->ch_cnt_per_piece);
    if (spad_id_in_piece >= 144U) {
        *row = 0xFFFFU;
        *col = 0xFFFFU;
        return;
    }

    spad_id_in_piece = gray_spad_array[spad_id_in_piece];
    piece_id_in_group = dctx->remap_slot_id * dctx->piece_cnt_per_block +
                        piece_id_in_slot / dctx->piece_cnt_per_block * dctx->slot_cnt_per_group * dctx->piece_cnt_per_block +
                        piece_id_in_slot % dctx->piece_cnt_per_block;

    *row = dctx->phy_group_id * 12U + spad_id_in_piece / 12U;
    *col = piece_id_in_group * 12U + spad_id_in_piece % 12U;
}

int32_t algo_gray_calc_cfg(algo_context_t* ctx, int32_t ab_idx)
{
    int ret = FS_SUCCESS;

    CHECK_PTR(ctx);
    CHECK_PTR(ctx->parse_cfg);

    const rx_parse_cfg_t* parse_cfg = ctx->parse_cfg;
    uint32_t total_len;
    uint8_t row_num = 12, col_num = 12;
    int32_t i;

    fs_lidar_data_info_t *data_info;
    data_info = &gray_lidar_data_info[ab_idx];
    memset(data_info, 0, sizeof(fs_lidar_data_info_t));
    total_len = 0;

    for (i = 0; i < parse_cfg->frm_cfg[ab_idx].group_cnt; i++) {

        uint16_t width;
        uint16_t pixel_len;

        pixel_len    = sizeof(gl_pix_info_t);

        // 计算最大宽度、高度和最大bin长度
        width = parse_cfg->frm_cfg[ab_idx].piece_cnt_per_slot * parse_cfg->frm_cfg[ab_idx].slot_cnt_per_group * col_num;
        if (width > data_info->ROI_info[0].width) {
            data_info->ROI_info[0].width = width;
        }
        data_info->ROI_info[0].height += row_num;

        // 计算每个group的pixel信息
        ctx->group_cfg[ab_idx].group[i].pixel_len = pixel_len;
        ctx->group_cfg[ab_idx].group[i].pixel_cnt = row_num * width;
        total_len += pixel_len * ctx->group_cfg[ab_idx].group[i].pixel_cnt;
    }

    // 计算每个group的addr_base
    uint32_t base_addr = 0;
    for (i = 0; i < parse_cfg->frm_cfg[ab_idx].group_cnt; i++) {
        for (int32_t j = 0; j < parse_cfg->frm_cfg[ab_idx].group_cnt; j++) {
            uint8_t phy_group_id = parse_cfg->frm_cfg[ab_idx].group_cfg[j].group_id - parse_cfg->frm_cfg[ab_idx].start_group;
            if (i == phy_group_id) {
                if (i == 0) {
                    ctx->group_cfg[ab_idx].group[j].addr_base = 0;
                } else {
                    ctx->group_cfg[ab_idx].group[j].addr_base = base_addr;
                }
                base_addr += ctx->group_cfg[ab_idx].group[j].pixel_len * ctx->group_cfg[ab_idx].group[j].pixel_cnt;
                break;
            }
        }
    }

    ctx->gray_buffer.buffer_size = total_len;
    if (ctx->gray_buffer.buffer) {
        free(ctx->gray_buffer.buffer);
        ctx->gray_buffer.buffer = NULL;
    }
    ctx->gray_buffer.buffer = (uint8_t*)malloc(ctx->gray_buffer.buffer_size);
    if (!ctx->gray_buffer.buffer) {
        FS_LOGE("Failed to malloc gray buffer, size=%u", ctx->gray_buffer.buffer_size);
        return FS_ERR_NOMEM;
    }

    data_info->data_type = DATA_TYPE_GL_DATA;
    data_info->max_bin_cnt = sizeof(gl_pix_info_t);;

    data_info->ab_frame_idx = (uint8_t)ab_idx;
    data_info->max_sub_frame_cnt = (1 << parse_cfg->frm_cfg[ab_idx].piece_roll_ctrl);
    data_info->peak_num = 0;
    data_info->group_cnt = parse_cfg->frm_cfg[ab_idx].group_cnt;
    data_info->group_row_unit = 12;
    memset(data_info->peak_type, 0, sizeof(data_info->peak_type));
    data_info->roi_cnt = 1;

    for (i = 0; i < data_info->roi_cnt; i++) {
        data_info->ROI_info[i].bin_cnt       = 0;
        data_info->ROI_info[i].col_unit      = 1;
        data_info->ROI_info[i].row_unit      = 1;
        data_info->ROI_info[i].group_start   = parse_cfg->frm_cfg[ab_idx].start_group;                 // FIXME:
        data_info->ROI_info[i].sub_frame_cnt = (1 << ctx->parse_cfg->frm_cfg[ab_idx].piece_roll_ctrl); // FIXME:

        if (parse_cfg->frm_cfg[ab_idx].pixel_cfg.ROI_cfg[i].bin_cnt_ns > 0) {
            data_info->ROI_info[i].range_max = parse_cfg->frm_cfg[ab_idx].auto_cfg.ROI_cfg[i].max_bin_cnt * (15.0f / parse_cfg->frm_cfg[ab_idx].pixel_cfg.ROI_cfg[i].bin_cnt_ns);
        }
        else {
            data_info->ROI_info[i].range_max = parse_cfg->frm_cfg[ab_idx].auto_cfg.ROI_cfg[i].max_bin_cnt * 15.0f;
        }

        data_info->ROI_info[i].pcl_unit = fs_calc_pcl_unit(data_info->ROI_info[i].range_max, FS_PCL_UNIT_ROUND_CEIL);
    }

    FS_LOGI("Gray Lidar Data Info: width=%u, height=%u, max_bin_cnt=%u, total_len=%u",
             data_info->ROI_info[0].width,
             data_info->ROI_info[0].height,
             data_info->max_bin_cnt,
             total_len);

    return ret;
}

FS_HOT_OPTIMIZE int32_t algo_gray_analyze_slot_data(algo_context_t *ctx,        int32_t ab_idx,
                                                    pkt_info_t     *pkt_info,   uint32_t pkt_cnt,
                                                    pixel_data_t   *pixel_data, uint32_t pixel_cnt,
                                                    gray_data_t    *gray_data , uint32_t gray_cnt,
                                                    ev_info_t      *ev_info,    int32_t flag)
{
    int32_t ret = FS_SUCCESS;
    uint16_t row, col;
    gl_pix_info_t *pix_info;
    rx_ROI_cfg_e ROI_sel;
    uint32_t pixel_len;
    uint32_t vir_group_index/* , phy_group_index */;
    uint8_t *buffer = NULL;
    uint32_t row_offset = 0;
    uint8_t col_num = 12;
    uint16_t pixel_num_per_row;
    uint16_t row_start;
    const rx_parse_frm_cfg_t* frm_cfg;
    uint32_t row_stride;
    uint32_t invalid_row_col_cnt = 0;

    UNUSED_VAR(pkt_info);
    UNUSED_VAR(pkt_cnt);
    UNUSED_VAR(pixel_data);
    UNUSED_VAR(pixel_cnt);
    UNUSED_VAR(gray_data);
    UNUSED_VAR(gray_cnt);
    UNUSED_VAR(ev_info);

    if (ctx->parse_ctx.slot_complete == false) {
        // FS_LOGW("Current slot is not complete, skip processing");
        return FS_SUCCESS;
    }

    vir_group_index   = pkt_info[0].vir_group_index;
    // phy_group_index   = pkt_info[0].phy_group_index;
    frm_cfg           = &ctx->parse_cfg->frm_cfg[ab_idx];
    ROI_sel           = frm_cfg->group_cfg[vir_group_index].ROI_sel;
    pixel_num_per_row = frm_cfg->piece_cnt_per_slot * frm_cfg->slot_cnt_per_group * col_num;
    row_start         = gray_lidar_data_info[ab_idx].ROI_info[ROI_sel].group_start *
                        gray_lidar_data_info[ab_idx].group_row_unit /
                        gray_lidar_data_info[ab_idx].ROI_info[ROI_sel].row_unit;


    pixel_len    = ctx->group_cfg[ab_idx].group[vir_group_index].pixel_len;
    buffer       = ctx->gray_buffer.buffer + ctx->group_cfg[ab_idx].group[vir_group_index].addr_base;
    row_offset   = frm_cfg->group_cfg[vir_group_index].row_offset;
    row_stride   = pixel_num_per_row * pixel_len;

    for (uint32_t i = 0; i < gray_cnt; i++) {
        gray_decode_ctx_t dctx;
        const uint32_t *gray_values = gray_data[i].data.frame.gray_data;
        uint32_t gray_len = gray_data[i].data.frame.got_len;
        uint32_t piece_group_idx_lut[64] = {0};
        uint32_t area_span;
        uint32_t piece_group_stride;
        uint32_t row_base;

        if (algo_gray_prepare_decode_ctx(frm_cfg, &gray_data[i], &dctx) != FS_SUCCESS) {
            continue;
        }

        if (dctx.channel_num > 64U) {
            continue;
        }

        area_span = 144U / dctx.ch_cnt_per_piece;
        piece_group_stride = dctx.slot_cnt_per_group * dctx.piece_cnt_per_block;
        row_base = (uint32_t)dctx.phy_group_id * 12U;

        for (uint32_t ch = 0; ch < dctx.channel_num; ch++) {
            uint32_t piece_id_in_slot = (dctx.ch_cnt_per_piece == 1U) ? ch : (ch >> 1U);
            piece_group_idx_lut[ch] = dctx.remap_slot_id * dctx.piece_cnt_per_block
                                      + (piece_id_in_slot / dctx.piece_cnt_per_block) * piece_group_stride
                                      + (piece_id_in_slot % dctx.piece_cnt_per_block);
        }

        uint32_t ch_id = 0;
        uint32_t cap_idx_in_channel = 0;

        for (uint32_t j = 0; j < gray_len; j++) {
            uint32_t spad_id_in_piece;
            uint32_t mapped_spad;
            uint32_t area_id;

            if ((j & 0x3U) == 0U && j != 0U) {
                ch_id++;
                if (ch_id >= dctx.channel_num) {
                    ch_id = 0;
                    cap_idx_in_channel++;
                }
            }

            area_id = (dctx.ch_cnt_per_piece == 1U) ? 0U : (ch_id & 0x1U);
            spad_id_in_piece = (cap_idx_in_channel << 2U) + (j & 0x3U) + area_id * area_span;
            if (spad_id_in_piece >= 144U) {
                invalid_row_col_cnt++;
                continue;
            }

            mapped_spad = gray_spad_array[spad_id_in_piece];
            row = (uint16_t)(row_base + mapped_spad / 12U);
            col = (uint16_t)(piece_group_idx_lut[ch_id] * 12U + mapped_spad % 12U);

            if (row >= gray_lidar_data_info[ab_idx].ROI_info[0].height || col >= gray_lidar_data_info[ab_idx].ROI_info[0].width) {
                invalid_row_col_cnt++;
                continue;
            }

            pix_info = (gl_pix_info_t*)(buffer + (row - row_offset) * row_stride + col * pixel_len);
            pix_info->fid = pkt_info->frame_index;
            pix_info->row = row;
            pix_info->col = col;
            pix_info->laser_all_cnt = gray_values[j];

            if (row >= (row_start + gray_lidar_data_info[ab_idx].ROI_info[ROI_sel].height) ||
                col >= gray_lidar_data_info[ab_idx].ROI_info[ROI_sel].width) {
                FS_LOGE("Invalid row/col: row=%u, col=%u", row, col);
                continue;
            }
        }
    }

    if (invalid_row_col_cnt > 0U) {
        FS_LOGE("Invalid row/col count=%u", invalid_row_col_cnt);
    }

    if (gray_lidar_frm_info.find_start_frm) {
        // if (pkt_info->frame_index % 2 == 0) 
        {
            if (flag == FS_ERR_GROUP_CHANGE || flag == FS_SUCCESS) {
                // 发送UDP数据包
                gray_lidar_data_info[ab_idx].max_sub_frame_cnt = (1 << ctx->parse_cfg->frm_cfg[ab_idx].piece_roll_ctrl); // 子帧数量应当使用最大子帧数的 group 的值
                gray_lidar_data_info[ab_idx].group_info.vir_group_idx = vir_group_index;
                gray_lidar_data_info[ab_idx].group_info.phy_group_idx = ctx->parse_cfg->frm_cfg[ab_idx].group_cfg[vir_group_index].group_id - ctx->parse_cfg->frm_cfg[ab_idx].start_group;
                gray_lidar_data_info[ab_idx].group_info.pixel_len = pixel_len;
                gray_lidar_data_info[ab_idx].group_info.pixel_count = ctx->group_cfg[ab_idx].group[vir_group_index].pixel_cnt / (1 << (pkt_info->piece_roll_mode & 0x03)); // 真实点数应当使用当前 group 的子帧数量计算
                gray_lidar_data_info[ab_idx].group_info.roi_index = ROI_sel & 0x03;
                gray_lidar_data_info[ab_idx].ROI_info[ROI_sel].sub_frame_cnt = (1 << (pkt_info->piece_roll_mode & 0x03));
                gray_lidar_data_info[ab_idx].ROI_info[ROI_sel].sub_frame_idx = pkt_info->sub_frame_idx;
                gray_lidar_data_info[ab_idx].data = buffer;
                gray_lidar_data_info[ab_idx].frm_id = pkt_info->frame_index;
                gray_lidar_data_info[ab_idx].ab_frame_idx = pkt_info->ab_frame_sel & 0x01;

                // 深拷贝数据到新分配的内存，避免多线程竞态
                do {
                    fs_lidar_data_info_t* data_copy = NULL;
                    uint8_t* data_buffer = NULL;
                    uint32_t data_size = gray_lidar_data_info[ab_idx].group_info.pixel_count * gray_lidar_data_info[ab_idx].group_info.pixel_len;

                    data_copy = (fs_lidar_data_info_t*)malloc(sizeof(fs_lidar_data_info_t));
                    if (!data_copy) {
                        FS_LOGE("Failed to allocate memory for GRAY data_copy");
                        ret = FS_ERR_NOMEM;
                        break;
                    }

                    data_buffer = (uint8_t*)malloc(data_size);
                    if (!data_buffer) {
                        FS_LOGE("Failed to allocate memory for GRAY data_buffer, size=%u", data_size);
                        free(data_copy);
                        ret = FS_ERR_NOMEM;
                        break;
                    }

                    memcpy(data_copy, &gray_lidar_data_info[ab_idx], sizeof(fs_lidar_data_info_t));
                    memcpy(data_buffer, gray_lidar_data_info[ab_idx].data, data_size);
                    data_copy->data = data_buffer;

                    ret = algo_gray_queue_push_data(data_copy);
                    if (ret != FS_SUCCESS) {
                        FS_LOGE("Failed to send GRAY data one group, ret=%d", ret);
                        free(data_buffer);
                        free(data_copy);
                        return ret;
                    }
                } while (0);
            }
        }
    }

    return ret;
}

FS_HOT_OPTIMIZE int32_t algo_gray_data_analyze(algo_context_t* ctx, int32_t vi, uint8_t *data, uint32_t data_len)
{
    int32_t ret = FS_SUCCESS;
    uint32_t csi_width = 0;
    uint32_t csi_height = 0;
    const uint16_t* next_ptr = NULL;
    const uint16_t* data_end = NULL;
    uint32_t total_pkt_cnt = 0;
    uint32_t total_pixel_cnt = 0;
    uint32_t total_gray_cnt = 0;

    CHECK_PTR(ctx);
    CHECK_PTR(data);

    csi_width = ctx->parse_cfg->frm_cfg[vi].width;
    csi_height = ctx->parse_cfg->frm_cfg[vi].height;

    if (csi_width * csi_height * sizeof(uint16_t) > data_len) {
        return FS_ERR_BAD_PARAMS;
    }

    next_ptr = (const uint16_t* )data;
    data_end = (const uint16_t*)data + csi_width * csi_height;

    // FS_LOGI("Analyzing CICL data: width=%u, height=%u, data_len=%u", csi_width, csi_height, data_len);

    while (next_ptr < data_end) {
        uint32_t remain_len = (uint32_t)(data_end - next_ptr);
        uint32_t max_pkt_cnt = (csi_width == 0) ? 0 : (remain_len / csi_width);
        if (max_pkt_cnt > csi_height) {
            max_pkt_cnt = csi_height;
        }

        pkt_info_t* pkt_info = NULL;
        uint32_t pkt_cnt = 0;
        pixel_data_t* pixel_data = NULL;
        uint32_t pixel_cnt = 0;
        gray_data_t* gray_data = NULL;
        uint32_t gray_cnt = 0;
        ev_info_t ev_info = {0};

        ret = rx_parse_one_frame(&ctx->parse_ctx, ctx->parse_cfg, next_ptr, remain_len, csi_width, max_pkt_cnt, &pkt_info,
                                 &pkt_cnt, &pixel_data, &pixel_cnt, &gray_data, &gray_cnt, &ev_info, &next_ptr);

        total_pkt_cnt += pkt_cnt;
        total_pixel_cnt += pixel_cnt;
        total_gray_cnt += gray_cnt;

        // FS_LOGI("Partial frame parsed: pkt_cnt=%u, gray_cnt=%u, got_len=%u, gid=%u, sid=%u, flag=%d",
        //              pkt_cnt, gray_cnt, gray_data->data.frame.got_len, gray_data->group_index, gray_data->slot_index, ret);

        if (ret == FS_ERR_SLOT_CHANGE || ret == FS_ERR_GROUP_CHANGE) {
            // 继续解析下一个 slot/group 片段
            if (gray_data->group_index == 0 && gray_data->slot_index == 0) {
                if (!gray_lidar_frm_info.find_start_frm) {
                    gray_lidar_frm_info.find_start_frm = true;
                }
            }

            if (!gray_lidar_frm_info.find_start_frm) {
                // 未找到起始帧，丢弃数据
                continue;
            }

            algo_gray_analyze_slot_data(ctx, pkt_info->ab_frame_sel, pkt_info, pkt_cnt, pixel_data, pixel_cnt, gray_data, gray_cnt, &ev_info, ret);
            continue;
        } else if (ret == FS_ERR_FRAME_CHANGE) {
            // 完成一帧
            break;
        } else if (ret == FS_SUCCESS) {
            // 原始数据解析完成
            if (!gray_lidar_frm_info.find_start_frm) {
                // 未找到起始帧，丢弃数据
                continue;
            }

            algo_gray_analyze_slot_data(ctx, pkt_info->ab_frame_sel, pkt_info, pkt_cnt, pixel_data, pixel_cnt, gray_data, gray_cnt, &ev_info, ret);
            break;
        } else {
            // 其他错误直接失败
            FS_LOGE("Unexpected parse error: %d", ret);
            break;
        }
    }

    return 0;
}

int32_t algo_gray_data_process(algo_context_t* ctx, int32_t vi, uint8_t *data, uint32_t data_len)
{
    int32_t ret;

#if ALGO_GRAY_TIME_STAT_EN
    uint64_t ts_start = 0;
    uint64_t ts_end = 0;
    uint64_t cost_ms = 0;

    ts_start = fs_util_timestamp_ms();
#endif

    ret = algo_gray_data_analyze(ctx, vi, data, data_len);

#if ALGO_GRAY_TIME_STAT_EN
    ts_end = fs_util_timestamp_ms();
    cost_ms = (ts_end >= ts_start) ? (ts_end - ts_start) : 0;

    FS_LOGI("algo_gray_data_process cost: %llu ms, vi=%d, data_len=%u, ret=%d",
            (unsigned long long)cost_ms,
            vi,
            data_len,
            ret);
#endif

    if (ret < 0) {
        return ret;
    }

    return ret;
}
