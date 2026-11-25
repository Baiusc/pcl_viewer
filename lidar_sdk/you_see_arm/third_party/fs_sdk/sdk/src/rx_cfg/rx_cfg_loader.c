#include <stdlib.h>
#include <string.h>

#include "rx_cfg_loader.h"
#include "fs_err.h"
#include "fs_log.h"
#include "fs_util.h"

#undef  LOG_TAG
#define LOG_TAG "rx_cfg_loader"

#include "preset_cfg_idle.h"          // 预设的 idle 模式配置
#include "preset_cfg_gray.h"          // 预设的 gray 模式配置(AA 调焦)
#include "preset_cfg_range_roi0.h"    // 预设的 range 模式配置
#include "preset_cfg_range_roi1.h"    // 预设的 range 模式配置
#if 1
#include "preset_cfg_range_roi01_80.h"// 预设的 range 模式配置(80°+30°)
#else
#include "preset_cfg_range_roi01_90.h"// 预设的 range 模式配置(90°+30°)
#endif
#include "preset_cfg_gray_custom.h"

static const rx_staged_reg_list_t *preset_cfg_list[] = {
    &preset_reg_list_idle,
    &preset_reg_list_gray,
    &preset_reg_list_range_roi0,
    &preset_reg_list_range_roi1,
    &preset_reg_list_range_roi01,
    &preset_reg_list_gray_custom,
};

static const rx_parse_cfg_t *preset_parse_list[] = {
    &preset_rx_parse_idle,
    &preset_rx_parse_gray,
    &preset_rx_parse_range_roi0,
    &preset_rx_parse_range_roi1,
    &preset_rx_parse_range_roi01,
    &preset_rx_parse_gray_custom,
};

typedef struct {
    rx_lidar_mode_e             *user_cfg_list;   // 用户自定义的模式 ID 列表
    uint32_t                     user_cfg_cnt;    // 用户自定义的模式数量
    rx_staged_reg_list_t        *loaded_reg_list; // 当前已加载的寄存器配置
    rx_parse_cfg_t              *loaded_rx_parse; // 当前已加载的解析配置
} rx_cfg_loader_ctx_t;

static rx_cfg_loader_ctx_t s_ctx = {0};

static int32_t rx_cfg_check_devid_list(const int16_t *devid_list, int32_t devid_cnt, const char *label)
{
    bool valid = false;

    for (int32_t i = 0; i < devid_cnt; i++) {
        if (devid_list[i] == -1) {
            continue;
        }

        valid = true;

        for (int32_t j = 0; j < i; j++) {
            if ((devid_list[j] != -1) && (devid_list[j] == devid_list[i])) {
                FS_LOGE("Invalid %s cfg: duplicated devid %d", label, devid_list[i]);
                return FS_ERR_BAD_PARAMS;
            }
        }
    }

    if (!valid) {
        FS_LOGE("Invalid %s cfg", label);
        return FS_ERR_BAD_PARAMS;
    }

    return FS_SUCCESS;
}

static int32_t rx_cfg_check_channel_map_rows_hs(const uint8_t map[TX_HS_MAX_CHIP_CNT][TX_HS_MAX_CHANNEL_CNT], int32_t row_cnt, const char *label)
{
    const uint8_t invalid_value = (uint8_t)-1;

    for (int32_t i = 0; i < row_cnt; i++) {
        bool valid = false;

        for (int32_t j = 0; j < TX_HS_MAX_CHANNEL_CNT; j++) {
            if (map[i][j] != invalid_value) {
                valid = true;
                break;
            }
        }

        if (!valid) {
            FS_LOGE("Invalid %s cfg: row %d is all -1", label, i);
            return FS_ERR_BAD_PARAMS;
        }
    }

    return FS_SUCCESS;
}

static int32_t rx_cfg_check_channel_map_rows_ls(const uint8_t map[TX_LS_MAX_CHIP_CNT][TX_LS_MAX_CHANNEL_CNT], int32_t row_cnt, const char *label)
{
    const uint8_t invalid_value = (uint8_t)-1;

    for (int32_t i = 0; i < row_cnt; i++) {
        bool valid = false;

        for (int32_t j = 0; j < TX_LS_MAX_CHANNEL_CNT; j++) {
            if (map[i][j] != invalid_value) {
                valid = true;
                break;
            }
        }

        if (!valid) {
            FS_LOGE("Invalid %s cfg: row %d is all -1", label, i);
            return FS_ERR_BAD_PARAMS;
        }
    }

    return FS_SUCCESS;
}

static int32_t rx_cfg_gen_tx_channel_id(uint8_t map_len, uint8_t map_col, const uint8_t rx_ch_map[map_len][map_col], uint8_t tx_channel_id[map_len][map_col])
{
    int32_t err = FS_SUCCESS;
    uint8_t dst_col = 0U;

    if ((map_len == 0U) || (map_col == 0U)) {
        return FS_SUCCESS;
    }

    if ((rx_ch_map == NULL) || (tx_channel_id == NULL)) {
        FS_LOGE("[HUB] Error: Invalid ch map buffer! rx_ch_map=%p, tx_channel_id=%p", rx_ch_map, tx_channel_id);
        return FS_ERR_BAD_PARAMS;
    }

    uint8_t present[256] = {0};
    for (uint8_t row = 0U; row < map_len; row++) {
        for (uint8_t col = 0U; col < map_col; col++) {
            if (rx_ch_map[row][col] != (uint8_t)0xFF) {
                present[rx_ch_map[row][col]] = 1;
            }
        }
    }

    uint8_t rank_map[256] = {0};
    uint8_t current_rank = 0;
    for (uint16_t i = 0; i < 256; i++) {
        if (present[i]) {
            rank_map[i] = current_rank++;
        }
    }

    for (uint8_t row = 0U; row < map_len; row++) {
        for (uint8_t col = 0U; col < map_col; col++) {
            if (rx_ch_map[row][col] != (uint8_t)0xFF) {
                tx_channel_id[row][col] = rank_map[rx_ch_map[row][col]];
            }
            else {
                tx_channel_id[row][col] = (uint8_t)0xFF;
            }
        }
    }

    return err;
}

static int32_t rx_cfg_gen_tx_hi_seq(const uint8_t *chns, uint16_t chn_num, bool has_trp1,
                                    uint16_t seq_len, const uint64_t *ref_seq, uint64_t *seq_buf)
{
    int32_t err = FS_SUCCESS;
    CHECK_PTR(chns);
    CHECK_PTR(ref_seq);
    CHECK_PTR(seq_buf);

    if (chn_num == 0) {
        FS_LOGE("Set TX Hi Seq Mode: Channel number is 0");
        return FS_ERR_BAD_PARAMS;
    }
    else if (seq_len == 0) {
        FS_LOGE("Set TX Hi Seq Mode: Sequence length is 0");
        return FS_ERR_BAD_PARAMS;
    }

    memset(seq_buf, 0, seq_len * sizeof(*seq_buf));

    for (int32_t i = 0; i < chn_num; i++) {
        int32_t start;

        if (chns[i] >= seq_len) {
            FS_LOGE("Set TX Hi Seq Mode: Invalid channel[%d]=%d for single channel mode, seq_len=%d", i, chns[i], seq_len);
            return FS_ERR_BAD_PARAMS;
        }

        start = chns[i] - 1;

        if (start < 0) {
            start = 0;
        }
        else if (start > seq_len - 3) {
            start = seq_len - 3;
        }

        if (has_trp1) {
            seq_buf[start + 1] = ref_seq[chns[i]];
        }
        else {
            for (int32_t j = 0; j < 3; j++) {
                seq_buf[start + j] = ref_seq[chns[i]];
            }
        }
    }

    return err;
}

static int32_t rx_cfg_gen_tx_lo_seq(const uint8_t *chns, uint16_t chn_num,
                                    uint16_t seq_len, const uint64_t *ref_seq, uint64_t *seq_buf)
{
    int32_t err = FS_SUCCESS;
    CHECK_PTR(chns);
    CHECK_PTR(seq_buf);
    UNUSED_VAR(ref_seq);

    if (seq_len == 0) {
        FS_LOGE("Set TX Lo Seq Mode: Sequence length is 0");
        return FS_ERR_BAD_PARAMS;
    }

    memset(seq_buf, 0, seq_len * sizeof(*seq_buf));

    for (int32_t i = 0; i < chn_num; i++) {
        seq_buf[0] |= (1UL << chns[i]);
    }

    return err;
}

int32_t rx_cfg_check_validity(const trx_hw_cfg_t *hw_cfg, const rx_parse_cfg_t *rx_parse)
{
    int32_t err = FS_SUCCESS;
    CHECK_PTR(hw_cfg);
    CHECK_PTR(rx_parse);

    // 检查 tx_hs_ic_cnt 配置是否有效
    if ((hw_cfg->tx_hs_ic_cnt <= 0) || (hw_cfg->tx_hs_ic_cnt > TX_HS_MAX_CHIP_CNT)) {
        FS_LOGE("Invalid tx_hs_ic_cnt: %u", hw_cfg->tx_hs_ic_cnt);
        return FS_ERR_BAD_PARAMS;
    }
    else {
        err = rx_cfg_check_devid_list(hw_cfg->tx_hs_devid, hw_cfg->tx_hs_ic_cnt, "tx_hs_devid");

        if (err != FS_SUCCESS) {
            return FS_ERR_BAD_PARAMS;
        }

        err = rx_cfg_check_channel_map_rows_hs(hw_cfg->tx_hs_ch_map, hw_cfg->tx_hs_ic_cnt, "tx_hs_ch_map");
        if (err != FS_SUCCESS) {
            return FS_ERR_BAD_PARAMS;
        }
    }

    // 检查 tx_ls_ic_cnt 配置是否有效
    if ((hw_cfg->tx_ls_ic_cnt <= 0) || (hw_cfg->tx_ls_ic_cnt > TX_LS_MAX_CHIP_CNT)) {
        FS_LOGE("Invalid tx_ls_ic_cnt: %u", hw_cfg->tx_ls_ic_cnt);
        return FS_ERR_BAD_PARAMS;
    }
    else {
        err = rx_cfg_check_devid_list(hw_cfg->tx_ls_devid, hw_cfg->tx_ls_ic_cnt, "tx_ls_devid");

        if (err != FS_SUCCESS) {
            return FS_ERR_BAD_PARAMS;
        }

        err = rx_cfg_check_channel_map_rows_ls(hw_cfg->tx_ls_ch_map, hw_cfg->tx_ls_ic_cnt, "tx_ls_ch_map");
        if (err != FS_SUCCESS) {
            return FS_ERR_BAD_PARAMS;
        }
    }

    // 检查 tx_hs_grp_pre_ch 配置是否有效
    if (hw_cfg->tx_hs_grp_pre_ch == 0) {
        FS_LOGE("Invalid tx_hs_grp_pre_ch: %u", hw_cfg->tx_hs_grp_pre_ch);
        return FS_ERR_BAD_PARAMS;
    }
    else {
        uint32_t max_frame = rx_parse->is_ab_frame ? (uint32_t)(sizeof(rx_parse->frm_cfg) / sizeof(rx_parse->frm_cfg[0])) : 1;
        for (uint32_t frame = 0; frame < max_frame; frame++) {
            const rx_parse_frm_cfg_t *frm_cfg = &rx_parse->frm_cfg[frame];

            for (uint32_t i = 0; i < frm_cfg->group_cnt; i += hw_cfg->tx_hs_grp_pre_ch) {
                uint8_t group_id[hw_cfg->tx_hs_grp_pre_ch];
                bool found = false;
                int16_t devid;

                for (int32_t j = 0; j < hw_cfg->tx_hs_grp_pre_ch; j++) {
                    if (i + j < frm_cfg->group_cnt) {
                        group_id[j] = frm_cfg->group_cfg[i + j].group_id;
                    }
                    else {
                        group_id[j] = frm_cfg->group_cfg[frm_cfg->group_cnt - 1].group_id;
                    }
                }

                for (int32_t row = 0; row < hw_cfg->tx_hs_ic_cnt && !found; row++) {
                    for (int32_t col = 0; col < TX_HS_MAX_CHANNEL_CNT; col++) {
                        int32_t match_count = 0;

                        for (int32_t j = 0; j < hw_cfg->tx_hs_grp_pre_ch; j++) {
                            if ((uint32_t)group_id[j] - hw_cfg->tx_hs_ch_map[row][col] < hw_cfg->tx_hs_grp_pre_ch) {
                                match_count++;
                            }
                        }

                        if (match_count == hw_cfg->tx_hs_grp_pre_ch) {
                            found = true;
                            devid = hw_cfg->tx_hs_devid[row];
                            break;
                        }
                        else if (match_count > 0) {
                            FS_LOGE("Invalid tx_hs_ch_map cfg: group_id partially matched (%d/%u) for frm_cfg[%u].group_cfg[%u] at map[%d][%d]=%d",
                                    match_count, hw_cfg->tx_hs_grp_pre_ch, frame, i, row, col, hw_cfg->tx_hs_ch_map[row][col]);
                            return FS_ERR_BAD_PARAMS;
                        }
                    }
                }

                if (!found) {
                    FS_LOGE("Invalid tx_hs_ch_map cfg: group_id %u not found for frm_cfg[%u].group_cfg[%u]", group_id[0], frame, i);
                    return FS_ERR_BAD_PARAMS;
                }
                else if (hw_cfg->has_trp1 && (hw_cfg->tx_hs_ic_cnt == 2) && ((int16_t)((i % 4) / 2) != devid)) {
                    FS_LOGE("Invalid frm_cfg[%u].group_cfg[%u].group_id(%u): idx(%u) not aligned with tx_hs chip(%u)", frame, i, group_id[0], i, devid);
                    return FS_ERR_BAD_PARAMS;
                }
            }
        }
    }

    // FS_LOGD("rx_cfg_check_validity completed successfully");
    return err;
}

int32_t rx_cfg_gen_tx_seq(const trx_hw_cfg_t *hw_cfg, const rx_parse_cfg_t *rx_parse, tx_sequence_t *tx_seq)
{
    int32_t err = FS_SUCCESS;
    uint32_t max_frames;
    uint64_t temp_hs_seq[sizeof(rx_parse->frm_cfg) / sizeof(rx_parse->frm_cfg[0])][sizeof(tx_seq->hs_seq.sequence) / sizeof(tx_seq->hs_seq.sequence[0])] = {0};
    uint64_t temp_ls_seq[sizeof(rx_parse->frm_cfg) / sizeof(rx_parse->frm_cfg[0])][sizeof(tx_seq->ls_seq.sequence) / sizeof(tx_seq->ls_seq.sequence[0])] = {0};
    uint8_t  temp_hs_seq_cnt[sizeof(rx_parse->frm_cfg) / sizeof(rx_parse->frm_cfg[0])] = {0};
    uint8_t  temp_ls_seq_cnt[sizeof(rx_parse->frm_cfg) / sizeof(rx_parse->frm_cfg[0])] = {0};
    CHECK_PTR(rx_parse);
    CHECK_PTR(tx_seq);

    if (hw_cfg->tx_hs_grp_pre_ch <= 0) {
        FS_LOGE("Invalid tx_hs_grp_pre_ch: %u", hw_cfg->tx_hs_grp_pre_ch);
        return FS_ERR_BAD_PARAMS;
    }

    tx_seq->has_trp1  = hw_cfg->has_trp1;
    tx_seq->lock_slot = false;
    memset(tx_seq->hs_ch_id, 0xFF, sizeof(tx_seq->hs_ch_id));
    memset(tx_seq->ls_ch_id, 0xFF, sizeof(tx_seq->ls_ch_id));
    CHECK_ERR(rx_cfg_gen_tx_channel_id(hw_cfg->tx_hs_ic_cnt, sizeof(tx_seq->hs_ch_id[0]) / sizeof(tx_seq->hs_ch_id[0][0]), hw_cfg->tx_hs_ch_map, tx_seq->hs_ch_id));
    CHECK_ERR(rx_cfg_gen_tx_channel_id(hw_cfg->tx_ls_ic_cnt, sizeof(tx_seq->ls_ch_id[0]) / sizeof(tx_seq->ls_ch_id[0][0]), hw_cfg->tx_ls_ch_map, tx_seq->ls_ch_id));
    max_frames = rx_parse->is_ab_frame ? (uint32_t)(sizeof(rx_parse->frm_cfg) / sizeof(rx_parse->frm_cfg[0])) : 1;

    for (uint32_t frame = 0; frame < max_frames; frame++) {
        const rx_parse_frm_cfg_t *frm_cfg = &rx_parse->frm_cfg[frame];
        uint8_t block_cnt_per_slot;
        temp_hs_seq_cnt[frame] = (frm_cfg->group_cnt + hw_cfg->tx_hs_grp_pre_ch - 1) / hw_cfg->tx_hs_grp_pre_ch;
        FS_LOGI("Generating TX sequence for frame %u: group_cnt=%u, tx_hs_grp_pre_ch=%u, temp_hs_seq_cnt=%u",
                frame, frm_cfg->group_cnt, hw_cfg->tx_hs_grp_pre_ch, temp_hs_seq_cnt[frame]);
        for (uint16_t i = 0; i < temp_hs_seq_cnt[frame]; i++) {
            uint32_t group_idx = i * hw_cfg->tx_hs_grp_pre_ch;
            uint8_t  group_id[hw_cfg->tx_hs_grp_pre_ch];
            uint8_t  match_hs_ch_id = 0xFF;

            for (int32_t j = 0; j < hw_cfg->tx_hs_grp_pre_ch; j++) {
                if (group_idx + j < frm_cfg->group_cnt) {
                    group_id[j] = frm_cfg->group_cfg[group_idx + j].group_id;
                }
                else {
                    group_id[j] = frm_cfg->group_cfg[frm_cfg->group_cnt - 1].group_id;
                }
            }

            for (uint8_t row = 0; row < hw_cfg->tx_hs_ic_cnt; row++) {
                for (uint8_t col = 0; col < sizeof(hw_cfg->tx_hs_ch_map[0]) / sizeof(hw_cfg->tx_hs_ch_map[0][0]); col++) {
                    int32_t match_count = 0;

                    for (int32_t j = 0; j < hw_cfg->tx_hs_grp_pre_ch; j++) {
                        if ((uint32_t)group_id[j] - hw_cfg->tx_hs_ch_map[row][col] < hw_cfg->tx_hs_grp_pre_ch) {
                            match_count++;
                        }
                    }

                    if (match_count == hw_cfg->tx_hs_grp_pre_ch) {
                        match_hs_ch_id = tx_seq->hs_ch_id[row][col];
                        break;
                    }
                    else if (match_count > 0) {
                        FS_LOGE("Invalid tx_hs_ch_map cfg: group_id partially matched (%d/%u) for frm_cfg[%u].group_cfg[%u] at map[%d][%d]=%d",
                                match_count, hw_cfg->tx_hs_grp_pre_ch, frame, i, row, col, hw_cfg->tx_hs_ch_map[row][col]);
                        return FS_ERR_BAD_PARAMS;
                    }
                }

                if (match_hs_ch_id != 0xFF) {
                    break;
                }
            }

            if (match_hs_ch_id != 0xFF) {
                temp_hs_seq[frame][i] = (1ULL << match_hs_ch_id);
            }
            else {
                temp_hs_seq[frame][i] = 0;
            }
        }

        temp_ls_seq_cnt[frame] = frm_cfg->slot_cnt_per_group;

        if (frm_cfg->piece_cnt_per_block != hw_cfg->tx_hs_grp_pre_ch) {
            FS_LOGE("Invalid tx_hs_grp_pre_ch: tx_hs_grp_pre_ch(%u) != piece_cnt_per_block(%u)", hw_cfg->tx_hs_grp_pre_ch, frm_cfg->piece_cnt_per_block);
            return FS_ERR_BAD_PARAMS;
        }

        block_cnt_per_slot = frm_cfg->piece_cnt_per_slot / frm_cfg->piece_cnt_per_block;

        switch (block_cnt_per_slot) {
            case 6: {
                if (frm_cfg->piece_cnt_per_block == 2) {
                    /* slot_0 */
                    temp_ls_seq[frame][0] = (
                        (1UL <<  20)
                      | (1UL <<  16)
                      | (1UL <<  12)
                      | (1UL <<   8)
                      | (1UL <<   4)
                      | (1UL <<   0));
                    /* slot_1 */
                    temp_ls_seq[frame][1] = (
                        (1UL <<  21)
                      | (1UL <<  17)
                      | (1UL <<  13)
                      | (1UL <<   9)
                      | (1UL <<   5)
                      | (1UL <<   1));
                    /* slot_2 */
                    temp_ls_seq[frame][2] = (
                        (1UL <<  22)
                      | (1UL <<  18)
                      | (1UL <<  14)
                      | (1UL <<  10)
                      | (1UL <<   6)
                      | (1UL <<   2));
                    /* slot_3 */
                    temp_ls_seq[frame][3] = (
                        (1UL <<  23)
                      | (1UL <<  19)
                      | (1UL <<  15)
                      | (1UL <<  11)
                      | (1UL <<   7)
                      | (1UL <<   3));
                }
                else {
                    /* slot_0 */
                    temp_ls_seq[frame][0] = (
                        (1UL <<  44)
                      | (1UL <<  40)
                      | (1UL <<  36)
                      | (1UL <<  32)
                      | (1UL <<  28)
                      | (1UL <<  24)
                      | (1UL <<  20)
                      | (1UL <<  16)
                      | (1UL <<  12)
                      | (1UL <<   8)
                      | (1UL <<   4)
                      | (1UL <<   0));
                    /* slot_1 */
                    temp_ls_seq[frame][1] = (
                        (1UL <<  45)
                      | (1UL <<  41)
                      | (1UL <<  37)
                      | (1UL <<  33)
                      | (1UL <<  29)
                      | (1UL <<  25)
                      | (1UL <<  21)
                      | (1UL <<  17)
                      | (1UL <<  13)
                      | (1UL <<   9)
                      | (1UL <<   5)
                      | (1UL <<   1));
                    /* slot_2 */
                    temp_ls_seq[frame][2] = (
                        (1UL <<  46)
                      | (1UL <<  42)
                      | (1UL <<  38)
                      | (1UL <<  34)
                      | (1UL <<  30)
                      | (1UL <<  26)
                      | (1UL <<  22)
                      | (1UL <<  18)
                      | (1UL <<  14)
                      | (1UL <<  10)
                      | (1UL <<   6)
                      | (1UL <<   2));
                    /* slot_3 */
                    temp_ls_seq[frame][3] = (
                        (1UL <<  47)
                      | (1UL <<  43)
                      | (1UL <<  39)
                      | (1UL <<  35)
                      | (1UL <<  31)
                      | (1UL <<  27)
                      | (1UL <<  23)
                      | (1UL <<  19)
                      | (1UL <<  15)
                      | (1UL <<  11)
                      | (1UL <<   7)
                      | (1UL <<   3));
                }

                break;
            }

            case 12: {
                /* slot_0 */
                temp_ls_seq[frame][0] = (
                    (1UL <<  44)
                  | (1UL <<  40)
                  | (1UL <<  36)
                  | (1UL <<  32)
                  | (1UL <<  28)
                  | (1UL <<  24)
                  | (1UL <<  20)
                  | (1UL <<  16)
                  | (1UL <<  12)
                  | (1UL <<   8)
                  | (1UL <<   4)
                  | (1UL <<   0));
                /* slot_1 */
                temp_ls_seq[frame][1] = (
                    (1UL <<  45)
                  | (1UL <<  41)
                  | (1UL <<  37)
                  | (1UL <<  33)
                  | (1UL <<  29)
                  | (1UL <<  25)
                  | (1UL <<  21)
                  | (1UL <<  17)
                  | (1UL <<  13)
                  | (1UL <<   9)
                  | (1UL <<   5)
                  | (1UL <<   1));
                /* slot_2 */
                temp_ls_seq[frame][2] = (
                    (1UL <<  46)
                  | (1UL <<  42)
                  | (1UL <<  38)
                  | (1UL <<  34)
                  | (1UL <<  30)
                  | (1UL <<  26)
                  | (1UL <<  22)
                  | (1UL <<  18)
                  | (1UL <<  14)
                  | (1UL <<  10)
                  | (1UL <<   6)
                  | (1UL <<   2));
                /* slot_3 */
                temp_ls_seq[frame][3] = (
                    (1UL <<  47)
                  | (1UL <<  43)
                  | (1UL <<  39)
                  | (1UL <<  35)
                  | (1UL <<  31)
                  | (1UL <<  27)
                  | (1UL <<  23)
                  | (1UL <<  19)
                  | (1UL <<  15)
                  | (1UL <<  11)
                  | (1UL <<   7)
                  | (1UL <<   3));
                break;
            }

            default: {
                FS_LOGE("Invalid block_cnt_per_slot: %u", block_cnt_per_slot);
                return FS_ERR_BAD_PARAMS;
            }
        }
    }

    if (max_frames == 1) {
        tx_seq->hs_seq.seq_cnt = temp_hs_seq_cnt[0];
        tx_seq->ls_seq.seq_cnt = temp_ls_seq_cnt[0];
        memcpy(tx_seq->hs_seq.sequence, temp_hs_seq[0], temp_hs_seq_cnt[0] * sizeof(uint64_t));
        memcpy(tx_seq->ls_seq.sequence, temp_ls_seq[0], temp_ls_seq_cnt[0] * sizeof(uint64_t));
    }
    else {
        uint8_t min_cnt = (temp_hs_seq_cnt[0] < temp_hs_seq_cnt[1]) ? temp_hs_seq_cnt[0] : temp_hs_seq_cnt[1];

        for (uint8_t i = 0; i < min_cnt; i++) {
            if (temp_hs_seq[0][i] != temp_hs_seq[1][i]) {
                FS_LOGE("AB frame tx_hs_seq mismatch at idx %d, expected 0x%08x%08x, got 0x%08x%08x",
                    i,
                    (uint32_t)(temp_hs_seq[0][i] >> 32), (uint32_t)(temp_hs_seq[0][i] & 0xFFFFFFFF),
                    (uint32_t)(temp_hs_seq[1][i] >> 32), (uint32_t)(temp_hs_seq[1][i] & 0xFFFFFFFF));
                return FS_ERR_BAD_PARAMS;
            }
        }

        uint32_t final_frame = (temp_hs_seq_cnt[0] >= temp_hs_seq_cnt[1]) ? 0 : 1;
        tx_seq->hs_seq.seq_cnt = temp_hs_seq_cnt[final_frame];
        memcpy(tx_seq->hs_seq.sequence, temp_hs_seq[final_frame], temp_hs_seq_cnt[final_frame] * sizeof(uint64_t));

        if (temp_ls_seq_cnt[0] != temp_ls_seq_cnt[1]) {
            FS_LOGE("AB frame tx_ls_seq count mismatch: expected %u, got %u",
                temp_ls_seq_cnt[0],
                temp_ls_seq_cnt[1]);
            return FS_ERR_BAD_PARAMS;
        }

        for (uint8_t i = 0; i < temp_ls_seq_cnt[0]; i++) {
            if (temp_ls_seq[0][i] != temp_ls_seq[1][i]) {
                FS_LOGE("AB frame tx_ls_seq mismatch at idx %d, expected 0x%08x%08x, got 0x%08x%08x",
                    i,
                    (uint32_t)(temp_ls_seq[0][i] >> 32), (uint32_t)(temp_ls_seq[0][i] & 0xFFFFFFFF),
                    (uint32_t)(temp_ls_seq[1][i] >> 32), (uint32_t)(temp_ls_seq[1][i] & 0xFFFFFFFF));
                return FS_ERR_BAD_PARAMS;
            }
        }

        tx_seq->ls_seq.seq_cnt = temp_ls_seq_cnt[final_frame];
        memcpy(tx_seq->ls_seq.sequence, temp_ls_seq[final_frame], temp_ls_seq_cnt[final_frame] * sizeof(uint64_t));
    }

    return err;
}

int32_t rx_cfg_gen_trx_seq(tx_cfg_seq_mode_e mode, const uint8_t *chns, uint16_t chn_num, tx_sequence_t *tx_seq)
{
    int32_t err = FS_SUCCESS;
    static tx_cfg_seq_mode_e last_mode = TX_CFG_SEQ_MODE_FORWARD;
    static uint16_t last_x_chn_num       =  0,  last_y_chn_num       =  0;
    static uint8_t  last_x_chns_list[48] = {0}, last_y_chns_list[48] = {0};
    uint16_t         now_x_chn_num       =  0,   now_y_chn_num       =  0;
    uint8_t          now_x_chns_list[48] = {0},  now_y_chns_list[48] = {0};

    // 恢复上一次的工作模式
    if (mode == TX_CFG_SEQ_MODE_TX_RESUME) {
        mode = last_mode;

        if ((last_mode == TX_CFG_SEQ_MODE_SINGLE_Y_CHN) || (last_mode == TX_CFG_SEQ_MODE_MULTI_Y_CHN)) {
            chn_num = last_y_chn_num;
            chns    = last_y_chns_list;
        }
        else if ((last_mode == TX_CFG_SEQ_MODE_SINGLE_X_CHN) || (last_mode == TX_CFG_SEQ_MODE_MULTI_X_CHN)) {
            chn_num = last_x_chn_num;
            chns    = last_x_chns_list;
        }
    }

    switch (mode) {
        case TX_CFG_SEQ_MODE_REVERSE: {
            tx_hs_seq_t hs_seq;
            last_mode = mode;
            last_x_chn_num = 0;
            last_y_chn_num = 0;
            memset(last_x_chns_list, 0, sizeof(last_x_chns_list));
            memset(last_y_chns_list, 0, sizeof(last_y_chns_list));
            hs_seq = tx_seq->hs_seq;

            for (int i = 0; i < hs_seq.seq_cnt; i++) {
                hs_seq.sequence[i] = tx_seq->hs_seq.sequence[hs_seq.seq_cnt - 1 - i];
            }

            tx_seq->hs_seq = hs_seq;
            break;
        }

        case TX_CFG_SEQ_MODE_FORWARD: {
            last_mode = mode;
            last_x_chn_num = 0;
            last_y_chn_num = 0;
            memset(last_x_chns_list, 0, sizeof(last_x_chns_list));
            memset(last_y_chns_list, 0, sizeof(last_y_chns_list));
            break;
        }

        case TX_CFG_SEQ_MODE_SINGLE_Y_CHN:
            chn_num = 1;
            /* fall through */
        case TX_CFG_SEQ_MODE_MULTI_Y_CHN: {
            if (chn_num > (uint16_t)(sizeof(now_y_chns_list) / sizeof(now_y_chns_list[0]))) {
                FS_LOGE("Invalid Y channel number: %u > %u", chn_num, (uint16_t)(sizeof(now_y_chns_list) / sizeof(now_y_chns_list[0])));
                return FS_ERR_BAD_PARAMS;
            }

            now_y_chn_num = chn_num;
            memcpy(now_y_chns_list, chns, chn_num);
            break;
        }

        case TX_CFG_SEQ_MODE_BLOCK_CHN: {
            now_x_chn_num = 1;
            now_y_chn_num = 1;
            now_x_chns_list[0] = chns[0];
            now_y_chns_list[0] = chns[0];
            break;
        }

        case TX_CFG_SEQ_MODE_SINGLE_X_CHN:
            chn_num = 1;
            /* fall through */
        case TX_CFG_SEQ_MODE_MULTI_X_CHN: {
            if (chn_num > (uint16_t)(sizeof(now_x_chns_list) / sizeof(now_x_chns_list[0]))) {
                FS_LOGE("Invalid X channel number: %u > %u", chn_num, (uint16_t)(sizeof(now_x_chns_list) / sizeof(now_x_chns_list[0])));
                return FS_ERR_BAD_PARAMS;
            }

            now_x_chn_num = chn_num;
            memcpy(now_x_chns_list, chns, chn_num);
            break;
        }

        case TX_CFG_SEQ_MODE_CTRL_TX: {
            now_x_chn_num = chn_num;
            memcpy(now_x_chns_list, chns, chn_num);
            break;
        }

        case TX_CFG_SEQ_MODE_TX_RX_MERGE: {
            last_y_chn_num = 0;
            memset(last_y_chns_list, 0, sizeof(last_y_chns_list));
            break;
        }

        case TX_CFG_SEQ_MODE_TX_SUSPEND: {
            tx_ls_seq_t ls_seq;
            uint8_t chns = 0xFF;
            BREAK_ON_ERR(err = rx_cfg_gen_tx_lo_seq(&chns, 0, tx_seq->ls_seq.seq_cnt, tx_seq->ls_seq.sequence, ls_seq.sequence));
            ls_seq.seq_cnt = ((tx_seq->ls_seq.seq_cnt == 0) ? tx_seq->ls_seq.seq_cnt : 1);
            tx_seq->ls_seq = ls_seq;
            tx_seq->lock_slot = true;
            break;
        }

        case TX_CFG_SEQ_MODE_TX_RESUME: {
            break;
        }

        default: {
            FS_LOGE("Invalid TX sequence mode: %d", mode);
            return FS_ERR_BAD_PARAMS;
        }
    }

    if (mode == TX_CFG_SEQ_MODE_CTRL_TX) {
        tx_ls_seq_t ls_seq;
        uint8_t  tx_x_chns_list[48] = {0};
        uint16_t  tx_x_chn_num = 6;
        // 固定 23,19,15,11,7,3
        tx_x_chns_list[0] = 23;
        tx_x_chns_list[1] = 19;
        tx_x_chns_list[2] = 16;
        tx_x_chns_list[3] = 11;
        tx_x_chns_list[4] = 7;
        tx_x_chns_list[5] = 3;
        CHECK_ERR(rx_cfg_gen_tx_lo_seq(tx_x_chns_list, tx_x_chn_num, tx_seq->ls_seq.seq_cnt, tx_seq->ls_seq.sequence, ls_seq.sequence));
        FS_LOGI("tx_seq->ls_seq.seq_cnt=%u tx_seq->ls_seq.sequence=0x%016lx ls_seq.sequence=0x%016lx", tx_seq->ls_seq.seq_cnt, tx_seq->ls_seq.sequence[0], ls_seq.sequence[0]);
        ls_seq.seq_cnt = ((tx_seq->ls_seq.seq_cnt == 0) ? tx_seq->ls_seq.seq_cnt : 1);
        tx_seq->ls_seq = ls_seq;
        tx_seq->lock_slot = true;
    } else if (((mode >= TX_CFG_SEQ_MODE_SINGLE_Y_CHN) && (mode <= TX_CFG_SEQ_MODE_MULTI_X_CHN)) || (mode == TX_CFG_SEQ_MODE_TX_RX_MERGE)) {
        if (now_y_chn_num > 0) {
            tx_hs_seq_t hs_seq;
            CHECK_ERR(rx_cfg_gen_tx_hi_seq(now_y_chns_list, now_y_chn_num, tx_seq->has_trp1, tx_seq->hs_seq.seq_cnt, tx_seq->hs_seq.sequence, hs_seq.sequence));
            hs_seq.seq_cnt = tx_seq->hs_seq.seq_cnt;
            tx_seq->hs_seq = hs_seq;
        }
        else if (last_y_chn_num > 0) {
            tx_hs_seq_t hs_seq;
            CHECK_ERR(rx_cfg_gen_tx_hi_seq(last_y_chns_list, last_y_chn_num, tx_seq->has_trp1, tx_seq->hs_seq.seq_cnt, tx_seq->hs_seq.sequence, hs_seq.sequence));
            hs_seq.seq_cnt = tx_seq->hs_seq.seq_cnt;
            tx_seq->hs_seq = hs_seq;
        }

        if (now_x_chn_num > 0) {
            tx_ls_seq_t ls_seq;
            CHECK_ERR(rx_cfg_gen_tx_lo_seq(now_x_chns_list, now_x_chn_num, tx_seq->ls_seq.seq_cnt, tx_seq->ls_seq.sequence, ls_seq.sequence));
            ls_seq.seq_cnt = ((tx_seq->ls_seq.seq_cnt == 0) ? tx_seq->ls_seq.seq_cnt : 1);
            tx_seq->ls_seq = ls_seq;
            tx_seq->lock_slot = true;
        }
        else if (last_x_chn_num > 0) {
            tx_ls_seq_t ls_seq;
            CHECK_ERR(rx_cfg_gen_tx_lo_seq(last_x_chns_list, last_x_chn_num, tx_seq->ls_seq.seq_cnt, tx_seq->ls_seq.sequence, ls_seq.sequence));
            ls_seq.seq_cnt = ((tx_seq->ls_seq.seq_cnt == 0) ? tx_seq->ls_seq.seq_cnt : 1);
            tx_seq->ls_seq = ls_seq;
            tx_seq->lock_slot = true;
        }

        last_mode = mode;
        last_x_chn_num = now_x_chn_num;
        last_y_chn_num = now_y_chn_num;
        memcpy(last_x_chns_list, now_x_chns_list, sizeof(now_x_chns_list));
        memcpy(last_y_chns_list, now_y_chns_list, sizeof(now_y_chns_list));
    }

    return err;
}

int32_t rx_cfg_get_hw_cfg(tx_hw_mode_e hw_mode, trx_hw_cfg_t *hw_cfg)
{
    int32_t err = FS_SUCCESS;
    CHECK_PTR(hw_cfg);
    hw_cfg->tx_hw_mode = TX_HW_MODE_UNKNOWN;

    switch (hw_mode) {
        case TX_HW_MODE_3H_2L: {
            trx_hw_cfg_t cfg_3h_2l = {
                .tx_hw_mode = TX_HW_MODE_3H_2L,
                .tx_hs_grp_pre_ch = 1,
                .has_trp1 = false,
                .tx_hs_ic_cnt = 3,
                .tx_hs_devid = { 0, 1, 2, }, // 0: chip 0, 1: chip 1, 2: chip 2, -1: not device
                .tx_hs_ch_map = {
                    {  0,  1,  2,  3,  4,  5,  6,  7,  8,  9, 10, 11, 12, 13, 14, 15, -1, -1, },
                    { 16, 17, 18, 19, 20, 21, 22, 23, 24, 25, 26, 27, 28, 29, 30, 31, -1, -1, },
                    { 32, 33, 34, 35, 36, 37, 38, 39, 40, 41, 42, 43, 44, 45, 46, 47, -1, -1, },
                },
                .tx_ls_ic_cnt = 2,
                .tx_ls_devid = { 0, 1,     }, // 0: chip 0, 1: chip 1,            -1: not device
                .tx_ls_ch_map = {
                    {  0,  1,  2,  3,  4,  5,  6,  7,  8,  9, 10, 11, 12, 13, 14, 15, 16, 17, 18, 19, 20, 21, 22, 23, }, // chip 0: forward
                    { 47, 46, 45, 44, 43, 42, 41, 40, 39, 38, 37, 36, 35, 34, 33, 32, 31, 30, 29, 28, 27, 26, 25, 24, }, // chip 1: reverse
                },
            };
            *hw_cfg = cfg_3h_2l;
            break;
        }

        case TX_HW_MODE_1H_2L_R: {
            trx_hw_cfg_t cfg_1h_2l_r = {
                .tx_hw_mode = TX_HW_MODE_1H_2L_R,
                .tx_hs_grp_pre_ch = 2,
                .has_trp1 = false,
                .tx_hs_ic_cnt = 1,
                .tx_hs_devid = { 0, -1, -1, }, // 0: chip 0, 1: chip 1, 2: chip 2, -1: not device
                .tx_hs_ch_map = {
                    { 40, 38, 36, 34, 32, 30, 28, 26, 24, 22, 20, 18, 16, 14, 12, 10, 8, 6, },
                },
                .tx_ls_ic_cnt = 2,
                .tx_ls_devid = { 0, 1,     }, // 0: chip 0, 1: chip 1,            -1: not device
                .tx_ls_ch_map = {
                    {  0,  2,  4,  6,  8, 10, 12, 14, 16, 18, 20, 22, 24, 26, 28, 30, 32, 34, 36, 38, 40, 42, 44, 46, }, // chip 0: forward
                    { 46, 44, 42, 40, 38, 36, 34, 32, 30, 28, 26, 24, 22, 20, 18, 16, 14, 12, 10,  8,  6,  4,  2,  0, }, // chip 1: reverse
                },
            };
            *hw_cfg = cfg_1h_2l_r;
            break;
        }

        case TX_HW_MODE_1H_2L_C: {
            trx_hw_cfg_t cfg_1h_2l_c = {
                .tx_hw_mode = TX_HW_MODE_1H_2L_C,
                .tx_hs_grp_pre_ch = 2,
                .has_trp1 = false,
                .tx_hs_ic_cnt = 1,
                .tx_hs_devid = { 0, -1, -1, }, // 0: chip 0, 1: chip 1, 2: chip 2, -1: not device
                .tx_hs_ch_map = {
                    { 40, 36, 32, 28, 24, 20, 16, 12, 8, 38, 34, 30, 26, 22, 18, 14, 10, 6, },
                },
                .tx_ls_ic_cnt = 2,
                .tx_ls_devid = { 0, 1,     }, // 0: chip 0, 1: chip 1,            -1: not device
                .tx_ls_ch_map = {
                    {  0,  2,  4,  6,  8, 10, 12, 14, 16, 18, 20, 22, 24, 26, 28, 30, 32, 34, 36, 38, 40, 42, 44, 46, }, // chip 0: forward
                    { 46, 44, 42, 40, 38, 36, 34, 32, 30, 28, 26, 24, 22, 20, 18, 16, 14, 12, 10,  8,  6,  4,  2,  0, }, // chip 1: reverse
                },
            };
            *hw_cfg = cfg_1h_2l_c;
            break;
        }

        case TX_HW_MODE_2H_2L_TRP01: {
            trx_hw_cfg_t cfg_2h_2l_trp01 = {
                .tx_hw_mode = TX_HW_MODE_2H_2L_TRP01,
                .tx_hs_grp_pre_ch = 2,
                .has_trp1 = true,
                .tx_hs_ic_cnt = 2,
                .tx_hs_devid = { 0, 1, -1, }, // 0: chip 0, 1: chip 1, 2: chip 2, -1: not device
                .tx_hs_ch_map = {
                    { 40, 36, 32, 28, 24, 20, 16, 12, 8, -1, -1, -1, -1, -1, -1, -1, -1, -1, },
                    { 38, 34, 30, 26, 22, 18, 14, 10, 6, -1, -1, -1, -1, -1, -1, -1, -1, -1, },
                },

                .tx_ls_ic_cnt = 2,
                .tx_ls_devid = { 0, 1,     }, // 0: chip 0, 1: chip 1,            -1: not device
                .tx_ls_ch_map = {

                    {  0,  2,  4,  6,  8, 10, 12, 14, 16, 18, 20, 22, 24, 26, 28, 30, 32, 34, 36, 38, 40, 42, 44, 46, }, // chip 0: forward
                    { 46, 44, 42, 40, 38, 36, 34, 32, 30, 28, 26, 24, 22, 20, 18, 16, 14, 12, 10,  8,  6,  4,  2,  0, }, // chip 1: reverse
                }
            };
            *hw_cfg = cfg_2h_2l_trp01;
            break;
        }

        case TX_HW_MODE_3H_0L_S:
        case TX_HW_MODE_3H_0L_D:
        case TX_HW_MODE_2H_2L_TRP0:
        default: {
            FS_LOGE("Invalid hardware mode: %d", hw_mode);
            return FS_ERR_BAD_PARAMS;
        }
    }

    return err;
}

int32_t rx_cfg_get_reg_list(rx_lidar_mode_e lidar_mode, const rx_staged_reg_list_t **reg_list)
{
    // 先从 user_cfg_list 查找
    if (s_ctx.user_cfg_list != NULL) {
        for (uint32_t i = 0; i < s_ctx.user_cfg_cnt; i++) {
            if (s_ctx.user_cfg_list[i] == lidar_mode) {
                FS_LOGD("Found lidar_mode %d in user_cfg_list", lidar_mode);

                if ((s_ctx.loaded_reg_list == NULL) || (s_ctx.loaded_reg_list->lidar_mode != lidar_mode)) {
                    // TODO: 从用户配置文件中加载完整的寄存器列表并缓存到 s_ctx.loaded_reg_list, 以便后续快速访问
                }

                *reg_list = s_ctx.loaded_reg_list;
                return FS_SUCCESS;
            }
        }
    }

    // 再从 preset_cfg_list 加载
    uint32_t preset_cnt = sizeof(preset_cfg_list) / sizeof(preset_cfg_list[0]);
    for (uint32_t i = 0; i < preset_cnt; i++) {
        if (preset_cfg_list[i]->lidar_mode == lidar_mode) {
            FS_LOGD("Found lidar_mode %d in preset_cfg_list", lidar_mode);
            *reg_list = preset_cfg_list[i];
            return FS_SUCCESS;
        }
    }

    *reg_list = NULL;
    return FS_ERR_NOT_FOUND;
}

int32_t rx_cfg_get_rx_parse(rx_lidar_mode_e lidar_mode, const rx_parse_cfg_t **rx_parse)
{
    // 先从 user_cfg_list 查找
    if (s_ctx.user_cfg_list != NULL) {
        for (uint32_t i = 0; i < s_ctx.user_cfg_cnt; i++) {
            if (s_ctx.user_cfg_list[i] == lidar_mode) {
                FS_LOGD("Found lidar_mode %d in user_cfg_list", lidar_mode);

                if ((s_ctx.loaded_rx_parse == NULL) || (s_ctx.loaded_rx_parse->lidar_mode != lidar_mode)) {
                    // TODO: 从用户配置文件中加载完整的寄存器列表并缓存到 s_ctx.loaded_rx_parse, 以便后续快速访问
                }

                *rx_parse = s_ctx.loaded_rx_parse;
                return FS_SUCCESS;
            }
        }
    }

    // 再从 preset_parse_list 加载
    uint32_t preset_cnt = sizeof(preset_parse_list) / sizeof(preset_parse_list[0]);
    for (uint32_t i = 0; i < preset_cnt; i++) {
        if (preset_parse_list[i]->lidar_mode == lidar_mode) {
            FS_LOGD("Found lidar_mode %d in preset_parse_list", lidar_mode);
            *rx_parse = preset_parse_list[i];
            return FS_SUCCESS;
        }
    }

    *rx_parse = NULL;
    return FS_ERR_NOT_FOUND;
}

int32_t rx_cfg_patching_reg_list(const rx_staged_reg_list_t *reg_list, const trx_hw_cfg_t *hw_cfg, rx_staged_reg_list_t *patched_reg_list)
{
    int32_t err = FS_SUCCESS;
    int32_t i;
    CHECK_PTR(reg_list);
    CHECK_PTR(hw_cfg);
    CHECK_PTR(patched_reg_list);

    for (i = 0; i < (int32_t)reg_list->stage_count; i++) {
        if (patched_reg_list->stage_regs[i].regs != NULL) {
            FS_LOGE("Invalid patched_reg_list: stage_regs[%d].regs is not NULL", i);
            return FS_ERR_BAD_PARAMS;
        }

        patched_reg_list->stage_regs[i] = reg_list->stage_regs[i];

        if (patched_reg_list->stage_regs[i].stage == RX_STAGE_CONFIG) {
            /**
             * FIXME: 这里会修改到 preset_cfg_xxx 申请的内存, 但 preset_cfg_xxx 的定义是 const 的, 理论上不应该被修改.
             *        后续可以考虑在本函数为修改到的 stage 申请新的内存并复制数据, 以避免修改到 preset_cfg_xxx 定义的内存, 但要求调用方释放内存, 这容易造成内存泄露.
             */
            if (hw_cfg->has_trp1) {
                for (int32_t j = 0; j < (int32_t)patched_reg_list->stage_regs[i].count; j++) {
                    if (patched_reg_list->stage_regs[i].regs[j].addr == 0x00009188) {
                        patched_reg_list->stage_regs[i].regs[j].data |= 0x10; // set trp1_en bit
                    }
                }
            }
            else {
                for (int32_t j = 0; j < (int32_t)patched_reg_list->stage_regs[i].count; j++) {
                    if (patched_reg_list->stage_regs[i].regs[j].addr == 0x00009188) {
                        patched_reg_list->stage_regs[i].regs[j].data &= ~0x10U; // clear trp1_en bit
                    }
                }
            }
        }
    }

    patched_reg_list->lidar_mode  = reg_list->lidar_mode;
    patched_reg_list->stage_count = reg_list->stage_count;
    return err;
}

int32_t rx_cfg_get_stage_cfg(const rx_staged_reg_list_t *cfg, rx_stage_e stage, const rx_reg_t **stage_cfg, uint16_t *cfg_cnt)
{
    int32_t i;

    if (cfg == NULL || stage_cfg == NULL || cfg_cnt == NULL) {
        return FS_ERR_BAD_PARAMS;
    }

    *stage_cfg = NULL;
    *cfg_cnt   = 0;

    if (stage >= RX_STAGE_MAX) {
        return FS_ERR_BAD_PARAMS;
    }

    for (i = 0; i < (int32_t)cfg->stage_count; i++) {
        if (cfg->stage_regs[i].stage == stage) {
            *stage_cfg = cfg->stage_regs[i].regs;
            *cfg_cnt   = (uint16_t)cfg->stage_regs[i].count;
            return FS_SUCCESS;
        }
    }

    return FS_ERR_NOT_FOUND;
}

int32_t rx_cfg_loader_init(const char *cfg_load_path)
{
    UNUSED_VAR(cfg_load_path);
    rx_cfg_loader_cleanup(); // 先清理之前的用户配置
    // TODO: 在配置文件存储路径下加载所有配置(仅加载模式列表, 完整配置在需要使用时再去加载), 若模式与 preset_cfg_list 中的某项匹配, 则覆盖默认配置;
    return FS_SUCCESS;
}

void rx_cfg_loader_cleanup(void)
{
    if (s_ctx.user_cfg_list != NULL) {
        free(s_ctx.user_cfg_list);
        s_ctx.user_cfg_list = NULL;
    }

    if (s_ctx.loaded_reg_list != NULL) {
        free(s_ctx.loaded_reg_list);
        s_ctx.loaded_reg_list = NULL;
    }

    if (s_ctx.loaded_rx_parse != NULL) {
        free(s_ctx.loaded_rx_parse);
        s_ctx.loaded_rx_parse = NULL;
    }

    s_ctx.user_cfg_cnt  = 0;
    return;
}
