#ifndef __PRESET_CFG_IDLE_H__
#define __PRESET_CFG_IDLE_H__

#include "rx_cfg_loader.h"

#ifdef __LIDAR_MODE__
#undef __LIDAR_MODE__
#endif

#ifdef __PRESET_REG_LIST__
#undef __PRESET_REG_LIST__
#endif

#ifdef __PRESET_RX_PARSE__
#undef __PRESET_RX_PARSE__
#endif

#define __LIDAR_MODE__         RX_LIDAR_MODE_IDLE
#define __PRESET_REG_LIST__    preset_reg_list_idle
#define __PRESET_RX_PARSE__    preset_rx_parse_idle

const rx_staged_reg_list_t __PRESET_REG_LIST__ = {
    .lidar_mode = __LIDAR_MODE__,
    .stage_count = 8,
    .stage_regs = {
        {
            .stage = RX_STAGE_INIT,
            .count = 2,
            .capacity = 2,
            .regs = (rx_reg_t[]) {
                { 0x0000300C, 0x00000000, 2000, },
                { 0x0000300C, 0x00000001, 2000, },
            },
        },
        {
            .stage = RX_STAGE_WAKEUP,
            .count = 4,
            .capacity = 4,
            .regs = (rx_reg_t[]) {
                { 0x00001134, 0x00000001,    0, },
                { 0x00001110, 0x00000001,    0, },
                { 0x00003004, 0x000152AA,    0, },
                { 0x00003008, 0x00000038,    0, },
            },
        },
        {
            .stage = RX_STAGE_CONFIG,
            .count = 0,
            .capacity = 0,
            .regs = NULL,
        },
        {
            .stage = RX_STAGE_START,
            .count = 0,
            .capacity = 0,
            .regs = NULL,
        },
        {
            .stage = RX_STAGE_PAUSE,
            .count = 0,
            .capacity = 0,
            .regs = NULL,
        },
        {
            .stage = RX_STAGE_RESTART,
            .count = 0,
            .capacity = 0,
            .regs = NULL,
        },
        {
            .stage = RX_STAGE_STOP,
            .count = 0,
            .capacity = 0,
            .regs = NULL,
        },
        {
            .stage = RX_STAGE_SLEEP,
            .count = 0,
            .capacity = 0,
            .regs = NULL,
        },
    },
};

const rx_parse_cfg_t __PRESET_RX_PARSE__ = {
    .lidar_mode = __LIDAR_MODE__,
    .is_ab_frame = false,
    .frm_cfg = {
        {
            .is_b_frame = false,
            .vc = 0,
            .width = 3504,
            .height = 144,
            .mipi_frame_cnt = 1,
            .loop_num = 1,
            .slot_cnt_per_group = 4,
            .channel_cnt_per_slot = 12,
            .pixel_cnt_per_pkg = 192,
            .half_piece_per_ch = 2,
            .piece_cnt_per_block = 2,
            .piece_cnt_per_slot = 12,
            .start_group = 6,
            .piece_roll_ctrl = 0,
            .ev_pkt_cfg = { .en_ev_pkt = false, .en_laser_ev = false },
            .pkt_info_cfg = { .en_pkt_info = true, .en_ev_info = true, .pkt_ascii = 0x47C },
            .group_cnt = 36,
            .group_cfg = {
                { .group_id = 41, .ROI_sel = 0, .row_offset = 420, },
                { .group_id = 40, .ROI_sel = 0, .row_offset = 408, },
                { .group_id = 39, .ROI_sel = 0, .row_offset = 396, },
                { .group_id = 38, .ROI_sel = 0, .row_offset = 384, },
                { .group_id = 37, .ROI_sel = 0, .row_offset = 372, },
                { .group_id = 36, .ROI_sel = 0, .row_offset = 360, },
                { .group_id = 35, .ROI_sel = 0, .row_offset = 348, },
                { .group_id = 34, .ROI_sel = 0, .row_offset = 336, },
                { .group_id = 33, .ROI_sel = 0, .row_offset = 324, },
                { .group_id = 32, .ROI_sel = 0, .row_offset = 312, },
                { .group_id = 31, .ROI_sel = 0, .row_offset = 300, },
                { .group_id = 30, .ROI_sel = 0, .row_offset = 288, },
                { .group_id = 29, .ROI_sel = 0, .row_offset = 276, },
                { .group_id = 28, .ROI_sel = 0, .row_offset = 264, },
                { .group_id = 27, .ROI_sel = 0, .row_offset = 252, },
                { .group_id = 26, .ROI_sel = 0, .row_offset = 240, },
                { .group_id = 25, .ROI_sel = 0, .row_offset = 228, },
                { .group_id = 24, .ROI_sel = 0, .row_offset = 216, },
                { .group_id = 23, .ROI_sel = 0, .row_offset = 204, },
                { .group_id = 22, .ROI_sel = 0, .row_offset = 192, },
                { .group_id = 21, .ROI_sel = 0, .row_offset = 180, },
                { .group_id = 20, .ROI_sel = 0, .row_offset = 168, },
                { .group_id = 19, .ROI_sel = 0, .row_offset = 156, },
                { .group_id = 18, .ROI_sel = 0, .row_offset = 144, },
                { .group_id = 17, .ROI_sel = 0, .row_offset = 132, },
                { .group_id = 16, .ROI_sel = 0, .row_offset = 120, },
                { .group_id = 15, .ROI_sel = 0, .row_offset = 108, },
                { .group_id = 14, .ROI_sel = 0, .row_offset =  96, },
                { .group_id = 13, .ROI_sel = 0, .row_offset =  84, },
                { .group_id = 12, .ROI_sel = 0, .row_offset =  72, },
                { .group_id = 11, .ROI_sel = 0, .row_offset =  60, },
                { .group_id = 10, .ROI_sel = 0, .row_offset =  48, },
                { .group_id =  9, .ROI_sel = 0, .row_offset =  36, },
                { .group_id =  8, .ROI_sel = 0, .row_offset =  24, },
                { .group_id =  7, .ROI_sel = 0, .row_offset =  12, },
                { .group_id =  6, .ROI_sel = 0, .row_offset =   0, },
            },
            .noise_cfg = { .r_amb = 64, .o_amb = 0, .env_det_time = 11 },
            .pixel_cfg = {
                .en_pixel_data = false,
                .is_range_data = false,
                .data_type_cfg = { .hist_cfg = {
                    .en_pixel_info = false,
                    .en_peak = false,
                    .en_peak_sum = false,
                    .en_echo_bin = false,
                    .en_hist_main = false,
                    .en_hist_sub = false,
                    .peak_cnt = 0,
                    .peak_type = {0, 0, 0, 0},
                } },
                .ROI_cnt = 0,
                .ROI_cfg = {
                },
            },
            .gray_cfg = { .en_gray_data = true, .gray_type = 6, .frame_gray_len = 1728, .mixed_gray_len = {0, 0} },
            .auto_cfg = {
                .packet_info_len = 40,
                .ROI_cfg = {
                },
            },
        },
    },
};

#endif /* __PRESET_CFG_IDLE_H__ */
