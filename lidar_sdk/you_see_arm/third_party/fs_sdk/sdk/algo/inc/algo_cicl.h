#ifndef __FS_ALGO_CICL_H__
#define __FS_ALGO_CICL_H__

#include <stdint.h>
#include <stdbool.h>
#include "algo.h"

#pragma pack(push)
#pragma pack(1)

// Re-packetizing Packet Information
typedef struct {
    uint16_t packet_type;
    uint16_t frame_index;       // laser帧编号，从1开始
    uint16_t packet_id;         // MIPI帧内的packet编号，从1开始
    uint16_t group_index;       // 当前的group index
    uint16_t slot_index;        // 当前的slot index
    uint32_t timestamp;         // 时间戳
    uint8_t ab_frame_sel;       // 是否AB帧
    uint8_t sub_frame_idx;      // sub-frame 编号
    uint8_t piece_roll_mode;    // sub-frame 模式: 最大 sub-frame 数 = 2^n, n = piece_roll_mode -> {0, 1, 2}
    uint8_t tdc_reso_mode;      // tdc 分辨率模式
} pkt_info_unpack_t;

/*
 * Cicl Pixel Information after unpacketizing
 */
typedef struct {
    pkt_info_unpack_t pkt_info;

    uint16_t        pixel_id;           // 本Pixel在当前Slot内的编号，从0开始
    uint32_t        noise_amb;          // 寻峰算出的噪声阈值
    uint16_t        amb_num;            // laness统计的环境光总计数
    uint32_t        det_sum;            // laness统计的直方图总计数
    uint16_t        real_tx_num;        // laness统计的真实tx次数。
    uint16_t        lpwr_real_tx_num;   // 高低功率分开存储时，低功率的统计tx次数；其他情况为0。
    bool            en_algo_echo_bin;   // echo_bin 开关
    rx_peak_num_e   peak_num;           // peak 数量;
    uint8_t         echo_sample_l[RX_DATA_MAX_PEAK_CNT]; // echo 左采样 bin 数: 1~127
    uint8_t         echo_sample_r[RX_DATA_MAX_PEAK_CNT]; // echo 右采样 bin 数: 1~127
    rx_binning_e    binning_type;       // 当前Pixel所在组的bining模式
    pixel_decode_t  pixel_decode;       // 根据group_id, slot_id, pixel_id转换的物理坐标信息
    uint16_t row;                       // 物理行坐标
    uint16_t col;                       // 物理列坐标
    uint8_t data[0];                    // echo bin 数据起始地址
} cicl_pix_info_t;

#pragma pack(pop)

#ifdef __cplusplus
extern "C" {
#endif

int32_t algo_pcl_init_param(int32_t ab_idx, int32_t roi_cnt);
int32_t algo_pcl_uninit(int32_t ab_idx);
void algo_pcl_reset_globals(void);
int32_t algo_pcl_update_tx_temp(float temp_tx);
int32_t algo_pcl_update_rx_temp(const float *temp_rx, uint32_t temp_rx_cnt);
int32_t algo_cicl_calc_cfg(algo_context_t* ctx, int32_t ab_idx);
int32_t algo_cicl_data_process(algo_context_t* ctx, int32_t vi, uint8_t *data, uint32_t data_len, uint64_t timestamp);

#ifdef __cplusplus
}
#endif

#endif // __FS_ALGO_CICL_H__
