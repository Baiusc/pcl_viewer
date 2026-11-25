#ifndef __FS_ALGO_H__
#define __FS_ALGO_H__

#include <stdint.h>
#include <stdbool.h>
#include "algo_data_parse.h"
#include "fspc_support.h"
#include "concurrent_queue.h"
#include "fs_net_protocol.h"

#define RX_MAX_GROUP_CNT                  72   // 最大虚拟 group 数量
#define RX_MAX_FRAME_CFG                   2   // A/B 帧配置项最大数量
#define RX_DATA_MAX_PEAK_CNT               4   // 每个 pixel 最多包含的 peak 个数
#define RX_DATA_MAX_PIECE_CNT_PRE_GROUP   48   // 每个 group 内的最大 piece 数量
#define RX_ROI_INFO_VERSION             0x01

typedef enum {
    ALGO_DATA_TYPE_HIST = 0,
    ALGO_DATA_TYPE_ECHO,
    ALGO_DATA_TYPE_ECHO_HIST,
    ALGO_DATA_TYPE_CICL,
    ALGO_DATA_TYPE_GRAY_LEVEL,
} algo_data_type_e;

typedef enum {
    RX_PEAK_NUM_1 = 0,    // peak num = 1
    RX_PEAK_NUM_2,        // peak num = 2
    RX_PEAK_NUM_3,        // peak num = 3
    RX_PEAK_NUM_4,        // peak num = 4

    RX_PEAK_NUM_CNT,
} rx_peak_num_e;

/**
 * @brief PCL 单位计算模式
 */
typedef enum {
    FS_PCL_UNIT_ROUND_NEAREST = 0,  // 四舍五入（默认）
    FS_PCL_UNIT_ROUND_CEIL    = 1,  // 进一法（保证量程充足）
} fs_pcl_unit_round_mode_t;

typedef struct {
    uint32_t vi;
    uint32_t data_len;
    uint8_t* data;
    void *frame;
    uint64_t timestamp;
} algo_data_queue_t;

#pragma pack(push)
#pragma pack(1)

typedef struct {
    float row_idx;
    float col_idx;
    float distance;  // 距离
    float x;         // X Y Z 坐标信息
    float y;
    float z;
    float refl;      // 反射率
} FL6x32_algo_pcl_output_t;

typedef struct {
    uint8_t  sub_group_idx;
    uint32_t frame_id;
    uint16_t lidar_width;
    uint16_t lidar_height;
    uint8_t  *pcl_output;
} FL6x32_algo_pcl_result_t;

typedef struct {
    unsigned char version;
    unsigned char roi_cnt_a_frm;
    unsigned char roi_cnt_b_frm;
    unsigned char reserved[8];
    unsigned char data[0];
} FL6x32_algo_roi_msg_t;

typedef struct {
    unsigned short  group_start;                        // roi起始group
    unsigned short  piece_start;                        // roi起始piece
    unsigned short  row_start;                          // roi起始行号
    unsigned short  col_start;                          // roi起始列号
    unsigned short  row_size;                           // roi行size
    unsigned short  col_size;                           // roi列size
    unsigned short  rw_hist_length;                     // rw直方图长度
    unsigned short  rw_data_size;                       // rw数据size（目前直方图长度rw+4）
    unsigned char   echo_peak_num;                      // echo峰数
    unsigned short  echo_peak_width;                    // echo峰宽
    unsigned short  echo_data_size;                     // echo峰宽  noise peakaddr echo(峰宽57)
    unsigned short  TDC;                                // TDC:1,2,4,8
    unsigned short  binning_model[2];                   // [0]列binning-size,[1]行binning-size
    unsigned char   roll_mode;                          // roll 模式
    unsigned char   roll_idx;
    unsigned char   reserved[32];
} FL6x32_algo_roi_info_t;

typedef struct {
    uint8_t  channel_id;       // 将 pixel_id 转换为 channel id;
    uint8_t  chan_pix_id;      // 将 pixel_id 转换为 channel 中的 pixel id;
    uint8_t  slot_piece_id;    // 将 pixel_id 转换 slot 中的 piece id(不是物理 piece)
    uint8_t  phy_piece_id;     // 根据 slot_index, 将 slot_piece_id 转换为物理 piece id
    uint8_t  piece_pix_id;     // 将 pixel_id 转换为 piece 中的 pixel id
} pixel_decode_t;

typedef struct {
    uint16_t th_peak_addr;   // 寻峰结果 peak 地址(也就是 peak, 12bit)
    bool     crosstalk;      // th_peak_addr 是否为串扰峰: false -> 否; true -> 是
    uint32_t dist_rp;        // 信号峰上升沿位置对应的距离(16bit+8bit, 整数部分单位 cm)
    uint16_t fwhm;           // 信号峰峰宽(5bit+8bit, 整数部分单位 bin)
    uint32_t conv_intensity; // 信号峰卷积峰高(18bit)
    uint32_t dist_cent;      // 信号峰质心结果对应的距离(16bit+8bit, 整数部分单位 cm)
    uint16_t hist_peak_max;  // 信号峰原始 bin 最大值(信号峰高, 12bit)
} algo_result_unpack_t;

#pragma pack(pop)

typedef struct {
    uint8_t *buffer;
    uint32_t buffer_size;
} algo_buffer_t;

typedef struct {
    uint16_t group_cnt;
    struct {
        uint32_t addr_base;
        uint16_t pixel_len;
        uint16_t pixel_cnt;
    } group[RX_MAX_GROUP_CNT];
} algo_group_cfg_t;

typedef struct {
    algo_buffer_t pixel_buffer;
    algo_buffer_t gray_buffer;
    rx_frame_parse_ctx_t parse_ctx;
    fs_sdk_work_mode_e work_mode;
    const rx_parse_cfg_t *parse_cfg;
    algo_group_cfg_t group_cfg[RX_MAX_FRAME_CFG];

    ConcurrentQueue* algo_queue;
    pthread_t algo_thread_id;
    int32_t algo_thread_run_flag;

    ConcurrentQueue* pixel_queue;
    pthread_t pixel_thread_id;

    ConcurrentQueue* gray_queue;
    pthread_t gray_thread_id;
} algo_context_t;

typedef struct {
    uint32_t cur_lidar_frm_idx;  // 当前Lidar帧ID，1~65535
    uint32_t cur_mipi_frm_idx;   // 当前Lidar帧的MIPI帧ID
    uint32_t cur_ab_flag;        // 当前Lidar帧的A/B帧标志，0->A帧，1->B帧
    bool lidar_frm_ready;       // Lidar帧接收完整
    bool find_start_frm;        // 指示是否找到第一个Lidar帧的第一个mipi
    uint32_t pixel_count;
    uint32_t pixel_len;
    uint32_t total_len;
} lidar_frm_info_t;

#ifdef __cplusplus
extern "C" {
#endif

int32_t algo_init(const rx_parse_cfg_t *rx_parse, fs_sdk_work_mode_e work_mode);
int32_t algo_deinit(void);
int32_t algo_queue_push_data(algo_data_queue_t* data);
int32_t algo_queue_pop_data(algo_data_queue_t** data, int timeout_ms);
int32_t algo_pixel_queue_push_data(fs_lidar_data_info_t* data);
int32_t algo_pixel_queue_pop_data(fs_lidar_data_info_t** data, int timeout_ms);

int32_t algo_gray_queue_push_data(fs_lidar_data_info_t* data);
int32_t algo_gray_queue_pop_data(fs_lidar_data_info_t** data, int timeout_ms);

int32_t algo_get_roi_info(FL6x32_algo_roi_info_t *info, uint8_t *info_cnt);
int32_t algo_get_roi_msg(FL6x32_algo_roi_msg_t* info, uint32_t *info_size);
int32_t algo_update_tx_temp(float temp_tx);
int32_t algo_update_rx_temp(const float *temp_rx, uint32_t temp_rx_cnt);

/**
 * @brief 根据最大测距范围自动计算点云数据单位
 *
 * @param range_max 最大测距范围，单位：cm
 * @param mode 舍入模式：FS_PCL_UNIT_ROUND_NEAREST(四舍五入) 或 FS_PCL_UNIT_ROUND_CEIL(进一法)
 * @return uint8_t pcl_unit 编码值 (bit[6:5]:整数部分，bit[4:0]:小数部分)
 *
 * @note 计算公式：pcl_unit = range_max / 65535 (uint16_t 最大值)
 *       编码规则：整数部分占 2bit(bit[6:5])，小数部分占 5bit(bit[4:0])
 *       支持的小数精度：0.00000, 0.03125, 0.06250, ..., 0.96875
 *
 *       - 四舍五入：精度优先，有溢出风险
 *       - 进一法：量程优先，确保 uint16_t 最大值能覆盖 range_max，避免溢出
 */
uint8_t fs_calc_pcl_unit(float range_max, fs_pcl_unit_round_mode_t mode);

/**
 * @brief 从 pcl_unit 编码值解码实际单位
 *
 * @param pcl_unit 编码值
 * @return float 实际单位值 (cm)
 *
 * @note 解码公式：unit = integer_part + frac_bits * 0.25
 */
float fs_decode_pcl_unit(uint8_t pcl_unit);

fs_pcl_peak_type_t algo_conver_peak_type(uint32_t cfg_type);

#ifdef __cplusplus
}
#endif

#endif // __FS_ALGO_H__
