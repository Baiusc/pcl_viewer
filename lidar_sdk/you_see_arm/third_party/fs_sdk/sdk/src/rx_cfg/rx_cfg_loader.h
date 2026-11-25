#ifndef __RX_CFG_LOADER_H__
#define __RX_CFG_LOADER_H__

#include <stdint.h>
#include <stdbool.h>

//================================== enum & define===================================

#define RX_MAX_FRAME_CFG               2   // A/B 帧配置项最大数量
#define RX_MAX_GROUP_CNT              72   // 最大虚拟 group 数量
#define RX_MAX_PEAK_CNT                4   // 每个 pixel 最多包含的 peak 个数(echo 模式)
#define RX_MAX_ALGO_PEAK_CNT           2   // 每个 pixel 最多包含的 peak 个数(range 模式)
#define RX_ROI_CFG_CNT                 4   // 最大 ROI 个数

#define TX_HS_MAX_CHIP_CNT             3   // TX High Side 最大芯片个数
#define TX_HS_MAX_CHANNEL_CNT         18   // TX High Side 使用的 channel 的个数(每颗芯片)
#define TX_HS_MAX_CHANNEL_SEQUENCE    22   // TX High Side 使用的 sequence 的个数(每颗芯片)
#define TX_HS_SEQUENCE_MAX_LEN (TX_HS_MAX_CHIP_CNT * TX_HS_MAX_CHANNEL_SEQUENCE) // TX High Side 工作序列最大长度(单位: 64-bit)
#define TX_LS_MAX_CHIP_CNT             2   // TX Low  Side 最大芯片个数
#define TX_LS_MAX_CHANNEL_CNT         24   // TX Low  Side 使用的 channel 的个数(每颗芯片)
#define TX_LS_SEQUENCE_MAX_LEN         4   // TX Low Side 工作序列最大长度(单位: 64-bit)

typedef enum {
    TX_HW_MODE_UNKNOWN     = 0,                       // 未知模式，非法值
    TX_HW_MODE_3H_0L_S     = 1,                       // 3高 0 低 - 高边正序连接(trp0  ), 低边使用第3方芯片(单边);
    TX_HW_MODE_3H_0L_D     = 2,                       // 3高 0 低 - 高边正序连接(trp0  ), 低边使用第3方芯片(双边);
    TX_HW_MODE_3H_2L       = 3,                       // 3高 2 低 - 高边正序连接(trp0  ), 低边正序+反序连接(单边);
    TX_HW_MODE_1H_2L_R     = 4,                       // 1高1/2低 - 高边反序连接(trp0  ), 低边正序+反序连接(双边);
    TX_HW_MODE_1H_2L_C     = 5,                       // 1高1/2低 - 高边交叉连接(trp0  ), 低边正序+反序连接(双边);
    TX_HW_MODE_2H_2L_TRP0  = 6,                       // 2高1/2低 - 高边反序连接(trp0  ), 低边正序+反序连接(双边);
    TX_HW_MODE_2H_2L_TRP01 = 7,                       // 2高1/2低 - 高边反序连接(trp0/1), 低边正序+反序连接(双边);

    TX_HW_MODE_DEFAULT     = TX_HW_MODE_2H_2L_TRP01 , // TODO: 默认使用的硬件模式(双高双低或双高单低), 客户需按实际情况配置
} tx_hw_mode_e;

// RX 配置工作模式定义
typedef enum {
    RX_LIDAR_MODE_IDLE       =  0,         // RX 待机模式

    RX_LIDAR_MODE_GRAY       = 10,         // RX 灰度图模式(AA 调焦)
    RX_LIDAR_MODE_GRAY_CUSTOM = 11,         // RX 灰度图模式(自定义)

    RX_LIDAR_MODE_RANGE_0    = 20,         // RX Range 模式(ROI-0)
    RX_LIDAR_MODE_RANGE_1    = 21,         // RX Range 模式(ROI-1)
    RX_LIDAR_MODE_RANGE_2    = 22,         // RX Range 模式(ROI-2)
    RX_LIDAR_MODE_RANGE_3    = 23,         // RX Range 模式(ROI-3)
    RX_LIDAR_MODE_RANGE_01   = 24,         // RX Range 模式(ROI-0/1)
    RX_LIDAR_MODE_RANGE_02   = 25,         // RX Range 模式(ROI-0/2)
    RX_LIDAR_MODE_RANGE_03   = 26,         // RX Range 模式(ROI-0/3)
    RX_LIDAR_MODE_RANGE_12   = 27,         // RX Range 模式(ROI-1/2)
    RX_LIDAR_MODE_RANGE_13   = 28,         // RX Range 模式(ROI-1/3)
    RX_LIDAR_MODE_RANGE_23   = 29,         // RX Range 模式(ROI-2/3)
    RX_LIDAR_MODE_RANGE_012  = 30,         // RX Range 模式(ROI-0/1/2)
    RX_LIDAR_MODE_RANGE_013  = 31,         // RX Range 模式(ROI-0/1/3)
    RX_LIDAR_MODE_RANGE_023  = 32,         // RX Range 模式(ROI-0/2/3)
    RX_LIDAR_MODE_RANGE_123  = 33,         // RX Range 模式(ROI-1/2/3)
    RX_LIDAR_MODE_RANGE_0123 = 34,         // RX Range 模式(ROI-0/1/2/3)

    RX_LIDAR_MODE_ECHO_0     = 40,         // RX Echo  模式(ROI-0)
    RX_LIDAR_MODE_ECHO_1     = 41,         // RX Echo  模式(ROI-1)
    RX_LIDAR_MODE_ECHO_2     = 42,         // RX Echo  模式(ROI-2)
    RX_LIDAR_MODE_ECHO_3     = 43,         // RX Echo  模式(ROI-3)
    RX_LIDAR_MODE_ECHO_01    = 44,         // RX Echo  模式(ROI-0/1)
    RX_LIDAR_MODE_ECHO_02    = 45,         // RX Echo  模式(ROI-0/2)
    RX_LIDAR_MODE_ECHO_03    = 46,         // RX Echo  模式(ROI-0/3)
    RX_LIDAR_MODE_ECHO_12    = 47,         // RX Echo  模式(ROI-1/2)
    RX_LIDAR_MODE_ECHO_13    = 48,         // RX Echo  模式(ROI-1/3)
    RX_LIDAR_MODE_ECHO_23    = 49,         // RX Echo  模式(ROI-2/3)
    RX_LIDAR_MODE_ECHO_012   = 50,         // RX Echo  模式(ROI-0/1/2)
    RX_LIDAR_MODE_ECHO_013   = 51,         // RX Echo  模式(ROI-0/1/3)
    RX_LIDAR_MODE_ECHO_023   = 52,         // RX Echo  模式(ROI-0/2/3)
    RX_LIDAR_MODE_ECHO_123   = 53,         // RX Echo  模式(ROI-1/2/3)
    RX_LIDAR_MODE_ECHO_0123  = 54,         // RX Echo  模式(ROI-0/1/2/3)

    RX_LIDAR_MODE_HIST_0     = 50,         // RX Hist  模式(ROI-0)
    RX_LIDAR_MODE_HIST_1     = 51,         // RX Hist  模式(ROI-1)
    RX_LIDAR_MODE_HIST_2     = 52,         // RX Hist  模式(ROI-2)
    RX_LIDAR_MODE_HIST_3     = 53,         // RX Hist  模式(ROI-3)
    RX_LIDAR_MODE_HIST_01    = 54,         // RX Hist  模式(ROI-0/1)
    RX_LIDAR_MODE_HIST_02    = 55,         // RX Hist  模式(ROI-0/2)
    RX_LIDAR_MODE_HIST_03    = 56,         // RX Hist  模式(ROI-0/3)
    RX_LIDAR_MODE_HIST_12    = 57,         // RX Hist  模式(ROI-1/2)
    RX_LIDAR_MODE_HIST_13    = 58,         // RX Hist  模式(ROI-1/3)
    RX_LIDAR_MODE_HIST_23    = 59,         // RX Hist  模式(ROI-2/3)
    RX_LIDAR_MODE_HIST_012   = 60,         // RX Hist  模式(ROI-0/1/2)
    RX_LIDAR_MODE_HIST_013   = 61,         // RX Hist  模式(ROI-0/1/3)
    RX_LIDAR_MODE_HIST_023   = 62,         // RX Hist  模式(ROI-0/2/3)
    RX_LIDAR_MODE_HIST_123   = 63,         // RX Hist  模式(ROI-1/2/3)
    RX_LIDAR_MODE_HIST_0123  = 64,         // RX Hist  模式(ROI-0/1/2/3)
} rx_lidar_mode_e;

// 工作阶段定义
typedef enum {
    RX_STAGE_INIT    = 0,         // 初始化阶段
    RX_STAGE_WAKEUP  = 1,         // 唤醒阶段
    RX_STAGE_CONFIG  = 2,         // 配置阶段
    RX_STAGE_START   = 3,         // 启动阶段
    RX_STAGE_PAUSE   = 4,         // 暂停阶段
    RX_STAGE_RESTART = 5,         // 再次启动阶段
    RX_STAGE_STOP    = 6,         // 停止阶段
    RX_STAGE_SLEEP   = 7,         // 睡眠阶段

    RX_STAGE_MAX                  // 最大阶段数
} rx_stage_e;

typedef enum {
    RX_REG_DELAY_US = 0xFFFFFFFF, // 功能码: 寄存器写入后延时
} rx_reg_op_e;

typedef enum {
    TX_CFG_SEQ_MODE_REVERSE       = 0,    // 全扫反序点亮
    TX_CFG_SEQ_MODE_FORWARD       = 1,    // 全扫正序点亮
    TX_CFG_SEQ_MODE_SINGLE_Y_CHN  = 2,    // 单通道(Y)点亮
    TX_CFG_SEQ_MODE_MULTI_Y_CHN   = 3,    // 多通道(Y)点亮

    TX_CFG_SEQ_MODE_SINGLE_X_CHN  = 4,    // 单通道(X)点亮
    TX_CFG_SEQ_MODE_BLOCK_CHN     = 5,    // 块(X,Y)扫描点亮
    TX_CFG_SEQ_MODE_MULTI_X_CHN   = 6,    // 多通道(X)点亮
    TX_CFG_SEQ_MODE_CTRL_TX       = 7,    // TX 光斑控制
    TX_CFG_SEQ_MODE_TX_RX_MERGE   = 8,    // TX & RX 融合

    TX_CFG_SEQ_MODE_TX_SUSPEND    = 50,   // TX 暂停: 关闭
    TX_CFG_SEQ_MODE_TX_RESUME     = 51,   // TX 恢复: 开启
} tx_cfg_seq_mode_e;

//================================== struct ===================================

#pragma pack(push)
#pragma pack(1) // one byte-aligned

//---------------------------------- reg cfg list -----------------------------------

// 单个寄存器配置
typedef struct {
    uint32_t addr;   // 寄存器地址或功能码(rx_reg_op_e)
    uint32_t data;   // 寄存器数据
    uint32_t delay;  // 写入后延时(单位: us), 0 表示无延时
} rx_reg_t;

// 单个工作阶段的寄存器列表
typedef struct {
    rx_stage_e        stage;       // 工作阶段
    uint32_t          count;       // 当前寄存器数量
    uint32_t          capacity;    // 寄存器数组容量
    rx_reg_t         *regs;        // 寄存器数组
} rx_reg_list_t;

typedef struct {
    uint8_t  seq_cnt;                                                 // TX High side sequence count
    uint64_t sequence[TX_HS_SEQUENCE_MAX_LEN];                        // TX High side sequence
} tx_hs_seq_t;

typedef struct {
    uint8_t  seq_cnt;                                                 // TX Low  side sequence count
    uint64_t sequence[TX_LS_SEQUENCE_MAX_LEN];                        // TX Low  side sequence
} tx_ls_seq_t;

typedef struct {
    bool        has_trp1;
    bool        lock_slot;
    tx_hs_seq_t hs_seq;                                               // TX 高边时序
    tx_ls_seq_t ls_seq;                                               // TX 低边时序
    uint8_t     hs_ch_id[TX_HS_MAX_CHIP_CNT][TX_HS_MAX_CHANNEL_CNT];  // TX 高边通道全局序号
    uint8_t     ls_ch_id[TX_LS_MAX_CHIP_CNT][TX_LS_MAX_CHANNEL_CNT];  // TX 低边通道全局序号
} tx_sequence_t;

typedef struct {
    uint8_t  tx_hw_mode;                                              // see tx_hw_mode_e
    uint8_t  tx_hs_grp_pre_ch;                                        // TX High side 1 channel connected with N group
    bool     has_trp1;                                                // TX High side has trp0 and trp1
    uint8_t  tx_hs_ic_cnt;                                            // TX High side ic chip count
    int16_t  tx_hs_devid[TX_HS_MAX_CHIP_CNT];                         // TX High side device ID
    uint8_t  tx_ls_ic_cnt;                                            // TX Low  side ic chip count
    int16_t  tx_ls_devid[TX_LS_MAX_CHIP_CNT];                         // TX Low  side device ID
    uint8_t  tx_hs_ch_map[TX_HS_MAX_CHIP_CNT][TX_HS_MAX_CHANNEL_CNT]; // TX High side channel map: channel_id(idx) -> group_id(val), -1: not connected
    uint8_t  tx_ls_ch_map[TX_LS_MAX_CHIP_CNT][TX_LS_MAX_CHANNEL_CNT]; // TX Low  side channel map: channel_id(idx) -> piece_id(val), -1: not connected
} trx_hw_cfg_t;

// 多阶段寄存器配置容器
typedef struct {
    uint16_t lidar_mode;                                              // 雷达工作模式(rx_lidar_mode_e)
    uint32_t stage_count;                                             // 实际使用的阶段数
    rx_reg_list_t stage_regs[RX_STAGE_MAX];                           // 各阶段的寄存器列表
} rx_staged_reg_list_t;

//---------------------------------- parse cfg list -----------------------------------

// 数据解析配置
typedef struct {
    bool     is_b_frame;
    uint8_t  vc;
    uint32_t width;
    uint32_t height;
    uint32_t mipi_frame_cnt;
    uint16_t loop_num;
    uint8_t  slot_cnt_per_group;
    uint8_t  channel_cnt_per_slot;
    uint8_t  pixel_cnt_per_pkg;
    uint8_t  half_piece_per_ch;
    uint8_t  piece_cnt_per_block;
    uint8_t  piece_cnt_per_slot;
    uint8_t  start_group;
    uint8_t  piece_roll_ctrl;

    struct {
        bool en_ev_pkt;
        bool en_laser_ev;
    } ev_pkt_cfg;

    struct {
        bool     en_pkt_info;
        bool     en_ev_info;
        uint16_t pkt_ascii;
    } pkt_info_cfg;

    uint8_t          group_cnt;
    struct {
        uint8_t group_id;
        uint32_t ROI_sel;
        uint32_t row_offset;
    } group_cfg[RX_MAX_GROUP_CNT];

    struct {
        uint16_t        r_amb;
        uint32_t        o_amb;
        uint8_t         env_det_time;
    } noise_cfg;

    struct {
        bool en_pixel_data;
        bool is_range_data;

        union {
            struct {
                bool               en_pixel_info;
                bool               en_peak;
                bool               en_peak_sum;
                bool               en_echo_bin;
                bool               en_hist_main;
                bool               en_hist_sub;
                uint8_t            peak_cnt;
                uint32_t           peak_type[RX_MAX_PEAK_CNT];
            } hist_cfg;
            struct {
                bool     en_echo_bin;
                uint8_t  peak_cnt;

                bool     en_algo_pix_info;
                struct {
                    bool en_pixel_id;
                    bool en_noise_amb_h;
                    bool en_noise_amb_l;
                    bool en_amb_num_h;
                    bool en_amb_num_l;
                    bool en_det_num_h;
                    bool en_det_num_l;
                    bool en_real_hp_tx_num;
                    bool en_real_lp_tx_num;
                } algo_pix_info_cfg;

                bool     en_algo_result;
                struct {
                    bool en_th_peak_addr;
                    bool en_fwhm_h;
                    bool en_fwhm_l;
                    bool en_dist_rp_h;
                    bool en_dist_rp_l;
                    bool en_conv_int_h;
                    bool en_conv_int_l;
                    bool en_dist_cent_h;
                    bool en_dist_cent_l;
                    bool en_hist_peak_max;
                    uint32_t peak_type;
                } algo_result_cfg[RX_MAX_ALGO_PEAK_CNT];
            } range_cfg;
        } data_type_cfg;

        uint8_t  ROI_cnt;
        struct {
            struct {
                uint8_t  bin_cnt_left ;
                uint8_t  bin_cnt_right;
            } echo_cfg[RX_MAX_PEAK_CNT];

            struct {
                uint16_t main_hist_cnt;
                uint16_t sub_hist_cnt;
            } hist_cfg;

            struct {
                uint8_t  pix_cnt_per_channel;
            } location_cfg;
            uint8_t binning_type;
            uint8_t bin_cnt_ns;
            uint8_t sub_frame_cnt;
        } ROI_cfg[RX_ROI_CFG_CNT];
    } pixel_cfg;

    struct {
        bool en_gray_data;
        uint32_t gray_type;
        uint16_t frame_gray_len;
        uint16_t mixed_gray_len[2];
    } gray_cfg;

    struct {
        uint16_t packet_info_len;

        struct {
            uint16_t one_pixel_data_len;
            uint8_t  one_pixel_dummy_len;
            uint16_t max_bin_cnt;
        } ROI_cfg[RX_ROI_CFG_CNT];
    } auto_cfg;
} rx_parse_frm_cfg_t;

typedef struct {
    uint16_t lidar_mode;
    bool is_ab_frame;
    rx_parse_frm_cfg_t frm_cfg[RX_MAX_FRAME_CFG];
} rx_parse_cfg_t;

#pragma pack(pop)

//================================== function ===================================

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief 检查 TX/RX 配置是否错误
 * @param hw_cfg TX/RX 硬件配置
 * @param rx_parse 数据解析配置
 * @return 0 表示配置有效，非 0 表示配置无效
 */
int32_t rx_cfg_check_validity(const trx_hw_cfg_t *hw_cfg, const rx_parse_cfg_t *rx_parse);

/**
 * @brief 根据 hw_cfg 和 rx_parse 生成 TX 时序
 * @param hw_cfg TX/RX 硬件配置
 * @param rx_parse 数据解析配置
 * @param tx_seq 输出参数, 返回生成的 TX 时序
 * @return 0 表示成功，非 0 表示失败
 */
int32_t rx_cfg_gen_tx_seq(const trx_hw_cfg_t *hw_cfg, const rx_parse_cfg_t *rx_parse, tx_sequence_t *tx_seq);

/**
 * @brief 根据 reg_list 和 rx_parse 生成 TX/RX 时序
 * @param mode 时序模式
 * @param chns 通道列表
 * @param chn_num 通道数量
 * @param tx_seq 输出参数, 返回生成的 TX 时序
 * @return 0 表示成功，非 0 表示失败
 */
int32_t rx_cfg_gen_trx_seq(tx_cfg_seq_mode_e mode, const uint8_t *chns, uint16_t chn_num, tx_sequence_t *tx_seq);

/**
 * @brief 获取指定硬件模式的 TX/RX 硬件配置
 * @param hw_mode 硬件模式(tx_hw_mode_e)
 * @param hw_cfg 输出参数, 返回对应硬件模式的 TX/RX 硬件配置
 * @return 0 表示成功，非 0 表示失败
 */
int32_t rx_cfg_get_hw_cfg(tx_hw_mode_e hw_mode, trx_hw_cfg_t *hw_cfg);

/**
 * @brief 获取指定雷达模式的寄存器配置列表
 * @param lidar_mode 雷达工作模式(rx_lidar_mode_e)
 * @param reg_list 输出参数, 返回对应雷达模式的寄存器配置列表
 * @param hw_cfg 输出参数, 返回对应雷达模式的 TX/RX 硬件配置
 * @return 0 表示成功，非 0 表示失败
 */
int32_t rx_cfg_get_reg_list(rx_lidar_mode_e lidar_mode, const rx_staged_reg_list_t **reg_list);

/**
 * @brief 获取指定雷达模式的数据解析配置
 * @param lidar_mode 雷达工作模式(rx_lidar_mode_e)
 * @param rx_parse 输出参数, 返回对应雷达模式的数据解析配置
 * @return 0 表示成功，非 0 表示失败
 */
int32_t rx_cfg_get_rx_parse(rx_lidar_mode_e lidar_mode, const rx_parse_cfg_t       **rx_parse);

/**
 * @brief 根据输入的原始寄存器配置列表，修补并生成适配当前硬件配置的寄存器配置列表
 * @param reg_list         输入: 原始寄存器配置列表
 * @param hw_cfg           输入: 原始 TX/RX 硬件配置
 * @param patched_reg_list 输出: 返回适配当前硬件配置的寄存器配置列表
 * @return 0 表示成功，非 0 表示失败
 */
int32_t rx_cfg_patching_reg_list(const rx_staged_reg_list_t *reg_list, const trx_hw_cfg_t *hw_cfg, rx_staged_reg_list_t *patched_reg_list);

/**
 * @brief 获取指定工作阶段的寄存器配置
 * @param cfg 多阶段寄存器配置容器
 * @param stage 工作阶段(rx_stage_e)
 * @param stage_cfg 输出参数, 返回对应工作阶段的寄存器配置列表
 * @param cfg_cnt 输出参数, 返回寄存器配置数量
 * @return 0 表示成功，非 0 表示失败
 */
int32_t rx_cfg_get_stage_cfg(const rx_staged_reg_list_t *cfg, rx_stage_e stage, const rx_reg_t **stage_cfg, uint16_t *cfg_cnt);

/**
 * @brief 初始化 RX 配置加载器
 * @param cfg_load_path 配置文件存储路径, SDK 将从该路径加载用户自定义的配置文件;
 *                      若传入 NULL, 则仅使用 SDK 内置的预设配置;
 * @return 0 表示成功，非 0 表示失败
 */
int32_t rx_cfg_loader_init(const char *cfg_load_path);

/**
 * @brief 清理 RX 配置加载器
 * @return void
 */
void rx_cfg_loader_cleanup(void);

#ifdef __cplusplus
}
#endif

#endif /* __RX_CFG_LOADER_H__ */
