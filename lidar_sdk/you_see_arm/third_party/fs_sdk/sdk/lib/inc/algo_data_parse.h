#ifndef __ALGO_DATA_PARSE_H__
#define __ALGO_DATA_PARSE_H__

#include <stdint.h>
#include "rx_cfg_loader.h"

#define RX_DATA_ADC_CHANNEL_CNT           32   // 芯片内置 ADC 通道数
#define RX_DATA_MIXED_GRAY_CNT             2   // 最大混合灰度图数量 MAX[env/hist/ext/(env+hist)/(env+ext)] = 2
#define RX_MAX_PEAK_CNT                    4   // 每个 pixel 最多包含的 peak 个数(hist 模式)
#define RX_MAX_ALGO_PEAK_CNT               2   // 每个 pixel 最多包含的 peak 个数(range 模式)

typedef enum {
    RX_PEAK_TYPE_UNDEF = 0,           // 未定义峰类型
    RX_PEAK_TYPE_1ST_STRONG,          // 最强峰(strongest)
    RX_PEAK_TYPE_2ND_STRONG,          // 次强峰(second strongest)
    RX_PEAK_TYPE_3RD_STRONG,          // 第 3 强峰(third strongest)
    RX_PEAK_TYPE_4TH_STRONG,          // 第 4 强峰(fourth strongest)
    RX_PEAK_TYPE_1ST_CLOSE,           // 最近峰(closest)
    RX_PEAK_TYPE_2ND_CLOSE,           // 次近峰(second closest)
    RX_PEAK_TYPE_3RD_CLOSE,           // 第 3 次近峰(third closest)
    RX_PEAK_TYPE_4TH_CLOSE,           // 第 4 次近峰(fourth closest)
    RX_PEAK_TYPE_1ST_FAR,             // 最远峰(farthest)
} rx_peak_type_e;

typedef enum {
    RX_BINNING_TYPE_3X3  =   0,    // 3x3 binning
    RX_BINNING_TYPE_2X6  =   1,    // 2x6 binning
    RX_BINNING_TYPE_2X2  =   2,    // 2x2 binning
    RX_BINNING_TYPE_6X6  =   3,    // 6x6 binning
    RX_BINNING_TYPE_AUTO = 0xF,    // 保留字
} rx_binning_e;

typedef enum {
    RX_ROI_CFG_1 = 0,
    RX_ROI_CFG_2,
    RX_ROI_CFG_3,
    RX_ROI_CFG_4,
} rx_ROI_cfg_e;

typedef enum {
    GRAY_TYPE_UNDEF = 0,
    GRAY_TYPE_FRAME,   // 纯灰度图帧
    GRAY_TYPE_MIXED,   // 混合灰度图
} gray_type_t;

typedef enum {
    GRAY_MIXED_TYPE_UNDEF = 0,
    GRAY_MIXED_TYPE_ENV,       // 仅环境光灰度图
    GRAY_MIXED_TYPE_HIST,      // 仅 hist(TX) 灰度图
    GRAY_MIXED_TYPE_EXT,       // 仅 ext(外部光源) 灰度图
} gray_mixed_type_t;

// --- 数据包信息结构体定义 ---

typedef struct {
    bool has_ev;                                 // 是否包含 EV 信息
    uint16_t adc_data[RX_DATA_ADC_CHANNEL_CNT];  // 10bit adc_data(温度、电压、电流信息等)
} ev_info_t;

typedef struct {
    uint16_t      packet_type;            // 12bit, 包头
    uint8_t       sub_frame_idx;          //  2bit, sub-frame 计数
    uint8_t       piece_roll_mode;        //  2bit, sub-frame 数量: 0 -> 1 个子帧; 1 -> 2 个子帧; 2 -> 4 个子帧;
    uint8_t       tdc_reso_mode;          //  2bit, tdc 分辨率模式: 0 -> 1ns; 1 -> 500ps; 2 -> 250ps;
    uint8_t       ab_frame_sel;           //  1bit, A/B 帧: 0 -> A 帧; 1 -> B 帧
    uint16_t      frame_index;            // 16bit, laser 帧索引(从 1 开始)
    uint32_t      timestamp;              // 25bit, 本 slot 首次 TX 相对于 frame_sync 拉高后所经历的时间(单位: 40 ns)
    uint16_t      packet_id;              // 16bit, MIPI 帧内的分包 ID(从 1 开始)
    uint8_t       slot_index;             //  4bit, slot  索引(从 0 开始)
    uint16_t      vir_group_index;        // 12bit, 虚拟 group 索引(从 0 开始)
    uint16_t      phy_group_index;        // 12bit, 物理 group 索引(从 0 开始)
    ev_info_t     ev_info;                // EV 信息
} pkt_info_t;

typedef struct {
    uint16_t pixel_id;           // 从 0 开始的 pixel ID(12bit), slot 内循环递增, 跨 slot 时重置为 0
    uint8_t  channel_id;         // pixel 所属 channel 的 ID(4bit, 0~11)
    uint8_t  piece_id;           // pixel 所属物理 piece 的 ID(6bit, 0~47)
    uint8_t  pixel_col_in_group; // pixel 在 group 内的列号(取决于binning, 3x3: 0~191、2x2/2x6: 0~287、6x6: 0~95)
    uint8_t  pixel_row_in_group; // pixel 在 group 内的行号(取决于binning, 3x3: 0~3、2x2: 0~5、2x6/6x6: 0~1)
    uint16_t ambient_num;        // 环境光总计数(16bit)
    uint32_t det_sum;            // 直方图总计数(22bit)
    uint16_t real_tx_num;        // 高功率真实 TX 次数(11bit)
    uint16_t lpwr_real_tx_num;   // 低功率真实 TX 次数(11bit)
} pixel_info_t;

typedef struct {
    bool               crosstalk;      // peak 是否为串扰峰: false -> 否; true -> 是
    uint16_t           peak;           // 寻峰结果 peak 地址(12bit)
    rx_peak_type_e     peak_type;      // 峰类型
} peak_info_t;

typedef struct {
    uint16_t    hist_main_cnt;  // 主直方图的 bin 个数
    uint16_t    hist_sub_cnt;   // 子直方图的 bin 个数
    uint16_t   *main_hist;      // 主直方图数据指针
    uint16_t   *sub_hist;       // 子直方图数据指针
} hist_info_t;

typedef struct {
    uint8_t     bin_cnt_left ;  // echo_bin 峰左侧 bin 数量
    uint8_t     bin_cnt_right;  // echo_bin 峰右侧 bin 数量
    uint16_t   *echo_bin;       // 信号峰对应的 echo 直方图指针
} echo_info_t;

typedef struct {
    peak_info_t peak_info;      // 寻峰结果
    echo_info_t echo_info;      // echo 信息
    uint32_t    peak_sum;       // 信号峰累和值(24bit)
} peak_search_result_t;

typedef struct {
    peak_info_t peak_info;      // 寻峰结果
    echo_info_t echo_info;      // echo 信息
    uint16_t    fwhm;           // 信号峰峰宽(5bit+8bit, 整数部分单位 bin)
    uint32_t    dist_rp;        // 信号峰上升沿位置对应的距离(16bit+8bit, 整数部分单位 cm)
    uint32_t    conv_intensity; // 信号峰卷积峰高(18bit)
    uint32_t    dist_cent;      // 信号峰质心结果对应的距离(16bit+8bit, 整数部分单位 cm)
    uint16_t    hist_peak_max;  // 信号峰原始 bin 最大值(信号峰高, 12bit)
} range_result_t;

typedef struct {
    const pkt_info_t *pkt_info;         // 指向所属数据包的包信息结构体指针
    bool              is_range_data;    // 数据类型: false -> hist_data; true -> range_data;
    uint8_t           peak_cnt;         // 每个 pixel 包含的 peak 个数
    pixel_info_t      pixel_info;

    union {
        struct {
            peak_search_result_t peak_result[RX_MAX_PEAK_CNT];
            hist_info_t          hist_info;
        } hist_data;

        struct {
            range_result_t  range_result[RX_MAX_ALGO_PEAK_CNT];
            uint32_t        noise_amb; // 仅 algo_pixel_info 开启时有效: 寻峰算出的噪声阈值(调整系数后每个bin的平均噪声, 10bit+8bit)
        } range_data;
    } data;

    uint32_t        buf_len;
    uint16_t       *buffer;
} pixel_data_t;

typedef struct {
    gray_type_t     gray_type;              // 灰度图类型
    uint8_t         slot_index;             //  4bit, slot  索引(从 0 开始)
    uint16_t        group_index;            // 12bit, group 索引(从 0 开始)
    uint16_t        frame_index;            // 16bit, laser 帧索引(从 1 开始)

    union {
        struct {
            uint32_t           gray_len;    // 纯灰度图帧数据块最大长度(单位: 24bits)
            uint32_t           got_len;     // 已获取的纯灰度图帧长度(单位: 24bits)
            uint32_t          *gray_data;   // 纯灰度图帧数据指针(24bits)
        } frame;

        struct {
            gray_mixed_type_t  mixed_type;  // 混合灰度图类型
            uint32_t           gray_len;    // 混合灰度图数据块最大长度(单位: 12bits)
            uint32_t           got_len;     // 已获取的混合灰度图长度(单位: 12bits)
            uint16_t          *gray_data;   // 混合灰度图数据指针(12bits)
        } mixed[RX_DATA_MIXED_GRAY_CNT];
    } data;

    uint32_t        buf_len;
    uint16_t       *buffer;          // 灰度图缓冲区指针（动态分配）
} gray_data_t;   // 可存储 1 slot 的灰度图数据, 也可存储 1 个 group 甚至 1 帧, 具体取决于 gray_len 和 buf_len 的长度

// --- MIPI frame 解析上下文结构体 ---

typedef struct {
    // 状态追踪
    pkt_info_t last_pkt_info;               // 上一个包的包信息
    uint16_t   last_pixel_id;               // 上一个 pixel_id 从 0 开始(12bit)

    // pixel 解析进度
    uint32_t total_pixel_in_slot;           // 当前 slot 内已解析的 pixel 总数
    bool     slot_pixel_complete;           // 当前 slot 的 pixel 是否完整解析完成

    // gray 解析进度(跨包累积状态)
    gray_data_t *gray_accumulator;          // 累积的 gray_data (跨包时保存状态)
    bool        gray_in_progress;           // 是否正在跨包解析 gray

    // 统计信息
    uint32_t total_packets;                 // 当前 slot 内的 packet 总数
    uint32_t total_pixels;                  // 当前 slot 内的 pixel 总数
    uint32_t total_gray_slots;              // 当前 slot 内的 gray_data 块数

    // 动态分配的缓冲区（1 个 slot 的数据量）
    pkt_info_t   *pkt_info_buffer;          // 包信息缓冲区
    pixel_data_t *pixel_data_buffer;        // 像素数据缓冲区
    gray_data_t  *gray_data_buffer;         // 灰度图数据缓冲区

    uint32_t max_pkt_cnt;                   // 包信息缓冲区容量
    uint32_t max_pixel_cnt;                 // 像素数据缓冲区容量
    uint32_t max_gray_cnt;                  // 灰度图数据缓冲区容量

    // 当前输出索引
    uint32_t curr_pkt_idx;                  // 当前包索引
    uint32_t curr_pixel_idx;                // 当前像素索引
    uint32_t curr_gray_idx;                 // 当前灰度图索引

    // 标志位
    bool is_initialized;                    // 是否已初始化过此上下文
    bool slot_complete;                     // 当前 slot 是否已完整接收
} rx_frame_parse_ctx_t;

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief 根据 spad_id 解码出 pixel 的物理行列坐标
 * @param frm_cfg         输入: 当前帧的解析配置
 * @param gray_data       输入: 当前解析的 gray_data
 * @param spad_id_in_slot 输入: 当前 slot 内的 spad_id（从 0 开始）
 * @param row             输出: 解码得到的 pixel 行坐标（从 0 开始）
 * @param col             输出: 解码得到的 pixel 列坐标（从 0 开始）
 * @return FS_SUCCESS          解码成功
 * @return FS_ERR_BAD_PARAMS   参数错误（如空指针、spad_id 超出范围等）
 * @return FS_ERR_BAD_PROTOCOL 数据格式错误（如 frm_cfg 中的 binning 信息不合法等）
 * @return 其他值              其他错误
 */
int32_t rx_parse_decode_gray_spad_id(const rx_parse_frm_cfg_t *frm_cfg, gray_data_t *gray_data,
                                     uint32_t spad_id_in_slot, uint16_t *row, uint16_t *col);

/**
 * @brief 根据 bining_type 获取 1 个 pixel 的 spad 数量(行/列方向)
 * @param bining_type 输入: binning 类型
 * @param row_spad    输出: pixel 行方向的 spad 数量
 * @param col_spad    输出: pixel 列方向的 spad 数量
 * @return FS_SUCCESS          获取成功
 * @return FS_ERR_BAD_PARAMS   参数错误
 * @return 其他值              其他错误
 */
int32_t rx_parse_get_pixel_binning(rx_binning_e bining_type, uint8_t *row_spad, uint8_t *col_spad);

/**
 * @brief 根据 binning 类型获取一个 piece 内的 pixel 数量(行/列方向)
 * @param bining_type 输入: binning 类型
 * @param row_num     输出: 一个 piece 内行方向的 pixel 数量
 * @param col_num     输出: 一个 piece 内列方向的 pixel 数量
 * @return FS_SUCCESS          获取成功
 * @return FS_ERR_BAD_PARAMS   参数错误
 * @return 其他值              其他错误
 */
int32_t rx_parse_get_row_col_num_in_piece(rx_binning_e bining_type, uint8_t *row_num, uint8_t *col_num);

/**
 * @brief 根据 pixel_data 中的 pixel_id 解码出 pixel 的物理行列坐标
 * @param frm_cfg    输入: 当前帧的解析配置
 * @param pixel_data 输入: 当前解析的 pixel_data（包含 pixel_id）
 * @param row        输出: 解码得到的 pixel 行坐标（从 0 开始）
 * @param col        输出: 解码得到的 pixel 列坐标（从 0 开始）
 * @return FS_SUCCESS          解码成功
 * @return FS_ERR_BAD_PARAMS   参数错误
 * @return FS_ERR_BAD_PROTOCOL 数据格式错误
 * @return 其他值              其他错误
 */
int32_t rx_parse_decode_pixel_id(const rx_parse_frm_cfg_t *frm_cfg, pixel_data_t *pixel_data, uint16_t *row, uint16_t *col);

/**
 * @brief 解析 1 个 MIPI 帧 (bin 文件)
 *
 * @details 本函数解析一个完整的 MIPI frame（也即 1 个 bin 文件）。
 *          内存由上下文自动管理，每次返回 1 个 slot 的数据。
 *          当检测到 slot 切换时，返回 FS_ERR_SLOT_CHANGE，调用方应取走数据后继续调用。
 *          当检测到 frame 切换时，返回 FS_ERR_FRAME_CHANGE，调用方应处理完当前 frame。
 *
 * @param ctx           输入/输出: 解析上下文（包含内存管理和状态追踪）
 * @param parse_cfg     输入: 解析配置
 * @param data          输入: MIPI 帧数据指针
 * @param dlen          输入: MIPI 帧数据长度(单位: raw12，即 uint16_t 个数)
 * @param max_pkt_len   输入: 每个长包的最大长度(单位: raw12，即 uint16_t 个数)
 * @param remin_pkt_cnt 输入: data 中剩余的长包数量
 * @param pkt_info      输出: 指向上下文中的 pkt_info 缓冲区指针
 * @param pkt_cnt       输出: 实际解析的 pkt_info 个数
 * @param pixel_data    输出: 指向上下文中的 pixel_data 缓冲区指针
 * @param pixel_cnt     输出: 实际解析的 pixel_data 个数
 * @param gray_data     输出: 指向上下文中的 gray_data 缓冲区指针
 * @param gray_cnt      输出: 实际解析的 gray_data 个数
 * @param next          输出: 指向下一个未解析数据的指针（用于继续解析或下一个 bin 文件）
 *
 * @return FS_SUCCESS          解析成功，bin 文件已处理完毕
 * @return FS_ERR_SLOT_CHANGE  检测到 slot 切换，调用方应取走数据后继续调用
 * @return FS_ERR_FRAME_CHANGE 检测到 frame 切换，调用方应处理完当前 frame
 * @return FS_ERR_BAD_PARAMS   参数错误（空指针、索引越界等）
 * @return FS_ERR_BAD_PROTOCOL 数据格式错误
 * @return 其他值              其他错误
 */
int32_t rx_parse_one_frame(rx_frame_parse_ctx_t *ctx,
                           const rx_parse_cfg_t *parse_cfg,
                           const uint16_t *data, uint32_t dlen,
                           uint32_t max_pkt_len, uint32_t remin_pkt_cnt,
                           pkt_info_t   **pkt_info  , uint32_t *pkt_cnt,
                           pixel_data_t **pixel_data, uint32_t *pixel_cnt,
                           gray_data_t  **gray_data , uint32_t *gray_cnt,
                           ev_info_t     *ev_info, const uint16_t **next);

/**
 * @brief 初始化 MIPI frame 解析上下文
 *
 * @details 在开始解析 frame 前调用，初始化解析状态机上下文。
 *          自动分配 1 个 slot 所需的内存（pkt_info、pixel_data、gray_data）。
 *          支持跨多个 bin 文件解析单个 laser frame 的场景。
 *
 * @param ctx        输出: 初始化后的上下文指针
 * @param parse_cfg  输入: 解析配置（用于计算缓冲区大小）
 *
 * @return FS_SUCCESS        初始化成功
 * @return FS_ERR_BAD_PARAMS 参数错误
 * @return FS_ERR_NOMEM      内存分配失败
 */
int32_t rx_frame_parse_ctx_init(rx_frame_parse_ctx_t *ctx, const rx_parse_cfg_t *parse_cfg);

/**
 * @brief 销毁 MIPI frame 解析上下文（释放内部资源）
 *
 * @param ctx        输入: 待销毁的上下文指针
 *
 * @return FS_SUCCESS        销毁成功
 * @return FS_ERR_BAD_PARAMS 参数错误
 */
int32_t rx_frame_parse_ctx_destroy(rx_frame_parse_ctx_t *ctx);

#ifdef __cplusplus
}
#endif

#endif /* __ALGO_DATA_PARSE_H__ */

