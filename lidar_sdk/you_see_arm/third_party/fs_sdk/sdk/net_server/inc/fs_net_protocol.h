#ifndef __FS_NET_PROTOCOL_H__
#define __FS_NET_PROTOCOL_H__

#include <stdint.h>
#include <string.h>
#include "fs_err.h"

#define FS_PROTOCOL_UDP_MTU_SIZE 1472 // 3582 // 1472 // Maximum Transmission Unit

#define PROTOCOL_VERSION_MAJOR           2
#define PROTOCOL_FIXED_HEAD         0xA55A
#define PROTOCOL_PCL_VER_MINOR      2

typedef enum {
    DATA_TYPE_POINTCLOUD     = 0x21, /* 经过SOC算法处理后的点云数据 */

    DATA_TYPE_RAWDATA        = 0x23, /* 芯片出来的 Rawdata，没有经过任何处理 */
    DATA_TYPE_IMU_DATA       = 0x24, /* IMU数据 */
    DATA_TYPE_EV             = 0x25, /* EV包， 环境数据 */

    DATA_TYPE_FACTORY_PCL    = 0x2A, /* 工厂测试模式下的点云数据*/

    DATA_TYPE_CICL_DATA      = 0x2E, /* 芯片出来的 RANGE 数据 */
    DATA_TYPE_GL_DATA        = 0x2F, /* 芯片出来的纯灰度图数据 */
} fs_lidar_data_type_t;

typedef enum {
    FS_PCL_DISTANCE = 0x00, /* 距离数据 */

    FS_PCL_XYZ      = 0x01, /* 空间坐标XYZ */
    FS_PCL_DIS_XYZ  = 0x02, /* 距离 + XYZ */

    FS_CICL_DATA    = 0x05, /* CICL 数据 */
    FS_GL_DATA      = 0x07, /* Gray Level 数据 */
} fs_pcl_data_info_type_t;

typedef enum {
    FS_PCL_PEAK_NONE            = 0x0, /* 无回波      */
    FS_PCL_PEAK_1ST_STRONG      = 0x1, /* 最强回波    */
    FS_PCL_PEAK_2ND_STRONG      = 0x2, /* 次强回波    */
    FS_PCL_PEAK_3RD_STRONG      = 0x3, /* 第 3 强回波 */
    FS_PCL_PEAK_4TH_STRONG      = 0x4, /* 第 4 强回波 */
    FS_PCL_PEAK_1ST_CLOSE       = 0x5, /* 最近回波    */
    FS_PCL_PEAK_2ND_CLOSE       = 0x6, /* 次近回波    */
    FS_PCL_PEAK_3RD_CLOSE       = 0x7, /* 第 3 近回波 */
    FS_PCL_PEAK_4TH_CLOSE       = 0x8, /* 第 4 近回波 */
    FS_PCL_PEAK_1ST_FAR         = 0x9, /* 最远回波    */
    FS_PCL_PEAK_2ND_FAR         = 0xA, /* 次远回波    */
    FS_PCL_PEAK_3RD_FAR         = 0xB, /* 第 3 远回波 */
    FS_PCL_PEAK_4TH_FAR         = 0xC, /* 第 4 远回波 */
    // FS_PCL_PEAK_UNUSED_0        = 0xD,
    // FS_PCL_PEAK_UNUSED_1        = 0xE,

    FS_PCL_PEAK_MAX             = 0xF, /* 最大回波数 */
} fs_pcl_peak_type_t;

typedef enum {
    FS_PCL_FRAME_FLAG_START     = 0x00,
    FS_PCL_FRAME_FLAG_NORMAL    = 0x01,
    FS_PCL_FRAME_FLAG_END       = 0x02,
} fs_pcl_frame_flags_e;

#pragma pack(push)
#pragma pack(1) // one byte-aligned

typedef struct fs_protocol_head {
    uint16_t head;                // 0xA55A
    uint8_t  protocol_ver_major;  // 协议主版本，当前为 2
    uint8_t  data_type;           // 数据类型，描述当前数据是哪种数据类型，如点云、echo、IMU、EV、Rawdata、控制指令等。
    uint8_t  lidar_module;        // [7:6] 0 - 补盲面阵lidar 1 - 面阵主lidar 2 - 线阵主lidar 3 - 保留,
                                  // [5:0] 0 - FL6031 1 - FL6032 .....
    uint8_t  reserved[1];         // Reserved
} fs_protocol_head_t;

typedef struct {
    fs_lidar_data_type_t data_type; // 数据类型
    uint8_t *data;                  // pixel数据
    uint16_t frm_id;                // Lidar帧ID
    uint8_t  ab_frame_idx;          // AB 帧标记，0 - A 帧， 1 - B 帧
    uint8_t  max_sub_frame_cnt;     // 最大子帧数(本帧所有 group 中)
    uint8_t  peak_num;              // 回波个数(默认: 点云 2 个, echo 4 个)
    uint16_t max_bin_cnt;           // 本帧最大 bin 长
    uint16_t group_cnt;             // 本帧的虚拟 group 总数
    uint8_t  group_row_unit;        // group 在 row 方向(Y)的分辨率，单位: spad 个数
    uint8_t  roi_cnt;               // 本帧的 ROI 区域个数
    uint64_t timestamp_base;        // 本帧基准时间戳，单位 us
    fs_pcl_peak_type_t peak_type[4];// 回波类型, 最多 4 个回波

    struct {
        uint16_t width;             // 本 ROI 每行的 pixel 数
        uint16_t height;            // 本 ROI 每列的 pixel 数(行数)
        uint16_t bin_cnt;           // 本 ROI 的 bin 长
        uint8_t  col_unit;          // col 方向(X)的 pixel 分辨率，单位: spad 个数
        uint8_t  row_unit;          // row 方向(Y)的 pixel 分辨率，单位: spad 个数
        uint8_t  group_start;       // 本 ROI 对应的第一个物理 group 索引值
        uint8_t  sub_frame_cnt;     // 子帧数
        uint8_t  sub_frame_idx;     // 子帧索引
        float    range_max;         // 本 ROI 的测距上限，单位 cm
        uint8_t  pcl_unit;          // 本 ROI 点云数值的单位, bit[6:5] 整数部分, bit[4:0] 小数部分，单位是 cm
    } ROI_info[4];

    struct {
        uint32_t pixel_count;       // pixel 数量
        uint16_t pixel_len;         // 一个 pixel 的数据长度
        uint16_t vir_group_idx;     // 虚拟 group 索引
        uint16_t phy_group_idx;     // 物理 group 索引
        uint8_t  roi_index;         // 当前数据所属的 ROI 区域索引值
    } group_info;
} fs_lidar_data_info_t;

typedef struct fs_lidar_pcl_echo_data {
    uint8_t  protocol_ver_minor;  // 协议子版本，当前为0
    uint16_t data_sequence;       // 数据序列，可用于统计接收数据是否丢包
    uint16_t frame_cnt;           // frame 序号
    uint16_t dot_num;             // 当前数据包的data字段包含的点的数量
    uint8_t  type;                // 点云数据类型，（Distance/XYZ/(Distance+XYZ) ）+ 反射率 + 标签，默认XYZ
    uint8_t  peak_type_2_1;       // 点云数据的回波类型: bit[3:0] 第 1 个回波类型, bit[7:4] 第 2 个回波类型
    uint8_t  frame_flag;          // 帧标记。分别是frame start = 0x00,normal frame = 0x01, frame end = 0x02.
    uint16_t width;               // 一个group最大的pixel数
    uint16_t height;              // 一个雷达帧的pixel行数
    uint16_t max_bin_cnt;         // 雷达帧内最大的bin长
    uint8_t  ab_frame_idx;        // bit0: AB 帧标记，0 - A 帧，1 - B 帧; bit[7:6]: 点云数据单位 - 整数部分, bit[5:1]: 点云数据单位 - 小数部分;
    uint8_t  max_sub_frame_cnt;   // bit[3:0]: 最大子帧数量(所有 group 中的最大值), 取值范围 1~8; bit[7:4]: Reserved;
    uint8_t  roi_index;           // 当前数据所属的 roi 区域索引值, 取值范围 0~3; bit[7:2]: Reserved;
    uint16_t pixel_len;           // 本包 1 个 pixel 的数据长度
    uint16_t data_offset;         // 本包数据的 offset(仅 hist 或者 echo + hist 模式有效, 应对直方图长度超过一个包能承载的长度的情况)
    uint16_t data_len;            // 本包数据的长度(仅 hist 或者 echo + hist 模式有效, 应对直方图长度超过一个包能承载的长度的情况)
    uint8_t  peak_type_4_3;       // 点云数据的回波类型: bit[3:0] 第 3 个回波类型, bit[7:4] 第 4 个回波类型
    uint64_t timestamp_base;      // 帧同步时刻的时间戳
    uint8_t  data[0];
} fs_lidar_pcl_echo_data_t;

#ifndef FS_LIDAR_IMU_TYPES_DEFINED
#define FS_LIDAR_IMU_TYPES_DEFINED
typedef struct {
    int64_t timestamp; // Unit: ns
    int16_t  temperature;   // Unit: 1/256 ℃
    float  gyro_x;  // Unit: rad/s
    float  gyro_y;  // Unit: rad/s
    float  gyro_z;  // Unit: rad/s
    float  acc_x;   // Unit: g
    float  acc_y;   // Unit: g
    float  acc_z;   // Unit: g
} fs_lidar_imu_data_t;

typedef struct {
    uint8_t  crc8_header;
    uint8_t  crc8_imu_dat;
    uint8_t  version;
    uint16_t len;
    uint8_t  time_type;
    fs_lidar_imu_data_t data[0];
} fs_lidar_imu_header_t;
#endif

#pragma pack(pop)

#ifdef __cplusplus
extern "C" {
#endif

static inline void get_head_pkg(fs_protocol_head_t *phead_pkg, uint32_t type)
{
    memset(phead_pkg, 0, sizeof(fs_protocol_head_t));
    phead_pkg->head = PROTOCOL_FIXED_HEAD;
    phead_pkg->protocol_ver_major = PROTOCOL_VERSION_MAJOR;
    phead_pkg->data_type = type;
    return;
}

static inline int check_head_pkg(fs_protocol_head_t *phead_pkg, uint32_t type)
{
    if (phead_pkg->head != PROTOCOL_FIXED_HEAD) {
        FS_LOGE("PROTOCOL_FIXED_HEAD != head_data->head");
        return FS_ERR_PROTOCOL_HEAD;
    }

    if (phead_pkg->protocol_ver_major > PROTOCOL_VERSION_MAJOR) {
        FS_LOGE("PROTOCOL_VERSION_MAJOR < head_data->protocol_ver_major");
        return FS_ERR_PROTOCOL_HEAD;
    }

    if (phead_pkg->data_type != type) {
        FS_LOGE("head_data->data_type != 0x%02X", type);
        return FS_ERR_PROTOCOL_HEAD;
    }

    return FS_SUCCESS;
}

/**
 * @brief 发送 1 个 group 的 pixel 数据
 * @param info pixel 数据和该 group 的相关信息
 * @return int32_t 错误码
 */
int fs_send_pixel_data_one_group(fs_lidar_data_info_t *info);

/**
 * @brief 发送 1 个 group 的灰度图数据
 * @param info 灰度图数据和该 group 的相关信息
 * @return int32_t 错误码
 */
int fs_send_gray_data_one_group(fs_lidar_data_info_t *info);

#ifdef __cplusplus
}
#endif

#endif
