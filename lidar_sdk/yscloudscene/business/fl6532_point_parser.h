#ifndef YS_SCENE_FL6532_POINT_PARSER_H_
#define YS_SCENE_FL6532_POINT_PARSER_H_

#include <QObject>
#include <chrono>
#include <cstdint>
#include <map>
#include <memory>
#include <mutex>
#include <vector>

#include "ys_scene_def.h"

namespace ys_scene {

class PointcloudParser;

// mf150_fl6532 原始点云(UDP 点云解析)常量
namespace Fl6532RawParser {
    inline constexpr size_t udp_packet_size = 1462;        // UDP满包大小 (6B头+34B回波头+79点×18B)
    inline constexpr size_t packet_header_size = 40;       // 包头大小 (fs_protocol_head_t 6B + fs_lidar_pcl_echo_data_t 34B)
    inline constexpr size_t pixel_byte_size = 18;          // 单点数据大小 (fs_pcl_dist_xyz_info_t 8B + fs_pcl_xyz_info_t 10B)
    inline constexpr size_t pixels_per_packet = 79;        // 满包点数 ((1462-40)/18)
    inline constexpr size_t tail_packet_size = 166;        // 残差包大小 (40 + 7×18)
    inline constexpr size_t tail_pixels = 7;               // 残差包点数 (24576 - 311×79)
    inline constexpr size_t width = 192;                   // ROI列数
    inline constexpr size_t height = 128;                  // ROI行数
    inline constexpr size_t total_pixels = width * height; // 一个子帧完整点数 (24576)

    // 协议校验值
    inline constexpr uint16_t kProtocolHead = 0xA55A;      // 包头魔数
    inline constexpr uint8_t kProtocolVerMajor = 2;        // 协议主版本
    // 数据类型（fs_lidar_data_type_t）：0x21=DATA_TYPE_POINTCLOUD（SOC算法处理后的点云）
    // 其他: 0x23=RAWDATA 0x24=IMU 0x25=EV 0x2A=FACTORY_PCL 0x2E=CICL 0x2F=GL
    inline constexpr uint8_t kDataType = 0x21;
    inline constexpr uint8_t kProtocolVerMinor = 2;        // 协议次版本
    inline constexpr uint8_t kPointTypeDisXyz = 0x02;      // 点类型 DIS_XYZ

    // 帧标志
    inline constexpr uint8_t kFrameFlagStart = 0;          // 帧起始包
    inline constexpr uint8_t kFrameFlagNormal = 1;         // 帧普通包
    inline constexpr uint8_t kFrameFlagEnd = 2;            // 帧结束包

    // 帧重组超时
    inline constexpr auto kFrameTimeout = std::chrono::milliseconds(100);
}

#pragma pack(push, 1)
// 包结构（每包独立, pack(1)）
using Fl6532ProtocolHead = struct {
    uint16_t head;              // 0xA55A
    uint8_t protocol_ver_major; // 2
    uint8_t data_type;          // fs_lidar_data_type_t: 0x21=DATA_TYPE_POINTCLOUD
    uint8_t lidar_module;       // 雷达模块
    uint8_t reserved;           // 预留
};

using Fl6532EchoData = struct {
    uint8_t protocol_ver_minor; // 2
    uint16_t data_sequence;     // 帧内包序号（连续，可用于丢包检测）
    uint16_t frame_cnt;         // 帧号
    uint16_t dot_num;           // 本包点数
    uint8_t type;               // 0x02 DIS_XYZ
    uint8_t peak_type_2_1;      // 峰值类型
    uint8_t frame_flag;         // 0=START 1=NORMAL 2=END
    uint16_t width;             // ROI宽度
    uint16_t height;            // ROI高度
    uint16_t max_bin_cnt;       // 最大bin数
    uint8_t ab_frame_idx;       // 子帧序号（含距离单位换算系数）
    uint8_t max_sub_frame_cnt;  // 最大子帧数
    uint8_t roi_index;          // ROI索引
    uint16_t pixel_len;         // 单点字节数 (18)
    uint16_t data_offset;       // 点数据区偏移 (40)
    uint16_t data_len;          // 点数据区长度
    uint8_t peak_type_4_3;      // 峰值类型
    uint64_t timestamp_base;    // 基准时间戳 (us)
};

using Fl6532DistXyzInfo = struct {
    uint32_t timestamp_offset;  // 点时刻偏移 (us)
    uint16_t col;               // ROI内列坐标 [0,width)
    uint16_t row;               // ROI内行坐标 [0,height)
};

using Fl6532XyzInfo = struct {
    uint16_t distance;          // 距离原始值（无符号，同x/y/z单位）
    uint16_t x;                 // x原始值（有符号 int16 编码，解析时需符号扩展）
    uint16_t y;                 // y原始值（有符号 int16 编码，解析时需符号扩展）
    uint16_t z;                 // z原始值（有符号 int16 编码，解析时需符号扩展）
    uint8_t reflectivity;       // 反射率/强度
    uint8_t tag_label;          // 标签
};

using Fl6532Point = struct {
    Fl6532DistXyzInfo dist_xyz; // 8B
    Fl6532XyzInfo xyz;          // 10B
};

using Fl6532Packet = struct {
    Fl6532ProtocolHead head;    // 6B
    Fl6532EchoData echo;        // 34B
    Fl6532Point points[1];      // dot_num个点
};
#pragma pack(pop)

// 解析后的应用点（单位已换算）
struct Fl6532PointOut {
    float x{ 0.0f };            // 单位 m
    float y{ 0.0f };            // 单位 m
    float z{ 0.0f };            // 单位 m
    float distance_cm{ 0.0f };  // 距离换算值 (cm)，校验用
    uint8_t reflectivity{ 0 };
    uint8_t tag_label{ 0 };
    uint16_t row{ 0 };
    uint16_t col{ 0 };
    uint64_t timestamp_us{ 0 }; // 点时刻 = timestamp_base + timestamp_offset (us)
};

// 单包解析结果头
struct Fl6532PacketHeader {
    uint16_t frame_cnt{ 0 };
    uint16_t data_sequence{ 0 };
    uint16_t dot_num{ 0 };
    uint8_t frame_flag{ 0 };
    uint8_t ab_frame_idx{ 0 };
    uint16_t width{ Fl6532RawParser::width };
    uint16_t height{ Fl6532RawParser::height };
    uint64_t timestamp_base{ 0 };
    bool valid{ false };
};

class Fl6532PointParser : public QObject {
    Q_OBJECT
public:
    explicit Fl6532PointParser(QObject* parent = nullptr);
    ~Fl6532PointParser();

    // UDP 包入口（1462B/包，支持一次传入多个连续包）
    void ReceivePacket(const uint8_t* data, uint32_t length);

    void SetPointcloudParser(PointcloudParser* parser);

    // C 回调适配器：SDK 层 SetFl6532PacketCallback 注册入口
    static void ReceivePacketTrampoline(const uint32_t handle, void* src, uint32_t length, void* context);

    // ---- 静态解析工具（可独立测试/复用）----
    // 校验并解析包头，成功返回 true
    static bool ParsePacket(const uint8_t* data, uint32_t length, Fl6532PacketHeader& header);
    // 由 ab_frame_idx 计算距离单位换算系数 N
    static float CalcUnitScale(uint8_t ab_frame_idx);
    // 解析单包点数据（单位换算 + 时间戳）
    static std::vector<Fl6532PointOut> ParsePoints(const uint8_t* data, uint32_t length,
        const Fl6532PacketHeader& header);
    // 完整帧点集 → Cartesian 帧（单位 m，过滤无效点）
    static CloudFramePtr FrameToCartesian(uint16_t frame_cnt,
        const std::vector<Fl6532PointOut>& points);
    // 方式B：按 row*width+col 重建深度阵列（单位 m），无数据位置 NaN
    static std::vector<float> BuildDepthArray(const std::vector<Fl6532PointOut>& points,
        size_t width = Fl6532RawParser::width, size_t height = Fl6532RawParser::height);

private:
    struct FrameData;
    void HandleSinglePacket(const uint8_t* data, uint32_t length);
    void DispatchCompleteFrame(const std::shared_ptr<FrameData>& frame);
    void LogFrameDiagnostics(const std::vector<Fl6532PointOut>& points,
        size_t received, size_t rendered) const;

private:
    std::mutex mutex_;
    std::map<uint16_t, std::shared_ptr<FrameData>> frames_; // frame_cnt -> 帧缓存
    PointcloudParser* pointcloud_parser_{ nullptr };
    size_t frames_count_{ 0 };
};

} // namespace ys_scene

#endif // YS_SCENE_FL6532_POINT_PARSER_H_
