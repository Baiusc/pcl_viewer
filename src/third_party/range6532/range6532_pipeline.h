#pragma once

#include "range6532_protocol.h"

#include <array>
#include <cstddef>
#include <cstdint>
#include <functional>
#include <map>
#include <optional>
#include <string>
#include <tuple>
#include <vector>

namespace range6532 {

enum class EchoPolicy {
    // 与 build_pointcloud.py 当前实际执行逻辑一致：两个回波均有效时选择 echo0。
    PreferEcho0,
    // 便于实验的替代策略：选择 peak_aux 更大的有效回波。
    MaximumAux
};

struct RangeGroup {
    std::uint8_t roiIndex = kRoi0;
    std::uint16_t frameIndex = 0;
    std::uint16_t groupIndex = 0;
    std::uint8_t subFrameIndex = 0;
    std::size_t recordCount = kRecordsPerGroup;
    std::uint32_t timestamp = 0;
    // 前 recordCount 项有效；固定下标 = slot_index * 当前 ROI 每 slot 像素数 + pixel_id。
    std::array<PixelRecord, kRecordsPerGroup> pixels{};
};

struct PointXYZIRange {
    // 由当前 ROI group 拼成的物理点位。
    float x = std::numeric_limits<float>::quiet_NaN();
    float y = std::numeric_limits<float>::quiet_NaN();
    float z = std::numeric_limits<float>::quiet_NaN();
    float intensity = 0.0F;       // 当前为 selected peak_aux / 255 的显示强度
    float distanceMetres = 0.0F;  // 当前为选中峰 dist_rp / 25600
    float horizontalDeg = 0.0F;   // col 对应水平角
    float verticalDeg = 0.0F;     // row 对应俯仰角
    std::uint16_t row = 0;
    std::uint16_t col = 0;
    // 原始 278 B PixelRecord 的 packet timestamp；Frame CSV 第 3 列。
    std::uint32_t timestamp = 0;
    std::int8_t selectedEcho = -1; // 0=p0、1=p1、-1=没有有效峰
    bool valid = false;            // 由 aux/dist_rp 饱和值规则判定

    // 写 Frame*_ROI*.csv 时必须保留原始算法输出，而不能只保留 XYZ。
    // 这与 build_pointcloud.py 的 p0_*/p1_* 列一一对应。
    std::array<Echo, kEchoCount> echoes{};
    std::array<std::uint16_t, kEchoBinCount> echoBins{};
};

struct PointCloudFrame {
    std::uint8_t roiIndex = kRoi0;
    std::uint16_t frameIndex = 0;
    std::uint64_t serialNumber = 0;
    std::size_t validPointCount = 0;
    std::uint16_t rowStart = 0; // 点的 row 字段仍为物理全局行号。
    std::size_t rows = kRows;
    std::size_t cols = kCols;
    // 有组织点云，内存顺序为 ROI 局部 row-major。
    std::vector<PointXYZIRange> points;
};

struct PipelineStats {
    std::uint64_t udpDatagrams = 0;
    std::uint64_t rejectedDatagrams = 0;
    std::uint64_t ignoredRoi1Datagrams = 0;
    std::uint64_t parsedRecords = 0;
    std::uint64_t rejectedRecords = 0;
    std::uint64_t duplicateRecords = 0;
    std::uint64_t completedGroups = 0;
    std::uint64_t discardedGroups = 0;
    std::uint64_t completedFrames = 0;
    std::uint64_t discardedFrames = 0;
};

class RangePipeline {
public:
    using GroupCallback = std::function<void(const RangeGroup&)>;
    using FrameCallback = std::function<void(PointCloudFrame&&)>;

    explicit RangePipeline(EchoPolicy echoPolicy = EchoPolicy::PreferEcho0,
                           bool enableRoi1 = false);

    void setGroupCallback(GroupCallback callback);
    void setFrameCallback(FrameCallback callback);

    // 解析不带网络头的连续 278 B 记录。ARM 保存的 range* 文件，以及上层已经
    // 剥离 UDP 头后的数据，都应走此接口。
    bool ingestRawPayload(const std::uint8_t* data,
                          std::size_t size,
                          std::string* error = nullptr);

    // 用于无 UDP 头的 ROI1 range* 文件；roiIndex 必须为 0 或 1。
    bool ingestRawPayload(const std::uint8_t* data,
                          std::size_t size,
                          std::uint8_t roiIndex,
                          std::string* error = nullptr);

    // 解析一个完整 UDP datagram；headerBytes 为 nullopt 时优先严格解析 SDK 的
    // 固定 40 B 头，失败后才按像素特征扫描。指定数值时强制使用该头长度。
    bool ingestUdpDatagram(const std::uint8_t* data,
                           std::size_t size,
                           std::optional<std::size_t> headerBytes = std::nullopt,
                           std::string* error = nullptr);

    const PipelineStats& stats() const { return stats_; }
    void flushIncomplete();

private:
    struct GroupKey {
        std::uint8_t roi = kRoi0;
        std::uint16_t frame = 0;
        std::uint16_t group = 0;
        std::uint8_t subFrame = 0;
        bool operator<(const GroupKey& other) const {
            return std::tie(roi, frame, group, subFrame) <
                   std::tie(other.roi, other.frame, other.group, other.subFrame);
        }
    };

    struct PartialGroup {
        std::array<std::optional<PixelRecord>, kRecordsPerGroup> records;
        std::size_t count = 0;
        std::uint64_t lastTouch = 0;
    };

    struct PartialFrame {
        std::array<std::optional<RangeGroup>, kRoi1GroupsPerFrame * kRoi1SubFrames> groups;
        std::size_t count = 0;
        std::uint64_t lastTouch = 0;
    };

    struct FrameKey {
        std::uint8_t roi = kRoi0;
        std::uint16_t frame = 0;
        bool operator<(const FrameKey& other) const {
            return std::tie(roi, frame) < std::tie(other.roi, other.frame);
        }
    };

    bool ingestRecord(const std::uint8_t* data, std::uint8_t roiIndex, std::string* error);
    void finishGroup(const GroupKey& key, PartialGroup&& partial);
    void addGroupToFrame(RangeGroup&& group);
    PointCloudFrame buildPointCloud(const PartialFrame& frame, std::uint8_t roiIndex,
                                    std::uint16_t frameIndex);
    std::optional<std::size_t> detectPayloadOffset(const std::uint8_t* data,
                                                    std::size_t size) const;
    void evictStaleAssemblies();

    EchoPolicy echoPolicy_;
    bool enableRoi1_ = false;
    GroupCallback groupCallback_;
    FrameCallback frameCallback_;
    PipelineStats stats_;
    std::map<GroupKey, PartialGroup> partialGroups_;
    std::map<FrameKey, PartialFrame> partialFrames_;
    std::uint64_t touchCounter_ = 0;
    std::array<std::uint64_t, 2> frameSerialByRoi_{};
};

bool writeFrameCsv(const PointCloudFrame& frame,
                   const std::string& path,
                   std::string* error = nullptr);

} // namespace range6532

