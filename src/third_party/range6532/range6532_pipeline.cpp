#include "range6532_pipeline.h"

#include <algorithm>
#include <cmath>
#include <fstream>
#include <iomanip>
#include <sstream>
#include <utility>

namespace range6532 {
namespace {

constexpr std::uint32_t kInvalidDistanceMax = 0x00ffffffU;
constexpr std::uint8_t kInvalidAux = 255;

struct SelectedEcho {
    int index = -1;
    const Echo* echo = nullptr;
};

bool echoIsValid(const Echo& echo) {
    return echo.peakAux != kInvalidAux && echo.distanceRisingRaw > 0 &&
           echo.distanceRisingRaw < kInvalidDistanceMax;
}

SelectedEcho selectEcho(const PixelRecord& pixel, EchoPolicy policy) {
    const bool valid0 = echoIsValid(pixel.echoes[0]);
    const bool valid1 = echoIsValid(pixel.echoes[1]);
    if (valid0 && valid1) {
        if (policy == EchoPolicy::MaximumAux &&
            pixel.echoes[1].peakAux > pixel.echoes[0].peakAux) {
            return {1, &pixel.echoes[1]};
        }
        return {0, &pixel.echoes[0]};
    }
    if (valid0) {
        return {0, &pixel.echoes[0]};
    }
    if (valid1) {
        return {1, &pixel.echoes[1]};
    }
    return {};
}

void setError(std::string* error, const std::string& value) {
    if (error != nullptr) {
        *error = value;
    }
}

} // namespace

RangePipeline::RangePipeline(EchoPolicy echoPolicy, bool enableRoi1)
    : echoPolicy_(echoPolicy), enableRoi1_(enableRoi1) {}

void RangePipeline::setGroupCallback(GroupCallback callback) {
    groupCallback_ = std::move(callback);
}

void RangePipeline::setFrameCallback(FrameCallback callback) {
    frameCallback_ = std::move(callback);
}

bool RangePipeline::ingestRawPayload(const std::uint8_t* data,
                                     std::size_t size,
                                     std::string* error) {
    return ingestRawPayload(data, size, kRoi0, error);
}

bool RangePipeline::ingestRawPayload(const std::uint8_t* data,
                                     std::size_t size,
                                     std::uint8_t roiIndex,
                                     std::string* error) {
    if (data == nullptr || size == 0 || size % kRecordSize != 0) {
        setError(error, "raw payload size is not a positive multiple of 278 bytes");
        return false;
    }
    if (roiIndex != kRoi0 && roiIndex != kRoi1) {
        setError(error, "raw payload ROI must be 0 or 1");
        return false;
    }
    if (roiIndex == kRoi1 && !enableRoi1_) {
        return true; // 默认不解析 ROI1，但将其视为正常、已忽略的数据。
    }

    bool allOk = true;
    // 一个文件可含多个或不完整 group；真正的组装依据每条记录中的索引。
    for (std::size_t offset = 0; offset < size; offset += kRecordSize) {
        if (!ingestRecord(data + offset, roiIndex, error)) {
            ++stats_.rejectedRecords;
            allOk = false;
        }
    }
    evictStaleAssemblies();
    return allOk;
}

bool RangePipeline::ingestUdpDatagram(const std::uint8_t* data,
                                      std::size_t size,
                                      std::optional<std::size_t> headerBytes,
                                      std::string* error) {
    ++stats_.udpDatagrams;
    if (data == nullptr || size < kRecordSize) {
        ++stats_.rejectedDatagrams;
        setError(error, "UDP datagram is too short to contain a RANGE record");
        return false;
    }

    std::optional<std::size_t> offset = headerBytes;
    std::uint8_t roiIndex = kRoi0;
    UdpPacketMetadata metadata;
    std::string headerError;
    const bool hasArmHeader = parseArmUdpHeader(data, size, metadata, &headerError);
    if (!offset.has_value()) {
        // 首选路径：按 SDK 中 ARM 实际发送的两个 packed 头严格解析。特征扫描只
        // 用于兼容“已经剥掉一段或两段头”的旧抓包，不能替代正式协议校验。
        if (hasArmHeader) {
            offset = kUdpHeaderSize;
            roiIndex = metadata.roiIndex;
        } else {
            offset = detectPayloadOffset(data, size);
        }
    } else if (hasArmHeader) {
        // --header-bytes N 只控制 payload 起点；标准 40 B ARM 头仍用于区分 ROI。
        roiIndex = metadata.roiIndex;
    }
    if (!offset.has_value() || *offset > size ||
        (size - *offset) == 0 || (size - *offset) % kRecordSize != 0) {
        ++stats_.rejectedDatagrams;
        setError(error, "cannot locate an integral 278-byte RANGE payload in UDP datagram");
        return false;
    }
    if (roiIndex == kRoi1 && !enableRoi1_) {
        ++stats_.ignoredRoi1Datagrams;
        return true;
    }
    return ingestRawPayload(data + *offset, size - *offset, roiIndex, error);
}

std::optional<std::size_t> RangePipeline::detectPayloadOffset(const std::uint8_t* data,
                                                               std::size_t size) const {
    // ARM 正常发送的数据为 6 B fs_protocol_head_t + 34 B echo 头 + N*278 B。
    // 本函数不是正式解析路径，而是旧抓包兼容兜底：尝试有限前缀，利用 278 的
    // 整除关系和像素稳定字段寻找 payload 起点。
    const std::size_t maxHeader = std::min<std::size_t>(size - kRecordSize, 512);
    for (std::size_t offset = 0; offset <= maxHeader; ++offset) {
        const std::size_t payloadSize = size - offset;
        if (payloadSize == 0 || payloadSize % kRecordSize != 0) {
            continue;
        }
        const std::size_t recordCount = payloadSize / kRecordSize;
        if (!looksLikePixelRecord(data + offset, kRecordSize)) {
            continue;
        }

        // 同一个 ARM UDP 包中的记录来自同一 group；逐条验证 frame/group 一致，
        // 将随机字节恰好满足取模条件的误判概率降到很低。
        const std::uint16_t frame0 = static_cast<std::uint16_t>(data[offset + 2]) |
                                     (static_cast<std::uint16_t>(data[offset + 3]) << 8U);
        const std::uint16_t group0 = static_cast<std::uint16_t>(data[offset + 6]) |
                                     (static_cast<std::uint16_t>(data[offset + 7]) << 8U);
        bool consistent = true;
        for (std::size_t i = 1; i < recordCount; ++i) {
            const std::uint8_t* p = data + offset + i * kRecordSize;
            if (!looksLikePixelRecord(p, kRecordSize)) {
                consistent = false;
                break;
            }
            const std::uint16_t frame = static_cast<std::uint16_t>(p[2]) |
                                        (static_cast<std::uint16_t>(p[3]) << 8U);
            const std::uint16_t group = static_cast<std::uint16_t>(p[6]) |
                                        (static_cast<std::uint16_t>(p[7]) << 8U);
            if (frame != frame0 || group != group0) {
                consistent = false;
                break;
            }
        }
        if (consistent) {
            return offset;
        }
    }
    return std::nullopt;
}

bool RangePipeline::ingestRecord(const std::uint8_t* data, std::uint8_t roiIndex,
                                 std::string* error) {
    PixelRecord record;
    if (!parsePixelRecord(data, kRecordSize, record, error)) {
        return false;
    }
    ++stats_.parsedRecords;
    ++touchCounter_;

    const std::size_t pixelsPerSlot = roiIndex == kRoi1 ? kRoi1PixelsPerSlot : kPixelsPerSlot;
    const std::size_t recordsPerGroup = kSlotsPerGroup * pixelsPerSlot;
    if ((roiIndex == kRoi1 &&
         (record.packet.groupIndex < kRoi1FirstGroup ||
          record.packet.groupIndex >= kRoi1FirstGroup + kRoi1GroupsPerFrame ||
          record.packet.subFrameIndex >= kRoi1SubFrames || record.pixelId >= pixelsPerSlot)) ||
        (roiIndex == kRoi0 && record.pixelId >= pixelsPerSlot)) {
        setError(error, "record index is outside configured ROI layout");
        return false;
    }

    const std::uint8_t subFrame = roiIndex == kRoi1 ? record.packet.subFrameIndex : 0U;
    const GroupKey key{roiIndex, record.packet.frameIndex, record.packet.groupIndex, subFrame};
    PartialGroup& partial = partialGroups_[key];
    partial.lastTouch = touchCounter_;
    // 一个 group 的 4 slot × 192 pixel 以固定下标缓存，天然支持乱序和去重。
    const std::size_t index = static_cast<std::size_t>(record.packet.slotIndex) *
                                  pixelsPerSlot +
                              record.pixelId;
    if (partial.records[index].has_value()) {
        ++stats_.duplicateRecords;
        partial.records[index] = std::move(record); // 重传覆盖旧值，但不重复计数。
        return true;
    }

    partial.records[index] = std::move(record);
    ++partial.count;
    if (partial.count == recordsPerGroup) {
        PartialGroup completed = std::move(partial);
        partialGroups_.erase(key);
        finishGroup(key, std::move(completed));
    }
    return true;
}

void RangePipeline::finishGroup(const GroupKey& key, PartialGroup&& partial) {
    RangeGroup group;
    group.roiIndex = key.roi;
    group.frameIndex = key.frame;
    group.groupIndex = key.group;
    group.subFrameIndex = key.subFrame;
    group.recordCount = key.roi == kRoi1 ? kRoi1RecordsPerGroup : kRecordsPerGroup;
    for (std::size_t i = 0; i < group.recordCount; ++i) {
        group.pixels[i] = std::move(*partial.records[i]);
    }
    group.timestamp = group.pixels.front().packet.timestamp;
    ++stats_.completedGroups;

    if (groupCallback_) {
        groupCallback_(group);
    }
    addGroupToFrame(std::move(group));
}

void RangePipeline::addGroupToFrame(RangeGroup&& group) {
    ++touchCounter_;
    // RANGE_01 的 ROI1 将 sub_frame 0..3 分别放入连续四个 packet frame_index：
    // (F,0)、(F+1,1)、(F+2,2)、(F+3,3) 才是一幅完整 ROI1。因此用 F-sub_frame
    // 作为逻辑帧键；ROI0 仍直接使用 packet frame_index。
    const std::uint16_t logicalFrameIndex = group.roiIndex == kRoi1
        ? static_cast<std::uint16_t>(group.frameIndex - group.subFrameIndex)
        : group.frameIndex;
    const FrameKey key{group.roiIndex, logicalFrameIndex};
    PartialFrame& frame = partialFrames_[key];
    frame.lastTouch = touchCounter_;
    const std::size_t index = group.roiIndex == kRoi1
        ? static_cast<std::size_t>(group.subFrameIndex) * kRoi1GroupsPerFrame +
              (group.groupIndex - kRoi1FirstGroup)
        : group.groupIndex;
    if (!frame.groups[index].has_value()) {
        ++frame.count;
    }
    frame.groups[index] = std::move(group);
    const std::size_t expectedGroups = group.roiIndex == kRoi1
        ? kRoi1GroupsPerFrame * kRoi1SubFrames : kGroupsPerFrame;
    if (frame.count != expectedGroups) {
        return;
    }

    // ROI0 收齐 32 group；ROI1 收齐 12 group × 4 subframe 后各自产生一帧。
    const std::uint16_t frameIndex = logicalFrameIndex;
    const std::uint8_t roiIndex = group.roiIndex;
    // PartialFrame 含有大量原始 PixelRecord，不能复制到栈上；直接从 map 中的缓存重建。
    PointCloudFrame cloud = buildPointCloud(frame, roiIndex, frameIndex);
    partialFrames_.erase(key);
    ++stats_.completedFrames;
    if (frameCallback_) {
        frameCallback_(std::move(cloud));
    }
}

PointCloudFrame RangePipeline::buildPointCloud(const PartialFrame& frame, std::uint8_t roiIndex,
                                                std::uint16_t frameIndex) {
    PointCloudFrame cloud;
    cloud.roiIndex = roiIndex;
    cloud.frameIndex = frameIndex;
    cloud.serialNumber = ++frameSerialByRoi_[roiIndex];
    cloud.rowStart = roiIndex == kRoi1 ? kRoi1RowStart : 0;
    cloud.rows = roiIndex == kRoi1 ? kRoi1Rows : kRows;
    cloud.cols = kCols;
    cloud.points.resize(cloud.rows * cloud.cols);

    for (std::size_t row = 0; row < cloud.rows; ++row) {
        for (std::size_t col = 0; col < cloud.cols; ++col) {
            PointXYZIRange& point = cloud.points[row * cloud.cols + col];
            point.row = static_cast<std::uint16_t>(cloud.rowStart + row);
            point.col = static_cast<std::uint16_t>(col);
            point.horizontalDeg = static_cast<float>(horizontalAngleDeg(point.col));
            point.verticalDeg = static_cast<float>(verticalAngleDeg(point.row));
        }
    }

    constexpr double kPi = 3.14159265358979323846;
    // 逐 group/slot/pixel 映射至物理 row/col，而非沿原始记录顺序直接排列。
    const std::size_t groupCount = roiIndex == kRoi1
        ? kRoi1GroupsPerFrame * kRoi1SubFrames : kGroupsPerFrame;
    for (std::size_t groupIndex = 0; groupIndex < groupCount; ++groupIndex) {
        const RangeGroup& group = *frame.groups[groupIndex];
        for (std::size_t pixelIndex = 0; pixelIndex < group.recordCount; ++pixelIndex) {
            const PixelRecord& pixel = group.pixels[pixelIndex];
            PixelCoordinate coordinate;
            if (!decodePixelCoordinate(roiIndex, pixel.packet.groupIndex,
                                       pixel.packet.slotIndex,
                                       pixel.pixelId,
                                       group.subFrameIndex,
                                       coordinate)) {
                continue;
            }
            const std::size_t localRow = coordinate.row - cloud.rowStart;
            PointXYZIRange& point = cloud.points[localRow * cloud.cols + coordinate.col];

            // 时间戳属于单个 PixelRecord，不能用 RangeGroup 的首记录时间戳代替。
            point.timestamp = pixel.packet.timestamp;
            // 无论该像素最终是否有有效回波，都完整保存固件的双峰结果和
            // 91 个窗口化回波 bin。这样 CSV 才能与 Python 的 Frame CSV 等价，
            // 后续可重新选择 p0/p1 或离线分析，而无需再次接收原始 UDP 数据。
            point.echoes = pixel.echoes;
            point.echoBins = pixel.echoBins;

            const SelectedEcho selected = selectEcho(pixel, echoPolicy_);
            if (selected.echo == nullptr) {
                continue;
            }

            const double distance = selected.echo->distanceRisingRaw / kDistanceScale;
            const double horizontal = horizontalAngleDeg(coordinate.col) * kPi / 180.0;
            const double vertical = verticalAngleDeg(coordinate.row) * kPi / 180.0;
            const double horizontalRadius = distance * std::cos(vertical);

            // 坐标系约定：X 前、Y 左、Z 上。距离使用 dist_rp（与 Python 一致）；
            // 若产品标定要求使用 dist_cent，应在此处统一替换并同步标注文档。
            point.x = static_cast<float>(horizontalRadius * std::cos(horizontal));
            point.y = static_cast<float>(horizontalRadius * std::sin(horizontal));
            point.z = static_cast<float>(distance * std::sin(vertical));
            point.distanceMetres = static_cast<float>(distance);
            point.intensity = static_cast<float>(selected.echo->peakAux) / 255.0F;
            point.selectedEcho = static_cast<std::int8_t>(selected.index);
            point.valid = true;
            ++cloud.validPointCount;
        }
    }
    return cloud;
}

void RangePipeline::evictStaleAssemblies() {
    // UDP 丢包不能让不完整 group/frame 无限积累。128 个 group、4 个 frame
    // 的缓存可覆盖正常乱序；超过上限优先丢弃最久未更新项。
    while (partialGroups_.size() > 128) {
        auto oldest = std::min_element(partialGroups_.begin(), partialGroups_.end(),
            [](const auto& a, const auto& b) { return a.second.lastTouch < b.second.lastTouch; });
        partialGroups_.erase(oldest);
        ++stats_.discardedGroups;
    }
    while (partialFrames_.size() > 4) {
        auto oldest = std::min_element(partialFrames_.begin(), partialFrames_.end(),
            [](const auto& a, const auto& b) { return a.second.lastTouch < b.second.lastTouch; });
        partialFrames_.erase(oldest);
        ++stats_.discardedFrames;
    }
}

void RangePipeline::flushIncomplete() {
    stats_.discardedGroups += partialGroups_.size();
    stats_.discardedFrames += partialFrames_.size();
    partialGroups_.clear();
    partialFrames_.clear();
}

bool writeFrameCsv(const PointCloudFrame& frame,
                   const std::string& path,
                   std::string* error) {
    std::ofstream output(path);
    if (!output) {
        setError(error, "cannot open CSV output: " + path);
        return false;
    }
    // 列名、列顺序、每像素一行的双峰展开方式，严格对齐
    // data/固态雷达range模式data/build_pointcloud.py::frame_csv_fields()。
    // 请勿只输出 XYZ：软件工程师/算法工程师需要 p0/p1 原始算法块和
    // echo_bins 才能复核回波选择、量程切换及距离标定。
    output << "frame,roi,时间戳,行row,列col,水平角deg,俯仰角deg,距离m,峰高,反射率,有效,最优回波,"
              "p0_峰地址,p0_辅助值,p0_量程,p0_dist_rp_raw,p0_dist_rp_m,"
              "p0_fwhm_raw,p0_fwhm_bin,p0_conv_intensity,p0_dist_cent_raw,p0_dist_cent_m,"
              "p1_峰地址,p1_辅助值,p1_量程,p1_dist_rp_raw,p1_dist_rp_m,"
              "p1_fwhm_raw,p1_fwhm_bin,p1_conv_intensity,p1_dist_cent_raw,p1_dist_cent_m,"
              "echo_bins\n";

    // 外层列、内层 ROI 局部行；PointData::row 保持传感器的物理全局行号。
    for (std::size_t col = 0; col < frame.cols; ++col) {
        for (std::size_t row = 0; row < frame.rows; ++row) {
            const PointXYZIRange& p = frame.points[row * frame.cols + col];
            const int peak = p.valid
                ? static_cast<int>(p.echoes[static_cast<std::size_t>(p.selectedEcho)].peakAux)
                : 0;
            const double reflect = static_cast<double>(peak) / 255.0;

            // Python 的 frame 是“本次离线重建输出序号”，不是传感器 packet 中的
            // frame_index；CSV 文件名与此处保持同一 serialNumber 语义。
            // 与 build_pointcloud.py/readFrameCsv 一致：第 3 列是该像素的原始时间戳。
            output << frame.serialNumber << ',' << static_cast<unsigned>(frame.roiIndex) << ','
                   << p.timestamp << ',' << p.row << ',' << p.col << ','
                   << std::fixed << std::setprecision(3) << p.horizontalDeg << ','
                   << p.verticalDeg << ','
                   << std::setprecision(4) << p.distanceMetres << ','
                   << peak << ',' << std::setprecision(4) << reflect << ','
                   << (p.valid ? 1 : 0) << ',' << static_cast<int>(p.selectedEcho);

            // p0/p1 的定点数列紧跟其原始列输出解析值，保留 Python 的 1 位小数。
            // 即：dist_rp_raw -> dist_rp_m；fwhm_raw -> fwhm_bin；
            //     dist_cent_raw -> dist_cent_m。
            for (const Echo& echo : p.echoes) {
                const double distRpMetres = static_cast<double>(echo.distanceRisingRaw) / kDistanceScale;
                const double fwhmBins = static_cast<double>(echo.fwhmRaw) / 256.0;
                const double distCentMetres = static_cast<double>(echo.distanceCentroidRaw) / kDistanceScale;
                output << ',' << echo.thresholdPeakAddress
                       << ',' << static_cast<unsigned>(echo.peakAux)
                       << ',' << static_cast<unsigned>(echo.peakFlag)
                       << ',' << echo.distanceRisingRaw
                       << ',' << std::setprecision(1) << distRpMetres
                       << ',' << echo.fwhmRaw
                       << ',' << std::setprecision(1) << fwhmBins
                       << ',' << echo.convolutionIntensity
                       << ',' << echo.distanceCentroidRaw
                       << ',' << std::setprecision(1) << distCentMetres;
            }

            // echo_bins 与两个峰共享，一像素仅输出一次；以分号分隔，和 Python 保持一致。
            output << ',';
            for (std::size_t i = 0; i < p.echoBins.size(); ++i) {
                if (i != 0) {
                    output << ';';
                }
                output << p.echoBins[i];
            }
            output << '\n';
        }
    }
    return true;
}

} // namespace range6532

