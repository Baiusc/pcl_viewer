#pragma once

#include <array>
#include <cstddef>
#include <cstdint>
#include <limits>
#include <string>

namespace range6532 {

// RANGE_01 常量：由 preset_cfg_range_roi01_80.h、ARM 发送代码及
// parse_range_mode_bin.py 的逐字节实测共同确认。
constexpr std::size_t kRecordSize = 278;
constexpr std::size_t kEchoCount = 2;
constexpr std::size_t kEchoBinCount = 91;
constexpr std::size_t kSlotsPerGroup = 4;
constexpr std::size_t kPixelsPerSlot = 192;
constexpr std::size_t kRecordsPerGroup = kSlotsPerGroup * kPixelsPerSlot; // 768
constexpr std::size_t kGroupsPerFrame = 32;
constexpr std::size_t kRows = 128;
constexpr std::size_t kCols = 192;
constexpr std::uint8_t kRoi0 = 0;
constexpr std::uint8_t kRoi1 = 1;
constexpr std::uint16_t kRoi1FirstGroup = 4;
constexpr std::size_t kRoi1GroupsPerFrame = 12;
constexpr std::size_t kRoi1PixelsPerSlot = 48;
constexpr std::size_t kRoi1RecordsPerGroup = kSlotsPerGroup * kRoi1PixelsPerSlot;
constexpr std::size_t kRoi1SubFrames = 4;
constexpr std::size_t kRoi1Rows = 48;
constexpr std::uint16_t kRoi1RowStart = 16;
constexpr std::size_t kProtocolHeaderSize = 6;
constexpr std::size_t kPclEchoHeaderSize = 34;
constexpr std::size_t kUdpHeaderSize = kProtocolHeaderSize + kPclEchoHeaderSize; // 40
constexpr std::uint16_t kProtocolFixedHead = 0xa55a;
constexpr std::uint8_t kProtocolMajorVersion = 2;
constexpr std::uint8_t kDataTypeCicl = 0x2e;
constexpr std::uint8_t kCiclPayloadType = 0x05;
constexpr double kDistanceScale = 25600.0; // raw cm*256 -> metre
constexpr double kHorizontalStepDeg = 0.625;
constexpr double kVerticalStepDeg = 0.625;

struct PacketInfo {
    // pkt_info_unpack_t，占裸记录 0x00~0x11。
    std::uint16_t packetType = 0;
    std::uint16_t frameIndex = 0;
    std::uint16_t packetId = 0;
    std::uint16_t groupIndex = 0;
    std::uint16_t slotIndex = 0;
    std::uint32_t timestamp = 0;
    std::uint8_t abFrameSelect = 0;
    std::uint8_t subFrameIndex = 0;
    std::uint8_t pieceRollMode = 0;
    std::uint8_t tdcResolutionMode = 0;
};

struct Echo {
    // 19 B 算法结果块（p0/p1），对应裸记录 0x3A 或 0x4D。
    std::uint16_t thresholdPeakAddress = 0;
    std::uint8_t peakAux = 255;
    std::uint8_t peakFlag = 0;
    std::uint8_t reserved = 0;
    std::uint32_t distanceRisingRaw = 0;
    std::uint16_t fwhmRaw = 0;
    std::uint32_t convolutionIntensity = 0;
    std::uint32_t distanceCentroidRaw = 0;
};

struct PixelRecord {
    // 0x12 起的 cicl_pix_info_t 实测头字段。
    PacketInfo packet;
    std::uint16_t pixelId = 0;
    std::uint32_t noiseAmbient = 0;
    std::uint16_t ambientCount = 0;
    std::uint32_t detectionSum = 0;
    std::uint16_t realTxCount = 0;
    std::uint16_t lowPowerRealTxCount = 0;
    bool hasEchoBins = false;
    std::uint8_t peakCountRaw = 0;
    std::array<std::uint8_t, 4> echoSamplesLeft{};
    std::array<std::uint8_t, 4> echoSamplesRight{};
    std::uint8_t binningType = 0;
    std::array<std::uint8_t, 5> pixelDecodeRaw{};
    std::uint16_t rowRaw = 0;
    std::uint16_t colRaw = 0;
    std::uint16_t unknown0 = 0;
    std::uint16_t unknown1 = 0;
    std::array<Echo, kEchoCount> echoes{};              // p0、p1 两个算法块
    std::array<std::uint16_t, kEchoBinCount> echoBins{}; // 0x60 起的 91 个回波 bin
};

struct PixelCoordinate {
    std::uint16_t row = 0;
    std::uint16_t col = 0;
};

struct UdpPacketMetadata {
    // fs_lidar_pcl_echo_data_t 的网络头元数据；不属于 ARM 保存的 range* 文件。
    std::uint8_t protocolMinorVersion = 0;
    std::uint16_t dataSequence = 0;
    std::uint16_t frameCount = 0;
    std::uint16_t dotCount = 0;
    std::uint8_t frameFlag = 0;
    std::uint16_t width = 0;
    std::uint16_t height = 0;
    std::uint16_t maxBinCount = 0;
    std::uint8_t roiIndex = 0;
    std::uint16_t pixelLength = 0;
    std::uint64_t timestampBase = 0;
};

// 显式小端读取，而不是把网络字节强转为 packed C struct。这样可避免 ARM、
// MSVC、GCC 的对齐/pragma 差异，并让每一个偏移都可与 Python 脚本核对。
bool parsePixelRecord(const std::uint8_t* data,
                      std::size_t size,
                      PixelRecord& output,
                      std::string* error = nullptr);

// 自动定位 UDP 后裸像素数据时使用的轻量判别器，仅检查稳定字段，避免把随机
// 头字节误判为 278 B 像素记录。
bool looksLikePixelRecord(const std::uint8_t* data, std::size_t size);

// 解析并校验 fs_protocol_head_t + fs_lidar_pcl_echo_data_t。二者均为
// #pragma pack(1)，所以 pixel payload 从第 40 字节开始。
bool parseArmUdpHeader(const std::uint8_t* data,
                       std::size_t size,
                       UdpPacketMetadata& output,
                       std::string* error = nullptr);

// 复现 cicl_decode_pixel_id_fast()，将 (group, slot, pixel_id) 解码为
// roi01_80 配置下的物理 (row, col)。
bool decodePixelCoordinate(std::uint16_t groupIndex,
                           std::uint16_t slotIndex,
                           std::uint16_t pixelId,
                           PixelCoordinate& output);

// ROI1 使用 4 个 piece-roll 子帧填满物理行 16..63 的 48×192 聚焦子区。
bool decodePixelCoordinate(std::uint8_t roiIndex,
                           std::uint16_t groupIndex,
                           std::uint16_t slotIndex,
                           std::uint16_t pixelId,
                           std::uint8_t subFrameIndex,
                           PixelCoordinate& output);

double horizontalAngleDeg(std::uint16_t col);
double verticalAngleDeg(std::uint16_t row);

} // namespace range6532

