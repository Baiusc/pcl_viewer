#include "range6532_protocol.h"

#include <sstream>

namespace range6532 {
namespace {

constexpr std::size_t kPacketInfoSize = 18;
constexpr std::size_t kAlgorithmBase = 0x3a;
constexpr std::size_t kAlgorithmStep = 19;
constexpr std::size_t kBinsOffset = 0x60;
constexpr std::uint16_t kPieceMap3x3[16] = {
    0, 4, 1, 5, 8, 12, 9, 13, 2, 6, 3, 7, 10, 14, 11, 15
};

std::uint16_t readU16(const std::uint8_t* p) {
    return static_cast<std::uint16_t>(p[0]) |
           (static_cast<std::uint16_t>(p[1]) << 8U);
}

std::uint32_t readU32(const std::uint8_t* p) {
    return static_cast<std::uint32_t>(p[0]) |
           (static_cast<std::uint32_t>(p[1]) << 8U) |
           (static_cast<std::uint32_t>(p[2]) << 16U) |
           (static_cast<std::uint32_t>(p[3]) << 24U);
}

std::uint64_t readU64(const std::uint8_t* p) {
    return static_cast<std::uint64_t>(readU32(p)) |
           (static_cast<std::uint64_t>(readU32(p + 4)) << 32U);
}

void setError(std::string* error, const std::string& value) {
    if (error != nullptr) {
        *error = value;
    }
}

} // namespace

bool parsePixelRecord(const std::uint8_t* data,
                      std::size_t size,
                      PixelRecord& out,
                      std::string* error) {
    if (data == nullptr || size < kRecordSize) {
        setError(error, "pixel record is shorter than 278 bytes");
        return false;
    }

    // 固定 278 B 裸像素记录。以下偏移与 parse_range_mode_bin.py 注释中的
    // 0x00~0x60 布局逐项一致，切勿按 SDK 旧的变长结构直接 reinterpret_cast。
    out.packet.packetType = readU16(data + 0x00);
    out.packet.frameIndex = readU16(data + 0x02);
    out.packet.packetId = readU16(data + 0x04);
    out.packet.groupIndex = readU16(data + 0x06);
    out.packet.slotIndex = readU16(data + 0x08);
    out.packet.timestamp = readU32(data + 0x0a);
    out.packet.abFrameSelect = data[0x0e];
    out.packet.subFrameIndex = data[0x0f];
    out.packet.pieceRollMode = data[0x10];
    out.packet.tdcResolutionMode = data[0x11];

    out.pixelId = readU16(data + kPacketInfoSize + 0x00);
    out.noiseAmbient = readU32(data + kPacketInfoSize + 0x02);
    out.ambientCount = readU16(data + kPacketInfoSize + 0x06);
    out.detectionSum = readU32(data + kPacketInfoSize + 0x08);
    out.realTxCount = readU16(data + kPacketInfoSize + 0x0c);
    out.lowPowerRealTxCount = readU16(data + kPacketInfoSize + 0x0e);
    out.hasEchoBins = data[kPacketInfoSize + 0x10] != 0;
    out.peakCountRaw = data[kPacketInfoSize + 0x11];
    for (std::size_t i = 0; i < 4; ++i) {
        out.echoSamplesLeft[i] = data[kPacketInfoSize + 0x12 + i];
        out.echoSamplesRight[i] = data[kPacketInfoSize + 0x16 + i];
    }
    out.binningType = data[kPacketInfoSize + 0x1a];
    for (std::size_t i = 0; i < 5; ++i) {
        out.pixelDecodeRaw[i] = data[kPacketInfoSize + 0x1b + i];
    }
    out.rowRaw = readU16(data + kPacketInfoSize + 0x20);
    out.colRaw = readU16(data + kPacketInfoSize + 0x22);
    out.unknown0 = readU16(data + kPacketInfoSize + 0x24);
    out.unknown1 = readU16(data + kPacketInfoSize + 0x26);

    // 固件实测仅有两个 19 B 算法块：0x3A(p0)、0x4D(p1)。0x60 起就是 91 个
    // echo bin；绝不能按“四峰”继续向后读取，否则会把直方图误解析成算法结果。
    for (std::size_t i = 0; i < kEchoCount; ++i) {
        const std::uint8_t* p = data + kAlgorithmBase + i * kAlgorithmStep;
        Echo& echo = out.echoes[i];
        echo.thresholdPeakAddress = readU16(p + 0);
        echo.peakAux = p[2];
        echo.peakFlag = p[3];
        echo.reserved = p[4];
        echo.distanceRisingRaw = readU32(p + 5);
        echo.fwhmRaw = readU16(p + 9);
        echo.convolutionIntensity = readU32(p + 11);
        echo.distanceCentroidRaw = readU32(p + 15);
    }

    for (std::size_t i = 0; i < kEchoBinCount; ++i) {
        out.echoBins[i] = readU16(data + kBinsOffset + i * 2);
    }

    if (out.packet.groupIndex >= kGroupsPerFrame) {
        std::ostringstream oss;
        oss << "group_index out of ROI0 range: " << out.packet.groupIndex;
        setError(error, oss.str());
        return false;
    }
    if (out.packet.slotIndex >= kSlotsPerGroup) {
        std::ostringstream oss;
        oss << "slot_index out of range: " << out.packet.slotIndex;
        setError(error, oss.str());
        return false;
    }
    if (out.pixelId >= kPixelsPerSlot) {
        std::ostringstream oss;
        oss << "pixel_id out of range: " << out.pixelId;
        setError(error, oss.str());
        return false;
    }
    if (out.peakCountRaw > 4) {
        setError(error, "peak count is implausible");
        return false;
    }
    return true;
}

bool looksLikePixelRecord(const std::uint8_t* data, std::size_t size) {
    if (data == nullptr || size < kRecordSize) {
        return false;
    }
    const std::uint16_t group = readU16(data + 0x06);
    const std::uint16_t slot = readU16(data + 0x08);
    const std::uint16_t pixel = readU16(data + 0x12);
    const std::uint8_t peakCount = data[0x23];
    return group < kGroupsPerFrame && slot < kSlotsPerGroup &&
           pixel < kPixelsPerSlot && peakCount <= 4;
}

bool parseArmUdpHeader(const std::uint8_t* data,
                       std::size_t size,
                       UdpPacketMetadata& out,
                       std::string* error) {
    if (data == nullptr || size < kUdpHeaderSize) {
        setError(error, "UDP datagram is shorter than the packed 40-byte headers");
        return false;
    }
    if (readU16(data) != kProtocolFixedHead) {
        setError(error, "invalid fs_protocol_head_t.head (expected 0xA55A)");
        return false;
    }
    if (data[2] > kProtocolMajorVersion) {
        setError(error, "unsupported protocol major version");
        return false;
    }
    if (data[3] != kDataTypeCicl) {
        setError(error, "UDP data_type is not DATA_TYPE_CICL_DATA (0x2E)");
        return false;
    }

    const std::uint8_t* echo = data + kProtocolHeaderSize;
    out.protocolMinorVersion = echo[0];
    out.dataSequence = readU16(echo + 1);
    out.frameCount = readU16(echo + 3);
    out.dotCount = readU16(echo + 5);
    const std::uint8_t payloadType = echo[7];
    out.frameFlag = echo[9];
    out.width = readU16(echo + 10);
    out.height = readU16(echo + 12);
    out.maxBinCount = readU16(echo + 14);
    out.roiIndex = echo[18];
    out.pixelLength = readU16(echo + 19);
    out.timestampBase = readU64(echo + 26);

    if (payloadType != kCiclPayloadType) {
        setError(error, "fs_lidar_pcl_echo_data_t.type is not FS_CICL_DATA (0x05)");
        return false;
    }
    if (out.roiIndex != kRoi0 && out.roiIndex != kRoi1) {
        setError(error, "only RANGE ROI0/ROI1 are supported");
        return false;
    }
    if (out.pixelLength != kRecordSize) {
        std::ostringstream oss;
        oss << "pixel_len is " << out.pixelLength << ", expected 278";
        setError(error, oss.str());
        return false;
    }
    if (out.dotCount == 0) {
        setError(error, "dot_num is zero");
        return false;
    }
    const std::size_t expected = kUdpHeaderSize +
                                 static_cast<std::size_t>(out.dotCount) * out.pixelLength;
    if (size != expected) {
        std::ostringstream oss;
        oss << "UDP length mismatch: actual=" << size << " expected=" << expected
            << " (40 + dot_num*pixel_len)";
        setError(error, oss.str());
        return false;
    }
    return true;
}

bool decodePixelCoordinate(std::uint16_t groupIndex,
                           std::uint16_t slotIndex,
                           std::uint16_t pixelId,
                           PixelCoordinate& out) {
    return decodePixelCoordinate(kRoi0, groupIndex, slotIndex, pixelId, 0, out);
}

bool decodePixelCoordinate(std::uint8_t roiIndex,
                           std::uint16_t groupIndex,
                           std::uint16_t slotIndex,
                           std::uint16_t pixelId,
                           std::uint8_t subFrameIndex,
                           PixelCoordinate& out) {
    constexpr std::uint16_t channelCount = 12;
    constexpr std::uint16_t halfPiecesPerChannel = 2;
    constexpr std::uint16_t piecesPerBlock = 2;
    constexpr std::uint16_t pieceLoopCount = 8;
    constexpr std::uint16_t rowCountInPiece = 4;
    constexpr std::uint16_t colCountInPiece = 4;
    constexpr std::uint16_t maxPiecesPerGroup = 48;

    const bool isRoi1 = roiIndex == kRoi1;
    const std::uint16_t pixelsPerChannel = isRoi1 ? 4 : 16;
    const std::uint16_t pixelsPerSlot = isRoi1 ? kRoi1PixelsPerSlot : kPixelsPerSlot;
    if ((roiIndex != kRoi0 && !isRoi1) || slotIndex >= kSlotsPerGroup ||
        pixelId >= pixelsPerSlot ||
        (!isRoi1 && groupIndex >= kGroupsPerFrame) ||
        (isRoi1 && (groupIndex < kRoi1FirstGroup ||
                    groupIndex >= kRoi1FirstGroup + kRoi1GroupsPerFrame ||
                    subFrameIndex >= kRoi1SubFrames))) {
        return false;
    }

    // 对应 cicl_decode_pixel_id_fast() 的链路：pixel_id -> channel_id /
    // channel 内编号 -> slot piece -> 物理 piece -> piece 内 row/col。
    const std::uint16_t pixelInChannel = pixelId / channelCount;
    if (pixelInChannel >= pixelsPerChannel) {
        return false;
    }
    const std::uint16_t channelId = pixelId - pixelInChannel * channelCount;
    const std::uint16_t pixelInPiece = pixelInChannel; // one piece/channel here
    const std::uint16_t pieceIdInSlot = channelId * halfPiecesPerChannel / 2;
    const std::uint16_t loopIndex = pieceIdInSlot / piecesPerBlock;
    std::uint16_t loopOffset = pieceIdInSlot - loopIndex * piecesPerBlock;
    loopOffset = static_cast<std::uint16_t>(loopOffset + slotIndex * piecesPerBlock);
    const std::uint16_t physicalPiece = pieceLoopCount * loopIndex + loopOffset;
    if (physicalPiece >= maxPiecesPerGroup) {
        return false;
    }

    static constexpr std::uint8_t kRoi1PieceMaps[kRoi1SubFrames][4] = {
        {0, 5, 10, 15}, {1, 12, 6, 11}, {8, 13, 2, 7}, {4, 9, 3, 14}
    };
    const std::uint16_t mappedPixel = isRoi1
        ? kRoi1PieceMaps[subFrameIndex][pixelInPiece]
        : kPieceMap3x3[pixelInPiece];
    out.row = static_cast<std::uint16_t>(groupIndex * 4 + mappedPixel / rowCountInPiece);
    out.col = static_cast<std::uint16_t>(physicalPiece * colCountInPiece +
                                         mappedPixel % colCountInPiece);
    return out.row < kRows && out.col < kCols &&
           (!isRoi1 || (out.row >= kRoi1RowStart && out.row < kRoi1RowStart + kRoi1Rows));
}

double horizontalAngleDeg(std::uint16_t col) {
    // 当前为离线 Python 脚本的理想均匀角模型；量产显示应替换为出厂标定角表。
    return (static_cast<double>(col) - 95.5) * kHorizontalStepDeg;
}

double verticalAngleDeg(std::uint16_t row) {
    return (63.5 - static_cast<double>(row)) * kVerticalStepDeg;
}

} // namespace range6532

