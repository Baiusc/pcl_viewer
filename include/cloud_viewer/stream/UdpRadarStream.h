/**
 * @brief 面阵激光雷达 UDP 在线接收
 *
 * 协议来源：matrix_lidar_sdk/global.h 与 matrix_process.cpp
 *  - 本地绑定 6002，接收源端口 6001（兼容 6301）的 UDP 包
 *  - 包头 0xA55A，data_type=0x01 为点云，0x04 为 IMU
 *  - frame_flag：0=帧开始，1=中间，2=帧结束
 *  - 点类型：0=距离+反射率，1=XYZ+反射率，2=距离+XYZ+反射率
 *  - 坐标单位：distance 为 cm，pt3d 为 cm（写入 PointData 时统一转米）
 */
#pragma once

#include <atomic>
#include <cstdint>
#include <functional>
#include <map>
#include <mutex>
#include <memory>
#include "cloud_viewer/stream/RadarControl.h"
#include "cloud_viewer/stream/RangeStreamDecoder.h"
#include <string>
#include <thread>
#include <vector>

#include "cloud_viewer/core/CloudFrame.h"

namespace cloud_viewer
{

struct UdpRadarConfig
{
    std::string radarIp = "192.168.137.200";
    std::uint16_t radarPort = 6001;
    std::uint16_t localPort = 6002;
    RadarOutputMode mode = RadarOutputMode::Cloud;
    std::uint8_t roi = 0;
};

struct RadarStreamStats
{
    std::uint64_t packets = 0;
    std::uint64_t invalidPackets = 0;
    std::uint64_t frames = 0;
    double framesPerSecond = 0.0;
    std::string lastError;
};

class UdpRadarStream
{
public:
    using FrameSink = std::function<void(algo_lidar::PointMatrix &&, std::uint64_t timestampUs, int roi)>;

    ~UdpRadarStream();

    bool open(const UdpRadarConfig &config, FrameSink sink, std::string &error);

    void stop();

    bool running() const { return running_.load(); }

    RadarStreamStats stats() const;

    const UdpRadarConfig &config() const { return config_; }

private:
    void receiveLoop();
    void processDatagram(const std::uint8_t *data, std::size_t length);
    void flushFrame();

    // ---- FL6532 (data_type=0x21) ----
    struct Fl6532PointOut
    {
        float x = 0.0f;
        float y = 0.0f;
        float z = 0.0f;
        float distanceCm = 0.0f;
        std::uint8_t reflectivity = 0;
        std::uint8_t label = 0;
        std::uint16_t row = 0;
        std::uint16_t col = 0;
        std::uint64_t timestampUs = 0;
    };

    struct Fl6532Frame
    {
        std::uint16_t frameCnt = 0;
        std::uint8_t roi = 0;
        std::uint64_t touch = 0;
        std::vector<bool> seen;
        std::uint16_t width = 192;
        std::uint16_t height = 128;
        std::uint32_t received = 0;
        std::uint64_t timestampBase = 0;
        std::map<std::uint16_t, std::vector<Fl6532PointOut>> packets;
    };

    void processFl6532(const std::uint8_t *data, std::size_t length);
    void flushFl6532(std::uint32_t key);
    void receiveRangeFrame(range6532::PointCloudFrame&& frame);
    void emitMatrix(algo_lidar::PointMatrix &&matrix, std::uint64_t timestampUs, int roi = -1);

    std::map<std::uint32_t, Fl6532Frame> fl6532Frames_;
    std::unique_ptr<RangeStreamDecoder> rangeDecoder_;
    std::uint64_t cloudTick_ = 0;

    struct PacketRef
    {
        std::vector<std::uint8_t> payload;
        std::uint16_t startRow = 0;
        std::uint16_t startCol = 0;
        std::uint16_t dotNum = 0;
        std::uint8_t pointType = 0;
        std::uint64_t timestampUs = 0;
    };

    UdpRadarConfig config_;
    FrameSink sink_;

    int socket_ = -1;
    std::atomic<bool> running_{false};
    std::thread thread_;

    mutable std::mutex statsMutex_;
    RadarStreamStats stats_;

    std::vector<PacketRef> framePackets_;
    bool hasFrameStart_ = false;

    std::uint64_t fpsWindowStartMs_ = 0;
    std::uint64_t fpsWindowFrames_ = 0;
};

} // namespace cloud_viewer
