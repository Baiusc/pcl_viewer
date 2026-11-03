/**
 * @brief CS96 Beta 转镜雷达（96 线 × 800 slot）UDP 直收，支持三种 ARM 输出模式
 *
 * 模式由设备 /etc/ys_conf.ini 的 work_mode 决定，包长不同，可自动识别：
 *
 *   2 真实点云（默认）1266B = 18B 头 + 96 × 13B（float x,y,z + uint8 albedo）
 *   1 原始点云        1561B = 9B 头 + 16B slot 头 + 96 × 16B（ft/et/pk × 2 回波）
 *   0 直方图          4105B = 9B 头 + 4096B（4 像素 × 512 bins × uint16，24 包/帧）
 *
 * 原始点云距离：r = 0.000299792458 * (ft >> 1) 米（kLightSpeed * ft/2）。
 * 直方图没有物理量纲，按 “bin × 像素 × 强度” 构造可观察的地形点云。
 */
#pragma once

#include <cstdint>
#include <functional>
#include <memory>
#include <string>

#include "cloud_viewer/core/CloudFrame.h"
#include "cloud_viewer/preprocess/Cs96BetaCalibration.h"

namespace cloud_viewer
{

class BetaUdpStream
{
public:
    enum class WorkMode
    {
        Histogram = 0,
        OriginCloud = 1,
        CartesianCloud = 2
    };

    struct Config
    {
        std::string radarIp = "10.42.0.218";
        std::uint16_t localPort = 50007;
    };

    struct Stats
    {
        std::uint64_t packets = 0;
        std::uint64_t frames = 0;
        std::uint64_t invalidPackets = 0;
        std::uint64_t droppedSlots = 0;
        int detectedMode = -1;
    };

    using FrameSink = std::function<void(algo_lidar::PointMatrix &&, std::uint64_t)>;

    BetaUdpStream();
    ~BetaUdpStream();

    bool open(const Config &config, FrameSink sink, std::string &error);
    void stop();
    bool running() const;

    Config config() const;
    Stats stats() const;

    /// 原始点云模式使用的标定表；为空时退回简化公式。
    void setCalibration(std::shared_ptr<const Cs96BetaCalibration> calibration);

    /// 一键开关四项标定（距离系数/零点、逐线水平角、FPGA 旋转角、反射率查表）。
    /// 关闭后仍使用编码器表、面起始角、omega0、俯仰角等结构参数。
    void setCalibrationEnabled(bool enabled);
    bool calibrationEnabled() const;

    /// 逐项控制标定，便于单独定位某一项的影响。
    void setDecodeOptions(const Cs96BetaCalibration::DecodeOptions &options);
    Cs96BetaCalibration::DecodeOptions decodeOptions() const;

    /// 面选择独立于坐标标定：原始帧按起始 PA 判面，真实帧直接使用设备面 ID。
    /// bit0~bit3 对应 F1~F4，0xF 为全部。
    void setFaceMask(std::uint8_t mask);
    std::uint8_t faceMask() const;

    /// 解析真实点云包（1266B），保留给单元测试。
    static bool parsePacket(const std::uint8_t *data, std::size_t size,
                            std::uint16_t &frameIndex, std::uint16_t &slot,
                            algo_lidar::PointData *points, int &pointCount);

    /// PA 固定机械四分区：以 0/90/180/270 度为中心，不依赖标定文件。
    /// F1=[315,360)∪[0,45)，F2=[45,135)，F3=[135,225)，F4=[225,315)。
    static int faceFromPa(std::uint32_t mirrorAngle);

    static constexpr std::size_t kCartesianPacketSize = 1266;
    static constexpr std::size_t kOriginPacketSize = 1561;
    static constexpr std::size_t kHistogramPacketSize = 4105;
    static constexpr int kLines = 96;
    static constexpr int kSlots = 800;
    static constexpr int kBins = 512;

private:
    struct Impl;
    std::unique_ptr<Impl> impl_;
};

} // namespace cloud_viewer
