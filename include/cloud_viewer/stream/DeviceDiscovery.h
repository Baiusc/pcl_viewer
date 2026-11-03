/**
 * @brief 局域网雷达发现（监听厂商广播）
 *
 * 雷达上电后周期性向 UDP 55000 广播（源端口 65000）。实测包结构：
 *   [0]      0xAA             magic
 *   [1]      0x01             协议版本
 *   [2..3]   uint16 LE        data_len
 *   [4]      0x02             消息类型
 *   [11..]   ASCII            serial_number（如 CS96000B0126020001）
 *   [..]     uint8            work_mode
 *   [len-8]  uint16 LE        line_count（CS96 Beta = 96）
 *   [len-6]  uint16 LE        slots_per_frame（CS96 Beta = 800）
 *   [len-4]  uint32           CRC
 *
 * 仅做被动监听，不发送任何探测包，避免影响雷达状态。
 */
#pragma once

#include <cstdint>
#include <memory>
#include <string>
#include <vector>

namespace cloud_viewer
{

struct DiscoveredDevice
{
    std::string ip;
    std::string serialNumber;
    std::string model;        ///< "CS96 Beta" / "CS96" / "FL6532" / "LiDAR"
    std::uint16_t lineCount = 0;
    std::uint16_t slotsPerFrame = 0;
    std::uint8_t workMode = 0;
    std::uint64_t lastSeenMs = 0;
};

class DeviceDiscovery
{
public:
    DeviceDiscovery();
    ~DeviceDiscovery();

    /// 开始监听广播（默认 55000 端口）。重复调用会先停止旧监听。
    bool start(std::uint16_t port = 55000);
    void stop();
    bool running() const;

    /// 当前已发现的设备快照（按 IP 去重，最新广播覆盖旧值）。
    std::vector<DiscoveredDevice> snapshot() const;

    /// 解析单个广播报文；失败返回 false。公开以便单元测试。
    static bool parseBroadcast(const std::uint8_t *data, std::size_t size,
                               DiscoveredDevice &device);

private:
    struct Impl;
    std::unique_ptr<Impl> impl_;
};

} // namespace cloud_viewer
