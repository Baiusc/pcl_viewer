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
#include <utility>
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

    // 主动扫描补充的信息（广播发现时 sshReady 由调用方决定是否探测）
    std::string mac;          ///< ARP 学到的 MAC，形如 "98:03:8a:7f:32:90"
    bool sshReady = false;    ///< TCP 22 可连接（能远程改配置/起进程）
    bool fromScan = false;    ///< true=网段扫描发现，没有广播元数据
    bool fromProbe = false;   ///< true=按已知 IP 探测发现（FL6532 专用）
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

    /// 主动发现兜底：扫描本机所有 /24 IPv4 网段。
    ///
    /// 广播依赖雷达上的 you_see，进程没跑就完全搜不到；而 ARP 由内核应答，
    /// 只要设备网络栈起来了就能被看见。流程：
    ///   并行 ping 触发 ARP -> 读 /proc/net/arp -> 按 MAC 前缀筛选 -> 探测 TCP 22。
    ///
    /// @param macPrefix  厂商标识，如 "98:03:8a"；空串表示不按 MAC 过滤。
    /// @param note       输出人类可读的扫描摘要（网段数量、命中数、耗时）。
    /// @return 命中的设备；fromScan=true，model 未知时填 "LiDAR"。
    static std::vector<DiscoveredDevice> scanSubnet(const std::string &macPrefix,
                                                    std::string &note);

    /// 解析 /proc/net/arp 文本，返回 [IP, MAC] 列表；flags 非 0x2 的条目跳过。
    /// 公开以便单元测试。
    static std::vector<std::pair<std::string, std::string>> parseArpTable(const std::string &text);

    /// 非阻塞 connect，判断目标 TCP 端口能否建立连接。
    static bool tcpPortOpen(const std::string &ip, std::uint16_t port, int timeoutMs);

    /// FL6532 固态雷达不发送 55000 广播，只能按已知 IP 探测。
    ///
    /// 判据是 TCP 6508（控制口）可连接；CS96 Beta 不开放该端口，
    /// 因此不会把 ARM 平台雷达误判成 6532。
    static std::vector<DiscoveredDevice> probeFl6532(const std::vector<std::string> &candidates,
                                                     std::string &note);

private:
    struct Impl;
    std::unique_ptr<Impl> impl_;
};

} // namespace cloud_viewer
