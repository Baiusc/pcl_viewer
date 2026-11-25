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

/// 网口物理类型。笔记本通常同时有「有线以太网」和「无线 WLAN」两块，
/// 雷达可能接在任意一块上，扫描时必须两块都覆盖，并且有线优先。
enum class LinkKind
{
    Wired,
    Wireless,
    Other,
};

const char *linkKindName(LinkKind kind); ///< "有线" / "无线" / "其他"

/// 本机一个可用 IPv4 网口。
struct LocalInterface
{
    std::string name;         ///< 系统网口名，如 "以太网" / "WLAN" / "eth0" / "wlan0"
    std::string description;  ///< Windows 网卡描述（Linux 为空）
    std::string address;      ///< IPv4 地址
    int prefixLength = 24;    ///< 掩码前缀长度；169.254.x.x 自动私有地址为 16
    std::uint32_t ifIndex = 0;
    LinkKind kind = LinkKind::Other;
    bool linkLocal = false;   ///< 169.254.0.0/16（直连未配 IP 时的自动地址）
    bool virtualAdapter = false; ///< Wi-Fi Direct / Hyper-V / VMware 等虚拟网口
};

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
    std::string interfaceName; ///< 命中该设备的本机网口名（扫描发现时有效）
    LinkKind linkKind = LinkKind::Other; ///< 该网口是有线还是无线
    /// ARP/端口探测是否确认在线。false 表示只命中了系统邻居表缓存，
    /// 设备可能已经离线（缓存条目在 Windows 上会保留一段时间）。
    bool arpConfirmed = false;
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

    /// 本机所有可用 IPv4 网口（已排除回环、虚拟网卡；有线排在无线前面）。
    ///
    /// 笔记本常见双网口：有线接雷达（常是 169.254.x.x/16 自动地址）、
    /// 无线连公司网络（如 10.x/24）。只扫「第一个 /24」会永远扫不到接在
    /// 另一块网口上的雷达，所以扫描必须按网口逐个进行。
    static std::vector<LocalInterface> localInterfaces();

    /// 解析单个广播报文；失败返回 false。公开以便单元测试。
    static bool parseBroadcast(const std::uint8_t *data, std::size_t size,
                               DiscoveredDevice &device);

    /// 主动发现兜底：遍历本机所有物理网口（有线 + 无线）扫描邻居。
    ///
    /// 广播依赖雷达上的 you_see，进程没跑就完全搜不到；而 ARP 由内核应答，
    /// 只要设备网络栈起来了就能被看见。流程（按网口逐个执行，有线优先）：
    ///   1) 读系统邻居表（缓存命中即得，零成本）；
    ///   2) 对 hints 里的已知地址定向 ARP（可命中 /16 网段里跨 /24 的设备）；
    ///   3) 该网口还没命中时，再扫其所在 /24 兜底。
    /// Windows 下 ARP 请求按网口源地址发出，避免多网口时被错误网卡路由。
    ///
    /// @param macPrefix  厂商标识，如 "98:03:8a"；空串表示不按 MAC 过滤。
    /// @param hints      已知/上次连接的地址，优先定向探测，可跨 /24。
    /// @param note       输出人类可读的扫描摘要（网口、邻居数、命中数、耗时）。
    /// @return 命中的设备；fromScan=true，model 未知时填 "LiDAR"。
    static std::vector<DiscoveredDevice>
    scanSubnet(const std::string &macPrefix, const std::vector<std::string> &hints,
               std::string &note);

    /// 兼容旧调用：不带 hints 的扫描。
    static std::vector<DiscoveredDevice> scanSubnet(const std::string &macPrefix,
                                                    std::string &note);

    /// 解析 /proc/net/arp 文本，返回 [IP, MAC] 列表；flags 非 0x2 的条目跳过。
    /// 公开以便单元测试。
    static std::vector<std::pair<std::string, std::string>> parseArpTable(const std::string &text);

    /// 非阻塞 connect，判断目标 TCP 端口能否建立连接。
    /// localIp 非空时把源地址绑定到该网口（多网口时避免走错网卡）。
    static bool tcpPortOpen(const std::string &ip, std::uint16_t port, int timeoutMs,
                            const std::string &localIp = std::string());

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
