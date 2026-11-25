#include "cloud_viewer/stream/DeviceDiscovery.h"

#include <algorithm>
#include <atomic>
#include <cctype>
#include <cerrno>
#include <chrono>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <filesystem>
#include <fstream>
#include <iterator>
#include <map>
#include <mutex>
#include <set>
#include <sstream>
#include <system_error>
#include <thread>
#include <vector>

#ifndef _WIN32
#include <arpa/inet.h>
#include <fcntl.h>
#include <ifaddrs.h>
#include <net/if.h>
#include <netinet/in.h>
#include <sys/select.h>
#include <sys/socket.h>
#include <sys/time.h>
#include <unistd.h>
#else
#ifndef NOMINMAX
#define NOMINMAX
#endif
#include <winsock2.h>
#include <ws2tcpip.h>
#include <iphlpapi.h>
#include <ipifcons.h> // IF_TYPE_SOFTWARE_LOOPBACK / IfOperStatusUp
#include <netioapi.h> // GetIpNetTable2：读系统邻居表
#endif

namespace cloud_viewer
{
namespace
{

std::uint64_t nowMs()
{
    return static_cast<std::uint64_t>(
        std::chrono::duration_cast<std::chrono::milliseconds>(
            std::chrono::steady_clock::now().time_since_epoch())
            .count());
}

#ifdef _WIN32
/// Winsock 只需初始化一次；UDP 监听、探测与 SendARP 都依赖它。
void ensureWinsock()
{
    static std::once_flag initialized;
    std::call_once(initialized, []() {
        WSADATA data{};
        ::WSAStartup(MAKEWORD(2, 2), &data);
    });
}

/// SendARP 返回的 MAC 转成与 Linux 一致的 "aa:bb:cc:dd:ee:ff"。
std::string macToString(const unsigned char *bytes)
{
    static const char *kHex = "0123456789abcdef";
    std::string text;
    text.reserve(17);
    for (int i = 0; i < 6; ++i)
    {
        if (i > 0)
        {
            text.push_back(':');
        }
        text.push_back(kHex[(bytes[i] >> 4) & 0x0F]);
        text.push_back(kHex[bytes[i] & 0x0F]);
    }
    return text;
}
#endif

std::uint16_t readLe16(const std::uint8_t *data)
{
    return static_cast<std::uint16_t>(data[0]) |
           static_cast<std::uint16_t>(static_cast<std::uint16_t>(data[1]) << 8);
}

std::string classifyModel(const std::string &serial)
{
    if (serial.rfind("CS96", 0) == 0)
    {
        // 硬件版本位（位置随型号前缀略有变化，取前 12 字符内）：
        //   B/b = Beta（IMX459，96×800）；C/c = Gamma（SL100，120×800）。
        // Gamma 未试产，SN 仍沿用 CS96 前缀，正式命名应为 cs120 Gamma。
        const auto version = serial.find_first_of("BbCc", 4);
        if (version != std::string::npos && version < 12)
        {
            const char tag = serial[version];
            return (tag == 'C' || tag == 'c') ? "CS120 Gamma" : "CS96 Beta";
        }
        return "CS96";
    }
    if (serial.find("6532") != std::string::npos || serial.find("FL65") != std::string::npos)
    {
        return "FL6532";
    }
    return "LiDAR";
}

/// 本机可用网口（已排除回环/虚拟网口）。多网口是笔记本的常态：
/// 有线接雷达（常为 169.254.x.x/16 自动地址）、无线连公司网（10.x/24）。
/// 只扫「唯一一个 /24」会漏掉接在另一块网口上的雷达，因此必须逐网口扫描。
struct AdapterInfo
{
    std::string name;
    std::string description;
    std::string address;
    int prefixLength = 24;
    std::uint32_t ifIndex = 0;
    bool up = false;
    bool linkLocal = false;
    bool virtualAdapter = false;
    bool wireless = false;
};

/// 网卡描述里的虚拟/隧道关键字：这些网口（Wi-Fi Direct、Hyper-V、VMware 等）
/// 也会带 169.254/16 地址，混进来只会让扫描变慢并干扰网段判断。
bool looksVirtualAdapter(const std::string &text)
{
    static const char *kKeywords[] = {
        "wi-fi direct", "wifi direct", "virtual", "vmware", "virtualbox", "hyper-v",
        "vethernet",   "loopback",    "tap-",    "tap adapter", "tunnel",   "teredo",
        "bluetooth",   "npcap",       "wintun",  "zerotier",    "tailscale", "docker",
        "veth",        "br-",         "virbr",   "default switch",
    };
    for (const char *keyword : kKeywords)
    {
        if (text.find(keyword) != std::string::npos)
        {
            return true;
        }
    }
    return false;
}

bool looksWirelessAdapter(const std::string &text)
{
    static const char *kKeywords[] = {"wi-fi", "wifi", "wireless", "wlan", "802.11", "wlp"};
    for (const char *keyword : kKeywords)
    {
        if (text.find(keyword) != std::string::npos)
        {
            return true;
        }
    }
    return false;
}

std::string toLower(std::string text)
{
    std::transform(text.begin(), text.end(), text.begin(),
                   [](unsigned char ch) { return static_cast<char>(std::tolower(ch)); });
    return text;
}

#ifdef _WIN32
std::string narrow(const wchar_t *text)
{
    if (text == nullptr)
    {
        return std::string();
    }
    const int size = ::WideCharToMultiByte(CP_UTF8, 0, text, -1, nullptr, 0, nullptr, nullptr);
    if (size <= 1)
    {
        return std::string();
    }
    std::string out(static_cast<std::size_t>(size - 1), '\0');
    ::WideCharToMultiByte(CP_UTF8, 0, text, -1, &out[0], size, nullptr, nullptr);
    return out;
}

std::vector<AdapterInfo> enumerateAdapters()
{
    ensureWinsock();
    ULONG size = 16 * 1024;
    std::vector<unsigned char> buffer(size, 0);
    auto *addrs = reinterpret_cast<IP_ADAPTER_ADDRESSES *>(buffer.data());
    const ULONG flags = GAA_FLAG_SKIP_ANYCAST | GAA_FLAG_SKIP_MULTICAST | GAA_FLAG_SKIP_DNS_SERVER;
    ULONG status = ::GetAdaptersAddresses(AF_INET, flags, nullptr, addrs, &size);
    if (status == ERROR_BUFFER_OVERFLOW)
    {
        buffer.assign(size, 0);
        addrs = reinterpret_cast<IP_ADAPTER_ADDRESSES *>(buffer.data());
        status = ::GetAdaptersAddresses(AF_INET, flags, nullptr, addrs, &size);
    }
    if (status != NO_ERROR)
    {
        return {};
    }

    std::vector<AdapterInfo> adapters;
    for (IP_ADAPTER_ADDRESSES *adapter = addrs; adapter != nullptr; adapter = adapter->Next)
    {
        if (adapter->IfType == IF_TYPE_SOFTWARE_LOOPBACK)
        {
            continue;
        }
        AdapterInfo info;
        info.name = narrow(adapter->FriendlyName);
        info.description = narrow(adapter->Description);
        info.ifIndex = adapter->IfIndex;
        info.up = adapter->OperStatus == IfOperStatusUp;

        const std::string haystack = toLower(info.name + " " + info.description);
        info.virtualAdapter = looksVirtualAdapter(haystack) ||
                              adapter->IfType == IF_TYPE_TUNNEL || adapter->IfType == IF_TYPE_PPP;
        info.wireless = adapter->IfType == IF_TYPE_IEEE80211 || looksWirelessAdapter(haystack);

        for (IP_ADAPTER_UNICAST_ADDRESS *unicast = adapter->FirstUnicastAddress; unicast != nullptr;
             unicast = unicast->Next)
        {
            if (unicast->Address.lpSockaddr == nullptr ||
                unicast->Address.lpSockaddr->sa_family != AF_INET)
            {
                continue;
            }
            const auto *address = reinterpret_cast<const sockaddr_in *>(unicast->Address.lpSockaddr);
            char text[INET_ADDRSTRLEN] = {0};
            if (::inet_ntop(AF_INET, &address->sin_addr, text, sizeof(text)) == nullptr)
            {
                continue;
            }
            AdapterInfo entry = info;
            entry.address = text;
            entry.prefixLength = static_cast<int>(unicast->OnLinkPrefixLength);
            entry.linkLocal = entry.address.rfind("169.254.", 0) == 0;
            adapters.push_back(entry);
        }
    }
    return adapters;
}
#else
std::vector<AdapterInfo> enumerateAdapters()
{
    std::vector<AdapterInfo> adapters;
    ifaddrs *list = nullptr;
    if (::getifaddrs(&list) != 0)
    {
        return adapters;
    }
    for (ifaddrs *it = list; it != nullptr; it = it->ifa_next)
    {
        if (it->ifa_addr == nullptr || it->ifa_netmask == nullptr)
        {
            continue;
        }
        if (it->ifa_addr->sa_family != AF_INET)
        {
            continue;
        }
        if ((it->ifa_flags & IFF_LOOPBACK) != 0)
        {
            continue;
        }
        const auto *address = reinterpret_cast<const sockaddr_in *>(it->ifa_addr);
        const auto *mask = reinterpret_cast<const sockaddr_in *>(it->ifa_netmask);
        char text[INET_ADDRSTRLEN] = {0};
        if (::inet_ntop(AF_INET, &address->sin_addr, text, sizeof(text)) == nullptr)
        {
            continue;
        }
        const std::uint32_t maskBits = ntohl(mask->sin_addr.s_addr);
        int prefixLength = 0;
        for (std::uint32_t bit = 0x80000000u; bit != 0 && (maskBits & bit) != 0; bit >>= 1)
        {
            ++prefixLength;
        }

        AdapterInfo info;
        info.name = it->ifa_name != nullptr ? it->ifa_name : "";
        info.address = text;
        info.prefixLength = prefixLength;
        info.ifIndex = ::if_nametoindex(info.name.c_str());
        info.up = (it->ifa_flags & IFF_UP) != 0;
        info.linkLocal = info.address.rfind("169.254.", 0) == 0;

        const std::string haystack = toLower(info.name);
        // Linux 用网口名前缀区分：wl* 无线、en*/eth* 有线；虚拟网桥与容器网口跳过。
        info.virtualAdapter = haystack.rfind("lo", 0) == 0 || haystack.rfind("docker", 0) == 0 ||
                              haystack.rfind("veth", 0) == 0 || haystack.rfind("br-", 0) == 0 ||
                              haystack.rfind("virbr", 0) == 0 || haystack.rfind("tun", 0) == 0 ||
                              haystack.rfind("tap", 0) == 0;
        info.wireless = haystack.rfind("wl", 0) == 0;
        // /sys/class/net/<if>/wireless 存在即为无线网卡（比名字更可靠）。
        std::error_code ec;
        if (!info.wireless && std::filesystem::exists("/sys/class/net/" + info.name + "/wireless", ec))
        {
            info.wireless = true;
        }
        adapters.push_back(info);
    }
    ::freeifaddrs(list);
    return adapters;
}
#endif

std::uint32_t ipToUint(const std::string &ip)
{
    in_addr address{};
    if (::inet_pton(AF_INET, ip.c_str(), &address) != 1)
    {
        return 0;
    }
    return ntohl(address.s_addr);
}

std::string uintToIp(std::uint32_t value)
{
    in_addr address{};
    address.s_addr = htonl(value);
    char text[INET_ADDRSTRLEN] = {0};
    if (::inet_ntop(AF_INET, &address, text, sizeof(text)) == nullptr)
    {
        return std::string();
    }
    return text;
}

bool sameSubnet(const std::string &ip, const std::string &localIp, int prefixLength)
{
    const std::uint32_t target = ipToUint(ip);
    const std::uint32_t local = ipToUint(localIp);
    if (target == 0 || local == 0)
    {
        return false;
    }
    const int prefix = std::min(std::max(prefixLength, 1), 32);
    const std::uint32_t mask = (prefix == 32) ? 0xFFFFFFFFu : (0xFFFFFFFFu << (32 - prefix));
    return (target & mask) == (local & mask);
}

/// 该网口要扫的主机地址。
///
/// /24 及更小的网段全量扫；更宽的网段（如直连自动地址 169.254/16）只扫本机所在
/// /24 —— /16 全量是 6.5 万个地址，ARP 串行超时按秒计，不可接受。
/// 跨 /24 的已知设备靠 hints 定向探测解决。
std::vector<std::string> sweepTargetsFor(const AdapterInfo &adapter)
{
    std::vector<std::string> targets;
    const std::int64_t hostBits = 32 - std::min(std::max(adapter.prefixLength, 1), 32);
    const int effectivePrefix =
        (hostBits <= 10) ? adapter.prefixLength : 24; // > 1022 台主机时退化为本机 /24
    const std::uint32_t value = ipToUint(adapter.address);
    if (value == 0)
    {
        return targets;
    }
    const std::uint32_t mask =
        (effectivePrefix == 32) ? 0xFFFFFFFFu : (0xFFFFFFFFu << (32 - effectivePrefix));
    const std::uint32_t network = value & mask;
    const std::uint32_t broadcast = network | ~mask;
    for (std::uint32_t host = network + 1; host < broadcast; ++host)
    {
        targets.push_back(uintToIp(host));
    }
    return targets;
}

/// 邻居表里的组播/广播/保留地址不是设备：224/4 组播、255 广播、x.x.x.255 子网广播。
bool isUnicastHostAddress(const std::string &ip)
{
    in_addr address{};
    if (::inet_pton(AF_INET, ip.c_str(), &address) != 1)
    {
        return false;
    }
    const std::uint32_t value = ntohl(address.s_addr);
    const std::uint8_t first = static_cast<std::uint8_t>((value >> 24) & 0xFF);
    const std::uint8_t last = static_cast<std::uint8_t>(value & 0xFF);
    if (first == 0 || first >= 224 || last == 0 || last == 255)
    {
        return false;
    }
    return true;
}

/// 系统邻居表（缓存命中即可发现设备，成本为零）。
struct NeighborRow
{
    std::string ip;
    std::string mac;
    std::uint32_t ifIndex = 0;
    std::string interfaceName;
};

#ifdef _WIN32
std::vector<NeighborRow> readNeighborTable()
{
    std::vector<NeighborRow> rows;
    PMIB_IPNET_TABLE2 table = nullptr;
    if (::GetIpNetTable2(AF_INET, &table) != NO_ERROR || table == nullptr)
    {
        return rows;
    }
    for (ULONG i = 0; i < table->NumEntries; ++i)
    {
        const MIB_IPNET_ROW2 &row = table->Table[i];
        if (row.Address.si_family != AF_INET || row.PhysicalAddressLength < 6)
        {
            continue;
        }
        // Unreachable/Incomplete 是「扫过但没人应答」的残留，不能当发现结果。
        if (row.State == NlnsUnreachable || row.State == NlnsIncomplete)
        {
            continue;
        }
        char text[INET_ADDRSTRLEN] = {0};
        if (::inet_ntop(AF_INET, &row.Address.Ipv4.sin_addr, text, sizeof(text)) == nullptr)
        {
            continue;
        }
        if (!isUnicastHostAddress(text))
        {
            continue;
        }
        NeighborRow entry;
        entry.ip = text;
        entry.mac = macToString(row.PhysicalAddress);
        entry.ifIndex = row.InterfaceIndex;
        rows.push_back(entry);
    }
    ::FreeMibTable(table);
    return rows;
}
#else
std::vector<NeighborRow> readNeighborTable()
{
    std::vector<NeighborRow> rows;
    std::ifstream arp("/proc/net/arp");
    std::string line;
    bool headerSkipped = false;
    while (std::getline(arp, line))
    {
        if (!headerSkipped)
        {
            headerSkipped = true;
            continue;
        }
        std::istringstream fields(line);
        std::string ip;
        std::string hwType;
        std::string flags;
        std::string mac;
        std::string mask;
        std::string device;
        if (!(fields >> ip >> hwType >> flags >> mac >> mask >> device))
        {
            continue;
        }
        if (flags != "0x2" || mac == "00:00:00:00:00:00")
        {
            continue;
        }
        if (!isUnicastHostAddress(ip))
        {
            continue;
        }
        NeighborRow entry;
        entry.ip = ip;
        entry.mac = mac;
        entry.interfaceName = device;
        entry.ifIndex = ::if_nametoindex(device.c_str());
        rows.push_back(entry);
    }
    return rows;
}
#endif

#ifdef _WIN32
/// 对单个地址发一次 ARP 请求。
///
/// SendARP 的第二个参数是「用哪个源地址发」。多网口机器上两块网卡常常都是
/// 169.254/16（无线直连虚拟网卡 + 有线），不指定源地址时由系统按路由挑，
/// 很容易从错误的网卡发出去而收不到回应，因此这里必须按网口绑定源地址。
bool arpProbe(const std::string &ip, const std::string &sourceIp, std::string &mac)
{
    in_addr target{};
    in_addr source{};
    if (::inet_pton(AF_INET, ip.c_str(), &target) != 1)
    {
        return false;
    }
    const ULONG sourceAddress =
        (::inet_pton(AF_INET, sourceIp.c_str(), &source) == 1) ? source.s_addr : 0;
    ULONG macBytes[2] = {0, 0};
    ULONG length = sizeof(macBytes);
    if (::SendARP(target.s_addr, sourceAddress, macBytes, &length) != NO_ERROR || length < 6)
    {
        return false;
    }
    const auto *bytes = reinterpret_cast<const unsigned char *>(macBytes);
    const std::string text = macToString(bytes);
    if (text == "00:00:00:00:00:00")
    {
        return false;
    }
    mac = text;
    return true;
}

/// 对本机一个网口的地址列表做 SendARP 扫描，返回 (ip, mac)。
/// SendARP 走二层解析、不经过防火墙，适合作为「广播被拦截」时的兜底发现手段。
std::vector<std::pair<std::string, std::string>>
sendArpSweep(const std::vector<std::string> &targets, const std::string &sourceIp)
{
    std::vector<std::pair<std::string, std::string>> rows;
    if (targets.empty())
    {
        return rows;
    }

    std::mutex mutex;
    std::atomic<std::size_t> next{0};
    // SendARP 对不存在的地址要等 ARP 重传超时（约 3s），并发度决定了整轮耗时。
    const std::size_t workerCount = std::min<std::size_t>(48, targets.size());
    std::vector<std::thread> workers;
    workers.reserve(workerCount);
    for (std::size_t i = 0; i < workerCount; ++i)
    {
        workers.emplace_back([&]() {
            while (true)
            {
                const std::size_t index = next.fetch_add(1);
                if (index >= targets.size())
                {
                    break;
                }
                std::string mac;
                if (!arpProbe(targets[index], sourceIp, mac))
                {
                    continue;
                }
                std::lock_guard<std::mutex> lock(mutex);
                rows.emplace_back(targets[index], mac);
            }
        });
    }
    for (auto &worker : workers)
    {
        worker.join();
    }
    return rows;
}
#endif

/// 非阻塞 connect，判断目标 TCP 端口是否能建立连接。
/// localIp 非空时把套接字绑定到该源地址，多网口下强制走对应网卡。
bool tcpPortOpenImpl(const std::string &ip, std::uint16_t port, int timeoutMs,
                     const std::string &localIp)
{
#ifndef _WIN32
    const int fd = ::socket(AF_INET, SOCK_STREAM, 0);
    if (fd < 0)
    {
        return false;
    }
    if (!localIp.empty())
    {
        sockaddr_in local{};
        local.sin_family = AF_INET;
        local.sin_port = 0;
        if (::inet_pton(AF_INET, localIp.c_str(), &local.sin_addr) == 1)
        {
            ::bind(fd, reinterpret_cast<sockaddr *>(&local), sizeof(local));
        }
    }
    sockaddr_in address{};
    address.sin_family = AF_INET;
    address.sin_port = htons(port);
    if (::inet_pton(AF_INET, ip.c_str(), &address.sin_addr) != 1)
    {
        ::close(fd);
        return false;
    }
    const int flags = ::fcntl(fd, F_GETFL, 0);
    ::fcntl(fd, F_SETFL, flags | O_NONBLOCK);

    bool open = false;
    if (::connect(fd, reinterpret_cast<sockaddr *>(&address), sizeof(address)) == 0)
    {
        open = true;
    }
    else if (errno == EINPROGRESS)
    {
        fd_set writable;
        fd_set failed;
        FD_ZERO(&writable);
        FD_ZERO(&failed);
        FD_SET(fd, &writable);
        FD_SET(fd, &failed);
        timeval timeout{};
        timeout.tv_sec = timeoutMs / 1000;
        timeout.tv_usec = (timeoutMs % 1000) * 1000;
        if (::select(fd + 1, nullptr, &writable, &failed, &timeout) > 0 &&
            FD_ISSET(fd, &writable))
        {
            int error = 0;
            socklen_t length = sizeof(error);
            if (::getsockopt(fd, SOL_SOCKET, SO_ERROR, &error, &length) == 0 && error == 0)
            {
                open = true;
            }
        }
    }
    ::close(fd);
    return open;
#else
    // Windows：非阻塞 connect + select，语义与 Linux 分支一致。
    ensureWinsock();
    SOCKET fd = ::socket(AF_INET, SOCK_STREAM, IPPROTO_TCP);
    if (fd == INVALID_SOCKET)
    {
        return false;
    }
    if (!localIp.empty())
    {
        sockaddr_in local{};
        local.sin_family = AF_INET;
        local.sin_port = 0;
        if (::inet_pton(AF_INET, localIp.c_str(), &local.sin_addr) == 1)
        {
            // 绑定源地址即可固定出口网卡；失败不致命，退回系统默认路由。
            ::bind(fd, reinterpret_cast<sockaddr *>(&local), sizeof(local));
        }
    }
    u_long nonBlocking = 1;
    ::ioctlsocket(fd, FIONBIO, &nonBlocking);

    sockaddr_in address{};
    address.sin_family = AF_INET;
    address.sin_port = ::htons(port);
    if (::inet_pton(AF_INET, ip.c_str(), &address.sin_addr) != 1)
    {
        ::closesocket(fd);
        return false;
    }

    bool open = false;
    if (::connect(fd, reinterpret_cast<sockaddr *>(&address), sizeof(address)) == 0)
    {
        open = true;
    }
    else if (::WSAGetLastError() == WSAEWOULDBLOCK)
    {
        fd_set writable;
        fd_set failed;
        FD_ZERO(&writable);
        FD_ZERO(&failed);
        FD_SET(fd, &writable);
        FD_SET(fd, &failed);
        timeval timeout{};
        timeout.tv_sec = timeoutMs / 1000;
        timeout.tv_usec = (timeoutMs % 1000) * 1000;
        // Windows 下 select 的第一个参数被忽略，传 0 即可。
        if (::select(0, nullptr, &writable, &failed, &timeout) > 0 && FD_ISSET(fd, &writable) &&
            !FD_ISSET(fd, &failed))
        {
            int error = 0;
            int length = sizeof(error);
            if (::getsockopt(fd, SOL_SOCKET, SO_ERROR, reinterpret_cast<char *>(&error), &length) == 0 &&
                error == 0)
            {
                open = true;
            }
        }
    }
    ::closesocket(fd);
    return open;
#endif
}

} // namespace

struct DeviceDiscovery::Impl
{
    mutable std::mutex mutex;
    std::map<std::string, DiscoveredDevice> devices;
    std::thread worker;
    bool stopRequested = false;
    bool active = false;
    std::uint16_t port = 55000;

    void loop()
    {
#ifndef _WIN32
        const int sock = ::socket(AF_INET, SOCK_DGRAM, 0);
        if (sock < 0)
        {
            std::lock_guard<std::mutex> lock(mutex);
            active = false;
            return;
        }

        int reuse = 1;
        ::setsockopt(sock, SOL_SOCKET, SO_REUSEADDR, &reuse, sizeof(reuse));

        timeval timeout{};
        timeout.tv_sec = 0;
        timeout.tv_usec = 300000; // 300ms，便于及时退出
        ::setsockopt(sock, SOL_SOCKET, SO_RCVTIMEO, &timeout, sizeof(timeout));

        sockaddr_in address{};
        address.sin_family = AF_INET;
        address.sin_addr.s_addr = htonl(INADDR_ANY);
        address.sin_port = htons(port);

        if (::bind(sock, reinterpret_cast<sockaddr *>(&address), sizeof(address)) < 0)
        {
            ::close(sock);
            std::lock_guard<std::mutex> lock(mutex);
            active = false;
            return;
        }

        std::uint8_t buffer[2048];
        while (true)
        {
            {
                std::lock_guard<std::mutex> lock(mutex);
                if (stopRequested)
                {
                    break;
                }
            }

            sockaddr_in from{};
            socklen_t fromLength = sizeof(from);
            const ssize_t received = ::recvfrom(sock, buffer, sizeof(buffer), 0,
                                                reinterpret_cast<sockaddr *>(&from), &fromLength);
            if (received <= 0)
            {
                continue;
            }

            DiscoveredDevice device;
            if (!parseBroadcast(buffer, static_cast<std::size_t>(received), device))
            {
                continue;
            }

            char ipText[INET_ADDRSTRLEN] = {0};
            ::inet_ntop(AF_INET, &from.sin_addr, ipText, sizeof(ipText));
            device.ip = ipText;
            device.lastSeenMs = nowMs();

            std::lock_guard<std::mutex> lock(mutex);
            devices[device.ip] = device;
        }

        ::close(sock);
        std::lock_guard<std::mutex> lock(mutex);
        active = false;
#else
        // Windows：Winsock 版广播监听。绑定 INADDR_ANY 才能收到 255.255.255.255 与子网广播；
        // 注意入站 UDP 受防火墙约束，第一次运行需允许该程序（或放行 UDP 55000）。
        ensureWinsock();
        SOCKET sock = ::socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP);
        if (sock == INVALID_SOCKET)
        {
            std::lock_guard<std::mutex> lock(mutex);
            active = false;
            return;
        }

        int reuse = 1;
        ::setsockopt(sock, SOL_SOCKET, SO_REUSEADDR, reinterpret_cast<const char *>(&reuse),
                     sizeof(reuse));

        DWORD timeout = 300; // 300ms，便于及时退出
        ::setsockopt(sock, SOL_SOCKET, SO_RCVTIMEO, reinterpret_cast<const char *>(&timeout),
                     sizeof(timeout));

        sockaddr_in address{};
        address.sin_family = AF_INET;
        address.sin_addr.s_addr = htonl(INADDR_ANY);
        address.sin_port = htons(port);

        if (::bind(sock, reinterpret_cast<sockaddr *>(&address), sizeof(address)) == SOCKET_ERROR)
        {
            ::closesocket(sock);
            std::lock_guard<std::mutex> lock(mutex);
            active = false;
            return;
        }

        std::uint8_t buffer[2048];
        while (true)
        {
            {
                std::lock_guard<std::mutex> lock(mutex);
                if (stopRequested)
                {
                    break;
                }
            }

            sockaddr_in from{};
            int fromLength = sizeof(from);
            const int received = ::recvfrom(sock, reinterpret_cast<char *>(buffer), sizeof(buffer), 0,
                                            reinterpret_cast<sockaddr *>(&from), &fromLength);
            if (received <= 0)
            {
                continue;
            }

            DiscoveredDevice device;
            if (!parseBroadcast(buffer, static_cast<std::size_t>(received), device))
            {
                continue;
            }

            char ipText[INET_ADDRSTRLEN] = {0};
            ::inet_ntop(AF_INET, &from.sin_addr, ipText, sizeof(ipText));
            device.ip = ipText;
            device.lastSeenMs = nowMs();

            std::lock_guard<std::mutex> lock(mutex);
            devices[device.ip] = device;
        }

        ::closesocket(sock);
        std::lock_guard<std::mutex> lock(mutex);
        active = false;
#endif
    }
};

DeviceDiscovery::DeviceDiscovery() : impl_(std::make_unique<Impl>())
{
}

DeviceDiscovery::~DeviceDiscovery()
{
    stop();
}

bool DeviceDiscovery::start(std::uint16_t port)
{
    stop();

    {
        std::lock_guard<std::mutex> lock(impl_->mutex);
        impl_->devices.clear();
        impl_->stopRequested = false;
        impl_->port = port;
        impl_->active = true;
    }

    impl_->worker = std::thread([this] { impl_->loop(); });
    return true;
}

void DeviceDiscovery::stop()
{
    {
        std::lock_guard<std::mutex> lock(impl_->mutex);
        impl_->stopRequested = true;
    }
    if (impl_->worker.joinable())
    {
        impl_->worker.join();
    }
}

bool DeviceDiscovery::running() const
{
    std::lock_guard<std::mutex> lock(impl_->mutex);
    return impl_->active;
}

std::vector<DiscoveredDevice> DeviceDiscovery::snapshot() const
{
    std::lock_guard<std::mutex> lock(impl_->mutex);
    std::vector<DiscoveredDevice> result;
    result.reserve(impl_->devices.size());
    for (const auto &entry : impl_->devices)
    {
        result.push_back(entry.second);
    }
    std::sort(result.begin(), result.end(),
              [](const DiscoveredDevice &a, const DiscoveredDevice &b) { return a.ip < b.ip; });
    return result;
}

bool DeviceDiscovery::parseBroadcast(const std::uint8_t *data, std::size_t size,
                                     DiscoveredDevice &device)
{
    // 最小长度：11 字节头 + serial + work_mode + 8 字节尾部字段/CRC。
    if (data == nullptr || size < 24 || data[0] != 0xAA)
    {
        return false;
    }

    // serial_number 从偏移 11 开始，取连续可见 ASCII。
    std::string serial;
    constexpr std::size_t kSerialOffset = 11;
    constexpr std::size_t kSerialMaxLength = 24;
    for (std::size_t i = kSerialOffset; i < size && serial.size() < kSerialMaxLength; ++i)
    {
        const std::uint8_t ch = data[i];
        if (ch < 0x20 || ch > 0x7E)
        {
            break;
        }
        serial.push_back(static_cast<char>(ch));
    }
    if (serial.empty())
    {
        return false;
    }

    device.serialNumber = serial;
    device.model = classifyModel(serial);

    // work_mode 紧跟在 serial 之后。
    const std::size_t workModeOffset = kSerialOffset + serial.size();
    if (workModeOffset < size)
    {
        device.workMode = data[workModeOffset];
    }

    // 尾部：[-8..-7] 线数，[-6..-5] 每帧 slot 数，[-4..-1] CRC。
    if (size >= 9)
    {
        const std::uint16_t lines = readLe16(data + size - 8);
        const std::uint16_t slots = readLe16(data + size - 6);
        if (lines > 0 && lines <= 2048)
        {
            device.lineCount = lines;
        }
        if (slots > 0 && slots <= 16384)
        {
            device.slotsPerFrame = slots;
        }
    }
    return true;
}

bool DeviceDiscovery::tcpPortOpen(const std::string &ip, std::uint16_t port, int timeoutMs,
                                  const std::string &localIp)
{
    return tcpPortOpenImpl(ip, port, timeoutMs, localIp);
}

std::vector<DiscoveredDevice>
DeviceDiscovery::probeFl6532(const std::vector<std::string> &candidates, std::string &note)
{
    std::vector<DiscoveredDevice> devices;
    std::set<std::string> visited;
    const std::size_t total = candidates.size();

    for (const std::string &ip : candidates)
    {
        if (ip.empty() || !visited.insert(ip).second)
        {
            continue;
        }
        // 6508 是该型号的控制端口；CS96 Beta 不开放，可用于区分型号。
        if (!tcpPortOpenImpl(ip, 6508, 500, std::string()))
        {
            continue;
        }
        DiscoveredDevice device;
        device.ip = ip;
        device.model = "FL6532";
        device.fromProbe = true;
        device.lastSeenMs = nowMs();
        devices.push_back(device);
    }

    std::ostringstream detail;
    detail << "探测 " << total << " 个已知地址，命中 FL6532 " << devices.size() << " 台";
    note = detail.str();
    return devices;
}

std::vector<std::pair<std::string, std::string>>
DeviceDiscovery::parseArpTable(const std::string &text)
{
    std::vector<std::pair<std::string, std::string>> rows;
    std::istringstream input(text);
    std::string line;
    bool headerSkipped = false;
    while (std::getline(input, line))
    {
        if (!headerSkipped)
        {
            headerSkipped = true; // 首行是 "IP address HW type Flags ..."
            continue;
        }
        std::istringstream fields(line);
        std::string ip;
        std::string hwType;
        std::string flags;
        std::string mac;
        if (!(fields >> ip >> hwType >> flags >> mac))
        {
            continue;
        }
        // 0x2 = ATF_COM：只有完成解析的邻居才有可信 MAC。
        if (flags != "0x2")
        {
            continue;
        }
        if (mac.find(':') == std::string::npos || mac == "00:00:00:00:00:00")
        {
            continue;
        }
        rows.emplace_back(ip, mac);
    }
    return rows;
}

namespace
{

LinkKind kindOf(const AdapterInfo &adapter)
{
    if (adapter.wireless)
    {
        return LinkKind::Wireless;
    }
    // 有线的判据：非无线、且有实际物理链路（描述里带 Ethernet/以太网/Realtek/Intel 等）。
    return LinkKind::Wired;
}

/// CV_SCAN_SUBNETS：额外要扫的 /24，逗号分隔，如 "169.254.7,192.168.1.0/24"。
/// 用于雷达不在本机任何网口 /24 内、又确实同二层直连的场景。
std::vector<std::string> extraScanSubnets()
{
    std::vector<std::string> prefixes;
    const char *value = std::getenv("CV_SCAN_SUBNETS");
    if (value == nullptr)
    {
        return prefixes;
    }
    std::stringstream stream(value);
    std::string item;
    while (std::getline(stream, item, ','))
    {
        std::string normalized;
        for (char ch : item)
        {
            const auto uch = static_cast<unsigned char>(ch);
            if (std::isdigit(uch) || ch == '.')
            {
                normalized.push_back(ch);
            }
        }
        // 只取前三段：所有额外网段都按 /24 处理。
        std::vector<int> parts;
        std::stringstream octets(normalized);
        std::string octet;
        while (std::getline(octets, octet, '.') && parts.size() < 4)
        {
            if (!octet.empty())
            {
                parts.push_back(std::atoi(octet.c_str()));
            }
        }
        if (parts.size() >= 3)
        {
            prefixes.push_back(std::to_string(parts[0]) + "." + std::to_string(parts[1]) + "." +
                               std::to_string(parts[2]));
        }
    }
    return prefixes;
}

std::vector<std::string> subnetHosts(const std::string &prefix)
{
    std::vector<std::string> targets;
    for (int host = 1; host <= 254; ++host)
    {
        targets.push_back(prefix + "." + std::to_string(host));
    }
    return targets;
}

bool macMatches(const std::vector<std::string> &prefixes, const std::string &mac)
{
    if (prefixes.empty())
    {
        return true;
    }
    const std::string lower = toLower(mac);
    for (const std::string &prefix : prefixes)
    {
        if (lower.rfind(prefix, 0) == 0)
        {
            return true;
        }
    }
    return false;
}

} // namespace

std::vector<LocalInterface> DeviceDiscovery::localInterfaces()
{
    std::vector<LocalInterface> result;
    std::vector<AdapterInfo> adapters = enumerateAdapters();
    std::stable_sort(adapters.begin(), adapters.end(),
                     [](const AdapterInfo &a, const AdapterInfo &b) {
                         return static_cast<int>(kindOf(a)) < static_cast<int>(kindOf(b));
                     });
    for (const AdapterInfo &adapter : adapters)
    {
        if (!adapter.up || adapter.virtualAdapter)
        {
            continue;
        }
        LocalInterface entry;
        entry.name = adapter.name;
        entry.description = adapter.description;
        entry.address = adapter.address;
        entry.prefixLength = adapter.prefixLength;
        entry.ifIndex = adapter.ifIndex;
        entry.kind = kindOf(adapter);
        entry.linkLocal = adapter.linkLocal;
        entry.virtualAdapter = adapter.virtualAdapter;
        result.push_back(entry);
    }
    return result;
}

std::vector<DiscoveredDevice> DeviceDiscovery::scanSubnet(const std::string &macPrefix,
                                                          std::string &note)
{
    return scanSubnet(macPrefix, {}, note);
}

std::vector<DiscoveredDevice> DeviceDiscovery::scanSubnet(const std::string &macPrefix,
                                                          const std::vector<std::string> &hints,
                                                          std::string &note)
{
    std::vector<DiscoveredDevice> devices;
    const auto started = std::chrono::steady_clock::now();

    // 前缀允许逗号分隔多个（如 "98:03:8a,e0:be:03"），不同批次雷达的 OUI 可能不同。
    std::vector<std::string> prefixes;
    {
        std::stringstream stream(macPrefix);
        std::string item;
        while (std::getline(stream, item, ','))
        {
            std::string normalized;
            for (char ch : item)
            {
                const auto uch = static_cast<unsigned char>(ch);
                if (!std::isspace(uch))
                {
                    normalized.push_back(static_cast<char>(std::tolower(uch)));
                }
            }
            if (!normalized.empty())
            {
                prefixes.push_back(normalized);
            }
        }
    }

    // 物理网口列表：有线优先，无线其次；Wi-Fi Direct / Hyper-V 等虚拟网口直接跳过
    // （它们同样带 169.254/16 地址，混进来会误导网段判断并拖慢扫描）。
    std::vector<AdapterInfo> adapters;
    for (const AdapterInfo &adapter : enumerateAdapters())
    {
        if (!adapter.up || adapter.virtualAdapter)
        {
            continue;
        }
        adapters.push_back(adapter);
    }
    std::stable_sort(adapters.begin(), adapters.end(),
                     [](const AdapterInfo &a, const AdapterInfo &b) {
                         return static_cast<int>(kindOf(a)) < static_cast<int>(kindOf(b));
                     });

    if (adapters.empty())
    {
        note = "没有可用网口（回环/虚拟网卡已忽略）：请检查网线或无线连接";
        return devices;
    }

    // 已知地址（上次连接 / 环境变量）优先定向探测：可命中与网口不在同一 /24 的设备，
    // 例如本机 169.254.222.6/16、雷达 169.254.7.30 这种同二层不同 /24 的情况。
    std::vector<std::string> knownIps;
    for (const std::string &hint : hints)
    {
        if (!hint.empty() && ipToUint(hint) != 0)
        {
            knownIps.push_back(hint);
        }
    }

    const std::vector<NeighborRow> cache = readNeighborTable();
    std::map<std::string, DiscoveredDevice> found; // 按 IP 去重
    std::map<std::string, std::string> sourceByIp; // 命中设备的网口源地址，供事后确认

    // confirmed=false 表示只从系统邻居表缓存里拿到 MAC，设备可能已经离线；
    // 扫网段/定向探测拿到的都是当次有 ARP 回应的，直接算已确认。
    auto addHit = [&](const std::string &ip, const std::string &mac, const AdapterInfo &adapter,
                      bool confirmed) {
        if (!macMatches(prefixes, mac))
        {
            return false;
        }
        if (found.find(ip) != found.end())
        {
            return false;
        }
        DiscoveredDevice device;
        device.ip = ip;
        device.mac = toLower(mac);
        device.model = "LiDAR";
        device.fromScan = true;
        device.interfaceName = adapter.name;
        device.linkKind = kindOf(adapter);
        device.arpConfirmed = confirmed;
        device.sshReady = tcpPortOpenImpl(ip, 22, 400, adapter.address);
        device.lastSeenMs = nowMs();
        found.emplace(ip, device);
        sourceByIp[ip] = adapter.address;
        return true;
    };

    std::ostringstream ifaceDetail;
    int ifaceIndex = 0;
    for (const AdapterInfo &adapter : adapters)
    {
        if (ifaceIndex++ > 0)
        {
            ifaceDetail << "；";
        }
        ifaceDetail << linkKindName(kindOf(adapter)) << " " << adapter.name << " "
                    << adapter.address << "/" << adapter.prefixLength;
        std::size_t hitCount = 0;

        // 1) 系统邻居表：只要最近有过二层通信，这里就有 MAC，零成本命中。
        for (const NeighborRow &row : cache)
        {
#ifdef _WIN32
            const bool sameIface = row.ifIndex == adapter.ifIndex;
#else
            const bool sameIface = row.interfaceName == adapter.name;
#endif
            if (!sameIface)
            {
                continue;
            }
            if (addHit(row.ip, row.mac, adapter, /*confirmed=*/false))
            {
                ++hitCount;
            }
        }

        // 2) 已知地址定向探测（跨 /24 也能探到）。
        for (const std::string &ip : knownIps)
        {
            if (!sameSubnet(ip, adapter.address, adapter.prefixLength))
            {
                continue;
            }
            if (found.find(ip) != found.end())
            {
                continue;
            }
#ifdef _WIN32
            std::string mac;
            if (arpProbe(ip, adapter.address, mac) && addHit(ip, mac, adapter, /*confirmed=*/true))
            {
                ++hitCount;
            }
#else
            const std::string command = "ping -c 1 -W 1 -n -I " + adapter.name + " " + ip +
                                        " >/dev/null 2>&1";
            if (FILE *pipe = ::popen(command.c_str(), "r"))
            {
                ::pclose(pipe);
            }
            for (const NeighborRow &row : readNeighborTable())
            {
                if (row.interfaceName == adapter.name && row.ip == ip &&
                    addHit(row.ip, row.mac, adapter, /*confirmed=*/true))
                {
                    ++hitCount;
                }
            }
#endif
        }

        // 3) 该网口还没命中才扫网段：ARP 全扫在死网段上要等超时，能省则省。
        if (hitCount == 0)
        {
            std::vector<std::string> targets = sweepTargetsFor(adapter);
            // 额外网段（CV_SCAN_SUBNETS）只在有线口上补扫。
            if (kindOf(adapter) == LinkKind::Wired)
            {
                for (const std::string &extra : extraScanSubnets())
                {
                    const std::vector<std::string> hosts = subnetHosts(extra);
                    targets.insert(targets.end(), hosts.begin(), hosts.end());
                }
            }
#ifdef _WIN32
            for (const auto &row : sendArpSweep(targets, adapter.address))
            {
                if (addHit(row.first, row.second, adapter, /*confirmed=*/true))
                {
                    ++hitCount;
                }
            }
#else
            // ping 只为让内核发出 ARP 请求；ICMP 被挡也不影响邻居表，故不检查返回码。
            const std::string pingCommand = "xargs -P 100 -n 1 sh -c 'ping -c 1 -W 1 -n -I " +
                                            adapter.name + " \"$0\" >/dev/null 2>&1'";
            if (FILE *pipe = ::popen(pingCommand.c_str(), "w"))
            {
                for (const std::string &ip : targets)
                {
                    std::fprintf(pipe, "%s\n", ip.c_str());
                }
                ::pclose(pipe);
            }
            for (const NeighborRow &row : readNeighborTable())
            {
                if (row.interfaceName == adapter.name &&
                    addHit(row.ip, row.mac, adapter, /*confirmed=*/true))
                {
                    ++hitCount;
                }
            }
#endif
        }

        ifaceDetail << " → " << (hitCount > 0 ? "命中 " + std::to_string(hitCount) : std::string("无匹配"));
    }

    // 邻居表里的 MAC 可能来自几分钟前的缓存，设备断电后仍会残留。
    // 对「只靠缓存命中」的条目补一次定向确认：ARP 有回应或端口能连上才算真在线。
    std::size_t unconfirmed = 0;
    for (auto &entry : found)
    {
        if (entry.second.arpConfirmed)
        {
            continue;
        }
        bool alive = entry.second.sshReady;
        if (!alive)
        {
#ifdef _WIN32
            std::string mac;
            alive = arpProbe(entry.first, sourceByIp[entry.first], mac);
#else
            const std::string command = "ping -c 1 -W 1 -n -I " + entry.second.interfaceName + " " +
                                        entry.first + " >/dev/null 2>&1";
            if (FILE *pipe = ::popen(command.c_str(), "r"))
            {
                ::pclose(pipe);
            }
            for (const NeighborRow &row : readNeighborTable())
            {
                if (row.ip == entry.first && row.interfaceName == entry.second.interfaceName)
                {
                    alive = true;
                    break;
                }
            }
#endif
        }
        entry.second.arpConfirmed = alive;
        if (!alive)
        {
            ++unconfirmed;
        }
    }

    for (auto &entry : found)
    {
        devices.push_back(entry.second);
    }

    std::sort(devices.begin(), devices.end(),
              [](const DiscoveredDevice &a, const DiscoveredDevice &b) { return a.ip < b.ip; });

    const auto elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(
                             std::chrono::steady_clock::now() - started)
                             .count();

    std::ostringstream detail;
    detail << "扫描 " << adapters.size() << " 个网口[" << ifaceDetail.str() << "]";
    if (!prefixes.empty())
    {
        detail << "，MAC 前缀 ";
        for (std::size_t i = 0; i < prefixes.size(); ++i)
        {
            detail << (i == 0 ? "" : "+") << prefixes[i];
        }
        detail << " 匹配 " << devices.size() << " 台";
    }
    if (unconfirmed > 0)
    {
        detail << "（其中 " << unconfirmed << " 台仅邻居表缓存命中、未回应 ARP，可能已离线）";
    }
    detail << "，耗时 " << elapsed << " ms";
    note = detail.str();

    return devices;
}

const char *linkKindName(LinkKind kind)
{
    switch (kind)
    {
    case LinkKind::Wired:
        return "有线";
    case LinkKind::Wireless:
        return "无线";
    default:
        return "其他";
    }
}

} // namespace cloud_viewer
