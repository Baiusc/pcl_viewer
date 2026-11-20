#include "cloud_viewer/stream/DeviceDiscovery.h"

#include <algorithm>
#include <atomic>
#include <cctype>
#include <cerrno>
#include <chrono>
#include <cstdio>
#include <cstring>
#include <fstream>
#include <iterator>
#include <map>
#include <mutex>
#include <set>
#include <sstream>
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

/// 本机所有已启用、非回环、掩码为 /24 的网段前缀（如 "10.42.0"）。
std::vector<std::string> localSubnetPrefixes()
{
    std::vector<std::string> prefixes;
#ifdef _WIN32
    ensureWinsock();
    // GetAdaptersAddresses 需要先给定缓冲区，不够时会返回 ERROR_BUFFER_OVERFLOW 并回填所需大小。
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
        return prefixes;
    }

    std::set<std::string> seen;
    for (IP_ADAPTER_ADDRESSES *adapter = addrs; adapter != nullptr; adapter = adapter->Next)
    {
        if (adapter->IfType == IF_TYPE_SOFTWARE_LOOPBACK || adapter->OperStatus != IfOperStatusUp)
        {
            continue;
        }
        for (IP_ADAPTER_UNICAST_ADDRESS *unicast = adapter->FirstUnicastAddress; unicast != nullptr;
             unicast = unicast->Next)
        {
            if (unicast->Address.lpSockaddr == nullptr ||
                unicast->Address.lpSockaddr->sa_family != AF_INET)
            {
                continue;
            }
            // 只扫 /24：更宽的网段要探测的地址数量不可控。
            if (unicast->OnLinkPrefixLength != 24)
            {
                continue;
            }
            const auto *address = reinterpret_cast<const sockaddr_in *>(unicast->Address.lpSockaddr);
            char text[INET_ADDRSTRLEN] = {0};
            if (::inet_ntop(AF_INET, &address->sin_addr, text, sizeof(text)) == nullptr)
            {
                continue;
            }
            const std::string value(text);
            const auto dot = value.find_last_of('.');
            if (dot == std::string::npos)
            {
                continue;
            }
            const std::string prefix = value.substr(0, dot);
            if (seen.insert(prefix).second)
            {
                prefixes.push_back(prefix);
            }
        }
    }
#else
    ifaddrs *list = nullptr;
    if (::getifaddrs(&list) != 0)
    {
        return prefixes;
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
        if ((it->ifa_flags & IFF_LOOPBACK) != 0 || (it->ifa_flags & IFF_UP) == 0)
        {
            continue;
        }
        const auto *address = reinterpret_cast<const sockaddr_in *>(it->ifa_addr);
        const auto *mask = reinterpret_cast<const sockaddr_in *>(it->ifa_netmask);
        // 只扫 /24：更宽的网段要 ping 的地址数量不可控。
        if (ntohl(mask->sin_addr.s_addr) != 0xFFFFFF00u)
        {
            continue;
        }
        char text[INET_ADDRSTRLEN] = {0};
        if (::inet_ntop(AF_INET, &address->sin_addr, text, sizeof(text)) == nullptr)
        {
            continue;
        }
        const std::string value(text);
        const auto dot = value.find_last_of('.');
        if (dot == std::string::npos)
        {
            continue;
        }
        prefixes.push_back(value.substr(0, dot));
    }
    ::freeifaddrs(list);
#endif
    return prefixes;
}

#ifdef _WIN32
/// 对本机 /24 网段做一轮 SendARP 扫描，返回 (ip, mac)。
/// SendARP 走二层解析、不经过防火墙，适合作为「广播被拦截」时的兜底发现手段。
std::vector<std::pair<std::string, std::string>>
sendArpSweep(const std::vector<std::string> &subnets)
{
    std::vector<std::string> targets;
    for (const std::string &subnet : subnets)
    {
        for (int host = 1; host <= 254; ++host)
        {
            targets.push_back(subnet + "." + std::to_string(host));
        }
    }

    std::vector<std::pair<std::string, std::string>> rows;
    if (targets.empty())
    {
        return rows;
    }

    std::mutex mutex;
    std::atomic<std::size_t> next{0};
    // 并发度 32：ARP 请求本身很快，避免 254 个地址串行等待。
    const std::size_t workerCount = std::min<std::size_t>(32, targets.size());
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
                const std::string &ip = targets[index];
                in_addr address{};
                if (::inet_pton(AF_INET, ip.c_str(), &address) != 1)
                {
                    continue;
                }
                ULONG mac[2] = {0, 0};
                ULONG length = sizeof(mac);
                if (::SendARP(address.s_addr, 0, mac, &length) != NO_ERROR || length < 6)
                {
                    continue;
                }
                const auto *bytes = reinterpret_cast<const unsigned char *>(mac);
                const std::string macText = macToString(bytes);
                if (macText == "00:00:00:00:00:00")
                {
                    continue;
                }
                std::lock_guard<std::mutex> lock(mutex);
                rows.emplace_back(ip, macText);
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
bool tcpPortOpenImpl(const std::string &ip, std::uint16_t port, int timeoutMs)
{
#ifndef _WIN32
    const int fd = ::socket(AF_INET, SOCK_STREAM, 0);
    if (fd < 0)
    {
        return false;
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

bool DeviceDiscovery::tcpPortOpen(const std::string &ip, std::uint16_t port, int timeoutMs)
{
    return tcpPortOpenImpl(ip, port, timeoutMs);
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
        if (!tcpPortOpenImpl(ip, 6508, 500))
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

std::vector<DiscoveredDevice> DeviceDiscovery::scanSubnet(const std::string &macPrefix,
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

    const std::vector<std::string> subnets = localSubnetPrefixes();
    if (subnets.empty())
    {
        note = "没有可扫描的 /24 IPv4 网段";
        return devices;
    }

    std::vector<std::pair<std::string, std::string>> rows;
#ifndef _WIN32
    // ping 只为让内核发出 ARP 请求；ICMP 被防火墙挡住也不影响邻居表，故不检查返回码。
    for (const std::string &subnet : subnets)
    {
        const std::string command =
            "for i in $(seq 1 254); do echo " + subnet + ".$i; done"
            " | xargs -P 100 -n 1 sh -c 'ping -c 1 -W 1 -n \"$0\" >/dev/null 2>&1'";
        if (FILE *pipe = ::popen(command.c_str(), "r"))
        {
            ::pclose(pipe);
        }
    }

    std::ifstream arp("/proc/net/arp");
    const std::string text((std::istreambuf_iterator<char>(arp)),
                           std::istreambuf_iterator<char>());
    rows = parseArpTable(text);
#else
    // Windows：/proc 不存在，且防火墙会丢弃入站广播；用 SendARP 直接取二层 MAC，
    // 既不受防火墙影响，也不需要先 ping 一遍。
    rows = sendArpSweep(subnets);
#endif

    for (const auto &row : rows)
    {
        std::string mac = row.second;
        std::transform(mac.begin(), mac.end(), mac.begin(),
                       [](unsigned char ch) { return static_cast<char>(std::tolower(ch)); });
        if (!prefixes.empty())
        {
            bool matched = false;
            for (const std::string &prefix : prefixes)
            {
                if (mac.rfind(prefix, 0) == 0)
                {
                    matched = true;
                    break;
                }
            }
            if (!matched)
            {
                continue;
            }
        }
        DiscoveredDevice device;
        device.ip = row.first;
        device.mac = row.second;
        device.model = "LiDAR";
        device.fromScan = true;
        device.sshReady = tcpPortOpen(device.ip, 22, 400);
        device.lastSeenMs = nowMs();
        devices.push_back(device);
    }

    std::sort(devices.begin(), devices.end(),
              [](const DiscoveredDevice &a, const DiscoveredDevice &b) { return a.ip < b.ip; });

    const auto elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(
                             std::chrono::steady_clock::now() - started)
                             .count();

    std::ostringstream detail;
    detail << "扫描 " << subnets.size() << " 个网段(";
    for (std::size_t i = 0; i < subnets.size(); ++i)
    {
        detail << (i == 0 ? "" : ",") << subnets[i] << ".0/24";
    }
    detail << ")，ARP 邻居 " << rows.size() << " 台";
    if (!prefixes.empty())
    {
        detail << "，MAC 前缀 ";
        for (std::size_t i = 0; i < prefixes.size(); ++i)
        {
            detail << (i == 0 ? "" : "+") << prefixes[i];
        }
        detail << " 匹配 " << devices.size() << " 台";
    }
    detail << "，耗时 " << elapsed << " ms";
    note = detail.str();

    return devices;
}

} // namespace cloud_viewer
