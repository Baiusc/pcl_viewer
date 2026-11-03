#include "cloud_viewer/stream/DeviceDiscovery.h"

#include <algorithm>
#include <chrono>
#include <cstring>
#include <map>
#include <mutex>
#include <thread>

#ifndef _WIN32
#include <arpa/inet.h>
#include <netinet/in.h>
#include <sys/socket.h>
#include <sys/time.h>
#include <unistd.h>
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

std::uint16_t readLe16(const std::uint8_t *data)
{
    return static_cast<std::uint16_t>(data[0]) |
           static_cast<std::uint16_t>(static_cast<std::uint16_t>(data[1]) << 8);
}

std::string classifyModel(const std::string &serial)
{
    if (serial.rfind("CS96", 0) == 0)
    {
        // CS96000B...：硬件版本位 B/b 表示 Beta（位置随型号前缀略有变化，取前 12 字符内）。
        const auto version = serial.find_first_of("Bb", 4);
        if (version != std::string::npos && version < 12)
        {
            return "CS96 Beta";
        }
        return "CS96";
    }
    if (serial.find("6532") != std::string::npos || serial.find("FL65") != std::string::npos)
    {
        return "FL6532";
    }
    return "LiDAR";
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

} // namespace cloud_viewer
