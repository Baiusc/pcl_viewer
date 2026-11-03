#include <cmath>
#include <cstdint>
#include <cstring>
#include <iostream>
#include <vector>
#ifndef _WIN32
#include <arpa/inet.h>
#include <sys/socket.h>
#include <unistd.h>
#include <chrono>
#include <thread>
#endif

#include "cloud_viewer/stream/BetaUdpStream.h"

#ifndef _WIN32
static bool testSourceFilter(const std::vector<std::uint8_t> &packet)
{
    using namespace std::chrono_literals;
    int reserve = socket(AF_INET, SOCK_DGRAM, 0);
    if (reserve < 0) return false;
    sockaddr_in local{}; local.sin_family = AF_INET;
    local.sin_addr.s_addr = htonl(INADDR_LOOPBACK);
    if (bind(reserve, reinterpret_cast<sockaddr *>(&local), sizeof(local)) != 0)
    { close(reserve); return false; }
    socklen_t len = sizeof(local);
    if (getsockname(reserve, reinterpret_cast<sockaddr *>(&local), &len) != 0)
    { close(reserve); return false; }
    const auto port = ntohs(local.sin_port); close(reserve);
    cloud_viewer::BetaUdpStream stream;
    cloud_viewer::BetaUdpStream::Config config;
    config.radarIp = "127.0.0.1"; config.localPort = port;
    std::string error;
    if (!stream.open(config, {}, error)) return false;
    std::this_thread::sleep_for(200ms);
    auto sendFrom = [&](const char *ip) {
        int fd = socket(AF_INET, SOCK_DGRAM, 0);
        if (fd < 0) return false;
        sockaddr_in from{}; from.sin_family = AF_INET;
        inet_pton(AF_INET, ip, &from.sin_addr);
        if (bind(fd, reinterpret_cast<sockaddr *>(&from), sizeof(from)) != 0)
        { close(fd); return false; }
        for (int i = 0; i < 10; ++i)
            if (sendto(fd, packet.data(), packet.size(), 0,
                       reinterpret_cast<sockaddr *>(&local), sizeof(local)) < 0)
            { close(fd); return false; }
        close(fd); return true;
    };
    const bool sentForeign = sendFrom("127.0.0.2");
    std::this_thread::sleep_for(200ms);
    const auto foreign = stream.stats().packets;
    const bool sentSelected = sendFrom("127.0.0.1");
    std::this_thread::sleep_for(200ms);
    const auto selected = stream.stats().packets;
    stream.stop();
    std::cout << "SOURCE_FILTER foreign=" << foreign << " selected=" << selected << "\n";
    return sentForeign && sentSelected && foreign == 0 && selected == 10;
}
#endif

int main()
{
    // 构造一个与实测格式一致的 1266B 包：18B 头 + 96 × 13B 点。
    std::vector<std::uint8_t> packet(cloud_viewer::BetaUdpStream::kCartesianPacketSize, 0);
    // 真实点云包头的 frame 字段是面号（0~3），解析器会拒绝大于 3 的值。
    const std::uint16_t frameIndex = 2;
    const std::uint16_t slot = 45;
    const std::uint16_t totalSlots = 800;

    std::memcpy(packet.data() + 10, &frameIndex, 2);
    std::memcpy(packet.data() + 12, &totalSlots, 2);
    std::memcpy(packet.data() + 14, &slot, 2);

    for (int i = 0; i < 96; ++i)
    {
        std::uint8_t *raw = packet.data() + 18 + i * 13;
        const float x = 1.5f;
        const float y = 2.5f;
        const float z = 3.5f;
        std::memcpy(raw, &x, 4);
        std::memcpy(raw + 4, &y, 4);
        std::memcpy(raw + 8, &z, 4);
        raw[12] = 100;
    }

    std::uint16_t outFrame = 0;
    std::uint16_t outSlot = 0;
    algo_lidar::PointData points[96];
    int count = 0;

    if (!cloud_viewer::BetaUdpStream::parsePacket(packet.data(), packet.size(),
                                                  outFrame, outSlot, points, count))
    {
        std::cerr << "BETA_TEST_FAIL parse returned false\n";
        return 1;
    }

    const bool ok = outFrame == frameIndex && outSlot == slot && count == 96 &&
                    std::fabs(points[0].x - 1.5f) < 1e-5 &&
                    std::fabs(points[0].y - 2.5f) < 1e-5 &&
                    std::fabs(points[0].z - 3.5f) < 1e-5 &&
                    points[0].intensity == 100 && points[0].is_valid;
    if (!ok)
    {
        std::cerr << "BETA_TEST_FAIL frame=" << outFrame << " slot=" << outSlot
                  << " count=" << count << " x=" << points[0].x
                  << " intensity=" << static_cast<int>(points[0].intensity) << "\n";
        return 2;
    }

    // 错误长度应被拒绝。
    if (cloud_viewer::BetaUdpStream::parsePacket(packet.data(), 100, outFrame, outSlot,
                                                 points, count))
    {
        std::cerr << "BETA_TEST_FAIL short packet accepted\n";
        return 3;
    }

#ifndef _WIN32
    if (!testSourceFilter(packet))
    {
        std::cerr << "BETA_TEST_FAIL source filtering\n";
        return 4;
    }
#endif
    std::cout << "BETA_TEST_PASS frame=" << outFrame << " slot=" << outSlot << std::endl;
    return 0;
}
