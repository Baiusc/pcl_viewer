#include <cmath>
#include <cstdint>
#include <cstring>
#include <iostream>
#include <vector>

#include "cloud_viewer/stream/BetaUdpStream.h"

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

    std::cout << "BETA_TEST_PASS frame=" << outFrame << " slot=" << outSlot << std::endl;
    return 0;
}
