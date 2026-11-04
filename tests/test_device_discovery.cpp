#include <cstdint>
#include <iostream>
#include <string>

#include "cloud_viewer/stream/DeviceDiscovery.h"

namespace
{

// 实测 CS96 Beta 广播包（UDP 55000，源端口 65000）。
const std::uint8_t kCs96BetaBroadcast[] = {
    0xaa, 0x01, 0x1c, 0x00, 0x02, 0x00, 0x00, 0xd9, 0x60, 0x00, 0x00,
    0x43, 0x53, 0x39, 0x36, 0x30, 0x30, 0x30, 0x42, 0x30, 0x31, 0x32,
    0x36, 0x30, 0x32, 0x30, 0x30, 0x30, 0x31,
    0x02, 0x00, 0x01, 0x00, 0x60, 0x00, 0x20, 0x03,
    0x2a, 0xd9, 0xaf, 0x28};

} // namespace

int main()
{
    cloud_viewer::DiscoveredDevice device;
    if (!cloud_viewer::DeviceDiscovery::parseBroadcast(
            kCs96BetaBroadcast, sizeof(kCs96BetaBroadcast), device))
    {
        std::cerr << "DISCOVERY_TEST_FAIL parse returned false\n";
        return 1;
    }

    const bool ok = device.serialNumber == "CS96000B0126020001" &&
                    device.model == "CS96 Beta" &&
                    device.workMode == 0x02 &&
                    device.lineCount == 96 &&
                    device.slotsPerFrame == 800;
    if (!ok)
    {
        std::cerr << "DISCOVERY_TEST_FAIL serial=" << device.serialNumber
                  << " model=" << device.model
                  << " workMode=" << static_cast<int>(device.workMode)
                  << " lines=" << device.lineCount
                  << " slots=" << device.slotsPerFrame << "\n";
        return 2;
    }

    // 非广播包应被拒绝。
    cloud_viewer::DiscoveredDevice ignored;
    if (cloud_viewer::DeviceDiscovery::parseBroadcast(kCs96BetaBroadcast, 5, ignored))
    {
        std::cerr << "DISCOVERY_TEST_FAIL short packet accepted\n";
        return 3;
    }

    // /proc/net/arp 解析：只保留 flags=0x2 且有真实 MAC 的条目。
    const std::string arpText =
        "IP address       HW type     Flags       HW address            Mask     Device\n"
        "10.42.0.222      0x1         0x2         98:03:8a:7f:32:90     *        eth0\n"
        "10.42.0.218      0x1         0x0         00:00:00:00:00:00     *        eth0\n"
        "10.42.0.229      0x1         0x2         88:ae:dd:a1:45:f1     *        eth0\n"
        "broken line without enough fields\n";
    const auto arpRows = cloud_viewer::DeviceDiscovery::parseArpTable(arpText);
    if (arpRows.size() != 2 || arpRows[0].first != "10.42.0.222" ||
        arpRows[0].second != "98:03:8a:7f:32:90" || arpRows[1].first != "10.42.0.229")
    {
        std::cerr << "DISCOVERY_TEST_FAIL arp rows=" << arpRows.size() << "\n";
        return 4;
    }

    std::cout << "DISCOVERY_TEST_PASS " << device.model << " " << device.serialNumber
              << " " << device.lineCount << "x" << device.slotsPerFrame << std::endl;
    return 0;
}
