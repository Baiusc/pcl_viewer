#pragma once
#include <cstdint>
#include <string>
namespace cloud_viewer {
enum class RadarOutputMode { Cloud, Range };
// TCP 6508 公开控制协议；返回空串表示 ACK 成功，数据模式仍由接收端验证。
std::string setRadarOutputMode(const std::string& ip, std::uint16_t port, RadarOutputMode mode);
}
