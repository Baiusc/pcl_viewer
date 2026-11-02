/**
 * @brief 在线雷达后端注册与查询
 *
 * 真实 SDK 名称/函数签名由厂商提供，本文不杜撰。
 * 未启用的后端返回 BackendUnavailable，不自动降级为 mock。
 */
#pragma once

#include <string>
#include <vector>

namespace cloud_viewer
{

struct BackendDescriptor
{
    std::string id;
    std::string displayName;
    bool available = false;
    bool isMock = false;
};

class RadarBackendFactory
{
public:
    static RadarBackendFactory &instance();

    std::vector<BackendDescriptor> backends() const;

    /// 查找后端描述；未知 id 返回 nullptr。
    const BackendDescriptor *find(const std::string &id) const;

    bool isAvailable(const std::string &id) const;

    static constexpr const char *kAlphaSdk = "alpha_sdk";
    static constexpr const char *kBetaSdk = "beta_sdk";
    static constexpr const char *kFl6031Sdk = "fl6031_sdk";
    static constexpr const char *kFl6532Sdk = "fl6532_sdk";
    static constexpr const char *kMockReplay = "mock_replay";
    static constexpr const char *kMockSynthetic = "mock_synthetic";

private:
    RadarBackendFactory();
    std::vector<BackendDescriptor> backends_;
};

} // namespace cloud_viewer
