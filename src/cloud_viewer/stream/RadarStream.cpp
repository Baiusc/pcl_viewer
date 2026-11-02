#include "cloud_viewer/stream/RadarStream.h"

namespace cloud_viewer
{

RadarBackendFactory &RadarBackendFactory::instance()
{
    static RadarBackendFactory factory;
    return factory;
}

RadarBackendFactory::RadarBackendFactory()
{
    auto add = [this](const char *id, const char *name, bool available, bool isMock)
    {
        BackendDescriptor descriptor;
        descriptor.id = id;
        descriptor.displayName = name;
        descriptor.available = available;
        descriptor.isMock = isMock;
        backends_.push_back(std::move(descriptor));
    };

#ifdef CLOUD_VIEWER_WITH_ALPHA_SDK
    add(kAlphaSdk, "alpha 转镜雷达 SDK", true, false);
#else
    add(kAlphaSdk, "alpha 转镜雷达 SDK", false, false);
#endif

#ifdef CLOUD_VIEWER_WITH_BETA_SDK
    add(kBetaSdk, "beta 薄款转镜雷达 SDK", true, false);
#else
    add(kBetaSdk, "beta 薄款转镜雷达 SDK", false, false);
#endif

#ifdef CLOUD_VIEWER_WITH_FL6031_SDK
    add(kFl6031Sdk, "FL6031 SDK", true, false);
#else
    add(kFl6031Sdk, "FL6031 SDK", false, false);
#endif

#ifdef CLOUD_VIEWER_WITH_FL6532_SDK
    add(kFl6532Sdk, "FL6532 SDK", true, false);
#else
    add(kFl6532Sdk, "FL6532 SDK", false, false);
#endif

    add(kMockReplay, "Mock 回放（离线帧循环）", true, true);
    add(kMockSynthetic, "Mock 合成（固定帧率）", true, true);
}

std::vector<BackendDescriptor> RadarBackendFactory::backends() const
{
    return backends_;
}

const BackendDescriptor *RadarBackendFactory::find(const std::string &id) const
{
    for (const auto &backend : backends_)
    {
        if (backend.id == id)
        {
            return &backend;
        }
    }
    return nullptr;
}

bool RadarBackendFactory::isAvailable(const std::string &id) const
{
    const BackendDescriptor *backend = find(id);
    return backend != nullptr && backend->available;
}

} // namespace cloud_viewer
