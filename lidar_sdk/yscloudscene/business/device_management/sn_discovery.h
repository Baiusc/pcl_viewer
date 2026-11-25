#pragma once

#include "network_protocol.h"

namespace ys_scene::lidar_sn {

struct DiscoveryCheck
{
    QString host;
    bool conflicting_address = false;
};

DiscoveryCheck VerifyDiscovery(const lidar_network::Endpoint &endpoint, const QString &sn);

} // namespace ys_scene::lidar_sn
