#pragma once

#include "network_access.h"

namespace ys_scene::lidar_sn {

struct Identity
{
    QString mac;
    QString sn;
    QString source;
    QString identity;
    QString model;
    bool mdns_running = false;
    bool simulation = false;
};

struct WorkOrder
{
    QString number;
    QString operator_id;
    QString mac;
    QString sn;
};

enum class WriteDecision { KBlocked, KProvision, KAlreadyWritten };

QString NormalizeSn(const QString &text);
bool ParseIdentity(const lidar_network::Reply &reply, Identity &identity);
WriteDecision CheckTarget(const Identity &identity, const WorkOrder &order);

} // namespace ys_scene::lidar_sn
