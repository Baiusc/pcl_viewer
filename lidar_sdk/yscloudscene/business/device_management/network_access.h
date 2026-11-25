#pragma once

#include "network_protocol.h"

#include <QNetworkInterface>

class QNetworkDatagram;

namespace ys_scene::lidar_network {

inline constexpr quint16 kDiscoveryPort = 5354;

struct Announcement
{
    Endpoint endpoint;
    QString mac;
    QString identity;
    QString sn;
};

struct Reply
{
    long status = 0;
    QJsonObject snapshot;
    QString error;
};

QString SourceAddress(const Endpoint &endpoint);
Announcement ParseAnnouncement(const QNetworkDatagram &packet, const QNetworkInterface &nic);
Reply Query(Endpoint endpoint, QString source, QJsonObject request = {}, bool diagnostic = false);

} // namespace ys_scene::lidar_network
