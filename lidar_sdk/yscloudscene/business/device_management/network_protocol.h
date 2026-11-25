#pragma once

#include <QJsonObject>
#include <QString>

namespace ys_scene::lidar_network {

struct Endpoint
{
    QString interface_id;
    QString address;
    int port = 80;
};

struct ConfigResult
{
    QJsonObject value;
    QString error;
};

ConfigResult NormalizeConfig(const QJsonObject& input);
QString NormalizeMac(const QString& mac);
bool ParseIpv4(const QString& text, quint32& value);

enum class Decision { KObserve, KConfirm, KCommitted, KFailed, KStop };

struct Task
{
    Endpoint original;
    QString mac;
    QString identity;
    QString epoch;
    QString previous_operation;
    QJsonObject requested;
    bool simulation = false;
    QString operation;
    bool confirmation_sent = false;

    bool Accept(int status, const QJsonObject& snapshot);
    Decision Evaluate(const Endpoint& endpoint, const QJsonObject& snapshot) const;
};

bool IsSnapshot(const QJsonObject& snapshot);
QJsonObject SetRequest(const QJsonObject& normalized_config);
QJsonObject ConfirmRequest(const QString& operation);

} // namespace ys_scene::lidar_network
