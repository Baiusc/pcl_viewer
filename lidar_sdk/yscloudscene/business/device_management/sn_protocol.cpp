#include "sn_protocol.h"

#include <QRegularExpression>

namespace ys_scene::lidar_sn {

QString NormalizeSn(const QString &text)
{
    static const QRegularExpression separated(
        QStringLiteral(R"(\A([A-Z0-9]{7})-([A-Z0-9]{3})-([01])-([0-9]{4})-([0-9A-F]{4})\z)"));
    static const QRegularExpression canonical(QStringLiteral(
        R"(\A(?:CS|MF)[0-9]{2}[A-Z0-9]{3}[A-Z][0-9]{2}[01][0-9]{2}(?:0[1-9]|[1-4][0-9]|5[0-3])[0-9A-F]{4}\z)"));
    QString sn = text.trimmed();
    if (separated.match(sn).hasMatch()) {
        sn.remove('-');
    }
    return canonical.match(sn).hasMatch() && !sn.endsWith("0000") ? sn : QString();
}

bool ParseIdentity(const lidar_network::Reply &reply, Identity &identity)
{
    identity = {};
    const auto &json = reply.snapshot;
    if (!reply.error.isEmpty() || reply.status != 200 || json.value("code") != QJsonValue(200) ||
        json.value("schema_version") != QJsonValue(1) || !json.value("sn").isString() ||
        !json.value("mdns_running").isBool() || (json.contains("simulation") && !json.value("simulation").isBool())) {
        return false;
    }

    const auto mac = lidar_network::NormalizeMac(json.value("mac").toString());
    const auto sn = json.value("sn").toString();
    const auto source = json.value("identity_source").toString();
    const auto name = json.value("identity").toString();
    const auto model = json.value("model").toString();
    const bool sn_valid = !sn.isEmpty() && NormalizeSn(sn) == sn;
    if (mac.isEmpty() || name.isEmpty() || model.isEmpty() ||
        !((source == "mac" && sn.isEmpty()) || (source == "sn" && sn_valid && sn == name))) {
        return false;
    }
    identity = {mac, sn, source, name, model, json.value("mdns_running").toBool(), json.value("simulation").toBool()};
    return true;
}

WriteDecision CheckTarget(const Identity &identity, const WorkOrder &order)
{
    if (order.number.trimmed().isEmpty() || order.operator_id.trimmed().isEmpty() || order.mac.isEmpty() ||
        lidar_network::NormalizeMac(order.mac) != order.mac || identity.mac != order.mac || order.sn.isEmpty() ||
        NormalizeSn(order.sn) != order.sn || !order.sn.startsWith("CS96") ||
        identity.model.compare(QStringLiteral("CS96-beta"), Qt::CaseInsensitive) != 0 || identity.simulation) {
        return WriteDecision::KBlocked;
    }
    if (identity.source == "sn" && identity.sn == order.sn) {
        return WriteDecision::KAlreadyWritten;
    }
    return identity.source == "mac" && identity.sn.isEmpty() ? WriteDecision::KProvision : WriteDecision::KBlocked;
}

} // namespace ys_scene::lidar_sn
