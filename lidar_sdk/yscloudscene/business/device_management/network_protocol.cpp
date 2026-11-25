#include "network_protocol.h"

#include <QHostAddress>
#include <QRegularExpression>
#include <QStringList>

namespace ys_scene::lidar_network {
namespace {

QString Format(quint32 value)
{
    return QHostAddress(value).toString();
}

bool Overlaps(quint32 first, quint32 last, quint32 low, quint32 high)
{
    return first <= high && last >= low;
}

bool AllowedRange(quint32 first, quint32 last)
{
    return first >= 0x01000000u && last < 0xe0000000u &&
           !Overlaps(first, last, 0x7f000000u, 0x7fffffffu) &&
           !Overlaps(first, last, 0xa9fe0000u, 0xa9feffffu);
}

bool Host(quint32 ip, quint32 mask)
{
    const quint32 host = ip & ~mask;
    return AllowedRange(ip, ip) && host != 0 && host != ~mask;
}

bool Token(const QString& text)
{
    static const QRegularExpression token(QStringLiteral("^[0-9a-f]{32}$"));
    return token.match(text).hasMatch();
}

bool Integer(const QJsonValue& value, int minimum, int maximum)
{
    return value.isDouble() && value.toDouble() >= minimum && value.toDouble() <= maximum &&
           value.toDouble() == value.toInt();
}

bool ConfigMatches(const QJsonValue& input, const QJsonObject& expected)
{
    const auto normalized = NormalizeConfig(input.toObject());
    return normalized.error.isEmpty() && normalized.value == expected;
}

bool AddressMatches(const Endpoint& endpoint, const QJsonObject& snapshot, const QJsonObject& requested)
{
    quint32 address;
    quint32 connected;
    if (!ParseIpv4(snapshot.value("address").toString(), address) ||
        !ParseIpv4(endpoint.address, connected) || address != connected) {
        return false;
    }
    const auto mode = requested.value("mode").toString();
    const auto source = snapshot.value("source").toString();
    if (mode == "auto") {
        quint32 mask;
        return (source == "dhcp" && AllowedRange(address, address)) ||
               (source == "linklocal" && address >= 0xa9fe0100u && address <= 0xa9fefeffu &&
                ParseIpv4(snapshot.value("netmask").toString(), mask) && mask == 0xffff0000u);
    }

    if (!snapshot.value("netmask").isString() || !snapshot.value("gateway").isString()) {
        return false;
    }
    QJsonObject actual = requested;
    actual.insert("netmask", snapshot.value("netmask"));
    actual.insert("gateway", snapshot.value("gateway"));
    if (source != mode || !ConfigMatches(actual, requested)) {
        return false;
    }
    quint32 first = 0;
    quint32 last = 0;
    quint32 gateway = 0;
    if (mode == "static") {
        return ParseIpv4(requested.value("address").toString(), first) && address == first;
    }
    ParseIpv4(requested.value("gateway").toString(), gateway);
    return mode == "pool" && ParseIpv4(requested.value("start").toString(), first) &&
           ParseIpv4(requested.value("end").toString(), last) && address >= first &&
           address <= last && address != gateway;
}

bool SameDevice(const Task& task, const QJsonObject& snapshot)
{
    return NormalizeMac(snapshot.value("mac").toString()) == task.mac &&
           snapshot.value("identity").toString() == task.identity &&
           snapshot.value("epoch").toString() == task.epoch &&
           snapshot.value("simulation").toBool() == task.simulation;
}

} // namespace

bool ParseIpv4(const QString& text, quint32& value)
{
    const auto parts = text.trimmed().split('.');
    if (parts.size() != 4) {
        return false;
    }
    quint32 result = 0;
    for (const auto& part : parts) {
        if (part.isEmpty() || part.size() > 3) {
            return false;
        }
        quint32 octet = 0;
        for (QChar digit : part) {
            if (digit < '0' || digit > '9') {
                return false;
            }
            octet = octet * 10 + digit.unicode() - '0';
        }
        if (octet > 255) {
            return false;
        }
        result = (result << 8) | octet;
    }
    value = result;
    return true;
}

QString NormalizeMac(const QString& mac)
{
    static const QRegularExpression pattern(QStringLiteral("\\A[0-9a-f]{2}([:-])[0-9a-f]{2}(?:\\1[0-9a-f]{2}){4}\\z"),
                                            QRegularExpression::CaseInsensitiveOption);
    return pattern.match(mac).hasMatch() ? QString(mac).replace('-', ':').toLower() : QString();
}

ConfigResult NormalizeConfig(const QJsonObject& input)
{
    const auto mode = input.value("mode").toString();
    if (mode == "auto" && input.size() == 1) {
        return {QJsonObject{{"mode", mode}}, {}};
    }
    const bool pool = mode == "pool";
    QStringList allowed{"mode", "netmask", "gateway"};
    if (pool) {
        allowed << "start" << "end";
    } else {
        allowed << "address";
    }
    if ((!pool && mode != "static") || !input.contains("netmask")) {
        return {{}, QStringLiteral("网络模式或掩码无效")};
    }
    for (auto it = input.begin(); it != input.end(); ++it) {
        if (!allowed.contains(it.key()) || !it.value().isString()) {
            return {{}, QStringLiteral("配置包含不支持的字段或类型")};
        }
    }

    quint32 mask;
    if (!ParseIpv4(input.value("netmask").toString(), mask) || mask == 0 || ~mask < 3 ||
        (~mask & (~mask + 1)) != 0) {
        return {{}, QStringLiteral("请输入 /1 到 /30 的连续 IPv4 掩码")};
    }
    quint32 first;
    quint32 last;
    if (!ParseIpv4(input.value(pool ? "start" : "address").toString(), first) ||
        !ParseIpv4(input.value(pool ? "end" : "address").toString(), last) || first > last ||
        (first & mask) != (last & mask) || !Host(first, mask) || !Host(last, mask) ||
        !AllowedRange(first, last) || quint64(last) - first + 1 > 65534) {
        return {{}, QStringLiteral("地址或地址范围无效，请检查子网、主机地址和范围大小")};
    }
    quint32 gateway = 0;
    const auto gateway_text = input.value("gateway").toString().trimmed();
    if ((!gateway_text.isEmpty() && !ParseIpv4(gateway_text, gateway)) ||
        (gateway != 0 && (!Host(gateway, mask) || (gateway & mask) != (first & mask) ||
                          (first == last && first == gateway)))) {
        return {{}, QStringLiteral("网关必须是同一子网的有效主机，且不能占用唯一候选地址")};
    }
    QJsonObject result{{"mode", mode}, {"netmask", Format(mask)}, {"gateway", gateway ? Format(gateway) : QString()}};
    if (pool) {
        result.insert("start", Format(first));
        result.insert("end", Format(last));
    } else {
        result.insert("address", Format(first));
    }
    return {result, {}};
}

bool IsSnapshot(const QJsonObject& snapshot)
{
    for (const auto* key : {"simulation", "ready", "recovery", "configuration_valid", "pending"}) {
        if (!snapshot.value(key).isBool()) {
            return false;
        }
    }
    for (const auto* key : {"phase", "transaction_phase", "source", "address", "operation_id", "identity"}) {
        if (!snapshot.value(key).isString()) {
            return false;
        }
    }
    quint32 address;
    const auto ip = snapshot.value("address").toString();
    const auto operation = snapshot.value("operation_id").toString();
    return snapshot.value("schema_version") == QJsonValue(1) &&
           !NormalizeMac(snapshot.value("mac").toString()).isEmpty() &&
           Token(snapshot.value("epoch").toString()) && (operation.isEmpty() || Token(operation)) &&
           (ip.isEmpty() || ParseIpv4(ip, address)) &&
           Integer(snapshot.value("operation_code"), 0, 599) &&
           Integer(snapshot.value("confirm_remaining_ms"), 0, 2147483647) &&
           snapshot.value("confirmed_config").isObject() && snapshot.value("desired_config").isObject();
}

bool Task::Accept(int status, const QJsonObject& snapshot)
{
    const auto id = snapshot.value("operation_id").toString();
    if (!operation.isEmpty() || status != 202 || !IsSnapshot(snapshot) || !SameDevice(*this, snapshot) ||
        !Token(id) || id == previous_operation || !snapshot.value("pending").toBool() ||
        snapshot.value("operation_code") != QJsonValue(202) ||
        !ConfigMatches(snapshot.value("desired_config"), requested)) {
        return false;
    }
    operation = id;
    return true;
}

Decision Task::Evaluate(const Endpoint& endpoint, const QJsonObject& snapshot) const
{
    if (operation.isEmpty()) {
        return Decision::KObserve;
    }
    if (!IsSnapshot(snapshot) || endpoint.interface_id != original.interface_id ||
        endpoint.port != original.port || !SameDevice(*this, snapshot) ||
        snapshot.value("operation_id").toString() != operation) {
        return Decision::KStop;
    }
    const auto phase = snapshot.value("transaction_phase").toString();
    const bool pending = snapshot.value("pending").toBool();
    if (phase == "FAILED" || phase == "ROLLED_BACK" || snapshot.value("operation_code") == QJsonValue(408)) {
        return Decision::KFailed;
    }
    if (phase == "PROBING" || phase == "PREPARED" || phase == "APPLY_DELAY") {
        return Decision::KObserve;
    }
    if (!snapshot.value("configuration_valid").toBool() || snapshot.value("recovery").toBool() ||
        !snapshot.value("ready").toBool() || snapshot.value("phase").toString() != "READY" ||
        !AddressMatches(endpoint, snapshot, requested)) {
        return Decision::KStop;
    }
    if (phase == "WAIT_CONFIRM" && pending && snapshot.value("operation_code") == QJsonValue(202) &&
        snapshot.value("confirm_remaining_ms").toInt() > 0 &&
        ConfigMatches(snapshot.value("desired_config"), requested)) {
        return confirmation_sent ? Decision::KObserve : Decision::KConfirm;
    }
    if (phase == "COMMITTED" && !pending && snapshot.value("operation_code") == QJsonValue(200) &&
        ConfigMatches(snapshot.value("confirmed_config"), requested)) {
        return Decision::KCommitted;
    }
    return Decision::KStop;
}

QJsonObject SetRequest(const QJsonObject& normalized_config)
{
    return {{"op", "network_set"}, {"config", normalized_config}};
}

QJsonObject ConfirmRequest(const QString& operation)
{
    return Token(operation) ? QJsonObject{{"op", "network_confirm"}, {"operation_id", operation}} : QJsonObject{};
}

} // namespace ys_scene::lidar_network
