#include "management_protocol.h"
#include <QJsonArray>
#include <cmath>
#include <limits>

namespace ys_scene {
namespace {
bool Integer(const QJsonValue &value)
{
    const double number = value.toDouble();
    return value.isDouble() && std::isfinite(number) && std::floor(number) == number &&
           number >= std::numeric_limits<int>::min() && number <= std::numeric_limits<int>::max();
}
} // namespace

bool ParseManagementSnapshot(const QJsonObject &o, ManagementSnapshot &s)
{
    for (const auto *key : {"code", "actual_mode", "confirmed_mode", "desired_mode", "pid"})
        if (!Integer(o[key]))
            return false;
    for (const auto *key : {"message", "model", "operation_id", "phase", "error"})
        if (!o[key].isString())
            return false;
    for (const auto *key : {"simulation", "initialized", "pipeline_verified"})
        if (!o[key].isBool())
            return false;
    if ((!o["version"].isString() && !o["version"].isNull()) || !o.contains("version") ||
        !o["supported_modes"].isArray() || !o["validated_work_modes"].isArray() ||
        o["operation_id"].toString().isEmpty() || o["pid"].toInt() < 0)
        return false;
    const auto groups = o["supported_modes"].toArray();
    for (const auto &group : groups)
        if (!group.isString())
            return false;
    ManagementSnapshot next;
    for (const auto &mode : o["validated_work_modes"].toArray()) {
        if (!Integer(mode))
            return false;
        const int n = mode.toInt();
        if (n < 0 || n > 3)
            continue;
        const QString group = n == 3 ? "gray" : (n == 0 ? "histogram" : "pointcloud");
        if (groups.contains(group) && !next.modes.contains(n))
            next.modes.append(n);
    }
    next.phase = o["phase"].toString();
    next.operation_id = o["operation_id"].toString();
    next.error = o["error"].toString();
    next.model = o["model"].toString();
    next.version = o["version"].isNull() ? QStringLiteral("未标记版本") : o["version"].toString();
    next.actual = o["actual_mode"].toInt();
    next.confirmed = o["confirmed_mode"].toInt();
    next.desired = o["desired_mode"].toInt();
    next.simulation = o["simulation"].toBool();
    s = next;
    return true;
}

QString ManagementModeName(int mode)
{
    const QStringList names = {QStringLiteral("连续原始直方图"), QStringLiteral("原始点云"), QStringLiteral("解算点云"),
                               QStringLiteral("灰度图")};
    if (mode == -1)
        return QStringLiteral("切换中 / 未确认");
    return mode >= 0 && mode < names.size() ? names[mode] : QStringLiteral("未知模式（%1）").arg(mode);
}

QString ManagementOutcome(const ManagementSnapshot &s, int requested)
{
    if (s.phase == "FAULT")
        return QStringLiteral("设备故障：%1").arg(s.error);
    if (s.phase != "COMMITTED")
        return {};
    if (s.actual == requested && s.confirmed == requested)
        return s.error.isEmpty() ? QStringLiteral("切换成功")
                                 : QStringLiteral("已达到目标，但发生异常：%1").arg(s.error);
    return QStringLiteral("目标切换未成功，当前为%1：%2").arg(ManagementModeName(s.actual), s.error);
}
} // namespace ys_scene
