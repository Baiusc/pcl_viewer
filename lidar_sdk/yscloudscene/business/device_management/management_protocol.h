#pragma once
#include <QJsonObject>
#include <QList>
#include <QString>

namespace ys_scene {
struct ManagementSnapshot
{
    QString phase, operation_id, error, model, version;
    int actual = -1, confirmed = -1, desired = -1;
    bool simulation = false;
    QList<int> modes;
};

bool ParseManagementSnapshot(const QJsonObject &object, ManagementSnapshot &snapshot);
QString ManagementModeName(int mode);
QString ManagementOutcome(const ManagementSnapshot &snapshot, int requested);
} // namespace ys_scene
