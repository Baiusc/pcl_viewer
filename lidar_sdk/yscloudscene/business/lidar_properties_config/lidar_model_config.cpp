#include "business/lidar_properties_config/lidar_model_config.h"

#include <QCoreApplication>
#include <QString>

namespace ys_scene {

QString LidarModelConfigKey(const you_see::LidarInfo& node)
{
    if (node.model != you_see::LidarModel::KCS) {
        return QString();
    }

    const uint8_t first_hw = node.hardware_version[0];
    if (first_hw == 'B' || first_hw == 'b') {
        return QStringLiteral("cs96_beta");
    }

    return QString();
}

QString LidarModelConfigDir(const QString& model_key)
{
    return QCoreApplication::applicationDirPath()
           + QStringLiteral("/lidar_properties_config/")
           + model_key;
}

} // namespace ys_scene
