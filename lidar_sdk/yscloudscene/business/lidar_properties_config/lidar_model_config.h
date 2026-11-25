#ifndef YS_SCENE_LIDAR_MODEL_CONFIG_H_
#define YS_SCENE_LIDAR_MODEL_CONFIG_H_

#include "ys_lidar_type.h"

#include <QString>

namespace ys_scene {

// 根据广播信息(LidarInfo)判定设备型号对应的属性配置 key。
// key 同时作为运行时目录 <exe>/lidar_properties_config/<key>/ 下的子目录名。
// 目前支持：CS 系列且 hardware_version 首字符为 B(大小写均可) -> "cs96_beta"；
// 其余型号返回空串（暂无属性配置）。
QString LidarModelConfigKey(const you_see::LidarInfo& node);

// 返回可执行文件同级目录下该型号的属性配置目录：
// <exe>/lidar_properties_config/<model_key>
QString LidarModelConfigDir(const QString& model_key);

} // namespace ys_scene

#endif // YS_SCENE_LIDAR_MODEL_CONFIG_H_
