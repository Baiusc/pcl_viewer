#ifndef YS_SCENE_FIRMWARE_VERSION_CODEC_H_
#define YS_SCENE_FIRMWARE_VERSION_CODEC_H_

#include <QString>

#include "lumomo_ui/firmware_version/firmware_version_info.h"
#include "ys_lidar_type.h"

namespace ys_scene {

/**
 * @brief 0x0302 系统固件版本应答(C 结构体) → 面板展示用的 Qt 结构
 *
 * 完成定长 ASCII 字段的 NUL 截断、git_sha 按 sha_len 截断、
 * build_time 高低 32 位拼接，以及 component_count 的越界防护。
 *
 * @param ack 板端应答载荷(200 字节，见 FirmwareVersionInfoT)
 * @return 解析后的固件版本信息
 */
lumosy::FirmwareVersionInfo ToFirmwareVersionInfo(const you_see::FirmwareVersionInfoT& ack);

/**
 * @brief 64 位构建时刻(UTC Unix 秒) → "yyyy-MM-dd HH:mm:ss UTC"
 *
 * @param unix_seconds 构建时刻，0 表示未知
 * @return 格式化后的 UTC 时间串；入参为 0 时返回空串
 */
QString FormatFirmwareBuildTimeUtc(quint64 unix_seconds);

} // namespace ys_scene

#endif // YS_SCENE_FIRMWARE_VERSION_CODEC_H_
