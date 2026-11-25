#include "business/firmware_version/firmware_version_codec.h"

#include <QDateTime>

namespace ys_scene {

namespace {

// 字节布局护栏：与固件协议文档 §4 绑定（SDK 头文件中亦有同等断言，此处互为兜底）
static_assert(sizeof(you_see::FirmwareComponentInfoT) == 32, "0x0302 component record must be 32 bytes");
static_assert(sizeof(you_see::FirmwareVersionInfoT) == 200, "0x0302 ack payload must be 200 bytes");

// 板端 0x0302 固定上报 5 条组件记录
constexpr int kMaxComponentCount = 5;

/**
 * @brief 定长 ASCII 字段按首个 NUL 截断
 *
 * 板端以 NUL 结尾、尾部补 0，直接整段转字符串会把补齐字节带进来。
 */
QString Latin1FromNulTerminated(const uint8_t* data, int max_len)
{
    int len = 0;
    while (len < max_len && data[len] != 0) {
        ++len;
    }
    return QString::fromLatin1(reinterpret_cast<const char*>(data), len).trimmed();
}

lumosy::FirmwareComponentInfo ToFirmwareComponentInfo(const you_see::FirmwareComponentInfoT& src)
{
    lumosy::FirmwareComponentInfo dst;
    dst.id = src.id;
    dst.valid = (src.valid != 0);
    dst.major = src.major;
    dst.minor = src.minor;
    dst.patch = src.patch;
    dst.dirty = (src.dirty != 0);
    // sha_len 是有效长度，超过字段容量时按容量截断，避免越界
    dst.sha_len = static_cast<quint8>(qMin<int>(src.sha_len, sizeof(src.git_sha)));
    dst.git_sha = QString::fromLatin1(reinterpret_cast<const char*>(src.git_sha), dst.sha_len);
    // build_time 为 64 位语义拆分（低 32 在 build_time，高 32 在 build_time_hi），
    // 采用无符号拼接，避免有符号 32 位在 2038 回绕
    dst.build_time = (static_cast<quint64>(src.build_time_hi) << 32) | static_cast<quint64>(src.build_time);
    return dst;
}

} // namespace

lumosy::FirmwareVersionInfo ToFirmwareVersionInfo(const you_see::FirmwareVersionInfoT& ack)
{
    lumosy::FirmwareVersionInfo info;
    info.ret_code = ack.ret_code;
    info.ok = (ack.ret_code == 0x00);
    info.component_count = ack.component_count;
    info.fw_major = ack.fw_major;
    info.fw_minor = ack.fw_minor;
    info.fw_patch = ack.fw_patch;
    info.product = Latin1FromNulTerminated(ack.product, sizeof(ack.product));
    info.release_id = Latin1FromNulTerminated(ack.release_id, sizeof(ack.release_id));

    // component_count 来自板端，不可直接用作数组长度：
    // 与固定 5 条取小，异常值不会导致越界
    const int report_count = qMin<int>(ack.component_count, kMaxComponentCount);
    info.components.reserve(report_count);
    for (int i = 0; i < report_count; ++i) {
        info.components.append(ToFirmwareComponentInfo(ack.components[i]));
    }
    return info;
}

QString FormatFirmwareBuildTimeUtc(quint64 unix_seconds)
{
    if (unix_seconds == 0) {
        return QString();
    }
    // 板端上报的是 UTC Unix 秒，统一以 UTC 展示，便于与板端日志 / date -u 对照
    const QDateTime utc_time =
        QDateTime::fromSecsSinceEpoch(static_cast<qint64>(unix_seconds)).toUTC();
    return utc_time.toString(QStringLiteral("yyyy-MM-dd HH:mm:ss")) + QStringLiteral(" UTC");
}

} // namespace ys_scene
