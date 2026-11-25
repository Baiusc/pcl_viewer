#include "ip_utils.h"
#include <QStringList>

namespace lumosy {
QString IPUtils::Uint32ToIPString(uint32_t ip_val)
{
    unsigned char b1 = (ip_val) & 0xFF;
    unsigned char b2 = (ip_val >> 8) & 0xFF;
    unsigned char b3 = (ip_val >> 16) & 0xFF;
    unsigned char b4 = (ip_val >> 24) & 0xFF;

    return QString("%1.%2.%3.%4").arg(b1).arg(b2).arg(b3).arg(b4);
}

uint32_t IPUtils::IPStringToUint32(const QString &ip_str)
{
    QStringList parts = ip_str.split('.');
    if (parts.size() != 4) {
        return 0; // 无效IP返回0
    }

    uint32_t ip = 0;
    bool ok;

    // 从左到右读取每个字节
    // 192.168.1.100 应该转换为 0x64.0x01.0xA8.0xC0
    unsigned char bytes[4];
    for (int i = 0; i < 4; ++i) {
        bytes[i] = parts[i].toUInt(&ok);
        if (!ok || bytes[i] > 255) {
            return 0; // 转换失败或超出范围返回0
        }
    }

    // 按照与Uint32ToIPString相同的字节顺序组装
    ip = (bytes[3] << 24) | (bytes[2] << 16) | (bytes[1] << 8) | bytes[0];

    return ip;
}

bool IPUtils::IsValidIPString(const QString &ip_str)
{
    QStringList parts = ip_str.split('.');
    if (parts.size() != 4) {
        return false;
    }

    bool ok;
    for (const QString &part : parts) {
        uint32_t value = part.toUInt(&ok);
        if (!ok || value > 255) {
            return false;
        }
    }
    return true;
}
} // namespace lumosy