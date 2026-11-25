#ifndef LUMOSY_IP_UTILS_H_
#define LUMOSY_IP_UTILS_H_

#include <QString>
#include <cstdint>

namespace lumosy {

class IPUtils
{
 public:
    /**
     * @brief 将uint32_t格式的IP地址转换为字符串格式
     * @param ip_val IP地址的uint32_t表示
     * @return IP地址的字符串表示，格式为"xxx.xxx.xxx.xxx"
     */
    static QString Uint32ToIPString(uint32_t ip_val);

    /**
     * @brief 将字符串格式的IP地址转换为uint32_t格式
     * @param ip_str IP地址的字符串表示，格式为"xxx.xxx.xxx.xxx"
     * @return IP地址的uint32_t表示，转换失败返回0
     */
    static uint32_t IPStringToUint32(const QString &ip_str);

    /**
     * @brief 验证IP地址字符串格式是否有效
     * @param ip_str 待验证的IP地址字符串
     * @return 有效返回true，无效返回false
     */
    static bool IsValidIPString(const QString &ip_str);
private:
    IPUtils() = default; // 工具类不允许实例化
};


} // namespace lumosy

#endif // LUMOSY_IP_UTILS_H_