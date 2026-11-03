/**
 * @brief 雷达网段的自动检测与配置
 *
 * 雷达向本机 192.168.137.100 单播点云；该地址通常是临时添加的，
 * 电脑重启后会丢失，导致 viewer 永远停在 Waiting。
 *
 * 本模块在连接前检查地址是否存在，缺失时按以下顺序尝试配置：
 *   1. sudo -n /usr/local/bin/cv-radar-net <iface> <cidr>  （推荐，安装脚本写入免密规则）
 *   2. sudo -n /sbin/ip addr replace <cidr> dev <iface>    （用户已给 ip 配免密时）
 *   3. CV_SUDO_PASS 环境变量 + sudo -S                     （兜底，不推荐）
 */
#pragma once

#include <string>

namespace cloud_viewer
{

struct NetworkSetupResult
{
    bool hasAddress = false; ///< 检查时地址已存在
    bool configured = false; ///< 最终可用（原本就有或本次配置成功）
    std::string message;     ///< 面向用户的说明
};

/// 本机是否拥有指定 IPv4 地址。
bool hasLocalAddress(const std::string &address);

/// 确保本机拥有 address（属于 cidr）。已存在则直接返回成功。
NetworkSetupResult ensureLocalAddress(const std::string &iface,
                                      const std::string &cidr,
                                      const std::string &address);

} // namespace cloud_viewer
