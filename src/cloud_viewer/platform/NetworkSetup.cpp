#include "cloud_viewer/platform/NetworkSetup.h"

#include <cstdlib>
#include <string>
#include <vector>

#ifndef _WIN32
#include <arpa/inet.h>
#include <ifaddrs.h>
#include <netinet/in.h>
#include <sys/socket.h>
#endif

namespace cloud_viewer
{
namespace
{

std::string envOrDefault(const char *name, const char *fallback)
{
    const char *value = std::getenv(name);
    return (value != nullptr && value[0] != '\0') ? std::string(value) : std::string(fallback);
}

std::string shellQuote(const std::string &text)
{
    std::string out = "'";
    for (char ch : text)
    {
        if (ch == '\'')
        {
            out += "'\\''";
        }
        else
        {
            out.push_back(ch);
        }
    }
    out += "'";
    return out;
}

} // namespace

bool hasLocalAddress(const std::string &address)
{
#ifdef _WIN32
    (void)address;
    return false;
#else
    struct ifaddrs *addresses = nullptr;
    if (getifaddrs(&addresses) != 0)
    {
        return false;
    }

    bool found = false;
    for (struct ifaddrs *it = addresses; it != nullptr; it = it->ifa_next)
    {
        if (it->ifa_addr == nullptr || it->ifa_addr->sa_family != AF_INET)
        {
            continue;
        }
        char buffer[INET_ADDRSTRLEN] = {0};
        const auto *sin = reinterpret_cast<const struct sockaddr_in *>(it->ifa_addr);
        if (inet_ntop(AF_INET, &sin->sin_addr, buffer, sizeof(buffer)) != nullptr &&
            address == buffer)
        {
            found = true;
            break;
        }
    }
    freeifaddrs(addresses);
    return found;
#endif
}

NetworkSetupResult ensureLocalAddress(const std::string &iface,
                                      const std::string &cidr,
                                      const std::string &address)
{
    NetworkSetupResult result;
#ifdef _WIN32
    // Windows 无 ip 命令，无法自动配置；放行让 UDP 继续尝试，由用户手动设网段。
    result.configured = true;
    result.message = "Windows 平台请手动配置雷达网段地址";
    return result;
#else
    if (hasLocalAddress(address))
    {
        result.hasAddress = true;
        result.configured = true;
        result.message = "本机已有 " + address;
        return result;
    }

    const std::string helper = envOrDefault("CV_RADAR_NET_HELPER", "/usr/local/bin/cv-radar-net");
    const std::string ipCommand = "/sbin/ip addr replace " + cidr + " dev " + iface;

    std::vector<std::string> attempts;
    // 1) 安装脚本写入的免密助手（推荐路径）。
    attempts.push_back("sudo -n " + shellQuote(helper) + " " + shellQuote(iface) + " " + shellQuote(cidr));
    // 2) 用户可能已为 ip 配置 NOPASSWD。
    attempts.push_back("sudo -n " + ipCommand);
    // 3) 兜底：显式提供 sudo 密码（不落盘，仅进程环境）。
    const std::string sudoPass = envOrDefault("CV_SUDO_PASS", "");
    if (!sudoPass.empty())
    {
        attempts.push_back("printf '%s\\n' " + shellQuote(sudoPass) +
                           " | sudo -S -p '' " + ipCommand);
    }

    for (const std::string &command : attempts)
    {
        (void)std::system((command + " >/dev/null 2>&1").c_str());
        if (hasLocalAddress(address))
        {
            result.configured = true;
            result.message = "已自动配置 " + cidr + " -> " + iface;
            return result;
        }
    }

    result.message = "本机缺少 " + address +
                     "：请执行 sudo ip addr add " + cidr + " dev " + iface +
                     "，或运行 tools/setup_radar_autoconnect.sh 一次性安装免密助手";
    return result;
#endif
}

} // namespace cloud_viewer
