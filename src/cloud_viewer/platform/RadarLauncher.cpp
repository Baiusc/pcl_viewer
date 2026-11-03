#include "cloud_viewer/platform/RadarLauncher.h"

#include <cstdio>
#include <cstdlib>
#include <fstream>
#include <sstream>
#include <string>

namespace cloud_viewer
{
namespace
{

struct Credentials
{
    std::string host = "192.168.137.200";
    std::string user = "root";
    std::string pass;
    std::string script = "/root/Linux_pcl/fsLAA.sh";
    std::string workdir = "/root/Linux_pcl";
};

std::string envOrDefault(const char *name, const char *fallback)
{
    const char *value = std::getenv(name);
    return (value != nullptr && value[0] != '\0') ? std::string(value) : std::string(fallback);
}

bool commandExists(const std::string &command)
{
    return std::system(("command -v " + command + " >/dev/null 2>&1").c_str()) == 0;
}

std::string trim(const std::string &text)
{
    const auto begin = text.find_first_not_of(" \t\r\n");
    if (begin == std::string::npos)
    {
        return {};
    }
    const auto end = text.find_last_not_of(" \t\r\n");
    return text.substr(begin, end - begin + 1);
}

/// 读取 ~/.config/cloud_viewer/radar.conf 中的 KEY=VALUE。
void loadConfigFile(Credentials &credentials)
{
    const char *home = std::getenv("HOME");
    if (home == nullptr || home[0] == '\0')
    {
        return;
    }

    std::ifstream input(std::string(home) + "/.config/cloud_viewer/radar.conf");
    std::string line;
    while (std::getline(input, line))
    {
        const std::string trimmed = trim(line);
        if (trimmed.empty() || trimmed[0] == '#')
        {
            continue;
        }
        const auto equal = trimmed.find('=');
        if (equal == std::string::npos)
        {
            continue;
        }
        const std::string key = trim(trimmed.substr(0, equal));
        std::string value = trim(trimmed.substr(equal + 1));
        if (value.size() >= 2 &&
            ((value.front() == '"' && value.back() == '"') ||
             (value.front() == '\'' && value.back() == '\'')))
        {
            value = value.substr(1, value.size() - 2);
        }

        if (key == "RADAR_SSH_HOST")
        {
            credentials.host = value;
        }
        else if (key == "RADAR_SSH_USER")
        {
            credentials.user = value;
        }
        else if (key == "RADAR_SSH_PASS")
        {
            credentials.pass = value;
        }
        else if (key == "RADAR_SCRIPT")
        {
            credentials.script = value;
        }
        else if (key == "RADAR_WORKDIR")
        {
            credentials.workdir = value;
        }
    }
}

Credentials loadCredentials()
{
    Credentials credentials;
    loadConfigFile(credentials);

    // 环境变量优先级高于配置文件，便于临时切换设备。
    credentials.host = envOrDefault("CV_RADAR_SSH_HOST", envOrDefault("CV_RADAR_IP", credentials.host.c_str()).c_str());
    credentials.user = envOrDefault("CV_RADAR_SSH_USER", credentials.user.c_str());
    credentials.pass = envOrDefault("CV_RADAR_SSH_PASS", credentials.pass.c_str());
    credentials.script = envOrDefault("CV_RADAR_SCRIPT", credentials.script.c_str());
    credentials.workdir = envOrDefault("CV_RADAR_WORKDIR", credentials.workdir.c_str());
    return credentials;
}

/// 单引号包裹，避免 shell 解释密码中的特殊字符。
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

bool RadarLauncher::configured()
{
#ifdef _WIN32
    return false;
#else
    return !loadCredentials().pass.empty() && commandExists("sshpass");
#endif
}

std::string RadarLauncher::describe()
{
    const Credentials credentials = loadCredentials();
    return credentials.user + "@" + credentials.host + " " + credentials.script;
}

bool RadarLauncher::start(std::string &message)
{
#ifdef _WIN32
    message = "Windows 暂不支持自动启动，请在雷达端手动执行 fsLAA.sh";
    return false;
#else
    const Credentials credentials = loadCredentials();
    if (credentials.pass.empty())
    {
        message = "未配置雷达 SSH 密码（~/.config/cloud_viewer/radar.conf 或 CV_RADAR_SSH_PASS）";
        return false;
    }
    if (!commandExists("sshpass"))
    {
        message = "缺少 sshpass，无法自动启动雷达驱动（sudo apt install sshpass）";
        return false;
    }

    // 驱动已在运行时不要重复拉起，避免多个 soc_example 抢占 UDP 端口。
    const std::string remote =
        "if ps | grep -q '[s]oc_example'; then echo ALREADY_RUNNING; "
        "else cd " + credentials.workdir + " && nohup sh " + credentials.script +
        " > /tmp/fsLAA.log 2>&1 < /dev/null & sleep 3; fi; "
        "ps | grep soc_example | grep -v grep | head -1";

    const std::string command =
        "sshpass -p " + shellQuote(credentials.pass) +
        " ssh -o StrictHostKeyChecking=no -o UserKnownHostsFile=/dev/null -o ConnectTimeout=5 " +
        shellQuote(credentials.user + "@" + credentials.host) + " " + shellQuote(remote) + " 2>&1";

    FILE *pipe = popen(command.c_str(), "r");
    if (pipe == nullptr)
    {
        message = "无法执行 SSH 命令";
        return false;
    }

    std::string output;
    char buffer[512];
    while (std::fgets(buffer, sizeof(buffer), pipe) != nullptr)
    {
        output += buffer;
    }
    const int rc = pclose(pipe);

    if (rc == 0 && (output.find("soc_example") != std::string::npos ||
                    output.find("ALREADY_RUNNING") != std::string::npos))
    {
        message = output.find("ALREADY_RUNNING") != std::string::npos
                      ? "雷达驱动已在运行（" + credentials.host + "）"
                      : "已通过 SSH 启动雷达驱动（" + credentials.host + "）";
        return true;
    }

    message = "SSH 启动失败: " + output.substr(0, 200);
    return false;
#endif
}

} // namespace cloud_viewer
