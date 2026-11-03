/**
 * @brief 通过 SSH 启动雷达端驱动脚本（可选能力）
 *
 * 凭据只从环境变量读取，不写入代码或仓库：
 *   CV_RADAR_SSH_HOST   默认 192.168.137.200
 *   CV_RADAR_SSH_USER   默认 root
 *   CV_RADAR_SSH_PASS   必需，未设置则不启用自动启动
 *   CV_RADAR_SCRIPT     默认 /root/Linux_pcl/fsLAA.sh
 *   CV_RADAR_WORKDIR    默认 /root/Linux_pcl
 */
#pragma once

#include <string>

namespace cloud_viewer
{

class RadarLauncher
{
public:
    /// 是否具备自动启动条件（Linux + sshpass + 已配置密码）。
    static bool configured();

    /// 描述当前配置（不含密码），用于日志/诊断。
    static std::string describe();

    /// 启动雷达脚本；成功时 message 为说明。
    static bool start(std::string &message);
};

} // namespace cloud_viewer
