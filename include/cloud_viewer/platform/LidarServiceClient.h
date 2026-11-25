/**
 * @brief 新版 CS96 ARM 固件（lidar_mgrd）的管理接口客户端
 *
 * 固件把 you_see 交给 lidar_mgrd（runsv 守护）管理后，上位机的操作方式整体变了：
 *
 *   旧固件                                   新固件（ARM 侧改动后）
 *   /etc/ys_conf.ini                         /run/lidar-data/lidar/config/ys_conf.ini（基准配置）
 *   kill you_see + 手工 setsid 拉起           /usr/libexec/lidar/lidar-service restart
 *   改 INI 的 work_mode 再重启                lidar_mgrctl ... switch N（INI 里的值会被 mgrd 覆盖）
 *   无状态查询                                /cgi-bin/lidar_status.cgi 返回 JSON 状态
 *
 * 其中状态查询与模式切换有 HTTP 管理接口（lighttpd -> lidar_status/switch.cgi ->
 * mgrd 的 control.sock），不需要 SSH，也不依赖 plink，因此 Windows 上也能直接用。
 * 推流目标（center_ip / center_data_port）仍然只能通过基准配置 + restart 生效。
 */
#pragma once

#include <cstdint>
#include <string>
#include <vector>

namespace cloud_viewer
{

/// lidar_status.cgi 的解析结果。
struct LidarServiceStatus
{
    bool reachable = false;
    int code = 0;
    std::string model;      // CS96-beta
    std::string phase;      // STOPPING / STARTING / COMMITTED / ...
    int confirmedMode = -1; // mgrd 已确认的模式
    int actualMode = -1;    // you_see 实际运行的模式；切换中为 -1
    int desiredMode = -1;   // 目标模式
    int pid = -1;
    std::string error;
    std::vector<int> validatedWorkModes; // 该固件允许的 work_mode（如 1、2）

    bool committed() const { return phase == "COMMITTED"; }
    bool supports(int mode) const;
    /// 一行可读摘要，用于 UI 日志。
    std::string describe() const;
};

/// 该固件允许的 work_mode 位图（bit N 表示 mode N）；无信息时返回 {1,2}。
std::uint8_t lidarServiceModeMask(const LidarServiceStatus &status);

/// 查询设备管理状态。旧固件没有该接口，返回 false（此时应走原 SSH 流程）。
bool queryLidarService(const std::string &ip, std::uint16_t httpPort,
                       LidarServiceStatus &status, int timeoutMs = 1500);

/// 请求切换 work_mode，并轮询到 COMMITTED。仅在新固件上可用。
bool switchLidarServiceMode(const std::string &ip, std::uint16_t httpPort, int mode,
                            LidarServiceStatus &finalStatus, std::string &error,
                            int acceptTimeoutMs = 8000, int commitTimeoutMs = 30000);

/// 设备 HTTP 端口；环境变量 CV_BETA_HTTP_PORT 可覆盖（调试用）。
std::uint16_t lidarServiceHttpPort();

} // namespace cloud_viewer
