/**
 * @brief 软件目录与用户设置的持久化
 *
 * 标定表选择一次后复制到软件目录（<exe>/calib/），并在 <exe>/cloud_viewer.ini
 * 中记住文件名与开关状态，下次启动自动恢复。若软件目录不可写（例如装到
 * /usr/bin），自动回退到 ~/.config/cloud_viewer/。
 */
#pragma once

#include <filesystem>
#include <string>

namespace cloud_viewer
{

struct ViewerSettings
{
    std::string calibrationFile;          ///< 文件名，位于 calibrationDirectory()
    bool calibrationEnabled = false;      ///< 标定开关是否启用
};

/// 可执行文件所在目录。
std::filesystem::path executableDirectory();

/// 标定表存放目录（优先软件目录下的 calib/）。
std::filesystem::path calibrationDirectory();

/// 设置文件路径（优先软件目录下的 cloud_viewer.ini）。
std::filesystem::path settingsFilePath();

/// 把用户选择的标定表复制到标定目录，返回目标路径；失败返回空。
std::filesystem::path importCalibrationFile(const std::filesystem::path &source,
                                            std::string &error);

ViewerSettings loadViewerSettings();

bool saveViewerSettings(const ViewerSettings &settings, std::string &error);

} // namespace cloud_viewer
