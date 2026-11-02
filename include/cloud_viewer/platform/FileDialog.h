/**
 * @brief 跨平台原生文件对话框薄封装
 *
 * Linux 使用 zenity（运行时依赖，无编译期依赖）；
 * Windows 使用 comdlg32 / shell32。所有路径以 UTF-8 输出，内部转为 filesystem::path。
 */
#pragma once

#include <filesystem>
#include <optional>
#include <vector>

namespace cloud_viewer
{

class FileDialogService
{
public:
    /// 多选打开点云文件；取消返回空数组。
    static std::vector<std::filesystem::path> openFiles();

    /// 选择目录；取消返回 nullopt。
    static std::optional<std::filesystem::path> chooseDirectory();

    /// 平台后端是否可用（Linux 下取决于 zenity 是否存在）。
    static bool available();

    static const char *backendName();
};

} // namespace cloud_viewer
