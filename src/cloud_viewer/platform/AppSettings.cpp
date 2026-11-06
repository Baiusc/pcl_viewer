#include "cloud_viewer/platform/AppSettings.h"

#include <cstdlib>
#include <fstream>
#include <system_error>

#ifdef _WIN32
#include <windows.h>
#else
#include <limits.h>
#include <unistd.h>
#endif

namespace cloud_viewer
{
namespace
{

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

std::filesystem::path homeDirectory()
{
    const char *home = std::getenv("HOME");
#ifdef _WIN32
    if (home == nullptr || home[0] == '\0')
    {
        home = std::getenv("USERPROFILE");
    }
#endif
    if (home != nullptr && home[0] != '\0')
    {
        return std::filesystem::path(home);
    }
    std::error_code error;
    return std::filesystem::current_path(error);
}

bool directoryWritable(const std::filesystem::path &directory)
{
    std::error_code error;
    std::filesystem::create_directories(directory, error);
    if (error)
    {
        return false;
    }

    const auto probe = directory / ".cv_write_probe";
    {
        std::ofstream output(probe, std::ios::trunc);
        if (!output.is_open())
        {
            return false;
        }
    }
    std::filesystem::remove(probe, error);
    return true;
}

/// 优先软件目录；不可写时回退到用户配置目录。
const std::filesystem::path &baseDataDirectory()
{
    static const std::filesystem::path cached = []() -> std::filesystem::path
    {
        const auto exeDirectory = executableDirectory();
        if (directoryWritable(exeDirectory))
        {
            return exeDirectory;
        }

        auto fallback = homeDirectory() / ".config" / "cloud_viewer";
        std::error_code error;
        std::filesystem::create_directories(fallback, error);
        return fallback;
    }();
    return cached;
}

} // namespace

std::filesystem::path executableDirectory()
{
#ifdef _WIN32
    wchar_t buffer[MAX_PATH] = {0};
    const DWORD length = GetModuleFileNameW(nullptr, buffer, MAX_PATH);
    if (length > 0)
    {
        return std::filesystem::path(buffer).parent_path();
    }
#else
    char buffer[PATH_MAX] = {0};
    const ssize_t length = readlink("/proc/self/exe", buffer, sizeof(buffer) - 1);
    if (length > 0)
    {
        return std::filesystem::path(std::string(buffer, static_cast<std::size_t>(length)))
            .parent_path();
    }
#endif
    std::error_code error;
    return std::filesystem::current_path(error);
}

std::filesystem::path calibrationDirectory()
{
    auto directory = baseDataDirectory() / "calib";
    std::error_code error;
    std::filesystem::create_directories(directory, error);
    return directory;
}

std::filesystem::path settingsFilePath()
{
    return baseDataDirectory() / "cloud_viewer.ini";
}

std::filesystem::path importCalibrationFile(const std::filesystem::path &source,
                                            std::string &error)
{
    std::error_code ec;
    if (!std::filesystem::is_regular_file(source, ec))
    {
        error = "源文件不存在: " + source.string();
        return {};
    }

    const auto target = calibrationDirectory() / source.filename();
    if (std::filesystem::exists(target, ec) &&
        std::filesystem::equivalent(source, target, ec))
    {
        return target; // 已在标定目录中，无需复制。
    }

    std::filesystem::copy_file(source, target,
                               std::filesystem::copy_options::overwrite_existing, ec);
    if (ec)
    {
        error = "复制到 " + target.string() + " 失败: " + ec.message();
        return {};
    }
    return target;
}

ViewerSettings loadViewerSettings()
{
    ViewerSettings settings;
    std::ifstream input(settingsFilePath());
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
        const std::string value = trim(trimmed.substr(equal + 1));

        if (key == "CALIBRATION_FILE")
        {
            settings.calibrationFile = value;
        }
        else if (key == "CALIBRATION_ENABLED")
        {
            settings.calibrationEnabled = (value == "1" || value == "true" || value == "ON");
        }
        else if (key == "LAST_DEVICE_IP")
        {
            settings.lastDeviceIp = value;
        }
    }
    return settings;
}

bool saveViewerSettings(const ViewerSettings &settings, std::string &error)
{
    const auto path = settingsFilePath();
    std::ofstream output(path, std::ios::trunc);
    if (!output.is_open())
    {
        error = "无法写入设置文件: " + path.string();
        return false;
    }
    output << "CALIBRATION_FILE=" << settings.calibrationFile << "\n";
    output << "CALIBRATION_ENABLED=" << (settings.calibrationEnabled ? 1 : 0) << "\n";
    if (!settings.lastDeviceIp.empty())
    {
        output << "LAST_DEVICE_IP=" << settings.lastDeviceIp << "\n";
    }
    return true;
}

} // namespace cloud_viewer
