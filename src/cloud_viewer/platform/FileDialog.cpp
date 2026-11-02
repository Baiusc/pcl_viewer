#include "cloud_viewer/platform/FileDialog.h"

#include <algorithm>
#include <cstdio>
#include <cstdlib>
#include <string>

#ifdef _WIN32
#ifndef NOMINMAX
#define NOMINMAX
#endif
#include <windows.h>
#include <commdlg.h>
#include <shlobj.h>
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

#ifndef _WIN32

bool commandExists(const char *command)
{
    const std::string cmd = std::string("command -v ") + command + " >/dev/null 2>&1";
    return std::system(cmd.c_str()) == 0;
}

std::string runCommand(const std::string &command)
{
    FILE *pipe = popen(command.c_str(), "r");
    if (!pipe)
    {
        return {};
    }

    std::string output;
    char buffer[4096];
    while (std::fgets(buffer, sizeof(buffer), pipe) != nullptr)
    {
        output += buffer;
    }
    pclose(pipe);
    return trim(output);
}

#else

std::string toUtf8(const std::wstring &text)
{
    if (text.empty())
    {
        return {};
    }
    const int size = WideCharToMultiByte(CP_UTF8, 0, text.c_str(), -1, nullptr, 0, nullptr, nullptr);
    std::string result(static_cast<std::size_t>(size), '\0');
    WideCharToMultiByte(CP_UTF8, 0, text.c_str(), -1, result.data(), size, nullptr, nullptr);
    if (!result.empty() && result.back() == '\0')
    {
        result.pop_back();
    }
    return result;
}

#endif

} // namespace

bool FileDialogService::available()
{
#ifdef _WIN32
    return true;
#else
    return commandExists("zenity") || commandExists("kdialog");
#endif
}

const char *FileDialogService::backendName()
{
#ifdef _WIN32
    return "Win32 comdlg32";
#else
    if (commandExists("zenity"))
    {
        return "zenity";
    }
    if (commandExists("kdialog"))
    {
        return "kdialog";
    }
    return "unavailable";
#endif
}

std::vector<std::filesystem::path> FileDialogService::openFiles()
{
    std::vector<std::filesystem::path> files;

#ifdef _WIN32
    std::wstring buffer(64 * 1024, L'\0');
    OPENFILENAMEW dialog{};
    dialog.lStructSize = sizeof(dialog);
    dialog.lpstrFilter =
        L"点云文件 (*.csv;*.txt;*.pcd)\0*.csv;*.txt;*.pcd\0所有文件 (*.*)\0*.*\0\0";
    dialog.lpstrFile = buffer.data();
    dialog.nMaxFile = static_cast<DWORD>(buffer.size());
    dialog.lpstrTitle = L"打开点云文件";
    dialog.Flags = OFN_ALLOWMULTISELECT | OFN_EXPLORER | OFN_FILEMUSTEXIST | OFN_PATHMUSTEXIST;

    if (!GetOpenFileNameW(&dialog))
    {
        return files;
    }

    const wchar_t *first = buffer.c_str();
    const wchar_t *second = first + wcslen(first) + 1;
    if (*second == L'\0')
    {
        // 单选：缓冲区即完整路径。
        files.emplace_back(toUtf8(first));
        return files;
    }

    // 多选：目录 + 若干文件名。
    const std::wstring directory(first);
    const wchar_t *name = second;
    while (*name != L'\0')
    {
        files.emplace_back(std::filesystem::path(directory) / name);
        name += wcslen(name) + 1;
    }
#else
    if (commandExists("zenity"))
    {
        const std::string output = runCommand(
            "zenity --file-selection --multiple --separator='|' "
            "--title='打开点云文件' "
            "--file-filter='点云文件 (*.csv *.txt *.pcd) | *.csv *.txt *.pcd' "
            "--file-filter='所有文件 | *' 2>/dev/null");
        if (output.empty())
        {
            return files;
        }

        std::size_t start = 0;
        while (start <= output.size())
        {
            const std::size_t end = output.find('|', start);
            const std::string item = (end == std::string::npos)
                                         ? output.substr(start)
                                         : output.substr(start, end - start);
            const std::string path = trim(item);
            if (!path.empty())
            {
                files.emplace_back(path);
            }
            if (end == std::string::npos)
            {
                break;
            }
            start = end + 1;
        }
    }
    else if (commandExists("kdialog"))
    {
        const std::string output = runCommand(
            "kdialog --getopenfilename . '点云文件 (*.csv *.txt *.pcd)' "
            "--multiple --separate-output 2>/dev/null");
        if (!output.empty())
        {
            std::size_t start = 0;
            while (start <= output.size())
            {
                const std::size_t end = output.find('\n', start);
                const std::string item = (end == std::string::npos)
                                             ? output.substr(start)
                                             : output.substr(start, end - start);
                const std::string path = trim(item);
                if (!path.empty())
                {
                    files.emplace_back(path);
                }
                if (end == std::string::npos)
                {
                    break;
                }
                start = end + 1;
            }
        }
    }
#endif

    return files;
}

std::optional<std::filesystem::path> FileDialogService::chooseDirectory()
{
#ifdef _WIN32
    BROWSEINFOW browse{};
    browse.lpszTitle = L"选择点云目录";
    browse.ulFlags = BIF_RETURNONLYFSDIRS | BIF_NEWDIALOGSTYLE;

    LPITEMIDLIST item = SHBrowseForFolderW(&browse);
    if (!item)
    {
        return std::nullopt;
    }

    wchar_t path[MAX_PATH] = {0};
    const bool ok = SHGetPathFromIDListW(item, path) != FALSE;
    CoTaskMemFree(item);
    if (!ok)
    {
        return std::nullopt;
    }
    return std::filesystem::path(toUtf8(path));
#else
    if (commandExists("zenity"))
    {
        const std::string output = runCommand(
            "zenity --file-selection --directory --title='选择点云目录' 2>/dev/null");
        if (output.empty())
        {
            return std::nullopt;
        }
        return std::filesystem::path(output);
    }
    return std::nullopt;
#endif
}

} // namespace cloud_viewer
