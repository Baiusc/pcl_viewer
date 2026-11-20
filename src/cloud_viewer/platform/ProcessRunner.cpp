#include "cloud_viewer/platform/ProcessRunner.h"

#include <algorithm>
#include <cstring>
#include <thread>

#ifdef _WIN32
#ifndef WIN32_LEAN_AND_MEAN
#define WIN32_LEAN_AND_MEAN
#endif
#include <windows.h>
#else
#include <cerrno>
#include <fcntl.h>
#include <poll.h>
#include <signal.h>
#include <sys/types.h>
#include <sys/wait.h>
#include <unistd.h>
#endif

namespace cloud_viewer
{
namespace
{

#ifdef _WIN32

std::wstring toWide(const std::string &text)
{
    if (text.empty())
    {
        return {};
    }
    const int size = ::MultiByteToWideChar(CP_UTF8, 0, text.data(),
                                           static_cast<int>(text.size()), nullptr, 0);
    if (size <= 0)
    {
        return {};
    }
    std::wstring wide(static_cast<std::size_t>(size), L'\0');
    ::MultiByteToWideChar(CP_UTF8, 0, text.data(), static_cast<int>(text.size()),
                          wide.data(), size);
    return wide;
}

/// MSVCRT 的命令行转义规则：CreateProcess 只接受一整条命令行，
/// 空格与引号必须自己加转义，否则 plink 会把密码拆成多个参数。
std::wstring quoteArgument(const std::wstring &argument)
{
    if (!argument.empty() &&
        argument.find_first_of(L" \t\n\v\"") == std::wstring::npos)
    {
        return argument;
    }
    std::wstring result = L"\"";
    for (std::size_t i = 0; i < argument.size(); ++i)
    {
        std::size_t backslashes = 0;
        while (i < argument.size() && argument[i] == L'\\')
        {
            ++i;
            ++backslashes;
        }
        if (i == argument.size())
        {
            result.append(backslashes * 2, L'\\');
            break;
        }
        if (argument[i] == L'"')
        {
            result.append(backslashes * 2 + 1, L'\\');
            result.push_back(L'"');
        }
        else
        {
            result.append(backslashes, L'\\');
            result.push_back(argument[i]);
        }
    }
    result.push_back(L'"');
    return result;
}

#endif

} // namespace

ProcessOutput runProcessWithInput(const std::string &executable,
                                  const std::vector<std::string> &args,
                                  const std::string &stdinText,
                                  unsigned timeoutMs)
{
    ProcessOutput result;

#ifdef _WIN32
    SECURITY_ATTRIBUTES attributes{};
    attributes.nLength = sizeof(attributes);
    attributes.bInheritHandle = TRUE;

    HANDLE stdinRead = nullptr;
    HANDLE stdinWrite = nullptr;
    HANDLE stdoutRead = nullptr;
    HANDLE stdoutWrite = nullptr;
    if (!::CreatePipe(&stdinRead, &stdinWrite, &attributes, 0) ||
        !::CreatePipe(&stdoutRead, &stdoutWrite, &attributes, 0))
    {
        result.text = "无法创建管道";
        return result;
    }
    ::SetHandleInformation(stdinWrite, HANDLE_FLAG_INHERIT, 0);
    ::SetHandleInformation(stdoutRead, HANDLE_FLAG_INHERIT, 0);

    std::wstring commandLine = quoteArgument(toWide(executable));
    for (const std::string &argument : args)
    {
        commandLine += L" ";
        commandLine += quoteArgument(toWide(argument));
    }
    std::vector<wchar_t> commandBuffer(commandLine.begin(), commandLine.end());
    commandBuffer.push_back(L'\0');
    std::wstring program = toWide(executable);

    STARTUPINFOW startup{};
    startup.cb = sizeof(startup);
    startup.dwFlags = STARTF_USESTDHANDLES;
    startup.hStdInput = stdinRead;
    startup.hStdOutput = stdoutWrite;
    startup.hStdError = stdoutWrite;

    PROCESS_INFORMATION process{};
    const BOOL started = ::CreateProcessW(
        program.c_str(), commandBuffer.data(), nullptr, nullptr, TRUE,
        CREATE_NO_WINDOW, nullptr, nullptr, &startup, &process);
    ::CloseHandle(stdinRead);
    ::CloseHandle(stdoutWrite);
    if (!started)
    {
        result.text = "无法启动 " + executable + "（错误码 " +
                      std::to_string(::GetLastError()) + "）";
        ::CloseHandle(stdinWrite);
        ::CloseHandle(stdoutRead);
        return result;
    }

    std::thread writer([&]() {
        std::size_t offset = 0;
        while (offset < stdinText.size())
        {
            const DWORD chunk = static_cast<DWORD>(
                std::min<std::size_t>(stdinText.size() - offset, 4096));
            DWORD written = 0;
            if (!::WriteFile(stdinWrite, stdinText.data() + offset, chunk, &written, nullptr) ||
                written == 0)
            {
                break;
            }
            offset += written;
        }
        ::CloseHandle(stdinWrite);
    });

    char buffer[1024];
    DWORD read = 0;
    const ULONGLONG deadline = ::GetTickCount64() + timeoutMs;
    while (true)
    {
        DWORD available = 0;
        if (!::PeekNamedPipe(stdoutRead, nullptr, 0, nullptr, &available, nullptr))
        {
            break;
        }
        if (available > 0)
        {
            const DWORD want = std::min<DWORD>(available, sizeof(buffer));
            if (!::ReadFile(stdoutRead, buffer, want, &read, nullptr) || read == 0)
            {
                break;
            }
            result.text.append(buffer, read);
            continue;
        }
        if (::WaitForSingleObject(process.hProcess, 30) == WAIT_OBJECT_0)
        {
            // 进程已退出，把管道里剩下的内容读完。
            while (::ReadFile(stdoutRead, buffer, sizeof(buffer), &read, nullptr) && read > 0)
            {
                result.text.append(buffer, read);
            }
            break;
        }
        if (::GetTickCount64() > deadline)
        {
            result.timedOut = true;
            ::TerminateProcess(process.hProcess, 1);
            ::WaitForSingleObject(process.hProcess, 2000);
            break;
        }
    }

    writer.join();
    DWORD exitCode = 0;
    if (::GetExitCodeProcess(process.hProcess, &exitCode))
    {
        result.exitCode = static_cast<int>(exitCode);
    }
    if (result.timedOut)
    {
        result.exitCode = -1;
    }
    ::CloseHandle(stdoutRead);
    ::CloseHandle(process.hThread);
    ::CloseHandle(process.hProcess);
    return result;
#else
    int stdinPipe[2] = {-1, -1};
    int stdoutPipe[2] = {-1, -1};
    if (::pipe(stdinPipe) != 0 || ::pipe(stdoutPipe) != 0)
    {
        result.text = "无法创建管道";
        return result;
    }

    const pid_t pid = ::fork();
    if (pid < 0)
    {
        result.text = "无法 fork";
        return result;
    }
    if (pid == 0)
    {
        ::dup2(stdinPipe[0], STDIN_FILENO);
        ::dup2(stdoutPipe[1], STDOUT_FILENO);
        ::dup2(stdoutPipe[1], STDERR_FILENO);
        ::close(stdinPipe[0]);
        ::close(stdinPipe[1]);
        ::close(stdoutPipe[0]);
        ::close(stdoutPipe[1]);
        std::vector<char *> argv;
        argv.push_back(const_cast<char *>(executable.c_str()));
        for (const std::string &argument : args)
        {
            argv.push_back(const_cast<char *>(argument.c_str()));
        }
        argv.push_back(nullptr);
        ::execvp(executable.c_str(), argv.data());
        ::_exit(127);
    }

    ::close(stdinPipe[0]);
    ::close(stdoutPipe[1]);
    std::thread writer([&]() {
        std::size_t offset = 0;
        while (offset < stdinText.size())
        {
            const ssize_t written = ::write(stdinPipe[1], stdinText.data() + offset,
                                            stdinText.size() - offset);
            if (written <= 0)
            {
                break;
            }
            offset += static_cast<std::size_t>(written);
        }
        ::close(stdinPipe[1]);
    });

    char buffer[1024];
    while (true)
    {
        pollfd descriptor{};
        descriptor.fd = stdoutPipe[0];
        descriptor.events = POLLIN;
        const int ready = ::poll(&descriptor, 1, 50);
        if (ready > 0)
        {
            const ssize_t read = ::read(stdoutPipe[0], buffer, sizeof(buffer));
            if (read <= 0)
            {
                break;
            }
            result.text.append(buffer, static_cast<std::size_t>(read));
        }
    }
    writer.join();
    ::close(stdoutPipe[0]);

    int status = 0;
    ::waitpid(pid, &status, 0);
    result.exitCode = WIFEXITED(status) ? WEXITSTATUS(status) : -1;
    return result;
#endif
}

} // namespace cloud_viewer
