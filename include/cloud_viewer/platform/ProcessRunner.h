/**
 * @brief 直接执行外部程序并收集输出（不经过 shell）
 *
 * Windows 侧没有 sshpass，密码认证只能交给 plink 的 -pw 参数；
 * 而密码、远端脚本里都可能含有 cmd.exe 的元字符（& | ^ " %），
 * 所以必须绕开 shell 直接 CreateProcess。Linux 侧继续走原有 shell 管道，
 * 本模块只服务于 Windows 的 SSH 路径（POSIX 也提供等价实现，便于测试）。
 */
#pragma once

#include <string>
#include <vector>

namespace cloud_viewer
{

struct ProcessOutput
{
    int exitCode = -1; ///< 子进程退出码；启动失败或超时为 -1
    std::string text;  ///< stdout 与 stderr 合并后的文本
    bool timedOut = false;
};

/// 运行 executable，把 stdinText 写入其 stdin，并收集 stdout+stderr。
ProcessOutput runProcessWithInput(const std::string &executable,
                                  const std::vector<std::string> &args,
                                  const std::string &stdinText,
                                  unsigned timeoutMs = 40000);

} // namespace cloud_viewer
