#include "app_manager.h"

#ifdef WIN32
    #include <windows.h>
    #include <tlhelp32.h>
    #include <tchar.h>
#endif

namespace ys_scene {
AppManager::AppManager(QObject* parent) : QObject(parent){}

AppManager::~AppManager()
{
}

bool AppManager::StartApp(const QString& app_path, const QString& app_name)
{
    // 检查参数是否有效
    if (app_path.isEmpty() || app_name.isEmpty()) {
        qWarning() << "Invalid parameters: path or app name is empty";
        return false;
    }

    // 检查文件是否存在
    QString full_path = QDir(app_path).absoluteFilePath(app_name);
    if (!QFile::exists(full_path)) {
        qWarning() << "Application does not exist:" << full_path;
        return false;
    }

    // 检查进程是否已运行
    if (IsProcessRunning(app_name)) {
        qDebug() << app_name << "is already running";
        return true;
    }

    // 启动新进程
    QProcess* process = new QProcess(this);
    process->setWorkingDirectory(app_path);
    process->start(full_path);

    // 检查是否成功启动
    if (!process->waitForStarted(5000)) {  // 等待5秒
        qWarning() << "Failed to start" << app_name;
        delete process;
        return false;
    }

    child_processes_.append(process);
    qDebug() << "Successfully started" << app_name;
    return true;
}

bool AppManager::IsProcessRunning(const QString& process_name)
{
#ifdef WIN32
    // Windows 实现方式
    HANDLE snapshot = CreateToolhelp32Snapshot(TH32CS_SNAPPROCESS, 0);
    if (snapshot == INVALID_HANDLE_VALUE) {
        return false;
    }

    PROCESSENTRY32W process_entry;
    process_entry.dwSize = sizeof(process_entry);

    if (!Process32FirstW(snapshot, &process_entry)) {
        CloseHandle(snapshot);
        return false;
    }

    QString process_name_lower = process_name.toLower();
    do {
        QString current_process = QString::fromWCharArray(process_entry.szExeFile).toLower();
        if (current_process == process_name_lower) {
            CloseHandle(snapshot);
            return true;
        }
    } while (Process32NextW(snapshot, &process_entry));

    CloseHandle(snapshot);
    return false;

#else
    // Linux/Unix 实现方式
    try {
        QProcess process;
        QStringList arguments;

#ifdef Q_OS_MACOS
        arguments << "-A" << "-c" << "comm";
#else
        arguments << "aux";
#endif

        process.start("ps", arguments);

        if (!process.waitForStarted(3000) || !process.waitForFinished(3000)) {
            qWarning() << "Process detection timed out";
            return false;
        }

        QByteArray output = process.readAllStandardOutput();
        QStringList processes = QString::fromUtf8(output).split('\n');

        QString processNameLower = processName.toLower();
        for (const QString& line : processes) {
            if (line.toLower().contains(processNameLower)) {
                return true;
            }
        }
        return false;

    }
    catch (const std::exception& e) {
        qWarning() << "Error checking process:" << e.what();
        return false;
    }
#endif
}

} // namespace ys_scene