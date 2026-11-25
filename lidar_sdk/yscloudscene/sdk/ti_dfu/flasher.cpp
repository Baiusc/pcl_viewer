#include "flasher.h"

#include <QDir>
#include <QFileInfo>
#include <QRegularExpression>
#include <QStringList>

namespace ti_dfu {

namespace {

const QString kDeviceId = QStringLiteral("0451:6165");
constexpr int kProbeAttempts = 15;
constexpr int kFlashProbeAttempts = 45;

} // namespace

Flasher::Flasher(QObject* parent)
    : QObject(parent)
{
    process_.setProcessChannelMode(QProcess::MergedChannels);
    timeout_.setSingleShot(true);
    connect(&timeout_, &QTimer::timeout, this, [this]() {
        timed_out_ = true;
        process_.kill();
    });
    connect(&process_, &QProcess::readyRead, this, &Flasher::ReadOutput);
    connect(&process_, qOverload<int, QProcess::ExitStatus>(&QProcess::finished),
            this, &Flasher::ProcessFinished);
    connect(&process_, &QProcess::errorOccurred, this, [this](QProcess::ProcessError error) {
        if (error == QProcess::FailedToStart) {
            Finish(false, tr("无法启动 dfu-util：%1").arg(process_.errorString()));
        }
    });
}

bool Flasher::IsRunning() const
{
    return step_ >= 0;
}

bool Flasher::Start(const Options& options)
{
    if (IsRunning()) {
        return false;
    }

    const QDir boot(options.boot_directory);
    if (options.boot_directory.trimmed().isEmpty() || options.image_path.trimmed().isEmpty()) {
        emit Completed(false, tr("请指定 boot 文件目录和固件镜像。"));
        return false;
    }
    steps_ = {
        {tr("加载 tiboot3.bin"), QStringLiteral("bootloader"), boot.absoluteFilePath("tiboot3.bin"), 0},
    };
    const QString image = QFileInfo(options.image_path).absoluteFilePath();
    if (options.workflow == Workflow::KFlashWriter) {
        steps_.append({tr("加载 tispl.bin"), QStringLiteral("tispl.bin"),
                       boot.absoluteFilePath("tispl.bin"), 0, true});
        steps_.append({tr("加载 u-boot.img"), QStringLiteral("u-boot.img"),
                       boot.absoluteFilePath("u-boot.img"), 1});
        steps_.append({tr("写入 Flash 镜像"), QStringLiteral("flash"), image, -1});
    } else {
        steps_.append({tr("发送更新镜像"), QStringLiteral("tispl.bin"), image, 0, true});
    }

    const QFileInfo tool(options.tool_path);
    if (!tool.isFile() || !tool.isExecutable()) {
        emit Completed(false, tr("找不到可执行的 dfu-util：%1").arg(options.tool_path));
        return false;
    }
    for (const Step& step : steps_) {
        const QFileInfo file(step.file);
        if (!file.isFile() || !file.isReadable() || file.size() == 0) {
            emit Completed(false, tr("文件缺失、为空或不可读：%1").arg(step.file));
            return false;
        }
    }

    tool_ = tool.absoluteFilePath();
    device_path_.clear();
    step_ = 0;
    probe_attempts_ = 0;
    QTimer::singleShot(0, this, &Flasher::Probe);
    return true;
}

void Flasher::Probe()
{
    const Step& step = steps_.at(step_);
    if (probe_attempts_++ == 0) {
        emit StageChanged(step.name, step_ + 1, steps_.size());
        emit ProgressChanged(-1);
    }
    probing_ = true;
    Run({QStringLiteral("-l"), QStringLiteral("-d"), kDeviceId});
}

void Flasher::Run(const QStringList& arguments)
{
    output_.clear();
    progress_ = -1;
    timed_out_ = false;
    download_done_ = false;
    process_.setWorkingDirectory(QFileInfo(tool_).absolutePath());
    timeout_.start(probing_ ? 5000 : 120000);
    process_.start(tool_, arguments);
}

void Flasher::ReadOutput()
{
    const QString chunk = QString::fromLocal8Bit(process_.readAll());
    if (chunk.isEmpty()) {
        return;
    }
    output_.append(chunk);
    if (!probing_) {
        timeout_.start(120000);
        static const QRegularExpression progress(R"(Download[^\r\n]*?\]\s*(\d{1,3})%)");
        auto matches = progress.globalMatch(output_);
        while (matches.hasNext()) {
            const int value = matches.next().captured(1).toInt();
            if (value > progress_ && value <= 100) {
                progress_ = value;
                emit ProgressChanged(value);
            }
        }
        download_done_ |= output_.contains(QStringLiteral("Download done."));
        output_ = output_.right(8192);
    }
    emit LogReceived(chunk);
}

void Flasher::ProcessFinished(int exit_code, QProcess::ExitStatus status)
{
    ReadOutput();
    timeout_.stop();
    const bool verify_bootloader_transition = !probing_ && step_ == 0 && !timed_out_ &&
        status == QProcess::NormalExit && exit_code == 74 && download_done_ &&
        output_.contains(QStringLiteral("DFU state(6) = dfuMANIFEST-SYNC, status(0)")) &&
        output_.contains(QStringLiteral("unable to read DFU status after completion (LIBUSB_ERROR_PIPE)"));
    if (verify_bootloader_transition) {
        emit LogReceived(tr("\n引导文件已传输，工具读取结束状态失败（退出码 74）。正在检查第二阶段 DFU 接口。\n"));
    } else if (timed_out_ || status != QProcess::NormalExit || exit_code != 0) {
        const QString reason = timed_out_ ? tr("工具等待超时") : tr("工具异常退出（退出码 %1）").arg(exit_code);
        Finish(false, tr("%1：%2\n%3").arg(steps_.at(step_).name, reason, output_.trimmed().right(2000)));
        return;
    }
    if (probing_) {
        HandleProbe();
        return;
    }
    if (!download_done_) {
        Finish(false, tr("%1：dfu-util 未报告传输完成。\n%2").arg(steps_.at(step_).name, output_.trimmed()));
        return;
    }
    emit ProgressChanged(100);
    if (++step_ == steps_.size()) {
        Finish(true, tr("dfu-util 已确认镜像传输完成。请按设备流程重启并验证固件版本。"));
        return;
    }
    probe_attempts_ = 0;
    QTimer::singleShot(1000, this, &Flasher::Probe);
}

void Flasher::HandleProbe()
{
    static const QRegularExpression row(
        R"dfu(^Found DFU: \[0451:6165\][^\r\n]*?\bdevnum=(\d+),([^\r\n]*?)\balt=(\d+),\s*name="([^"]*)")dfu",
        QRegularExpression::MultilineOption | QRegularExpression::CaseInsensitiveOption);
    static const QRegularExpression path_pattern(R"dfu(\bpath="([^"]+)")dfu");

    const Step& step = steps_.at(step_);
    QString device;
    QString path;
    int alt = -1;
    bool has_tispl = false;
    bool has_uboot = false;
    QStringList visible_interfaces;
    auto matches = row.globalMatch(output_);
    while (matches.hasNext()) {
        const auto entry = matches.next();
        const QString entry_path = path_pattern.match(entry.captured(2)).captured(1);
        const QString identity = entry.captured(1) + '|' + entry_path;
        if (!device.isEmpty() && identity != device) {
            Finish(false, tr("检测到多台 TI DFU 设备。请只连接待烧录设备后重试。"));
            return;
        }
        device = identity;
        path = entry_path;
        const int number = entry.captured(3).toInt();
        const QString name = entry.captured(4);
        visible_interfaces.append(QStringLiteral("alt=%1 %2").arg(number).arg(name));
        if (name == step.interface_name && (step.alt < 0 || number == step.alt)) {
            if (alt >= 0) {
                Finish(false, tr("检测到重复的 DFU 目标接口：%1").arg(name));
                return;
            }
            alt = number;
        }
        has_tispl |= number == 0 && name == QStringLiteral("tispl.bin");
        has_uboot |= number == 1 && name == QStringLiteral("u-boot.img");
    }
    if (!path.isEmpty() && !device_path_.isEmpty() && path != device_path_) {
        Finish(false, tr("DFU 设备的 USB 路径已变化。请确认连接后重试。"));
        return;
    }
    if (alt < 0 || (step.require_spl_pair && !has_uboot)) {
        if (step_ == 0 && has_tispl && has_uboot) {
            Finish(false, tr("设备已进入第二阶段 DFU（alt=0 tispl.bin，alt=1 u-boot.img）。"
                             "请让设备重新进入 USB DFU 启动模式，确认出现 bootloader 后再开始烧录。"));
            return;
        }
        const bool waiting_for_flash = step.interface_name == QStringLiteral("flash");
        if (probe_attempts_ < (waiting_for_flash ? kFlashProbeAttempts : kProbeAttempts)) {
            QTimer::singleShot(1000, this, &Flasher::Probe);
            return;
        }
        if (waiting_for_flash) {
            const QString observed = visible_interfaces.isEmpty()
                ? tr("未发现 TI DFU 设备") : visible_interfaces.join(QStringLiteral("、"));
            Finish(false, tr("u-boot.img 传输后仍未出现 flash DFU 接口。最后一次 USB 枚举：%1。"
                             "固件镜像尚未写入 Flash。请检查 USB 连接及 DFU 驱动，"
                             "并用 dfu-util -l 核对设备是否提供 name=\"flash\"。"
                             "若未提供，请检查所选 boot 文件与设备的 Flash Writer 固件是否匹配。").arg(observed));
            return;
        }
        const QString expected = step.require_spl_pair ? QStringLiteral("alt=0 tispl.bin、alt=1 u-boot.img")
                                                       : step.interface_name;
        Finish(false, tr("%1：等待 DFU 接口超时，预期 %2。").arg(step.name, expected));
        return;
    }

    if (!path.isEmpty()) {
        device_path_ = path;
    }
    QStringList arguments{QStringLiteral("-d"), kDeviceId};
    if (!path.isEmpty()) {
        arguments << QStringLiteral("-p") << path;
    }
    arguments << QStringLiteral("-R") << QStringLiteral("-a") << QString::number(alt)
              << QStringLiteral("-D") << step.file;
    probing_ = false;
    emit ProgressChanged(0);
    Run(arguments);
}

void Flasher::Finish(bool success, const QString& detail)
{
    timeout_.stop();
    step_ = -1;
    emit Completed(success, detail);
}

} // namespace ti_dfu
