#include "sn_client.h"

#include <QDateTime>
#include <QDir>
#include <QFile>
#include <QJsonDocument>
#include <QNetworkDatagram>
#include <QStandardPaths>
#include <QUuid>
#include <QtConcurrentRun>

namespace ys_scene::lidar_sn {
namespace {

using namespace lidar_network;

constexpr int kObservationLifetimeMs = 20000;
constexpr int kRecoveryTimeoutMs = 60000;

QString FindSsh()
{
#ifdef Q_OS_WIN
    const QDir windows(qEnvironmentVariable("SystemRoot"));
    return QStandardPaths::findExecutable(
        QStringLiteral("ssh.exe"),
        {windows.filePath(QStringLiteral("System32/OpenSSH")), windows.filePath(QStringLiteral("Sysnative/OpenSSH"))});
#else
    return QStandardPaths::findExecutable(QStringLiteral("ssh"));
#endif
}

} // namespace

SnClient::SnClient(QObject *parent)
    : QObject(parent)
{
    clock_.start();
    connect(&udp_, &QUdpSocket::readyRead, this, &SnClient::Receive);
    udp_.bind(QHostAddress::AnyIPv4, kDiscoveryPort, QUdpSocket::ShareAddress | QUdpSocket::ReuseAddressHint);
    connect(&http_, &QFutureWatcher<Reply>::finished, this, &SnClient::OnHttpResult);
    connect(&discovery_, &QFutureWatcher<DiscoveryCheck>::finished, this, &SnClient::OnDiscoveryResult);
    timer_.start(1000);
    connect(&timer_, &QTimer::timeout, this, [this] {
        bool removed = false;
        for (auto it = devices_.begin(); it != devices_.end();) {
            if (clock_.elapsed() - it->seen_at >= kObservationLifetimeMs) {
                it = devices_.erase(it);
                removed = true;
            }
            else {
                ++it;
            }
        }
        if (removed) {
            emit DevicesChanged();
        }
        if (stage_ == Stage::KWaiting) {
            Poll();
        }
    });

    ssh_timeout_.setSingleShot(true);
    connect(&ssh_timeout_, &QTimer::timeout, this, [this] {
        ssh_timed_out_ = true;
        ssh_process_.kill();
    });
    connect(&ssh_process_, &QProcess::readyReadStandardError, this,
            [this] { ssh_errors_ = (ssh_errors_ + ssh_process_.readAllStandardError()).right(2000); });
    connect(&ssh_process_, &QProcess::readyReadStandardOutput, this, [this] { ssh_process_.readAllStandardOutput(); });
    connect(&ssh_process_, qOverload<int, QProcess::ExitStatus>(&QProcess::finished), this, &SnClient::OnSshResult);
    connect(&ssh_process_, &QProcess::errorOccurred, this, [this](QProcess::ProcessError error) {
        if (error == QProcess::FailedToStart) {
            ssh_errors_ = ssh_process_.errorString().toUtf8();
            OnSshResult(-1, QProcess::CrashExit);
        }
    });
}

bool SnClient::IsListening() const
{
    return udp_.state() == QAbstractSocket::BoundState;
}

bool SnClient::IsBusy() const
{
    return stage_ != Stage::KIdle;
}

QList<Announcement> SnClient::Devices() const
{
    QList<Announcement> devices;
    for (const auto &device : devices_) {
        devices.append(device.announcement);
    }
    return devices;
}

bool SnClient::IsDuplicate(const QString &sn, const QString &mac) const
{
    if (sn.isEmpty()) {
        return false;
    }
    for (const auto &device : devices_) {
        if (device.announcement.sn == sn && device.announcement.mac != mac) {
            return true;
        }
    }
    return false;
}

void SnClient::Receive()
{
    while (udp_.hasPendingDatagrams()) {
        const auto packet = udp_.receiveDatagram(4097);
        auto device = ParseAnnouncement(packet, QNetworkInterface::interfaceFromIndex(packet.interfaceIndex()));
        if (!device.mac.isEmpty()) {
            const auto sn = NormalizeSn(device.sn);
            // A legacy MAC-based identity does not satisfy the SN format.
            device.sn = sn.isEmpty() ? NormalizeSn(device.identity) : sn;
            Remember(device);
        }
    }
}

void SnClient::Remember(const Announcement &device)
{
    const auto &ep = device.endpoint;
    const auto key = QStringLiteral("%1/%2/%3/%4").arg(ep.interface_id, ep.address, device.mac).arg(ep.port);
    const auto previous = devices_.value(key).announcement;
    devices_.insert(key, {device, clock_.elapsed()});
    if (previous.identity != device.identity || previous.sn != device.sn) {
        emit DevicesChanged();
    }
}

void SnClient::Read(const Endpoint &endpoint, const QString &expected_mac, bool diagnostic)
{
    if (!IsBusy()) {
        expected_mac_ = expected_mac;
        Query(diagnostic ? Stage::KDiagnostic : Stage::KRead, endpoint);
    }
}

void SnClient::Start(const Endpoint &endpoint, const WorkOrder &order, WriteDecision requested)
{
    if (IsBusy() || requested == WriteDecision::KBlocked) {
        return;
    }
    original_ = endpoint;
    order_ = order;
    requested_ = requested;
    expected_mac_ = order.mac;
    command_issue_.clear();
    discovery_host_.clear();
    Query(Stage::KPreflight, endpoint);
}

void SnClient::Query(Stage stage, const Endpoint &endpoint)
{
    const auto source = SourceAddress(endpoint);
    if (source.isEmpty()) {
        if (stage != Stage::KPoll) {
            emit IdentityChanged({}, endpoint);
            Complete(QStringLiteral("unreachable"), tr("所选网卡没有到 %1 的同网段 IPv4。").arg(endpoint.address));
        }
        return;
    }
    const bool was_busy = IsBusy();
    stage_ = stage;
    queried_ = endpoint;
    if (!was_busy) {
        emit BusyChanged(true);
        emit StatusChanged(tr("正在读取设备身份…"));
    }
    http_.setFuture(
        QtConcurrent::run(lidar_network::Query, endpoint, source, QJsonObject{}, stage == Stage::KDiagnostic));
}

void SnClient::OnHttpResult()
{
    const auto reply = http_.result();
    Identity identity;
    const bool parsed = ParseIdentity(reply, identity);
    const bool matches = parsed && (expected_mac_.isEmpty() || expected_mac_ == identity.mac);
    if (parsed) {
        Remember({queried_, identity.mac, identity.identity, identity.sn});
    }
    if (stage_ == Stage::KPoll) {
        stage_ = Stage::KWaiting;
        if (!matches) {
            return;
        }
        emit IdentityChanged(identity, queried_);
        const auto decision = CheckTarget(identity, order_);
        if (!command_issue_.isEmpty() || decision == WriteDecision::KBlocked) {
            Complete(QStringLiteral("pending"), tr("已重新读取设备，写入结果待人工核验。"));
        }
        else if (decision == WriteDecision::KAlreadyWritten) {
            stage_ = Stage::KDiscovery;
            emit StatusChanged(tr("HTTP 回读一致，正在核对 DNS-SD TXT 记录…"));
            discovery_.setFuture(QtConcurrent::run(VerifyDiscovery, queried_, order_.sn));
        }
        return;
    }

    emit IdentityChanged(matches ? identity : Identity{}, queried_);
    if (!matches) {
        const auto detail = parsed ? tr("HTTP 返回的 MAC 与所选设备或工单不一致。")
                                   : tr("设备身份查询失败（HTTP %1） %2").arg(reply.status).arg(reply.error);
        Complete(QStringLiteral("read_failed"), detail);
        return;
    }
    if (stage_ == Stage::KRead || stage_ == Stage::KDiagnostic) {
        Complete({}, stage_ == Stage::KDiagnostic ? tr("已读取只读诊断状态。") : tr("已读取在线状态。"));
        return;
    }

    const auto decision = CheckTarget(identity, order_);
    if (decision == WriteDecision::KBlocked || IsDuplicate(order_.sn, order_.mac) ||
        (decision == WriteDecision::KProvision && requested_ != WriteDecision::KProvision)) {
        Complete(QStringLiteral("blocked"), tr("工单与在线设备不符，或发现重复 SN。"));
        return;
    }
    if (decision == WriteDecision::KProvision) {
        ssh_ = FindSsh();
        if (ssh_.isEmpty()) {
            Complete(QStringLiteral("blocked"), tr("找不到系统 OpenSSH 客户端。"));
            return;
        }
    }
    audit_path_ = QDir(AuditDirectory()).filePath(QUuid::createUuid().toString(QUuid::WithoutBraces) + ".jsonl");
    if (!SaveAudit(QStringLiteral("started"), QString())) {
        audit_path_.clear();
        Complete(QStringLiteral("blocked"), tr("无法保存本地工单记录。"));
        return;
    }
    if (decision == WriteDecision::KAlreadyWritten) {
        stage_ = Stage::KDiscovery;
        emit StatusChanged(tr("设备已写入此 SN，正在核对 DNS-SD TXT 记录…"));
        discovery_.setFuture(QtConcurrent::run(VerifyDiscovery, queried_, order_.sn));
        return;
    }
    RunSsh(Stage::KProvision);
}

void SnClient::RunSsh(Stage stage)
{
    const auto source = SourceAddress(original_);
    if (source.isEmpty()) {
        Complete(QStringLiteral("pending"), tr("设备所在网卡已失效，请重新读取后核验。"));
        return;
    }
    stage_ = stage;
    QString alias = order_.mac;
    alias.remove(':');
    const QStringList arguments{"-F",
                                "none",
                                "-4",
                                "-T",
                                "-n",
                                "-b",
                                source,
                                "-o",
                                "BatchMode=yes",
                                "-o",
                                "StrictHostKeyChecking=yes",
                                "-o",
                                "GlobalKnownHostsFile=none",
                                "-o",
                                "UpdateHostKeys=no",
                                "-o",
                                "ConnectTimeout=8",
                                "-o",
                                "PreferredAuthentications=publickey",
                                "-o",
                                "HostKeyAlias=lidar-mac-" + alias,
                                "root@" + original_.address};
    // The work-order MAC and SN have been validated against strict ASCII formats.
    const QString guard =
        QStringLiteral("grep -F -x -q '%1' /sys/class/net/*/address || exit 3; exec ").arg(order_.mac);
    const QString command = stage == Stage::KProvision
                                ? QStringLiteral("/usr/libexec/lidar/lidar_sn_provision ") + order_.sn
                                : QStringLiteral("/usr/libexec/lidar/lidar-network-service restart");
    emit StatusChanged(stage == Stage::KProvision ? tr("正在通过受控 SSH 写入 SN…") : tr("正在重启设备网络服务…"));
    ssh_errors_.clear();
    ssh_timed_out_ = false;
    ssh_timeout_.start(60000);
    ssh_process_.start(ssh_, arguments + QStringList{guard + command});
}

void SnClient::OnSshResult(int exit_code, QProcess::ExitStatus status)
{
    ssh_timeout_.stop();
    ssh_errors_ = (ssh_errors_ + ssh_process_.readAllStandardError()).right(2000);
    const bool ok = !ssh_timed_out_ && status == QProcess::NormalExit && exit_code == 0;
    if (ok && stage_ == Stage::KProvision) {
        RunSsh(Stage::KRestart);
        return;
    }
    if (!ok) {
        const auto operation = stage_ == Stage::KProvision ? tr("写码命令") : tr("重启命令");
        const auto reason = ssh_timed_out_ ? tr("超时") : tr("退出码 %1").arg(exit_code);
        command_issue_ = tr("%1未确认完成（%2）。%3").arg(operation, reason, QString::fromUtf8(ssh_errors_).trimmed());
    }
    next_poll_ = 0;
    recovery_.start();
    stage_ = Stage::KWaiting;
    emit StatusChanged(command_issue_.isEmpty() ? tr("正在等待网络服务恢复并回读 SN…")
                                                : tr("SSH 阶段异常，正在重新读取设备身份…"));
}

void SnClient::Poll()
{
    if (recovery_.elapsed() >= kRecoveryTimeoutMs) {
        Complete(QStringLiteral("pending"), tr("HTTP 回读等待超时，写入结果待核验。"));
        return;
    }
    QList<Endpoint> options{original_};
    for (const auto &seen : devices_) {
        const auto &device = seen.announcement;
        if (device.mac == order_.mac && device.endpoint.interface_id == original_.interface_id &&
            (device.endpoint.address != original_.address || device.endpoint.port != original_.port)) {
            options.append(device.endpoint);
        }
    }
    Query(Stage::KPoll, options.at(next_poll_++ % options.size()));
}

void SnClient::OnDiscoveryResult()
{
    const auto result = discovery_.result();
    discovery_host_ = result.host;
    if (result.conflicting_address || IsDuplicate(order_.sn, order_.mac)) {
        Complete(QStringLiteral("duplicate"), tr("HTTP 回读一致，但发现重复 SN 或冲突的 DNS-SD 地址。请核对工单。"));
    }
    else if (result.host.isEmpty()) {
        Complete(QStringLiteral("dns_pending"), tr("HTTP 回读一致，DNS-SD 暂未核对通过。可重新核验。"));
    }
    else {
        Complete(QStringLiteral("verified"),
                 tr("HTTP 和 DNS-SD 核验通过。设备 %1，服务主机 %2。").arg(queried_.address, result.host));
    }
}

QString SnClient::AuditDirectory()
{
    return QDir(QStandardPaths::writableLocation(QStandardPaths::AppLocalDataLocation)).filePath("sn_audit");
}

bool SnClient::SaveAudit(const QString &outcome, const QString &detail)
{
    if (!QDir().mkpath(AuditDirectory())) {
        return false;
    }
    QFile file(audit_path_);
    if (!file.open(QIODevice::WriteOnly | QIODevice::Append)) {
        return false;
    }
    const QJsonObject record{{"time_utc", QDateTime::currentDateTimeUtc().toString(Qt::ISODateWithMs)},
                             {"work_order", order_.number},
                             {"operator", order_.operator_id},
                             {"mac", order_.mac},
                             {"sn", order_.sn},
                             {"original_address", original_.address},
                             {"last_http_address", queried_.address},
                             {"port", queried_.port},
                             {"interface", original_.interface_id},
                             {"dns_host", discovery_host_},
                             {"result", outcome},
                             {"detail", detail},
                             {"requested", requested_ == WriteDecision::KProvision ? "provision" : "verify"}};
    const auto line = QJsonDocument(record).toJson(QJsonDocument::Compact) + '\n';
    return file.write(line) == line.size() && file.flush();
}

void SnClient::Complete(const QString &outcome, QString message)
{
    if (!audit_path_.isEmpty()) {
        if (!command_issue_.isEmpty()) {
            message += QStringLiteral("\n") + command_issue_;
        }
        if (!SaveAudit(outcome, message)) {
            message += QStringLiteral("\n") + tr("本地工单记录写入失败，请补录。");
        }
        audit_path_.clear();
    }
    stage_ = Stage::KIdle;
    emit StatusChanged(message);
    emit BusyChanged(false);
}

} // namespace ys_scene::lidar_sn
