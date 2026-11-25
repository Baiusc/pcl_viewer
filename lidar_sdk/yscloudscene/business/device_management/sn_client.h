#pragma once

#include "sn_discovery.h"
#include "sn_protocol.h"

#include <QElapsedTimer>
#include <QFutureWatcher>
#include <QHash>
#include <QProcess>
#include <QTimer>
#include <QUdpSocket>

namespace ys_scene::lidar_sn {

class SnClient : public QObject
{
    Q_OBJECT
  public:
    explicit SnClient(QObject *parent = nullptr);

    bool IsBusy() const;
    bool IsListening() const;
    QList<lidar_network::Announcement> Devices() const;
    bool IsDuplicate(const QString &sn, const QString &mac) const;
    void Read(const lidar_network::Endpoint &endpoint, const QString &expected_mac, bool diagnostic);
    void Start(const lidar_network::Endpoint &endpoint, const WorkOrder &order, WriteDecision requested);

    static QString AuditDirectory();

  signals:
    void DevicesChanged();
    void IdentityChanged(const Identity &identity, const lidar_network::Endpoint &endpoint);
    void StatusChanged(const QString &message);
    void BusyChanged(bool busy);
  private:
    enum class Stage { KIdle, KRead, KDiagnostic, KPreflight, KProvision, KRestart, KWaiting, KPoll, KDiscovery };

    struct SeenDevice
    {
        lidar_network::Announcement announcement;
        qint64 seen_at = 0;
    };

    void Receive();
    void Remember(const lidar_network::Announcement &device);
    void Query(Stage stage, const lidar_network::Endpoint &endpoint);
    void OnHttpResult();
    void RunSsh(Stage stage);
    void OnSshResult(int exit_code, QProcess::ExitStatus status);
    void Poll();
    void OnDiscoveryResult();
    void Complete(const QString &outcome, QString message);
    bool SaveAudit(const QString &outcome, const QString &detail);

    Stage stage_ = Stage::KIdle;
    lidar_network::Endpoint original_;
    lidar_network::Endpoint queried_;
    WorkOrder order_;
    WriteDecision requested_ = WriteDecision::KBlocked;
    QString expected_mac_;
    QString audit_path_;
    QString command_issue_;
    QString discovery_host_;
    QString ssh_;
    bool ssh_timed_out_ = false;
    int next_poll_ = 0;
    QElapsedTimer clock_;
    QElapsedTimer recovery_;
    QHash<QString, SeenDevice> devices_;
    QUdpSocket udp_;
    QTimer timer_;
    QTimer ssh_timeout_;
    QProcess ssh_process_;
    QByteArray ssh_errors_;
    QFutureWatcher<lidar_network::Reply> http_;
    QFutureWatcher<DiscoveryCheck> discovery_;
};

} // namespace ys_scene::lidar_sn
