#pragma once
#include "management_protocol.h"
#include <QElapsedTimer>
#include <QObject>
#include <QThread>
#include <QTimer>
#include <atomic>
#include <memory>

namespace ys_scene {
class ManagementClient : public QObject
{
    Q_OBJECT
  public:
    explicit ManagementClient(QObject *parent = nullptr);
    ~ManagementClient() override;
    void SetAddress(const QString &address);
    void SetActive(bool active);
    void Refresh();
    void Switch(int mode);

    const ManagementSnapshot &Snapshot() const
    {
        return snapshot_;
    }

    bool CanSwitch() const;

    bool IsFresh() const
    {
        return fresh_;
    }

    QString Address() const
    {
        return address_;
    }

  signals:
    void AddressChanged();
    void Changed();
    void Message(const QString &message);
    void Switching();
    void Stable(int mode);
  private:
    void Request(int mode);
    QThread thread_;
    QObject *worker_;
    QTimer timer_;
    QElapsedTimer observed_;
    std::shared_ptr<std::atomic<bool>> cancel_;
    ManagementSnapshot snapshot_;
    QString address_, operation_;
    quint64 generation_ = 0;
    int requested_ = -1;
    bool active_ = false, busy_ = false, fresh_ = false, uncertain_ = false;
};
} // namespace ys_scene
