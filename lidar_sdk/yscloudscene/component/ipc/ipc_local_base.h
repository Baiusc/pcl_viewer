#ifndef IPC_LOCAL_BASE_H_
#define IPC_LOCAL_BASE_H_

#include <QObject>
#include <QLocalSocket>
#include "ipc_json_protocol.h"

namespace ys_scene {
class IPCLocalBase : public QObject {
	Q_OBJECT
public:
    explicit IPCLocalBase(QObject* parent = nullptr);
    virtual ~IPCLocalBase();

    void SendMessage(const IPCMessage& message);

protected:
    QLocalSocket* socket_;
    QByteArray buffer_;

    virtual void Initialize() = 0;

signals:
    void MessageReceived(const IPCMessage& msg);

private slots:
    void HandleReadyRead();

};
} // namespace ys_scene

#endif // IPC_LOCAL_BASE_H_