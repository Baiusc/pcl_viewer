#ifndef IPC_LOCAL_SERVER_H_
#define IPC_LOCAL_SERVER_H_

#include "ipc_local_base.h"
#include <QLocalServer>
#include <QList>

namespace ys_scene {

    class IPCLocalConnection : public IPCLocalBase {
        Q_OBJECT
    public:
        explicit IPCLocalConnection(QLocalSocket* client_socket, QObject* parent = nullptr);
        ~IPCLocalConnection();

    signals:
        void Disconnected(IPCLocalConnection* connection);
        void MessageReceived(const IPCMessage& msg);

    public slots:
        void SendMessageToClient(const IPCMessage& message);

    private slots:
        void HandleDisconnected();
        void HandleMessageReceived(const IPCMessage& msg);

    private:
        void Initialize() override;
    };

    class IPCLocalServer : public QObject {
        Q_OBJECT
    public:
        explicit IPCLocalServer(const QString& server_name, QObject* parent = nullptr);
        ~IPCLocalServer();

        void SendMessageToAll(const IPCMessage& message);

    signals:
        void AllClientsDisconnected();
        void MessageReceived(const IPCMessage& msg);

    public slots:
        void SendMessage(const IPCMessage& message);

    private slots:
        void HandleNewConnection();
        void HandleClientDisconnected(IPCLocalConnection* connection);
        void HandleMessageReceived(const IPCMessage& msg);

    private:
        QLocalServer* local_server_;
        QString server_name_;
        QList<IPCLocalConnection*> connections_;
    };

} // namespace ys_scene

#endif // IPC_LOCAL_SERVER_H_