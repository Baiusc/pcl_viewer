#include "ipc_local_server.h"
#include <QDebug>

namespace ys_scene {

    // IPCLocalConnection 实现
    IPCLocalConnection::IPCLocalConnection(QLocalSocket* client_socket, QObject* parent)
        : IPCLocalBase(parent) {
        socket_ = client_socket;
        connect(socket_, &QLocalSocket::disconnected, this, &IPCLocalConnection::HandleDisconnected);
        connect(this, &IPCLocalBase::MessageReceived, this, &IPCLocalConnection::HandleMessageReceived);
    }

    IPCLocalConnection::~IPCLocalConnection() {
        socket_->disconnectFromServer();
    }

    void IPCLocalConnection::HandleMessageReceived(const IPCMessage& msg)
    {
        emit MessageReceived(msg);
    }

    void IPCLocalConnection::Initialize() {
        // 服务器端特有的初始化
    }

    void IPCLocalConnection::SendMessageToClient(const IPCMessage& message) {
        SendMessage(message);
    }

    void IPCLocalConnection::HandleDisconnected() {
        emit Disconnected(this);
    }

    // IPCLocalServer 实现
    IPCLocalServer::IPCLocalServer(const QString& server_name, QObject* parent)
        : QObject(parent),
        local_server_(new QLocalServer(this)),
        server_name_(server_name) {

        connect(local_server_, &QLocalServer::newConnection, this, &IPCLocalServer::HandleNewConnection);

        // 如果服务器已经存在，先移除
        if (QLocalServer::removeServer(server_name_)) {
            qDebug() << "Removed existing IPC server.";
        }

        if (!local_server_->listen(server_name_)) {
            qWarning() << "Unable to start the IPC server:" << local_server_->errorString();
        }
        else {
            qDebug() << "IPC Server started successfully.";
        }
    }

    IPCLocalServer::~IPCLocalServer() {
        foreach(auto connection, connections_) {
            connection->disconnect();
            connection->deleteLater();
        }
        local_server_->close();
    }

    void IPCLocalServer::HandleNewConnection() {
        while (local_server_->hasPendingConnections()) {
            QLocalSocket* client_socket = local_server_->nextPendingConnection();
            IPCLocalConnection* connection = new IPCLocalConnection(client_socket, this);
            connections_.append(connection);

            connect(connection, &IPCLocalConnection::MessageReceived, this, &IPCLocalServer::HandleMessageReceived);
            connect(connection, &IPCLocalConnection::Disconnected, this, &IPCLocalServer::HandleClientDisconnected);

            qDebug() << "New IPC client connected.";
        }
    }

    void IPCLocalServer::SendMessageToAll(const IPCMessage& message) {
        foreach(auto connection, connections_) {
            connection->SendMessageToClient(message);
        }
    }

    void IPCLocalServer::SendMessage(const IPCMessage& message) {
        SendMessageToAll(message);
    }

    void IPCLocalServer::HandleClientDisconnected(IPCLocalConnection* connection) {
        connections_.removeAll(connection);
        connection->deleteLater();
        qDebug() << "IPC client disconnected.";

        if (connections_.isEmpty()) {
            emit AllClientsDisconnected();
        }
    }

    void IPCLocalServer::HandleMessageReceived(const IPCMessage& msg) {
        // 处理收到的消息
        if (msg.command == "KnobValueChanged") {
            int value = msg.data["value"].toInt();
            // 例如，转发或通知业务逻辑层
            qDebug() << "Received KnobValueChanged with value:" << value;
            // 在此处添加业务处理逻辑
        }
        // 处理其他命令
    }

} // namespace ys_scene