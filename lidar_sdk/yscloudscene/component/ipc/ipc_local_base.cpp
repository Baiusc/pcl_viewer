#include "ipc_local_base.h"
#include <QDataStream>
#include <QDebug>

namespace ys_scene {
IPCLocalBase::IPCLocalBase(QObject* parent) : 
    QObject(parent), 
    socket_(new QLocalSocket(this)) {
    connect(socket_, &QLocalSocket::readyRead, this, &IPCLocalBase::HandleReadyRead);
}

IPCLocalBase::~IPCLocalBase(){
    socket_->disconnectFromServer();
}

void IPCLocalBase::SendMessage(const IPCMessage& message){
    QByteArray jsonBytes = message.ToJson().toJson(QJsonDocument::Compact);
    quint32 length = jsonBytes.size();

    QByteArray packet;
    QDataStream stream(&packet, QIODevice::WriteOnly);
    stream.setByteOrder(QDataStream::LittleEndian);
    stream << length;
    packet.append(jsonBytes);

    if (socket_->state() == QLocalSocket::ConnectedState) {
        socket_->write(packet);
        socket_->flush();
    }
    else {
        qWarning() << "Cannot send message, socket is not connected.";
    }
}

void IPCLocalBase::HandleReadyRead() {
    buffer_.append(socket_->readAll());

    while (buffer_.size() >= 4) { // 检查是否有足够的长度前缀
        QDataStream stream(buffer_);
        stream.setByteOrder(QDataStream::LittleEndian);
        quint32 messageSize;
        stream >> messageSize;

        if (buffer_.size() < 4 + messageSize) {
            // 数据不足，等待更多数据
            break;
        }

        QByteArray messageBytes = buffer_.mid(4, messageSize);
        buffer_ = buffer_.mid(4 + messageSize);

        IPCMessage msg = FromJson(messageBytes);
        //ProcessMessage(msg);
    }
}

}