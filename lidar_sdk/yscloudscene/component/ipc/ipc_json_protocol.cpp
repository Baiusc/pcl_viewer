#include "ipc_json_protocol.h"

namespace ys_scene{

QJsonDocument IPCMessage::ToJson() const
{
    QJsonObject obj;
    obj["command"] = command;
    obj["data"] = data;
    return QJsonDocument(obj);
}

IPCMessage FromJson(const QByteArray &bytes)
{
    QJsonDocument doc = QJsonDocument::fromJson(bytes);
    IPCMessage msg;
    if (doc.isObject()) {
        QJsonObject obj = doc.object();
        msg.command = obj["command"].toString();
        msg.data = obj["data"].toObject();
    }
    return msg;
}

} // namespace ys_scene
