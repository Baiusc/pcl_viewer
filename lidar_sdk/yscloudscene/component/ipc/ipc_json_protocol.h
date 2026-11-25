#ifndef IPC_JSON_PROTOCOL_h_
#define IPC_JSON_PROTOCOL_h_

#include <QString>
#include <QJsonDocument>
#include <QJsonObject>

namespace ys_scene{

struct IPCMessage{

    QJsonDocument ToJson() const;
    QString command;
    QJsonObject data;
};

IPCMessage FromJson(const QByteArray& bytes);

}// namespace ys_scene

#endif //IPC_JSON_PROTOCOL_h_
