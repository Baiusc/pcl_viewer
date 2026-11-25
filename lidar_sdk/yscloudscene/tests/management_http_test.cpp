#include "business/device_management/management_client.h"
#include <QFile>
#include <QJsonArray>
#include <QJsonDocument>
#include <QTcpServer>
#include <QTcpSocket>
#include <QtTest>
using namespace ys_scene;

class ManagementHttpTest : public QObject
{
    Q_OBJECT
  private slots:

    void DeviceStatus()
    {
        const auto endpoint = qEnvironmentVariable("YS_MANAGEMENT_TEST_URL");
        if (endpoint.isEmpty())
            QSKIP("Set YS_MANAGEMENT_TEST_URL for read-only device verification");
        ManagementClient client;
        client.SetActive(true);
        client.SetAddress(endpoint);
        QTRY_VERIFY_WITH_TIMEOUT(client.IsFresh(), 15000);
        QVERIFY(!client.Snapshot().operation_id.isEmpty());
        client.SetActive(false);
    }

    void DeviceModes()
    {
        if (qEnvironmentVariable("YS_MANAGEMENT_TEST_SWITCH") != "221")
            QSKIP("Explicit 221 mode-switch integration test is disabled");
        ManagementClient client;
        client.SetActive(true);
        client.SetAddress(QStringLiteral("http://10.42.0.221"));
        QTRY_VERIFY_WITH_TIMEOUT(client.CanSwitch(), 15000);
        QCOMPARE(client.Snapshot().actual, 2);
        client.Switch(1);
        QTRY_VERIFY_WITH_TIMEOUT(client.CanSwitch() && client.Snapshot().actual == 1, 60000);
        QCOMPARE(client.Snapshot().confirmed, 1);
        QVERIFY(client.Snapshot().error.isEmpty());
        client.Switch(2);
        QTRY_VERIFY_WITH_TIMEOUT(client.CanSwitch() && client.Snapshot().actual == 2, 60000);
        QCOMPARE(client.Snapshot().confirmed, 2);
        QVERIFY(client.Snapshot().error.isEmpty());
        client.SetActive(false);
    }

    void SwitchOnce_data()
    {
        QTest::addColumn<QString>("postCase");
        QTest::addColumn<QString>("finalCase");
        QTest::addColumn<int>("expectedMode");
        QTest::addColumn<QString>("expectedMessage");
        QTest::newRow("success") << "switch_accepted" << "switch_succeeded" << 1 << QStringLiteral("切换成功");
        QTest::newRow("rollback") << "switch_accepted" << "rolled_back" << 2 << QStringLiteral("目标切换未成功");
        QTest::newRow("operation-changed")
            << "switch_accepted" << "operation_changed" << 2 << QStringLiteral("操作标识已变化");
        QTest::newRow("rejected") << "busy_switch" << "ready_mode_2" << 2 << QStringLiteral("切换被拒绝");
        QTest::newRow("ambiguous") << "bridge_timeout" << "ready_mode_2" << 2 << QStringLiteral("请求结果待确认");
    }

    void SwitchOnce()
    {
        QFETCH(QString, postCase);
        QFETCH(QString, finalCase);
        QFETCH(int, expectedMode);
        QFETCH(QString, expectedMessage);
        QFile file(QStringLiteral(MANAGEMENT_EXAMPLES));
        QVERIFY(file.open(QIODevice::ReadOnly));
        QMap<QString, QJsonObject> examples;
        for (const auto &entry : QJsonDocument::fromJson(file.readAll()).object()["scenarios"].toArray())
            examples[entry.toObject()["name"].toString()] = entry.toObject()["body"].toObject();
        QTcpServer server;
        QVERIFY(server.listen(QHostAddress::LocalHost));
        int posts = 0;
        connect(&server, &QTcpServer::newConnection, this, [&] {
            auto *socket = server.nextPendingConnection();
            connect(socket, &QTcpSocket::disconnected, socket, &QObject::deleteLater);
            connect(socket, &QTcpSocket::readyRead, socket, [&, socket] {
                const auto request = socket->property("request").toByteArray() + socket->readAll();
                socket->setProperty("request", request);
                if (!request.contains("\r\n\r\n"))
                    return;
                const bool post = request.startsWith("POST ");
                if (post && !request.endsWith("}"))
                    return;
                if (post)
                    ++posts;
                const auto body = examples[post ? postCase : posts ? finalCase : QStringLiteral("ready_mode_2")];
                const auto data = QJsonDocument(body).toJson(QJsonDocument::Compact);
                socket->write("HTTP/1.1 " + QByteArray::number(post ? body["code"].toInt() : 200) +
                              " OK\r\nContent-Type: application/json\r\nContent-Length: " +
                              QByteArray::number(data.size()) + "\r\nConnection: close\r\n\r\n" + data);
                socket->disconnectFromHost();
            });
        });
        ManagementClient client;
        QSignalSpy messages(&client, &ManagementClient::Message);
        client.SetActive(true);
        client.SetAddress(QStringLiteral("http://127.0.0.1:%1").arg(server.serverPort()));
        QTRY_VERIFY_WITH_TIMEOUT(client.CanSwitch(), 5000);
        client.Switch(0);
        QCOMPARE(posts, 0);
        client.Switch(1);
        client.Switch(1);
        QTRY_VERIFY_WITH_TIMEOUT(
            [&] {
                for (const auto &message : messages)
                    if (message[0].toString().contains(expectedMessage))
                        return true;
                return false;
            }(),
            5000);
        QTRY_VERIFY_WITH_TIMEOUT(client.CanSwitch(), 5000);
        QCOMPARE(client.Snapshot().actual, expectedMode);
        QCOMPARE(posts, 1);
        QVERIFY(client.CanSwitch());
        bool success = false;
        for (const auto &message : messages)
            success |= message[0].toString().contains(expectedMessage);
        QVERIFY(success);
        client.SetActive(false);
    }
};
QTEST_GUILESS_MAIN(ManagementHttpTest)
#include "management_http_test.moc"
