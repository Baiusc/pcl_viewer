#include "business/device_management/sn_client.h"

#include <QJsonDocument>
#include <QNetworkDatagram>
#include <QTcpServer>
#include <QTcpSocket>
#include <QtTest>

using namespace ys_scene;
using namespace ys_scene::lidar_sn;

namespace {

const QString kSn = QStringLiteral("CS96XXXB01026020001");
const QString kMac = QStringLiteral("98:03:8a:7f:32:ee");

QJsonObject Snapshot(const QString &sn = {}, const QString &mac = kMac)
{
    return {{"code", 200},
            {"schema_version", 1},
            {"model", "CS96-beta"},
            {"mac", mac},
            {"sn", sn},
            {"identity_source", sn.isEmpty() ? "mac" : "sn"},
            {"identity", sn.isEmpty() ? mac : sn},
            {"mdns_running", true}};
}

QString LoopbackInterface()
{
    for (const auto &nic : QNetworkInterface::allInterfaces()) {
        for (const auto &entry : nic.addressEntries()) {
            if (entry.ip() == QHostAddress(QHostAddress::LocalHost)) {
                return nic.name();
            }
        }
    }
    return {};
}

class DeviceServer : public QTcpServer
{
  public:
    explicit DeviceServer(const QJsonObject &snapshot)
        : body_(QJsonDocument(snapshot).toJson(QJsonDocument::Compact))
    {
        connect(this, &QTcpServer::newConnection, this, [this] {
            auto *socket = nextPendingConnection();
            connect(socket, &QTcpSocket::readyRead, this, [this, socket] {
                if (!socket->peek(4096).contains("\r\n\r\n")) {
                    return;
                }
                request = socket->readAll();
                socket->write("HTTP/1.1 200 OK\r\nConnection: close\r\nContent-Length: " +
                              QByteArray::number(body_.size()) + "\r\n\r\n" + body_);
                socket->disconnectFromHost();
            });
        });
    }

    QByteArray request;
  private:
    QByteArray body_;
};

} // namespace

class SnManagementTest : public QObject
{
    Q_OBJECT

  private slots:
    void SnFormat();
    void StatusContractAndWriteGuard();
    void DiscoveryTracksReceivingInterface();
    void HttpReadDiagnosticAndDuplicate();
    void VerifyOnlyCannotWriteAfterDeviceChanges();
};

void SnManagementTest::SnFormat()
{
    QCOMPARE(NormalizeSn("CS96XXX-B01-0-2602-0001"), kSn);
    QCOMPARE(NormalizeSn("MF15XX1-A09-1-2653-FFFF"), QStringLiteral("MF15XX1A0912653FFFF"));
    for (const auto *invalid :
         {"CS96XXXB01026020000", "CS96XXXB01026000001", "CS96XXXB01026540001", "CS96XXXB01226020001",
          "CS96XXXB0102602000G", "CS96XXX-B01-0-2602-000a", "CS96XXX--B01-0-2602-0001", "98:03:8a:7f:32:ee"}) {
        QVERIFY2(NormalizeSn(QString::fromLatin1(invalid)).isEmpty(), invalid);
    }
}

void SnManagementTest::StatusContractAndWriteGuard()
{
    const WorkOrder order{"job-42", "operator-5", kMac, kSn};
    Identity identity;
    QVERIFY(ParseIdentity({200, Snapshot(), {}}, identity));
    QCOMPARE(identity.source, QStringLiteral("mac"));
    QVERIFY(identity.sn.isEmpty());
    QCOMPARE(CheckTarget(identity, order), WriteDecision::KProvision);
    QVERIFY(ParseIdentity({200, Snapshot(kSn), {}}, identity));
    QCOMPARE(CheckTarget(identity, order), WriteDecision::KAlreadyWritten);
    QVERIFY(ParseIdentity({200, Snapshot("CS96XXXB01026020002"), {}}, identity));
    QCOMPARE(CheckTarget(identity, order), WriteDecision::KBlocked);
    QVERIFY(ParseIdentity({200, Snapshot({}, "98:03:8a:7f:32:ef"), {}}, identity));
    QCOMPARE(CheckTarget(identity, order), WriteDecision::KBlocked);

    auto json = Snapshot();
    json.insert("simulation", true);
    QVERIFY(ParseIdentity({200, json, {}}, identity));
    QCOMPARE(CheckTarget(identity, order), WriteDecision::KBlocked);
    QVERIFY(!ParseIdentity({503, json, {}}, identity));
    json.insert("schema_version", 2);
    QVERIFY(!ParseIdentity({200, json, {}}, identity));
    json = Snapshot(kSn);
    json.insert("identity_source", "mac");
    QVERIFY(!ParseIdentity({200, json, {}}, identity));
}

void SnManagementTest::DiscoveryTracksReceivingInterface()
{
    const auto nic = QNetworkInterface::interfaceFromName(LoopbackInterface());
    QVERIFY(nic.isValid());
    const QJsonObject announcement{{"protocol", "lidar-network"},
                                   {"schema_version", 1},
                                   {"service", "http"},
                                   {"path", "/cgi-bin/lidar_network.cgi"},
                                   {"ready", true},
                                   {"recovery", false},
                                   {"identity", kSn},
                                   {"sn", kSn},
                                   {"mac", kMac},
                                   {"address", "192.168.1.100"},
                                   {"port", 8000}};
    QNetworkDatagram packet(QJsonDocument(announcement).toJson(QJsonDocument::Compact));
    packet.setSender(QHostAddress("192.168.1.100"), lidar_network::kDiscoveryPort);
    packet.setInterfaceIndex(nic.index());
    const auto device = lidar_network::ParseAnnouncement(packet, nic);
    QCOMPARE(device.endpoint.interface_id, nic.name());
    QCOMPARE(device.endpoint.address, QStringLiteral("192.168.1.100"));
    QCOMPARE(device.endpoint.port, 8000);
    QCOMPARE(device.mac, kMac);
    QCOMPARE(device.sn, kSn);

    packet.setInterfaceIndex(0);
    QVERIFY(lidar_network::ParseAnnouncement(packet, nic).mac.isEmpty());
    packet.setInterfaceIndex(nic.index());
    packet.setSender(QHostAddress("192.168.1.101"), lidar_network::kDiscoveryPort);
    QVERIFY(lidar_network::ParseAnnouncement(packet, nic).mac.isEmpty());
}

void SnManagementTest::HttpReadDiagnosticAndDuplicate()
{
    const auto nic = LoopbackInterface();
    QVERIFY(!nic.isEmpty());
    DeviceServer first(Snapshot(kSn));
    DeviceServer second(Snapshot(kSn, "98:03:8a:7f:32:ef"));
    QVERIFY(first.listen(QHostAddress::LocalHost));
    QVERIFY(second.listen(QHostAddress::LocalHost));

    Identity identity;
    SnClient client;
    QString message;
    connect(&client, &SnClient::IdentityChanged, this, [&identity](const Identity &next) { identity = next; });
    connect(&client, &SnClient::StatusChanged, this, [&message](const QString &next) { message = next; });
    client.Read({nic, "127.0.0.1", first.serverPort()}, kMac, false);
    QTRY_VERIFY(!client.IsBusy());
    QVERIFY2(first.request.startsWith("GET /cgi-bin/lidar_network.cgi "), qPrintable(message));
    QCOMPARE(identity.sn, kSn);
    QVERIFY(!client.IsDuplicate(kSn, kMac));

    client.Read({nic, "127.0.0.1", second.serverPort()}, {}, true);
    QTRY_VERIFY(!client.IsBusy());
    QVERIFY(second.request.startsWith("GET /cgi-bin/lidar_network_status.cgi "));
    QVERIFY(identity.mac != kMac);
    QVERIFY(client.IsDuplicate(kSn, kMac));
    QCOMPARE(client.Devices().size(), 2);

    client.Read({nic, "127.0.0.1", second.serverPort()}, kMac, false);
    QTRY_VERIFY(!client.IsBusy());
    QVERIFY(identity.mac.isEmpty());
}

void SnManagementTest::VerifyOnlyCannotWriteAfterDeviceChanges()
{
    const auto nic = LoopbackInterface();
    QVERIFY(!nic.isEmpty());
    DeviceServer server(Snapshot());
    QVERIFY(server.listen(QHostAddress::LocalHost));

    SnClient client;
    QString message;
    connect(&client, &SnClient::StatusChanged, this, [&message](const QString &next) { message = next; });
    client.Start({nic, "127.0.0.1", server.serverPort()}, {"job-42", "operator-5", kMac, kSn},
                 WriteDecision::KAlreadyWritten);
    QTRY_VERIFY(!client.IsBusy());
    QVERIFY2(server.request.startsWith("GET /cgi-bin/lidar_network.cgi "), qPrintable(message));
    QVERIFY(message.contains(QStringLiteral("工单与在线设备不符")));
}

QTEST_GUILESS_MAIN(SnManagementTest)
#include "sn_management_test.moc"
