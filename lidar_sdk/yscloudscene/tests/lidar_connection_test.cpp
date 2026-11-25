#include "log/logging.h"
#include "you_see/business/gray_image_processor.h"
#include "you_see/business/instruction_processor.h"
#include "you_see/business/instruction_set_type.h"
#include "you_see/business/node_founder.h"
#include "you_see/business/node_supervisor.h"
#include "you_see/business/origin_point_processor.h"
#include "you_see/business/point_processor.h"
#include "you_see/business/raw_histogram_processor.h"

#include <QScopeGuard>
#include <QUdpSocket>
#include <QtEndian>
#include <QtTest>
#include <atomic>
#include <cstring>
#include <memory>

namespace {
using namespace you_see;

uint64_t NowMicroseconds()
{
    return std::chrono::duration_cast<std::chrono::microseconds>(std::chrono::steady_clock::now().time_since_epoch())
        .count();
}

LidarInfo DiscoveredNode()
{
    LidarInfo node{};
    node.ip = qToBigEndian(QHostAddress(QHostAddress::LocalHost).toIPv4Address());
    node.unique_id = node.ip;
    node.last_seen_timestamp = NowMicroseconds();
    node.instruction_processor_stoped = true;
    node.data_processor_stoped = true;
    return node;
}

QByteArray Packet(InstructionType type, InstructionGeneral command, QByteArray payload)
{
    YeahPacketCommon packet{magic_header};
    packet.protocol_version = protocol_version;
    packet.command_type = static_cast<uint8_t>(type);
    packet.serial_number = 0;
    packet.command_set = static_cast<uint8_t>(InstructionSet::kInstructionSetGeneral);
    packet.command_id = static_cast<uint8_t>(command);
    packet.data = reinterpret_cast<uint8_t *>(payload.data());
    packet.data_len = head_size + payload.size();
    packet.real_data_len = payload.size();
    QByteArray bytes(max_command_buffer_size, '\0');
    uint32_t size = bytes.size();
    YeahPackager packager;
    packager.Pack(reinterpret_cast<uint8_t *>(bytes.data()), bytes.size(), &size, packet);
    bytes.resize(size);
    return bytes;
}

void StopDataListeners()
{
    PointProcessorGetInstance().StopListening();
    OriginPointProcessorGetInstance().StopListening();
    RawHistogramProcessorGetInstance().StopListening();
    GrayImageProcessorGetInstance().StopListening();
}
} // namespace

class LidarConnectionTest : public QObject
{
    Q_OBJECT
  private slots:
    void initTestCase();
    void cleanupTestCase();
    void DiscoveryPreservesConnectionState();
    void SerialLookupUsesFullWidth();
    void BroadcastInitializesStoppedFlags();
    void BroadcastParsesIdentity_data();
    void BroadcastParsesIdentity();
    void BroadcastRefreshesIdentity();
    void BroadcastRejectsUnknownLengths();
    void BroadcastRecognizesMode3AsGray_data();
    void BroadcastRecognizesMode3AsGray();
    void ReceiversReadyBeforeHandshake_data();
    void ReceiversReadyBeforeHandshake();
    void ReconnectClearsHeartbeatState();
    void InvalidHandshakeIsIgnored_data();
    void InvalidHandshakeIsIgnored();
    void MissingHandshakeTimesOutWithoutHeartbeat();

  private:
    QUdpSocket socket_runtime_;
    std::unique_ptr<NodeFounder> founder_;
    quint16 discovery_port_ = 0;
};

void LidarConnectionTest::initTestCase()
{
    InitLogger();
    SetLoggerLevel(2);
    // Initialize and retain the Windows socket runtime before SDK discovery starts.
    QVERIFY(socket_runtime_.bind(QHostAddress(QHostAddress::LocalHost), quint16{0}));
    QUdpSocket reservation;
    QVERIFY(reservation.bind(QHostAddress(QHostAddress::AnyIPv4), quint16{0}));
    discovery_port_ = reservation.localPort();
    reservation.close();
    // Keep discovery alive across rows, like the application. Recreating it for each
    // row can race queued broadcasts from real devices with listener destruction.
    founder_ = std::make_unique<NodeFounder>(discovery_port_);
    QVERIFY(founder_->StartFound());
    QTest::qWait(100);
}

void LidarConnectionTest::cleanupTestCase()
{
    founder_.reset();
}

void LidarConnectionTest::DiscoveryPreservesConnectionState()
{
    NodeSupervisor supervisor;
    auto node = DiscoveredNode();
    supervisor.AddDiscoveredNode(node);
    auto *current = supervisor.GetNode(node.ip);
    QVERIFY(current);
    QVERIFY(current->instruction_processor_stoped);
    QVERIFY(current->data_processor_stoped);

    current->instruction_port = 50000;
    current->data_port = 50001;
    current->instruction_processor_stoped = false;
    current->data_processor_stoped = false;
    current->node_work_mode = NodeWorkMode::KPointcloud;
    node.line_count = 96;
    node.slots_per_frame = 800;
    supervisor.AddDiscoveredNode(node);
    QCOMPARE(current->instruction_port, uint16_t{50000});
    QCOMPARE(current->data_port, uint16_t{50001});
    QVERIFY(!current->instruction_processor_stoped);
    QVERIFY(!current->data_processor_stoped);
    QVERIFY(!current->is_connected);
    QCOMPARE(current->node_work_mode, NodeWorkMode::KPointcloud);
    QCOMPARE(current->line_count, uint16_t{96});

    current->is_connected = true;
    supervisor.AddDiscoveredNode(node);
    QVERIFY(current->is_connected);
    QCOMPARE(current->instruction_port, uint16_t{50000});
    QCOMPARE(current->slots_per_frame, uint16_t{800});
}

void LidarConnectionTest::SerialLookupUsesFullWidth()
{
    NodeSupervisor supervisor;
    auto node = DiscoveredNode();
    const char *serial = "CS96XXXB01026020001";
    std::memcpy(node.serial_number, serial, sizeof(node.serial_number));
    std::memcpy(node.hardware_version, "B01", sizeof(node.hardware_version));
    supervisor.AddDiscoveredNode(node);
    QCOMPARE(supervisor.GetNode(serial), supervisor.GetNode(node.ip));
    const char *same_prefix = "CS96XXXB01026020002";
    QVERIFY(!supervisor.GetNode(same_prefix));
}

void LidarConnectionTest::BroadcastInitializesStoppedFlags()
{
    QUdpSocket sender;
    QVERIFY(sender.bind(QHostAddress(QHostAddress::LocalHost), quint16{0}));
    const auto observed = std::make_shared<std::atomic<bool>>(false);
    NodeSupervisorGetInstance().RegisterNodeFoundCallback([observed](const LidarInfo &node, void *) {
        if (std::memcmp(node.serial_number, "CS-LOOPBACK-TEST", 16) == 0) {
            observed->store(!node.is_connected && node.instruction_processor_stoped && node.data_processor_stoped);
        }
    });
    QByteArray payload(26, '\0');
    std::memcpy(payload.data(), "CS-LOOPBACK-TEST", 16);
    payload[18] = static_cast<char>(NodeWorkMode::KPointcloud);
    const auto broadcast = Packet(InstructionType::kInstructionTypeMsg, InstructionGeneral::kMsgBroadcast, payload);
    QCOMPARE(sender.writeDatagram(broadcast, QHostAddress::LocalHost, discovery_port_), qint64{broadcast.size()});
    QTRY_VERIFY_WITH_TIMEOUT(observed->load(), 2000);
}

void LidarConnectionTest::BroadcastRecognizesMode3AsGray_data()
{
    QTest::addColumn<QByteArray>("serial");
    QTest::addColumn<int>("format");
    QTest::newRow("beta") << QByteArray("CS96000B0126020001") << static_cast<int>(GrayFormat::KBeta);
    QTest::newRow("gamma") << QByteArray("CS96000C0126020001") << static_cast<int>(GrayFormat::KGamma);
}

void LidarConnectionTest::BroadcastParsesIdentity_data()
{
    QTest::addColumn<QByteArray>("identity");
    QTest::addColumn<int>("identity_length");
    QTest::addColumn<int>("model");
    QTest::addColumn<QByteArray>("hardware");
    QTest::newRow("legacy-beta") << QByteArray("CS96000B0126020001") << 18
                                << static_cast<int>(LidarModel::KCS) << QByteArray("B01");
    QTest::newRow("legacy-padded") << QByteArray("CS96000B01") << 18
                                  << static_cast<int>(LidarModel::KCS) << QByteArray("B01");
    QTest::newRow("current-beta") << QByteArray("CS96XXXB01026020001") << 19
                                 << static_cast<int>(LidarModel::KCS) << QByteArray("B01");
    QTest::newRow("current-last-digit") << QByteArray("CS96XXXB01026020002") << 19
                                       << static_cast<int>(LidarModel::KCS) << QByteArray("B01");
    QTest::newRow("current-gamma") << QByteArray("CS96XXXC0112653FFFF") << 19
                                  << static_cast<int>(LidarModel::KCS) << QByteArray("C01");
    QTest::newRow("current-mf") << QByteArray("MF150XXA01026020001") << 19
                               << static_cast<int>(LidarModel::KMF) << QByteArray("A01");
    QTest::newRow("mac-fallback") << QByteArray("MAC-98038A7F32EE00") << 19
                                 << static_cast<int>(LidarModel::KUnknown) << QByteArray(3, '\0');
}

void LidarConnectionTest::BroadcastParsesIdentity()
{
    QFETCH(QByteArray, identity);
    QFETCH(int, identity_length);
    QFETCH(int, model);
    QFETCH(QByteArray, hardware);
    QUdpSocket sender;
    QVERIFY(sender.bind(QHostAddress(QHostAddress::LocalHost), quint16{0}));
    QByteArray expected_sn(19, '\0');
    if (!identity.startsWith("MAC-")) {
        std::memcpy(expected_sn.data(), identity.constData(), identity.size());
    }
    const auto observed = std::make_shared<std::atomic<bool>>(false);
    const auto source_port = qToBigEndian(sender.localPort());
    NodeSupervisorGetInstance().RegisterNodeFoundCallback(
        [observed, expected_sn, model, hardware, source_port](const LidarInfo &node, void *) {
            if (node.broadcast_port == source_port) {
                observed->store(std::memcmp(node.serial_number, expected_sn.constData(), 19) == 0 &&
                                std::memcmp(node.hardware_version, hardware.constData(), 3) == 0 &&
                                static_cast<int>(node.model) == model && node.node_work_mode == NodeWorkMode::KGray &&
                                node.echo_mode == 1 && node.echo_count == 2 && node.echo_type == 2 &&
                                node.coordinate_mode == 1 && node.line_count == 0x1234 &&
                                node.slots_per_frame == 0x5678 && node.last_seen_timestamp > 0);
            }
        });
    QByteArray payload(identity_length, '\0');
    std::memcpy(payload.data(), identity.constData(), identity.size());
    // Literal wire bytes verify the shifted mode fields and little-endian counters.
    payload.append(QByteArray::fromHex("0301020134127856"));
    const auto broadcast = Packet(InstructionType::kInstructionTypeMsg, InstructionGeneral::kMsgBroadcast, payload);
    QCOMPARE(broadcast.size(), identity_length == 18 ? 41 : 42);
    QCOMPARE(qFromLittleEndian<quint16>(reinterpret_cast<const uchar *>(broadcast.constData() + 2)),
             quint16(identity_length == 18 ? 28 : 29));
    QCOMPARE(sender.writeDatagram(broadcast, QHostAddress::LocalHost, discovery_port_), qint64{broadcast.size()});
    QTRY_VERIFY_WITH_TIMEOUT(observed->load(), 2000);
}

void LidarConnectionTest::BroadcastRefreshesIdentity()
{
    QUdpSocket sender;
    QVERIFY(sender.bind(QHostAddress(QHostAddress::LocalHost), quint16{0}));
    const auto received = std::make_shared<std::atomic<int>>(0);
    const auto snapshot = std::make_shared<LidarInfo>();
    const auto source_port = qToBigEndian(sender.localPort());
    NodeSupervisorGetInstance().RegisterNodeFoundCallback(
        [received, snapshot, source_port](const LidarInfo &node, void *) {
            if (node.broadcast_port == source_port) {
                *snapshot = node;
                ++(*received);
            }
        });
    const QList<QByteArray> identities{"CS96XXXB01026020001", "MAC-98038A7F32EE00", "CS96XXXB01026020002"};
    for (int i = 0; i < identities.size(); ++i) {
        QByteArray payload(19, '\0');
        std::memcpy(payload.data(), identities[i].constData(), identities[i].size());
        payload.append(QByteArray::fromHex("030000006000A800"));
        const auto broadcast = Packet(InstructionType::kInstructionTypeMsg, InstructionGeneral::kMsgBroadcast, payload);
        QCOMPARE(sender.writeDatagram(broadcast, QHostAddress::LocalHost, discovery_port_), qint64{broadcast.size()});
        QTRY_COMPARE_WITH_TIMEOUT(received->load(), i + 1, 2000);
        QCOMPARE(snapshot->unique_id, snapshot->ip);
        QCOMPARE(snapshot->line_count, uint16_t{96});
        QCOMPARE(snapshot->slots_per_frame, uint16_t{168});
        if (i == 1) {
            QCOMPARE(QByteArray(reinterpret_cast<const char *>(snapshot->serial_number), 19), QByteArray(19, '\0'));
            QCOMPARE(QByteArray(reinterpret_cast<const char *>(snapshot->hardware_version), 3), QByteArray(3, '\0'));
            QCOMPARE(snapshot->model, LidarModel::KUnknown);
        }
        else {
            QCOMPARE(QByteArray(reinterpret_cast<const char *>(snapshot->serial_number), 19), identities[i]);
            QCOMPARE(snapshot->model, LidarModel::KCS);
            QCOMPARE(GrayFormatForNode(*snapshot), GrayFormat::KBeta);
        }
    }
}

void LidarConnectionTest::BroadcastRejectsUnknownLengths()
{
    QUdpSocket sender;
    QVERIFY(sender.bind(QHostAddress(QHostAddress::LocalHost), quint16{0}));
    const auto received = std::make_shared<std::atomic<int>>(0);
    const auto source_port = qToBigEndian(sender.localPort());
    NodeSupervisorGetInstance().RegisterNodeFoundCallback([received, source_port](const LidarInfo &node, void *) {
        if (node.broadcast_port == source_port) {
            ++(*received);
        }
    });
    // These frames have valid framing/CRCs, but unsupported broadcast payload lengths.
    for (const int length : {0, 18, 25, 28, 40}) {
        const auto broadcast = Packet(InstructionType::kInstructionTypeMsg, InstructionGeneral::kMsgBroadcast,
                                      QByteArray(length, '\0'));
        QCOMPARE(sender.writeDatagram(broadcast, QHostAddress::LocalHost, discovery_port_), qint64{broadcast.size()});
    }
    const auto valid = Packet(InstructionType::kInstructionTypeMsg, InstructionGeneral::kMsgBroadcast,
                              QByteArray(27, '\0'));
    QCOMPARE(sender.writeDatagram(valid, QHostAddress::LocalHost, discovery_port_), qint64{valid.size()});
    QTRY_VERIFY_WITH_TIMEOUT(received->load() > 0, 2000);
    QCOMPARE(received->load(), 1);
}

void LidarConnectionTest::BroadcastRecognizesMode3AsGray()
{
    QFETCH(QByteArray, serial);
    QFETCH(int, format);
    QUdpSocket sender;
    QVERIFY(sender.bind(QHostAddress(QHostAddress::LocalHost), quint16{0}));
    const auto observed = std::make_shared<std::atomic<bool>>(false);
    NodeSupervisorGetInstance().RegisterNodeFoundCallback([observed, serial, format](const LidarInfo &node, void *) {
        if (std::memcmp(node.serial_number, serial.constData(), serial.size()) == 0) {
            observed->store(node.node_work_mode == NodeWorkMode::KGray &&
                            static_cast<int>(GrayFormatForNode(node)) == format);
        }
    });
    QByteArray payload(26, '\0');
    std::memcpy(payload.data(), serial.constData(), serial.size());
    payload[18] = 3; // 使用真实协议值，防止枚举与测试一起改错而漏掉回归。
    const auto broadcast = Packet(InstructionType::kInstructionTypeMsg, InstructionGeneral::kMsgBroadcast, payload);
    QCOMPARE(sender.writeDatagram(broadcast, QHostAddress::LocalHost, discovery_port_), qint64{broadcast.size()});
    QTRY_VERIFY_WITH_TIMEOUT(observed->load(), 2000);

}

void LidarConnectionTest::ReceiversReadyBeforeHandshake_data()
{
    QTest::addColumn<int>("mode");
    QTest::newRow("pointcloud") << static_cast<int>(NodeWorkMode::KPointcloud);
    QTest::newRow("origin") << static_cast<int>(NodeWorkMode::KOriginCloud);
    QTest::newRow("histogram") << static_cast<int>(NodeWorkMode::KRawHistogram);
    QTest::newRow("gray") << static_cast<int>(NodeWorkMode::KGray);
}

void LidarConnectionTest::ReceiversReadyBeforeHandshake()
{
    QFETCH(int, mode);
    QUdpSocket command_reservation;
    QUdpSocket data_reservation;
    QVERIFY(command_reservation.bind(QHostAddress(QHostAddress::LocalHost), quint16{0}));
    QVERIFY(data_reservation.bind(QHostAddress(QHostAddress::LocalHost), quint16{0}));
    auto node = DiscoveredNode();
    node.instruction_port = command_reservation.localPort();
    node.data_port = data_reservation.localPort();
    node.node_work_mode = static_cast<NodeWorkMode>(mode);
    command_reservation.close();
    data_reservation.close();
    auto &supervisor = NodeSupervisorGetInstance();
    supervisor.AddNode(node);
    const auto connected = std::make_shared<std::atomic<bool>>(false);
    supervisor.RegisterNodeFoundCallback([connected](const LidarInfo &info, void *) {
        connected->store(info.is_connected && !info.instruction_processor_stoped && !info.data_processor_stoped);
    });
    auto &instruction = InstructionProcessorGetInstance();
    const auto cleanup = qScopeGuard([&instruction]() {
        StopDataListeners();
        instruction.StopListening();
    });
    instruction.SetPeerIP(node.ip);
    QVERIFY(instruction.StartListening(node.instruction_port));
    switch (node.node_work_mode) {
    case NodeWorkMode::KPointcloud:
        PointProcessorGetInstance().SetPeerIP(node.ip);
        QVERIFY(PointProcessorGetInstance().StartListening(node.data_port));
        break;
    case NodeWorkMode::KOriginCloud:
        OriginPointProcessorGetInstance().SetPeerIP(node.ip);
        QVERIFY(OriginPointProcessorGetInstance().StartListening(node.data_port));
        break;
    case NodeWorkMode::KRawHistogram:
        RawHistogramProcessorGetInstance().SetPeerIP(node.ip);
        QVERIFY(RawHistogramProcessorGetInstance().StartListening(node.data_port));
        break;
    case NodeWorkMode::KGray:
        GrayImageProcessorGetInstance().SetPeerIP(node.ip);
        QVERIFY(GrayImageProcessorGetInstance().StartListening(node.data_port));
        break;
    }
    auto *current = supervisor.GetNode(node.ip);
    QVERIFY(current);
    // StartListening 必须在返回前完成状态更新，而不是依赖测试等待线程调度。
    QVERIFY(!current->instruction_processor_stoped);
    QVERIFY(!current->data_processor_stoped);
    supervisor.AddDiscoveredNode(node);
    QVERIFY(!current->instruction_processor_stoped);
    QVERIFY(!current->data_processor_stoped);
    QCOMPARE(current->instruction_port, node.instruction_port);
    QCOMPARE(current->data_port, node.data_port);

    QUdpSocket sender;
    QVERIFY(sender.bind(QHostAddress(QHostAddress::LocalHost), quint16{0}));
    const auto ack = Packet(InstructionType::kInstructionTypeAck, InstructionGeneral::KCmdHandShake,
                            QByteArray(1, static_cast<char>(NodeStatus::KSuccess)));
    QCOMPARE(sender.writeDatagram(ack, QHostAddress::LocalHost, node.instruction_port), qint64{ack.size()});
    QTRY_VERIFY_WITH_TIMEOUT(connected->load(), 2000);
}

void LidarConnectionTest::ReconnectClearsHeartbeatState()
{
    QUdpSocket sender;
    QVERIFY(sender.bind(QHostAddress(QHostAddress::LocalHost), quint16{0}));
    QUdpSocket heartbeat_receiver;
    QVERIFY(heartbeat_receiver.bind(QHostAddress(QHostAddress::LocalHost), quint16{65000}));
    QUdpSocket reservation;
    QVERIFY(reservation.bind(QHostAddress(QHostAddress::LocalHost), quint16{0}));
    auto node = DiscoveredNode();
    node.instruction_port = reservation.localPort();
    reservation.close();
    NodeSupervisorGetInstance().AddNode(node);
    auto &instruction = InstructionProcessorGetInstance();
    const auto cleanup = qScopeGuard([&instruction]() { instruction.StopListening(); });
    instruction.SetPeerIP(node.ip);
    QVERIFY(instruction.StartListening(node.instruction_port));
    const auto ack = Packet(InstructionType::kInstructionTypeAck, InstructionGeneral::KCmdHandShake,
                            QByteArray(1, static_cast<char>(NodeStatus::KSuccess)));
    QCOMPARE(sender.writeDatagram(ack, QHostAddress::LocalHost, node.instruction_port), qint64{ack.size()});
    // 即使数据接收器尚未就绪，握手确认也必须立即发送保活。
    QTRY_VERIFY_WITH_TIMEOUT(heartbeat_receiver.hasPendingDatagrams(), 2000);
    instruction.StopListening();
    QVERIFY(!instruction.IsHeartbeatTimeout(node.ip));
    QVERIFY(instruction.StartListening(node.instruction_port));
    QVERIFY(instruction.IsHeartbeatTimeout(node.ip));
}

void LidarConnectionTest::InvalidHandshakeIsIgnored_data()
{
    QTest::addColumn<QByteArray>("payload");
    QTest::newRow("empty") << QByteArray();
    QTest::newRow("rejected") << QByteArray(1, static_cast<char>(NodeStatus::KFailed));
}

void LidarConnectionTest::InvalidHandshakeIsIgnored()
{
    QFETCH(QByteArray, payload);
    QUdpSocket sender;
    QVERIFY(sender.bind(QHostAddress(QHostAddress::LocalHost), quint16{0}));
    QUdpSocket reservation;
    QVERIFY(reservation.bind(QHostAddress(QHostAddress::LocalHost), quint16{0}));
    auto node = DiscoveredNode();
    node.instruction_port = reservation.localPort();
    node.data_processor_stoped = false;
    reservation.close();
    NodeSupervisorGetInstance().AddNode(node);
    auto &instruction = InstructionProcessorGetInstance();
    const auto cleanup = qScopeGuard([&instruction, node]() {
        instruction.StopListening();
        auto &supervisor = NodeSupervisorGetInstance();
        auto *current = supervisor.GetNode(node.ip);
        current->data_processor_stoped = true;
        current->is_connected = false;
        supervisor.AddNode(*current);
    });
    instruction.SetPeerIP(node.ip);
    QVERIFY(instruction.StartListening(node.instruction_port));
    const auto confirmations = std::make_shared<std::atomic<int>>(0);
    NodeSupervisorGetInstance().RegisterNodeFoundCallback([confirmations](const LidarInfo &info, void *) {
        if (info.is_connected) {
            confirmations->fetch_add(1);
        }
    });
    const auto ack = Packet(InstructionType::kInstructionTypeAck, InstructionGeneral::KCmdHandShake, payload);
    QCOMPARE(sender.writeDatagram(ack, QHostAddress::LocalHost, node.instruction_port), qint64{ack.size()});
    // 随后发送合法 ACK，以确认拒绝路径已经在同一个事件循环里处理完毕。
    const auto success = Packet(InstructionType::kInstructionTypeAck, InstructionGeneral::KCmdHandShake,
                                QByteArray(1, static_cast<char>(NodeStatus::KSuccess)));
    QCOMPARE(sender.writeDatagram(success, QHostAddress::LocalHost, node.instruction_port), qint64{success.size()});
    QTRY_COMPARE_WITH_TIMEOUT(confirmations->load(), 1, 2000);
}

void LidarConnectionTest::MissingHandshakeTimesOutWithoutHeartbeat()
{
    QUdpSocket heartbeat_receiver;
    QVERIFY(heartbeat_receiver.bind(QHostAddress(QHostAddress::LocalHost), quint16{65000}));
    QUdpSocket reservation;
    QVERIFY(reservation.bind(QHostAddress(QHostAddress::LocalHost), quint16{0}));
    auto node = DiscoveredNode();
    node.instruction_port = reservation.localPort();
    reservation.close();
    auto &supervisor = NodeSupervisorGetInstance();
    supervisor.AddNode(node);
    auto &instruction = InstructionProcessorGetInstance();
    const auto disconnected = std::make_shared<std::atomic<bool>>(false);
    const auto cleanup = qScopeGuard([&instruction, &supervisor]() {
        supervisor.SetOnDisconnectRequest(nullptr);
        instruction.StopListening();
    });
    supervisor.SetOnDisconnectRequest([&instruction, disconnected](const LidarInfo &pending) {
        instruction.StopListening();
        disconnected->store(!pending.is_connected);
    });
    instruction.SetPeerIP(node.ip);
    QVERIFY(instruction.StartListening(node.instruction_port));
    instruction.StartHandshakeTimeout();
    QTRY_VERIFY_WITH_TIMEOUT(disconnected->load(), 8000);
    QVERIFY(!heartbeat_receiver.hasPendingDatagrams());
    const auto *current = supervisor.GetNode(node.ip);
    QVERIFY(current);
    QVERIFY(!current->is_connected);
    QVERIFY(current->instruction_processor_stoped);
    QVERIFY(current->data_processor_stoped);
}

QTEST_GUILESS_MAIN(LidarConnectionTest)
#include "lidar_connection_test.moc"
