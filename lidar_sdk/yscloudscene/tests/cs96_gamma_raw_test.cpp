#include "business/cs96_gamma_raw.h"
#include "business/origin_point_parser.h"
#include "log/logging.h"
#include "you_see/business/node_supervisor.h"
#include "you_see/business/origin_point_processor.h"

#include <QDir>
#include <QFile>
#include <QScopeGuard>
#include <QTemporaryDir>
#include <QThreadPool>
#include <QUdpSocket>
#include <QUuid>
#include <QtEndian>
#include <QtTest>
#include <array>

namespace {
using namespace you_see;
using namespace ys_scene;
using namespace ys_scene::CS96GammaRawParser;

void WriteLe(uint8_t *data, uint32_t value, size_t bytes)
{
    for (size_t i = 0; i < bytes; ++i) {
        data[i] = static_cast<uint8_t>(value >> (8 * i));
    }
}

LidarInfo Node()
{
    LidarInfo node{};
    node.ip = qToBigEndian(QHostAddress(QHostAddress::LocalHost).toIPv4Address());
    node.unique_id = node.ip;
    node.model = LidarModel::KCS;
    node.hardware_version[0] = 'G';
    node.node_work_mode = NodeWorkMode::KOriginCloud;
    node.line_count = 120;
    node.slots_per_frame = 800;
    return node;
}

std::array<uint8_t, udp_packet_size> Packet(uint16_t frame, uint16_t slot, uint8_t fpga_frame = 254)
{
    std::array<uint8_t, udp_packet_size> bytes{};
    bytes[0] = 0xAA;
    WriteLe(bytes.data() + 1, frame, 2);
    WriteLe(bytes.data() + 3, slot, 2);
    WriteLe(bytes.data() + 5, slot % 10, 2);
    WriteLe(bytes.data() + 7, 1936, 2);
    auto *value = bytes.data() + 9;
    WriteLe(value, (uint32_t{0xABCDE} << 12) | slot, 4);
    WriteLe(value + 4, (0xFFFFF0u + slot) & 0xFFFFFFu, 3);
    for (size_t i = 7; i < 14; ++i) {
        value[i] = static_cast<uint8_t>(0x80 + i);
    }
    value[14] = fpga_frame;
    value[15] = 0xFE;
    for (size_t pixel = 0; pixel < 120; ++pixel) {
        auto *raw = value + 16 + pixel * 16;
        // 包括 0、旧版无效码 16、24bit/16bit 最大值及 et < ft，均必须原样保存。
        WriteLe(raw, pixel == 0 ? 0 : (pixel == 1 ? 16 : 0x800000u + slot * 120u + pixel), 3);
        WriteLe(raw + 3, pixel == 0 ? 0xFFFFFFu : 0x123456u + pixel, 3);
        WriteLe(raw + 6, 0x8000u + slot + pixel, 2);
        WriteLe(raw + 8, pixel == 2 ? 0 : 0xFEDCBAu - slot * 120u - pixel, 3);
        WriteLe(raw + 11, pixel == 2 ? 16 : 0xABCDEFu - pixel, 3);
        WriteLe(raw + 14, pixel == 2 ? 0xFFFFu : pixel, 2);
    }
    return bytes;
}

QByteArray ExpectedFrame(uint16_t frame, uint8_t fpga_frame = 254)
{
    QByteArray expected;
    expected.reserve(1936 * 800);
    for (uint16_t slot = 0; slot < 800; ++slot) {
        const auto packet = Packet(frame, slot, fpga_frame);
        expected.append(reinterpret_cast<const char *>(packet.data() + 9), 1936);
    }
    return expected;
}

Cs96GammaRawAssembler::FramePtr FeedFrame(Cs96GammaRawAssembler &assembler, uint16_t frame, uint8_t fpga = 254)
{
    Cs96GammaRawAssembler::FramePtr result;
    for (uint16_t slot = 0; slot < 800; ++slot) {
        const auto packet = Packet(frame, slot, fpga);
        if (auto complete = assembler.Feed(Node().ip, packet.data(), packet.size())) {
            result = std::move(complete);
        }
    }
    return result;
}

QByteArray FrameBytes(const Cs96GammaRawAssembler::FramePtr &frame)
{
    return QByteArray(reinterpret_cast<const char *>(frame->data.data()), frame->data.size());
}
} // namespace

class Cs96GammaRawTest : public QObject
{
    Q_OBJECT
  private slots:
    void initTestCase();
    void ValidatesPacketAndSource();
    void OrdersSlotsAndIgnoresDuplicates();
    void MissingSlotsAndFrameRollover();
    void ResetDiscardsPreviousSession();
    void SavesAllRawCodesAndSlotBytes();
    void UdpSaveUsesFpgaFace();
};

void Cs96GammaRawTest::initTestCase()
{
    InitLogger();
    SetLoggerLevel(2);
    QCOMPARE(udp_packet_size, size_t{1945});
    QCOMPARE(frame_data_size, size_t{1548800});
}

void Cs96GammaRawTest::ValidatesPacketAndSource()
{
    auto packet = Packet(1, 0);
    Cs96GammaRawAssembler assembler;
    const auto node = Node();
    QVERIFY(!assembler.Feed(node.ip, packet.data(), packet.size()));
    auto wrong = node;
    wrong.model = LidarModel::KMF;
    assembler.Reset(wrong);
    QVERIFY(!FeedFrame(assembler, 1));
    for (auto mode : {NodeWorkMode::KPointcloud, NodeWorkMode::KGray, NodeWorkMode::KRawHistogram}) {
        wrong = node;
        wrong.node_work_mode = mode;
        assembler.Reset(wrong);
        QVERIFY(!FeedFrame(assembler, 1));
    }
    assembler.Reset(node);
    QVERIFY(!assembler.Feed(node.ip, nullptr, packet.size()));
    QVERIFY(!assembler.Feed(node.ip, packet.data(), 0));
    for (size_t size : {size_t{1561}, size_t{1578}, size_t{1944}, size_t{1946}, size_t{3113}, size_t{3138}}) {
        QVERIFY(!assembler.Feed(node.ip, packet.data(), size));
    }
    for (uint16_t slot = 1; slot < 800; ++slot) {
        packet = Packet(1, slot);
        QVERIFY(!assembler.Feed(node.ip, packet.data(), packet.size()));
    }
    packet = Packet(1, 0);
    QVERIFY(!assembler.Feed(node.ip + 1, packet.data(), packet.size()));
    packet[0] = 0xAB;
    QVERIFY(!assembler.Feed(node.ip, packet.data(), packet.size()));
    packet = Packet(1, 0);
    WriteLe(packet.data() + 7, 1935, 2);
    QVERIFY(!assembler.Feed(node.ip, packet.data(), packet.size()));
    packet = Packet(1, 800);
    QVERIFY(!assembler.Feed(node.ip, packet.data(), packet.size()));
    packet = Packet(1, 0);
    packet[9] = 1; // UDP 槽号与 FPGA 原始头槽号不一致。
    QVERIFY(!assembler.Feed(node.ip, packet.data(), packet.size()));
    packet = Packet(1, 0);
    auto frame = assembler.Feed(node.ip, packet.data(), packet.size());
    QVERIFY(frame);
    QCOMPARE(FrameBytes(frame), ExpectedFrame(1));
}

void Cs96GammaRawTest::OrdersSlotsAndIgnoresDuplicates()
{
    Cs96GammaRawAssembler assembler;
    assembler.Reset(Node());
    for (int slot = 799; slot >= 1; --slot) {
        auto packet = Packet(42, slot);
        // seq_num 不参与寻址，即使不是通常的 0~9，也不能被当成切片号。
        WriteLe(packet.data() + 5, 65535, 2);
        QVERIFY(!assembler.Feed(Node().ip, packet.data(), packet.size()));
        packet[25] ^= 0xFF;
        QVERIFY(!assembler.Feed(Node().ip, packet.data(), packet.size()));
    }
    auto packet = Packet(42, 0);
    auto frame = assembler.Feed(Node().ip, packet.data(), packet.size());
    QVERIFY(frame);
    QCOMPARE(frame->frame_number, size_t{42});
    QCOMPARE(frame->receive_times, uint32_t{800});
    QCOMPARE(frame->received_slots.size(), size_t{800});
    QCOMPARE(FrameBytes(frame), ExpectedFrame(42));
    QVERIFY(!FeedFrame(assembler, 42));
}

void Cs96GammaRawTest::MissingSlotsAndFrameRollover()
{
    Cs96GammaRawAssembler assembler;
    assembler.Reset(Node());
    for (uint16_t slot = 0; slot < 799; ++slot) {
        const auto packet = Packet(65534, slot);
        QVERIFY(!assembler.Feed(Node().ip, packet.data(), packet.size()));
    }
    auto frame = FeedFrame(assembler, 65535, 255);
    QVERIFY(frame);
    QCOMPARE(frame->frame_number, size_t{65535});
    frame = FeedFrame(assembler, 0, 0);
    QVERIFY(frame);
    QCOMPARE(frame->frame_number, size_t{0});
    QCOMPARE(FrameBytes(frame), ExpectedFrame(0, 0));
    auto late = Packet(65534, 799);
    QVERIFY(!assembler.Feed(Node().ip, late.data(), late.size()));
    QVERIFY(!FeedFrame(assembler, 65535));

    assembler.Reset(Node());
    for (uint16_t id = 10; id < 15; ++id) {
        for (uint16_t slot = 0; slot < 799; ++slot) {
            const auto packet = Packet(id, slot);
            QVERIFY(!assembler.Feed(Node().ip, packet.data(), packet.size()));
        }
    }
    late = Packet(10, 799); // 超过缓存窗口的不完整帧不能复活。
    QVERIFY(!assembler.Feed(Node().ip, late.data(), late.size()));
    auto last = Packet(14, 799);
    QVERIFY(assembler.Feed(Node().ip, last.data(), last.size()));
}

void Cs96GammaRawTest::ResetDiscardsPreviousSession()
{
    Cs96GammaRawAssembler assembler;
    assembler.Reset(Node());
    for (uint16_t slot = 0; slot < 799; ++slot) {
        const auto packet = Packet(100, slot);
        assembler.Feed(Node().ip, packet.data(), packet.size());
    }
    assembler.Reset(Node());
    const auto last = Packet(100, 799);
    QVERIFY(!assembler.Feed(Node().ip, last.data(), last.size()));
    QVERIFY(FeedFrame(assembler, 100));
    assembler.Reset();
    QVERIFY(!FeedFrame(assembler, 101));
}

void Cs96GammaRawTest::SavesAllRawCodesAndSlotBytes()
{
    QTemporaryDir directory(QCoreApplication::applicationDirPath() + "/gamma-raw-XXXXXX");
    QVERIFY(directory.isValid());
    const auto expected = ExpectedFrame(123);
    const auto *raw = reinterpret_cast<const uint8_t *>(expected.constData());
    const auto path = directory.filePath("nested/gamma.txt");
    QVERIFY(!SaveCS96GammaRawFrameSync(nullptr, expected.size(), 123, 456, path.toStdString()));
    QVERIFY(!SaveCS96GammaRawFrameSync(raw, expected.size() - 1, 123, 456, path.toStdString()));
    QVERIFY(SaveCS96GammaRawFrameSync(raw, expected.size(), 123, 456, path.toStdString()));
    QFile binary(directory.filePath("nested/gamma.bin"));
    QVERIFY(binary.open(QIODevice::ReadOnly));
    QCOMPARE(binary.readAll(), expected); // 包括 SLOT 头保留位与所有 120 像素的上传顺序。
    QFile text(path);
    QVERIFY(text.open(QIODevice::ReadOnly | QIODevice::Text));
    const auto content = text.readAll();
    QVERIFY(content.contains("frame_id: 123\n"));
    QVERIFY(content.contains("cols: 800  rows: 120\n"));
    QVERIFY(content.contains("echo_fields: ft0 et0 pk0 ft1 et1 pk1\n"));
    QVERIFY(content.contains("mirror_angle: 703710\n"));
    QVERIFY(content.contains("relative_time_us: 16777200\n"));
    QVERIFY(content.contains("fpga_frame_id: 254\nface_id: 2\n"));
    size_t slot = 0;
    size_t pixel = 0;
    for (const auto &line : content.split('\n')) {
        if (!line.startsWith("pixel_number: ")) {
            continue;
        }
        // 像素编号跨越 9/10、99/100 时，各回波列仍保持相同的起止位置。
        QCOMPARE(line.size(), qsizetype{14 + 3 + 6 * 19});
        QCOMPARE(line.mid(14, 3).trimmed().toUInt(), static_cast<unsigned>(pixel));
        for (int field = 0; field < 6; ++field) {
            QCOMPARE(line.at(17 + field * 19), ' ');
            QVERIFY(!line.mid(18 + field * 19, 18).trimmed().isEmpty());
        }
        const auto fields = line.simplified().split(' ');
        QCOMPARE(fields.size(), qsizetype{8});
        QCOMPARE(fields[1].toUInt(), static_cast<unsigned>(pixel));
        const auto *echo = raw + slot * 1936 + 16 + pixel * 16;
        // 独立按字节加权计算期望值，检查每行的六个码值，避免只抽样首尾。
        for (size_t e = 0; e < 2; ++e) {
            const auto *p = echo + e * 8;
            QCOMPARE(fields[2 + e * 3].toUInt(), uint32_t{p[0]} + uint32_t{p[1]} * 256 + uint32_t{p[2]} * 65536);
            QCOMPARE(fields[3 + e * 3].toUInt(), uint32_t{p[3]} + uint32_t{p[4]} * 256 + uint32_t{p[5]} * 65536);
            QCOMPARE(fields[4 + e * 3].toUInt(), uint32_t{p[6]} + uint32_t{p[7]} * 256);
        }
        if (++pixel == 120) {
            pixel = 0;
            ++slot;
        }
    }
    QCOMPARE(slot, size_t{800});
    QCOMPARE(pixel, size_t{0});
}

void Cs96GammaRawTest::UdpSaveUsesFpgaFace()
{
    ys_scene::OriginPointParser parser;
    auto node = Node();
    node.is_connected = true;
    NodeSupervisorGetInstance().AddNode(node);
    parser.SetSourceNode(node);
    QUdpSocket reservation;
    QVERIFY(reservation.bind(QHostAddress::LocalHost, quint16{0}));
    const auto port = reservation.localPort();
    reservation.close();
    auto &processor = OriginPointProcessorGetInstance();
    processor.SetPeerIP(node.ip);
    processor.SetOriginCloudCallback([&parser](uint32_t handle, void *data, uint32_t size) {
        parser.OriginPointReceive2(handle, data, size, nullptr);
    });
    const QString prefix = "gamma_raw_test_" + QUuid::createUuid().toString(QUuid::Id128) + "_";
    const QDir root(QCoreApplication::applicationDirPath() + "/origin_cloud");
    const auto cleanup = qScopeGuard([&]() {
        processor.StopListening();
        processor.SetOriginCloudCallback({});
        QThreadPool::globalInstance()->waitForDone();
        for (const auto &folder : root.entryList({prefix + "*"}, QDir::Dirs | QDir::NoDotAndDotDot)) {
            const QDir saved(root.filePath(folder));
            for (const auto &name : saved.entryList(QDir::Files)) {
                QFile::remove(saved.filePath(name));
            }
            QDir().rmdir(saved.absolutePath());
        }
    });
    QVERIFY(processor.StartListening(port));
    parser.SetSaveOriginPoint({1, 3, prefix.toStdString()}); // 选择第 3 面，即 FPGA % 4 == 2。
    QUdpSocket sender;
    for (uint16_t frame_id : {uint16_t{10}, uint16_t{11}}) {
        for (int slot = 799; slot >= 0; --slot) {
            const auto bytes = Packet(frame_id, slot, frame_id == 10 ? 1 : 2);
            QCOMPARE(sender.writeDatagram(reinterpret_cast<const char *>(bytes.data()), bytes.size(),
                                          QHostAddress::LocalHost, port),
                     qint64{1945});
            if (slot % 16 == 0) {
                QTest::qWait(1);
            }
        }
    }
    QString saved_path;
    auto saved = [&]() {
        for (const auto &folder : root.entryList({prefix + "*"}, QDir::Dirs | QDir::NoDotAndDotDot)) {
            const auto path = root.filePath(folder + "/" + prefix + "1.txt");
            QFile text(path);
            if (text.open(QIODevice::ReadOnly | QIODevice::Text) && text.readAll().contains("end of slot [799]")) {
                saved_path = path;
                return true;
            }
        }
        return false;
    };
    QTRY_VERIFY_WITH_TIMEOUT(saved(), 10000);
    QFile text(saved_path);
    QVERIFY(text.open(QIODevice::ReadOnly | QIODevice::Text));
    QVERIFY(text.readAll().contains("frame_id: 11\n")); // ARM % 4 == 3，与 FPGA 面号不同。
    QFile binary(saved_path.chopped(4) + ".bin");
    QVERIFY(binary.open(QIODevice::ReadOnly));
    QCOMPARE(binary.readAll(), ExpectedFrame(11, 2));
}

QTEST_GUILESS_MAIN(Cs96GammaRawTest)
#include "cs96_gamma_raw_test.moc"
