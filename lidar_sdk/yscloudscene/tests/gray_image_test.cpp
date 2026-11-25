#include "business/gray_image_controller.h"
#include "component/raw_display_widget.h"
#include "log/logging.h"
#include "you_see/business/gray_frame_assembler.h"
#include "you_see/business/gray_image_processor.h"
#include "you_see/business/instruction_processor.h"
#include "you_see/business/node_supervisor.h"
#include "ys_lidar_user.h"

#include <QScopeGuard>
#include <QtEndian>

#include <QFile>
#include <QFontDatabase>
#include <QSignalSpy>
#include <QTemporaryDir>
#include <QUdpSocket>
#include <QtTest>
#include <algorithm>
#include <array>
#include <atomic>
#include <cmath>
#include <thread>

namespace {
using namespace you_see;
using namespace ys_scene;

void WriteLe(uint8_t *out, uint64_t value, size_t bytes)
{
    for (size_t i = 0; i < bytes; ++i) {
        out[i] = static_cast<uint8_t>(value >> (8 * i));
    }
}

std::array<uint8_t, GrayProtocol::kPacketSize> Packet(uint16_t id, uint16_t slot, uint32_t echo1 = 70000,
                                                      uint32_t echo2 = 900000)
{
    std::array<uint8_t, GrayProtocol::kPacketSize> bytes{};
    bytes[0] = 0xAA;
    WriteLe(bytes.data() + 1, id, 2);
    WriteLe(bytes.data() + 3, slot, 2);
    WriteLe(bytes.data() + 5, slot % 10, 2);
    WriteLe(bytes.data() + 7, 968, 2);
    WriteLe(bytes.data() + 9, slot | (uint64_t{0x12345} << 5), 8);
    for (size_t pixel = 0; pixel < 120; ++pixel) {
        WriteLe(bytes.data() + 17 + pixel * 8,
                echo1 | (static_cast<uint64_t>(echo2) << 20) | (uint64_t{0xABCDEF} << 40), 8);
    }
    return bytes;
}

std::array<uint8_t, GrayProtocol::kPacketSize> GammaPixelOrderPacket(uint16_t id, uint16_t slot)
{
    auto bytes = Packet(id, slot, 0, 0);
    constexpr size_t kPixelsPerGroup = GrayProtocol::kPixelsPerSlot / 4;
    constexpr std::array<size_t, 4> kGroupOffsets{0, 2, 1, 3};
    for (size_t wire_pixel = 0; wire_pixel < GrayProtocol::kPixelsPerSlot; ++wire_pixel) {
        const size_t pixel = (wire_pixel % kPixelsPerGroup) * 4 + kGroupOffsets[wire_pixel / kPixelsPerGroup];
        const uint64_t echo1 = 1000 + pixel;
        const uint64_t echo2 = 2000 + pixel;
        WriteLe(bytes.data() + 17 + wire_pixel * 8, echo1 | (echo2 << 20), 8);
    }
    return bytes;
}

std::array<uint8_t, 1165> BetaPacket(uint16_t id, uint16_t slot, uint16_t seq)
{
    std::array<uint8_t, 2312> raw{};
    WriteLe(raw.data(), slot, 2);
    for (size_t pixel = 0; pixel < 576; ++pixel) {
        // 包含高位保留位，同时覆盖分片交界及 22 位最大值。
        WriteLe(raw.data() + 8 + pixel * 4,
                (pixel == 575 ? 0x3FFFFF : 0x100000 + slot * 576 + pixel) | uint32_t{0xFFC00000}, 4);
    }
    std::array<uint8_t, 1165> bytes{};
    bytes[0] = 0xAA;
    WriteLe(bytes.data() + 1, id, 2);
    WriteLe(bytes.data() + 3, slot, 2);
    WriteLe(bytes.data() + 5, seq, 2);
    WriteLe(bytes.data() + 7, 1156, 2);
    std::copy(raw.begin() + seq * 1156, raw.begin() + (seq + 1) * 1156, bytes.begin() + 9);
    return bytes;
}

GrayEchoFramePtr CompleteFrame(uint16_t id = 1, uint64_t session = 1)
{
    auto frame = std::make_shared<GrayEchoFrame>();
    frame->device_ip = 123;
    frame->session_id = session;
    frame->frame_id = id;
    frame->received_slots.set();
    for (size_t i = 0; i < 2400; ++i) {
        frame->echo1[i] = static_cast<uint32_t>(i * 400);
        frame->echo2[i] = static_cast<uint32_t>(1000000 - i * 400);
    }
    return frame;
}
} // namespace

class GrayImageTest : public QObject
{
    Q_OBJECT

  private slots:
    void initTestCase();
    void PacketValidationAndTwentyBitEchoes();
    void GammaPixelOrderIsRestored();
    void OutOfOrderDuplicatesAndFrameBoundaries();
    void ExpirationRolloverAndSessionReset();
    void MappingAndMatrixOrientation();
    void OrientationPreservesPixelsAndEma();
    void PauseSelectionAndLatestFrameQueue();
    void RawCsvAndContinuousRecording();
    void WidgetModeCompatibility();
    void UdpReceptionAndRestart();
    void BetaFragmentsAndSlowFrames();
    void BetaDisplayAndExport();
    void BetaUdpReception();
    void InstructionListenerReadyBeforeHandshake();
    void LiveBetaConnection();
};

void GrayImageTest::initTestCase()
{
    const QString font = qEnvironmentVariable("YS_GRAY_TEST_FONT");
    if (!font.isEmpty()) {
        QVERIFY(QFontDatabase::addApplicationFont(font) >= 0);
    }
}

void GrayImageTest::PacketValidationAndTwentyBitEchoes()
{
    GrayFrameAssembler assembler;
    assembler.Reset(123, 7);
    auto bytes = Packet(100, 0);
    QVERIFY(!assembler.Feed(bytes.data(), 976));
    auto bad = bytes;
    bad[0] = 0;
    QVERIFY(!assembler.Feed(bad.data(), bad.size()));
    bad = bytes;
    bad[7] = 0;
    QVERIFY(!assembler.Feed(bad.data(), bad.size()));
    bad = Packet(100, 20);
    QVERIFY(!assembler.Feed(bad.data(), bad.size()));
    bad = bytes;
    bad[9] = 1;
    QVERIFY(!assembler.Feed(bad.data(), bad.size()));
    QCOMPARE(assembler.GetStats().invalid_packets, uint64_t{5});
    GrayEchoFramePtr frame;
    for (uint16_t slot = 0; slot < 20; ++slot) {
        bytes = Packet(100, slot, slot == 19 ? 0xFFFFF : 70000, slot == 19 ? 0 : 900000);
        frame = assembler.Feed(bytes.data(), bytes.size());
        if (slot < 19) {
            QVERIFY(!frame);
        }
    }
    QVERIFY(frame);
    QCOMPARE(frame->session_id, uint64_t{7});
    QCOMPARE(frame->device_ip, uint32_t{123});
    QCOMPARE(frame->echo1[0], uint32_t{70000});
    QCOMPARE(frame->echo2[0], uint32_t{900000});
    QCOMPARE(frame->echo1[2399], uint32_t{0xFFFFF});
    QCOMPARE(frame->echo2[2399], uint32_t{0});
}

void GrayImageTest::GammaPixelOrderIsRestored()
{
    GrayFrameAssembler assembler;
    assembler.Reset(123, 7, GrayFormat::KGamma);
    GrayEchoFramePtr frame;
    for (uint16_t slot = 0; slot < GrayProtocol::kSlotsPerFrame; ++slot) {
        auto bytes = GammaPixelOrderPacket(100, slot);
        frame = assembler.Feed(bytes.data(), bytes.size());
    }
    QVERIFY(frame);
    for (size_t slot = 0; slot < GrayProtocol::kSlotsPerFrame; ++slot) {
        for (size_t pixel = 0; pixel < GrayProtocol::kPixelsPerSlot; ++pixel) {
            const size_t index = slot * GrayProtocol::kPixelsPerSlot + pixel;
            QCOMPARE(frame->echo1[index], static_cast<uint32_t>(1000 + pixel));
            QCOMPARE(frame->echo2[index], static_cast<uint32_t>(2000 + pixel));
        }
    }
}

void GrayImageTest::OutOfOrderDuplicatesAndFrameBoundaries()
{
    GrayFrameAssembler assembler;
    auto first = Packet(30, 0, 100, 200);
    QVERIFY(!assembler.Feed(first.data(), first.size()));
    // 同 SLOT 不同 seq_num 仍为重复，冲突数据保留第一次有效值。
    auto duplicate = Packet(30, 0, 300, 400);
    duplicate[5] = 9;
    QVERIFY(!assembler.Feed(duplicate.data(), duplicate.size()));
    QCOMPARE(assembler.GetStats().duplicate_slots, uint64_t{1});
    QCOMPARE(assembler.GetStats().conflicting_slots, uint64_t{1});
    // 新帧 SLOT 0 到达不能把上一帧截断。
    auto next = Packet(31, 0, 500, 600);
    QVERIFY(!assembler.Feed(next.data(), next.size()));
    GrayEchoFramePtr frame;
    for (int slot = 19; slot >= 1; --slot) {
        auto bytes = Packet(30, static_cast<uint16_t>(slot), 100 + slot, 200 + slot);
        frame = assembler.Feed(bytes.data(), bytes.size());
    }
    QVERIFY(frame);
    QCOMPARE(frame->frame_id, uint16_t{30});
    QCOMPARE(frame->echo1[0], uint32_t{100});
    QCOMPARE(frame->echo1[19 * 120], uint32_t{119});
    QVERIFY(!assembler.Feed(first.data(), first.size()));
    QCOMPARE(assembler.GetStats().late_packets, uint64_t{1});
    for (int slot = 1; slot < 20; ++slot) {
        auto bytes = Packet(31, static_cast<uint16_t>(slot), 500, 600);
        frame = assembler.Feed(bytes.data(), bytes.size());
    }
    QVERIFY(frame);
    QCOMPARE(frame->echo1[0], uint32_t{500});
}

void GrayImageTest::ExpirationRolloverAndSessionReset()
{
    GrayFrameAssembler assembler(std::chrono::milliseconds(50));
    auto now = GrayFrameAssembler::Clock::now();
    auto bytes = Packet(65534, 0);
    QVERIFY(!assembler.Feed(bytes.data(), bytes.size(), now));
    assembler.Expire(now + std::chrono::milliseconds(51));
    QCOMPARE(assembler.GetStats().incomplete_frames, uint64_t{1});
    QVERIFY(!assembler.Feed(bytes.data(), bytes.size(), now + std::chrono::milliseconds(52)));
    for (uint16_t id : {uint16_t{65535}, uint16_t{0}}) {
        GrayEchoFramePtr frame;
        for (uint16_t slot = 0; slot < 20; ++slot) {
            bytes = Packet(id, slot);
            frame = assembler.Feed(bytes.data(), bytes.size(), now + std::chrono::milliseconds(53));
        }
        QVERIFY(frame);
        QCOMPARE(frame->frame_id, id);
    }
    assembler.Reset(321, 9);
    GrayEchoFramePtr frame;
    for (uint16_t slot = 0; slot < 20; ++slot) {
        bytes = Packet(0, slot);
        frame = assembler.Feed(bytes.data(), bytes.size());
    }
    QVERIFY(frame);
    QCOMPARE(frame->session_id, uint64_t{9});
    QCOMPARE(frame->device_ip, uint32_t{321});
}

void GrayImageTest::MappingAndMatrixOrientation()
{
    auto frame = CompleteFrame();
    auto intensity = ExtractGrayIntensity(*frame, GrayEchoSelection::KFirst);
    QCOMPARE(intensity.width, 120);
    QCOMPARE(intensity.height, 20);
    QCOMPARE(intensity.values[19 * 120 + 119], frame->echo1[2399]);
    GrayToneMapper mapper;
    GrayDisplayOptions options;
    options.mapping = GrayMappingMode::KFixed;
    QImage image = mapper.Map(intensity, options);
    QCOMPARE(image.size(), QSize(120, 20));
    QCOMPARE(image.constScanLine(19)[119], static_cast<uchar>(std::lround(959600.0 / 1048575.0 * 255.0)));
    QCOMPARE(SelectGrayEcho(70000, 900000, GrayEchoSelection::KMaximum), uint32_t{900000});
    QCOMPARE(SelectGrayEcho(70000, 900000, GrayEchoSelection::KFirst), uint32_t{70000});
    QCOMPARE(SelectGrayEcho(70000, 900000, GrayEchoSelection::KSecond), uint32_t{900000});
    options.mapping = GrayMappingMode::KAutomatic;
    intensity.values.assign(2400, 0);
    mapper.Reset();
    image = mapper.Map(intensity, options);
    QCOMPARE(image.constScanLine(0)[0], uchar{0});
    intensity.values.assign(2400, 70000);
    mapper.Reset();
    image = mapper.Map(intensity, options);
    QCOMPARE(image.constScanLine(0)[0], uchar{255});
    intensity.values[0] = 0xFFFFF;
    intensity.valid_pixels[0] = 0;
    mapper.Reset();
    image = mapper.Map(intensity, options);
    QCOMPARE(image.constScanLine(0)[0], uchar{0});
    QCOMPARE(image.constScanLine(0)[1], uchar{255});
    QCOMPARE(mapper.Map(intensity, options), image);
}

void GrayImageTest::PauseSelectionAndLatestFrameQueue()
{
    GrayImageController controller;
    controller.BeginSession(123, 1);
    QSignalSpy spy(&controller, &GrayImageController::ImageReadySig);
    std::thread producer([&controller]() {
        for (uint16_t id = 1; id <= 100; ++id) {
            controller.OfferFrame(CompleteFrame(id));
        }
    });
    producer.join();
    QTRY_VERIFY(controller.GetDisplayedFrame());
    QCOMPARE(controller.GetDisplayedFrame()->frame_id, uint16_t{100});
    QCOMPARE(spy.count(), 1);
    controller.SetPaused(true);
    controller.OfferFrame(CompleteFrame(101));
    QCoreApplication::processEvents();
    QCOMPARE(controller.GetDisplayedFrame()->frame_id, uint16_t{100});
    GrayDisplayOptions options;
    options.echo = GrayEchoSelection::KFirst;
    options.mapping = GrayMappingMode::KFixed;
    controller.SetOptions(options);
    const auto first_image = qvariant_cast<QImage>(spy.last().at(0));
    options.echo = GrayEchoSelection::KSecond;
    controller.SetOptions(options);
    const auto second_image = qvariant_cast<QImage>(spy.last().at(0));
    QVERIFY(first_image != second_image);
    options.orientation = GrayImageOrientation::KHorizontal;
    controller.SetOptions(options);
    const auto horizontal_image = qvariant_cast<QImage>(spy.last().at(0));
    QCOMPARE(horizontal_image.size(), QSize(120, 20));
    QCOMPARE(second_image.size(), QSize(20, 120));
    QCOMPARE(OrientGrayImage(horizontal_image, GrayImageOrientation::KVertical), second_image);
    QCOMPARE(controller.GetDisplayedFrame()->frame_id, uint16_t{100});
    controller.BeginSession(123, 2);
    controller.OfferFrame(CompleteFrame(102, 1));
    QCoreApplication::processEvents();
    QVERIFY(!controller.GetDisplayedFrame());
    controller.OfferFrame(CompleteFrame(0, 2));
    QTRY_VERIFY(controller.GetDisplayedFrame());
    QCOMPARE(controller.GetDisplayedFrame()->session_id, uint64_t{2});
}

void GrayImageTest::RawCsvAndContinuousRecording()
{
    QTemporaryDir directory;
    QVERIFY(directory.isValid());
    GrayImageController controller;
    controller.BeginSession(123, 1);
    controller.OfferFrame(CompleteFrame());
    QTRY_VERIFY(controller.GetDisplayedFrame());
    const QString path = directory.filePath("raw.csv");
    QString error;
    QVERIFY2(controller.SaveRawFrame(path, &error), qPrintable(error));
    QFile file(path);
    QVERIFY(file.open(QIODevice::ReadOnly | QIODevice::Text));
    const QByteArray content = file.readAll();
    QVERIFY(content.contains("1,0,0,0,1000000,1\n"));
    QVERIFY(content.contains("1,19,119,959600,40400,1\n"));
    QVERIFY(content.contains("display_orientation=vertical"));
    QCOMPARE(content.count('\n'), 2403);
    QSignalSpy saved(&controller, &GrayImageController::RecordingResultSig);
    QVERIFY(controller.StartRecording(3, directory.path()));
    // 不运行 GUI 事件循环也应记录三帧，显示刷新可以合并。
    for (uint16_t id = 2; id <= 4; ++id) {
        controller.OfferFrame(CompleteFrame(id));
    }
    QTRY_COMPARE(saved.count(), 3);
    for (const auto &result : saved) {
        QVERIFY(result.at(1).toString().isEmpty());
        QVERIFY(QFile::exists(result.at(0).toString()));
    }
}

void GrayImageTest::WidgetModeCompatibility()
{
    RawDisplayWidget widget;
    widget.BeginGraySession(123, 1);
    widget.GetGrayController()->OfferFrame(CompleteFrame());
    QTRY_VERIFY(widget.GetGrayController()->GetDisplayedFrame());
    const QImage image = widget.GetImageWidget()->GetImage();
    QCOMPARE(image.size(), QSize(20, 120));
    auto *orientation = widget.findChild<QComboBox *>("gray_orientation");
    QVERIFY(orientation);
    QCOMPARE(orientation->currentIndex(), 0);
    orientation->setCurrentIndex(1);
    const QImage horizontal = widget.GetImageWidget()->GetImage();
    QCOMPARE(horizontal.size(), QSize(120, 20));
    QCOMPARE(OrientGrayImage(horizontal, GrayImageOrientation::KVertical), image);
    orientation->setCurrentIndex(0);
    QCOMPARE(widget.GetImageWidget()->GetImage(), image);
    const QString preview = qEnvironmentVariable("YS_GRAY_PREVIEW");
    if (!preview.isEmpty()) {
        widget.resize(1200, 640);
        widget.show();
        QCoreApplication::processEvents();
        QVERIFY(widget.grab().save(preview));
    }
    // 已排队的旧直方图更新不能覆盖 Gray 图像。
    widget.GetImageWidget()->UpdateFullFrame(56, 96, std::vector<uint16_t>(56 * 96, 100));
    widget.GetImageWidget()->UpdataImage2(0, std::vector<uint16_t>(96, 100));
    QCOMPARE(widget.GetImageWidget()->GetImage(), image);
    widget.EndGraySession();
    widget.GetImageWidget()->UpdateFullFrame(56, 96, std::vector<uint16_t>(56 * 96, 100));
    QCOMPARE(widget.GetImageWidget()->GetImage().size(), QSize(56, 96));
}

void GrayImageTest::OrientationPreservesPixelsAndEma()
{
    auto frame = CompleteFrame();
    auto intensity = ExtractGrayIntensity(*frame, GrayEchoSelection::KFirst);
    GrayToneMapper mapper;
    GrayDisplayOptions options;
    mapper.Map(intensity, options);
    intensity.frame_id = 2;
    for (auto &value : intensity.values) {
        value += 10000;
    }
    const QImage canonical = mapper.Map(intensity, options);
    const QImage vertical = OrientGrayImage(canonical, GrayImageOrientation::KVertical);
    QCOMPARE(vertical.size(), QSize(20, 120));
    for (int slot = 0; slot < 20; ++slot) {
        for (int pixel = 0; pixel < 120; ++pixel) {
            QCOMPARE(vertical.constScanLine(pixel)[slot], canonical.constScanLine(slot)[pixel]);
            QCOMPARE(GrayDisplayToSourcePixel(slot, pixel, GrayImageOrientation::KVertical), QPoint(slot, pixel));
            QCOMPARE(GrayDisplayToSourcePixel(pixel, slot, GrayImageOrientation::KHorizontal), QPoint(slot, pixel));
        }
    }
    QCOMPARE(GrayDisplayToSourcePixel(20, 119, GrayImageOrientation::KVertical), QPoint(-1, -1));
    QCOMPARE(GrayDisplayToSourcePixel(19, 120, GrayImageOrientation::KVertical), QPoint(-1, -1));
    QCOMPARE(GrayDisplayToSourcePixel(120, 0, GrayImageOrientation::KHorizontal), QPoint(-1, -1));
    QCOMPARE(GrayDisplayToSourcePixel(-1, 0, GrayImageOrientation::KVertical), QPoint(-1, -1));
    // 自动映射已有 EMA 历史时，切换方向仍逐像素保持同一灰阶。
    options.orientation = GrayImageOrientation::KHorizontal;
    QCOMPARE(mapper.Map(intensity, options), canonical);
    options.orientation = GrayImageOrientation::KVertical;
    QCOMPARE(mapper.Map(intensity, options), canonical);
    QTemporaryDir directory;
    QVERIFY(directory.isValid());
    const QString png_path = directory.filePath("vertical.png");
    QVERIFY(vertical.save(png_path));
    const QImage exported(png_path);
    QCOMPARE(exported.size(), QSize(20, 120));
    QCOMPARE(exported.convertToFormat(QImage::Format_Grayscale8), vertical);
}

void GrayImageTest::UdpReceptionAndRestart()
{
    QUdpSocket reservation;
    QVERIFY(reservation.bind(QHostAddress(QHostAddress::LocalHost), quint16{0}));
    const uint16_t port = reservation.localPort();
    reservation.close();
    GrayImageProcessor processor;
    // SDK handle 与 sockaddr_in 一致，使用网络序 IP。
    processor.SetPeerIP(0x0100007F);
    std::atomic<int> received{0};
    processor.SetFrameCallback([&received](GrayEchoFramePtr frame) {
        if (frame->echo1[0] == 70000 && frame->echo2[0] == 900000) {
            ++received;
        }
    });
    QVERIFY(processor.StartListening(port));
    const uint64_t first_session = processor.GetSessionId();
    QUdpSocket sender;
    for (uint16_t slot = 0; slot < 20; ++slot) {
        auto bytes = Packet(1, slot);
        QCOMPARE(sender.writeDatagram(reinterpret_cast<const char *>(bytes.data()), bytes.size(),
                                      QHostAddress::LocalHost, port),
                 qint64{977});
    }
    QTRY_COMPARE(received.load(), 1);
    for (uint16_t slot = 0; slot < 19; ++slot) {
        auto bytes = Packet(2, slot);
        sender.writeDatagram(reinterpret_cast<const char *>(bytes.data()), bytes.size(), QHostAddress::LocalHost, port);
    }
    processor.StopListening();
    QVERIFY(!processor.IsListening());
    QVERIFY(processor.StartListening(port));
    QVERIFY(processor.GetSessionId() > first_session);
    auto tail = Packet(2, 19);
    sender.writeDatagram(reinterpret_cast<const char *>(tail.data()), tail.size(), QHostAddress::LocalHost, port);
    for (uint16_t slot = 0; slot < 20; ++slot) {
        auto bytes = Packet(3, slot);
        sender.writeDatagram(reinterpret_cast<const char *>(bytes.data()), bytes.size(), QHostAddress::LocalHost, port);
    }
    QTRY_COMPARE(received.load(), 2);
    processor.SetFrameCallback(nullptr);
    processor.StopListening();
}

void GrayImageTest::BetaFragmentsAndSlowFrames()
{
    GrayFrameAssembler assembler(std::chrono::milliseconds(50));
    assembler.Reset(123, 7, GrayFormat::KBeta);
    const auto start = GrayFrameAssembler::Clock::now();
    auto packet = BetaPacket(65535, 0, 0);
    auto bad = packet;
    bad[5] = 2;
    QVERIFY(!assembler.Feed(bad.data(), bad.size(), start));
    bad = packet;
    bad[9] = 1;
    QVERIFY(!assembler.Feed(bad.data(), bad.size(), start));
    bad = packet;
    bad[7] = 0;
    QVERIFY(!assembler.Feed(bad.data(), bad.size(), start));
    bad = packet;
    WriteLe(bad.data() + 3, 168, 2);
    QVERIFY(!assembler.Feed(bad.data(), bad.size(), start));
    auto gamma = Packet(65535, 0);
    QVERIFY(!assembler.Feed(gamma.data(), gamma.size(), start));
    QCOMPARE(assembler.GetStats().invalid_packets, uint64_t{5});
    QVERIFY(!assembler.Feed(packet.data(), packet.size(), start));
    QVERIFY(!assembler.Feed(packet.data(), packet.size(), start));
    bad = packet;
    bad[17] ^= 1;
    QVERIFY(!assembler.Feed(bad.data(), bad.size(), start));
    QCOMPARE(assembler.GetStats().duplicate_slots, uint64_t{2});
    QCOMPARE(assembler.GetStats().conflicting_slots, uint64_t{1});
    assembler.Expire(start + std::chrono::seconds(20));
    QCOMPARE(assembler.GetStats().incomplete_frames, uint64_t{0});
    GrayEchoFramePtr frame;
    for (int slot = 167; slot >= 0; --slot) {
        for (int seq = 1; seq >= 0; --seq) {
            if (slot == 0 && seq == 0) {
                continue;
            }
            packet = BetaPacket(65535, slot, seq);
            frame = assembler.Feed(packet.data(), packet.size(), start + std::chrono::seconds(21));
            if (slot != 0 || seq != 1) {
                QVERIFY(!frame);
            }
        }
    }
    QVERIFY(frame);
    QVERIFY(frame->IsComplete());
    QVERIFY(frame->echo2.empty());
    QCOMPARE(frame->echo1.size(), size_t{96768});
    QCOMPARE(frame->echo1[286], uint32_t{0x100000 + 286});
    QCOMPARE(frame->echo1[287], uint32_t{0x100000 + 287});
    QCOMPARE(frame->echo1.back(), uint32_t{0x3FFFFF});
    packet = BetaPacket(0, 0, 1);
    QVERIFY(!assembler.Feed(packet.data(), packet.size(), start + std::chrono::seconds(22)));
    auto old = BetaPacket(65535, 0, 0);
    QVERIFY(!assembler.Feed(old.data(), old.size(), start + std::chrono::seconds(22)));
    packet = BetaPacket(1, 0, 0);
    QVERIFY(!assembler.Feed(packet.data(), packet.size(), start + std::chrono::seconds(23)));
    packet = BetaPacket(2, 0, 0);
    QVERIFY(!assembler.Feed(packet.data(), packet.size(), start + std::chrono::seconds(24)));
    QCOMPARE(assembler.GetStats().incomplete_frames, uint64_t{1});
    assembler.Reset(123, 8, GrayFormat::KBeta);
    QCOMPARE(assembler.GetStats().completed_frames, uint64_t{0});
}

void GrayImageTest::BetaDisplayAndExport()
{
    LidarInfo node{};
    node.model = LidarModel::KCS;
    node.hardware_version[0] = 'B';
    QCOMPARE(GrayFormatForNode(node), GrayFormat::KBeta);
    node.hardware_version[0] = 'G';
    QCOMPARE(GrayFormatForNode(node), GrayFormat::KGamma);
    auto frame = std::make_shared<GrayEchoFrame>(GrayFormat::KBeta);
    frame->device_ip = 123;
    frame->session_id = 1;
    frame->frame_id = 1;
    frame->received_slots.set();
    frame->echo1.back() = 0x3FFFFF;
    auto intensity = ExtractGrayIntensity(*frame, GrayEchoSelection::KSecond);
    QCOMPARE(intensity.width, 576);
    QCOMPARE(intensity.height, 168);
    QCOMPARE(intensity.values.back(), uint32_t{0x3FFFFF});
    GrayDisplayOptions options;
    options.mapping = GrayMappingMode::KFixed;
    options.white_level = 0x3FFFFF;
    GrayToneMapper mapper;
    auto image = mapper.Map(intensity, options);
    QCOMPARE(image.size(), QSize(576, 168));
    QCOMPARE(image.constScanLine(167)[575], uchar{255});
    QCOMPARE(OrientGrayImage(image, GrayImageOrientation::KVertical).size(), QSize(168, 576));
    QCOMPARE(GrayDisplayToSourcePixel(167, 575, GrayImageOrientation::KVertical, 168, 576), QPoint(167, 575));
    QCOMPARE(GrayDisplayToSourcePixel(168, 575, GrayImageOrientation::KVertical, 168, 576), QPoint(-1, -1));
    QTemporaryDir directory;
    QVERIFY(directory.isValid());
    const QString path = directory.filePath("beta.csv");
    QVERIFY(SaveGrayFrameCsv(*frame, path, options));
    QFile csv(path);
    QVERIFY(csv.open(QIODevice::ReadOnly));
    const auto text = csv.readAll();
    QVERIFY(text.contains("width=576,height=168,bits=22"));
    QVERIFY(text.contains("frame_id,slot,pixel,gray,valid"));
    QVERIFY(text.trimmed().endsWith("1,167,575,4194303,1"));
    RawDisplayWidget widget;
    widget.BeginGraySession(123, 1, GrayFormat::KBeta);
    auto *echo = widget.findChild<QComboBox *>("gray_echo_selection");
    QCOMPARE(echo->count(), 1);
    QVERIFY(!echo->isEnabled());
    QCOMPARE(widget.findChild<QSpinBox *>("gray_white_level")->maximum(), 4194303);
    widget.GetGrayController()->OfferFrame(frame);
    QTRY_VERIFY(widget.GetGrayController()->GetDisplayedFrame());
    QCOMPARE(widget.GetImageWidget()->GetImage().size(), QSize(168, 576));
    auto *orientation = widget.findChild<QComboBox *>("gray_orientation");
    orientation->setCurrentIndex(1);
    QCOMPARE(widget.GetImageWidget()->GetImage().size(), QSize(576, 168));
    widget.EndGraySession();
    widget.BeginGraySession(123, 2, GrayFormat::KGamma);
    QCOMPARE(echo->count(), 3);
    QVERIFY(echo->isEnabled());
    QCOMPARE(widget.findChild<QSpinBox *>("gray_white_level")->maximum(), 1048575);
}

void GrayImageTest::BetaUdpReception()
{
    QUdpSocket reservation;
    QVERIFY(reservation.bind(QHostAddress::LocalHost, 0));
    const auto port = reservation.localPort();
    reservation.close();
    GrayImageProcessor processor;
    processor.SetPeerIP(0x0100007F);
    std::atomic<int> received{0};
    processor.SetFrameCallback([&received](GrayEchoFramePtr frame) {
        if (frame->format == GrayFormat::KBeta && frame->IsComplete() && frame->echo1.back() == 0x3FFFFF) {
            ++received;
        }
    });
    QUdpSocket sender;
    for (int cycle = 0; cycle < 3; ++cycle) {
        QVERIFY(processor.StartListening(port, std::chrono::milliseconds(50), GrayFormat::KBeta));
        const auto malformed = BetaPacket(1, 0, 0);
        QCOMPARE(sender.writeDatagram(reinterpret_cast<const char *>(malformed.data()), malformed.size() - 1,
                                      QHostAddress::LocalHost, port), qint64{1164});
        QTRY_COMPARE(processor.GetStats().invalid_packets, uint64_t{1});
        for (uint16_t slot = 0; slot < 168; ++slot) {
            for (uint16_t seq : {1, 0}) {
                const auto packet = BetaPacket(1, slot, seq);
                QCOMPARE(sender.writeDatagram(reinterpret_cast<const char *>(packet.data()), packet.size(),
                                              QHostAddress::LocalHost, port), qint64{1165});
            }
            QTest::qWait(3);
        }
        QTRY_COMPARE(received.load(), cycle + 1);
        QCOMPARE(processor.GetStats().completed_frames, uint64_t{1});
        // 在未完成帧的连续收包期间停止，覆盖队列清空与 context 回收后的重启。
        for (uint16_t slot = 0; slot < 100; ++slot) {
            const auto packet = BetaPacket(2, slot, 0);
            sender.writeDatagram(reinterpret_cast<const char *>(packet.data()), packet.size(),
                                 QHostAddress::LocalHost, port);
        }
        processor.StopListening();
        QVERIFY(!processor.IsListening());
    }
}

void GrayImageTest::InstructionListenerReadyBeforeHandshake()
{
    auto &supervisor = NodeSupervisorGetInstance();
    LidarInfo node{};
    node.ip = 0x0200007F;
    node.unique_id = node.ip;
    node.instruction_processor_stoped = true;
    node.data_processor_stoped = true;
    node.last_seen_timestamp =
        std::chrono::duration_cast<std::chrono::microseconds>(std::chrono::steady_clock::now().time_since_epoch())
            .count();
    supervisor.AddNode(node);
    QUdpSocket reservation;
    QVERIFY(reservation.bind(QHostAddress(QHostAddress::LocalHost), quint16{0}));
    const auto port = reservation.localPort();
    reservation.close();
    InstructionProcessor processor;
    processor.SetPeerIP(node.ip);
    QVERIFY(processor.StartListening(port));
    QVERIFY(processor.IsListening());
    QVERIFY(supervisor.GetNode(node.ip));
    QVERIFY(!supervisor.GetNode(node.ip)->instruction_processor_stoped);
    processor.StopListening();
    QVERIFY(!processor.IsListening());
    QVERIFY(supervisor.GetNode(node.ip));
    QVERIFY(supervisor.GetNode(node.ip)->instruction_processor_stoped);
}

void GrayImageTest::LiveBetaConnection()
{
    const QString address = qEnvironmentVariable("YS_LIVE_BETA_IP");
    if (address.isEmpty()) {
        QSKIP("Set YS_LIVE_BETA_IP to explicitly enable the hardware handshake test");
    }
    bool valid = false;
    const uint32_t ip = qToBigEndian(QHostAddress(address).toIPv4Address(&valid));
    QVERIFY(valid);

    struct LiveState
    {
        std::atomic<bool> connected{false};
        std::atomic<int> frames{0};
    };

    const auto state = std::make_shared<LiveState>();
    // 独立运行本用例时也由 Qt 初始化 Windows socket 运行时。
    QUdpSocket socket_runtime;
    QVERIFY(socket_runtime.bind(QHostAddress(QHostAddress::LocalHost), quint16{0}));
    InitLogger();
    SetLoggerLevel(2);
    QVERIFY(Initialization());
    const auto cleanup = qScopeGuard([]() {
        GrayImageProcessorGetInstance().SetFrameCallback(nullptr);
        Cleanup();
        InstructionProcessorGetInstance().StopListening();
    });
    QTest::qWait(200);
    auto &supervisor = NodeSupervisorGetInstance();
    supervisor.RegisterNodeFoundCallback([state, ip](const LidarInfo &node, void *) {
        if (node.ip == ip) {
            state->connected = node.is_connected && !node.instruction_processor_stoped && !node.data_processor_stoped;
        }
    });
    GrayImageProcessorGetInstance().SetFrameCallback([state](GrayEchoFramePtr frame) {
        if (frame->format == GrayFormat::KBeta && frame->IsComplete()) {
            ++state->frames;
        }
    });
    // 广播已关闭时，按用户明确指定的 Beta Gray 机型和 IP 发起现有 SDK 握手。
    LidarInfo node{};
    node.ip = ip;
    node.unique_id = ip;
    node.broadcast_port = qToBigEndian(quint16{65000});
    node.model = LidarModel::KCS;
    node.hardware_version[0] = 'B';
    node.node_work_mode = NodeWorkMode::KGray;
    node.instruction_processor_stoped = true;
    node.data_processor_stoped = true;
    node.last_seen_timestamp =
        std::chrono::duration_cast<std::chrono::microseconds>(std::chrono::steady_clock::now().time_since_epoch())
            .count();
    supervisor.AddNode(node);
    ::ConnectNode(ip);
    QTRY_VERIFY_WITH_TIMEOUT(state->connected.load(), 10000);
    qInfo() << "Live handshake confirmed with both receivers ready:" << address;
    QElapsedTimer deadline;
    deadline.start();
    while (state->frames.load() == 0 && deadline.elapsed() < 10000) {
        QTest::qWait(50);
    }
    const auto stats = GrayImageProcessorGetInstance().GetStats();
    qInfo() << "Live Beta complete frames:" << state->frames.load() << "invalid packets:" << stats.invalid_packets
            << "incomplete frames:" << stats.incomplete_frames << "late packets:" << stats.late_packets
            << "queue drops:" << stats.queue_drops;
    QVERIFY(state->frames.load() > 0);
}

QTEST_MAIN(GrayImageTest)
#include "gray_image_test.moc"
