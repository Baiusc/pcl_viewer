#include "business/cs96_gamma_histogram.h"
#include "business/raw_histogram_parse_imp.h"
#include "business/raw_histogram_parser.h"
#include "component/raw_display_widget.h"
#include "you_see/business/gamma_histogram_assembler.h"
#include "you_see/business/node_supervisor.h"
#include "you_see/business/raw_histogram_processor.h"

#include <QDir>
#include <QElapsedTimer>
#include <QFile>
#include <QScopeGuard>
#include <QSemaphore>
#include <QSignalSpy>
#include <QTemporaryDir>
#include <QThreadPool>
#include <QUdpSocket>
#include <QUuid>
#include <QtEndian>
#include <QtTest>

#include <array>
#include <atomic>
#include <thread>

namespace {
using namespace you_see;
using namespace ys_scene;
using PacketBytes = std::array<uint8_t, RawParser::kRawUdpPacketSizeCs96Gamma>;

uint16_t BinValue(size_t pixel, size_t bin)
{
    return static_cast<uint16_t>(pixel * 512 + bin);
}

void WriteLe16(uint8_t *data, uint16_t value)
{
    data[0] = static_cast<uint8_t>(value);
    data[1] = static_cast<uint8_t>(value >> 8);
}

PacketBytes Packet(uint16_t id, uint16_t seq)
{
    PacketBytes bytes{};
    bytes[0] = 0xAA;
    WriteLe16(bytes.data() + 1, id);
    WriteLe16(bytes.data() + 5, seq);
    WriteLe16(bytes.data() + 7, 4096);
    for (size_t pixel = 0; pixel < 4; ++pixel) {
        for (size_t bin = 0; bin < 512; ++bin) {
            WriteLe16(bytes.data() + 9 + (pixel * 512 + bin) * 2, BinValue(seq * 4 + pixel, bin));
        }
    }
    return bytes;
}

HistogramPackets FramePackets()
{
    HistogramPackets packets;
    for (uint16_t seq = 0; seq < 30; ++seq) {
        const auto bytes = Packet(1, seq);
        packets[0][seq] = std::vector<uint8_t>(bytes.begin() + 9, bytes.end());
    }
    return packets;
}

void OfferGammaFrame(RawHistogramParser &parser, HistogramPackets &packets, uint16_t id)
{
    RawHistogramFrameWrapperT wrapper{};
    wrapper.device_type = static_cast<uint32_t>(DeviceType::kCs96Gamma);
    wrapper.work_mode = static_cast<uint32_t>(NodeWorkMode::KRawHistogram);
    wrapper.frame_id = id;
    wrapper.frame_map_data_ptr = &packets;
    parser.DealRawHistogramFrame(&wrapper);
}

void VerifySavedFrame(const QString &path, uint16_t id)
{
    QFile text(path);
    QVERIFY(text.open(QIODevice::ReadOnly | QIODevice::Text));
    const auto contents = text.readAll();
    QVERIFY(contents.contains("frame_id: " + QByteArray::number(id) + '\n'));
    QVERIFY(contents.contains("cols: 1  rows: 120  bins: 512"));
    const auto lines = contents.split('\n');
    int pixel = 0;
    for (int i = 0; i < lines.size(); ++i) {
        if (!lines[i].startsWith("Start of vector")) {
            continue;
        }
        const auto values = lines.at(i + 1).trimmed().split(' ');
        QCOMPARE(values.size(), qsizetype{512});
        for (size_t bin = 0; bin < 512; ++bin) {
            QCOMPARE(values[bin].toUInt(), static_cast<unsigned>(BinValue(pixel, bin)));
        }
        ++pixel;
    }
    QCOMPARE(pixel, 120);
    QFile binary(path.chopped(4) + ".bin");
    QVERIFY(binary.open(QIODevice::ReadOnly));
    const auto bytes = binary.readAll();
    QCOMPARE(bytes.size(), qsizetype{122880});
    for (size_t pixel = 0; pixel < 120; ++pixel) {
        for (size_t bin = 0; bin < 512; ++bin) {
            const size_t offset = (pixel * 512 + bin) * 2;
            const uint16_t value = static_cast<uint8_t>(bytes[offset]) |
                                   (static_cast<uint16_t>(static_cast<uint8_t>(bytes[offset + 1])) << 8);
            QCOMPARE(value, BinValue(pixel, bin));
        }
    }
}
} // namespace

class Cs96GammaHistogramTest : public QObject
{
    Q_OBJECT
  private slots:
    void ReordersAndDeduplicates();
    void RejectsMalformedAndIncompleteFrames();
    void WrapsAndResets();
    void DisplaysAllPixelsAndBins();
    void SavesExactTextAndBinary();
    void SavesSingleFrameSnapshot();
    void BurstKeepsOnlyLatestFrame();
    void SustainedStreamRefreshesAtBoundedRate();
    void UdpDisplayAndSave_data();
    void UdpDisplayAndSave();
};

void Cs96GammaHistogramTest::ReordersAndDeduplicates()
{
    GammaHistogramAssembler assembler;
    for (int seq = 29; seq > 0; --seq) {
        const auto bytes = Packet(1, seq);
        QVERIFY(!assembler.Feed(bytes.data(), bytes.size()));
        QVERIFY(!assembler.Feed(bytes.data(), bytes.size()));
    }
    const auto first = Packet(1, 0);
    const auto frame = assembler.Feed(first.data(), first.size());
    QVERIFY(frame);
    QCOMPARE(frame->frame_number, uint16_t{1});
    QCOMPARE(frame->received_fragments, size_t{30});
    QCOMPARE(frame->frame_data.size(), size_t{1});
    const auto bins = DecodeGammaHistogram(frame->frame_data);
    QCOMPARE(bins.size(), size_t{120 * 512});
    for (size_t pixel = 0; pixel < 120; ++pixel) {
        for (size_t bin = 0; bin < 512; ++bin) {
            QCOMPARE(bins[pixel * 512 + bin], BinValue(pixel, bin));
        }
    }
    for (uint16_t seq = 0; seq < 30; ++seq) {
        const auto duplicate = Packet(1, seq);
        QVERIFY(!assembler.Feed(duplicate.data(), duplicate.size()));
    }
}

void Cs96GammaHistogramTest::RejectsMalformedAndIncompleteFrames()
{
    GammaHistogramAssembler assembler;
    auto bytes = Packet(10, 0);
    QVERIFY(!assembler.Feed(nullptr, bytes.size()));
    QVERIFY(!assembler.Feed(bytes.data(), 8));
    QVERIFY(!assembler.Feed(bytes.data(), bytes.size() - 1));
    bytes[0] = 0;
    QVERIFY(!assembler.Feed(bytes.data(), bytes.size()));
    bytes = Packet(10, 0);
    WriteLe16(bytes.data() + 3, 1);
    QVERIFY(!assembler.Feed(bytes.data(), bytes.size()));
    bytes = Packet(10, 30);
    QVERIFY(!assembler.Feed(bytes.data(), bytes.size()));
    bytes = Packet(10, 0);
    WriteLe16(bytes.data() + 7, 4095);
    QVERIFY(!assembler.Feed(bytes.data(), bytes.size()));
    // 只发 1..29：所有非法的 seq 0 都不能参与成帧。
    for (uint16_t seq = 1; seq < 30; ++seq) {
        bytes = Packet(10, seq);
        QVERIFY(!assembler.Feed(bytes.data(), bytes.size()));
    }
    // 另一个 frame_id 独立组帧，不能拿上一帧的分包补齐。
    bytes = Packet(11, 0);
    QVERIFY(!assembler.Feed(bytes.data(), bytes.size()));
    bytes = Packet(10, 0);
    QVERIFY(assembler.Feed(bytes.data(), bytes.size()));
    for (uint16_t seq = 1; seq < 30; ++seq) {
        bytes = Packet(11, seq);
        const auto frame = assembler.Feed(bytes.data(), bytes.size());
        QCOMPARE(static_cast<bool>(frame), seq == 29);
    }
}

void Cs96GammaHistogramTest::WrapsAndResets()
{
    GammaHistogramAssembler assembler;
    for (uint16_t id : {uint16_t{65535}, uint16_t{0}, uint16_t{1}}) {
        for (uint16_t seq = 0; seq < 30; ++seq) {
            const auto bytes = Packet(id, seq);
            const auto frame = assembler.Feed(bytes.data(), bytes.size());
            QCOMPARE(static_cast<bool>(frame), seq == 29);
            if (frame) {
                QCOMPARE(frame->frame_number, id);
            }
        }
    }
    const auto late = Packet(65535, 0);
    QVERIFY(!assembler.Feed(late.data(), late.size()));
    assembler.Reset();
    for (uint16_t seq = 0; seq < 30; ++seq) {
        const auto bytes = Packet(1, seq);
        QCOMPARE(static_cast<bool>(assembler.Feed(bytes.data(), bytes.size())), seq == 29);
    }
}

void Cs96GammaHistogramTest::DisplaysAllPixelsAndBins()
{
    const auto frame = FramePackets();
    RawHistogramParseImp processor;
    processor.SetBinRange(511, 512);
    std::vector<uint16_t> captured;
    connect(&processor, &RawHistogramParseImp::grayvec_full_frame_sig, this,
            [&](int cols, int rows, const std::vector<uint16_t> &gray) {
                QCOMPARE(cols, 1);
                QCOMPARE(rows, 120);
                captured = gray;
            });
    processor.RawDataProcessor2(0,
                                std::make_shared<std::unordered_map<uint16_t, std::vector<uint8_t>>>(frame.at(0)),
                                DeviceType::kCs96Gamma);
    QCOMPARE(captured.size(), size_t{120});
    for (size_t pixel = 0; pixel < 120; ++pixel) {
        QCOMPARE(captured[pixel], static_cast<uint16_t>(BinValue(pixel, 511) * 0.06f));
    }
    RawDisplayWidget widget;
    widget.GetFrame(frame, DeviceType::kCs96Gamma, 7);
    QCOMPARE(widget.GetImageWidget()->GetImage().size(), QSize(1, 120));
    widget.GetImageWidget()->PlotShowSig(0, {0, 119});
    auto *plot = widget.findChild<RawCustomplot *>("line_show_wid0");
    QVERIFY(plot);
    QCOMPARE(plot->graph(0)->dataCount(), 512);
    QCOMPARE(plot->graph(0)->data()->at(0)->key, 0.0);
    QCOMPARE(plot->graph(0)->data()->at(511)->value, static_cast<double>(BinValue(119, 511)));
    auto changed = frame;
    changed[0][29][4094] = 0xFF;
    changed[0][29][4095] = 0xFF;
    widget.GetFrame(changed, DeviceType::kCs96Gamma, 8);
    QCOMPARE(plot->graph(0)->data()->at(511)->value, 65535.0);
}

void Cs96GammaHistogramTest::SavesExactTextAndBinary()
{
    QTemporaryDir directory;
    QVERIFY(directory.isValid());
    const auto path = directory.filePath("gamma.txt");
    auto packets = FramePackets();
    QVERIFY(SaveGammaHistogram(packets, 65535, path.toStdString()));
    VerifySavedFrame(path, 65535);
    packets[0].erase(29);
    QVERIFY(!SaveGammaHistogram(packets, 0, directory.filePath("incomplete.txt").toStdString()));
    QVERIFY(!QFile::exists(directory.filePath("incomplete.bin")));
}

void Cs96GammaHistogramTest::SavesSingleFrameSnapshot()
{
    RawDisplayWidget widget;
    widget.GetFrame(FramePackets(), DeviceType::kCs96Gamma, 123);
    const QDir root(QCoreApplication::applicationDirPath() + "/raw_data");
    const auto before = root.entryList({"raw_data_cs96_gamma_*.txt"}, QDir::Files);
    QStringList created;
    const auto cleanup = qScopeGuard([&]() {
        QThreadPool::globalInstance()->waitForDone();
        for (const auto &name : created) {
            QFile::remove(root.filePath(name));
            QFile::remove(root.filePath(name.chopped(4) + ".bin"));
        }
    });
    QVERIFY(QMetaObject::invokeMethod(&widget, "SaveFrame", Qt::DirectConnection));
    widget.GetFrame(FramePackets(), DeviceType::kCs96Gamma, 124);
    QThreadPool::globalInstance()->waitForDone();
    for (const auto &name : root.entryList({"raw_data_cs96_gamma_*.txt"}, QDir::Files)) {
        if (!before.contains(name)) {
            created.push_back(name);
        }
    }
    QCOMPARE(created.size(), qsizetype{1});
    VerifySavedFrame(root.filePath(created.front()), 123);
}

void Cs96GammaHistogramTest::BurstKeepsOnlyLatestFrame()
{
    RawHistogramParser parser;
    RawDisplayWidget widget;
    connect(&parser, &RawHistogramParser::SendFrameSig, &widget, &RawDisplayWidget::GetFrame);
    connect(&widget, &RawDisplayWidget::BinRangeChangedSig, &parser, &RawHistogramParser::SetBinRange);
    std::vector<uint16_t> displayed;
    int gray_frames = 0;
    connect(&parser, &RawHistogramParser::SendFrameSig, &widget,
            [&](const HistogramPackets &, DeviceType, uint16_t id) { displayed.push_back(id); });
    for (auto *worker : parser.GetProcessors()) {
        connect(worker, &RawHistogramParseImp::grayvec_full_frame_sig, &widget,
                [&](int cols, int rows, const std::vector<uint16_t> &gray) {
                    ++gray_frames;
                    widget.GetImageWidget()->UpdateFullFrame(cols, rows, gray);
                });
    }
    // 模拟全局线程池正被其他业务占用，Gamma 显示不应等待其中的历史任务。
    auto *pool = QThreadPool::globalInstance();
    const int original_threads = pool->maxThreadCount();
    pool->setMaxThreadCount(1);
    QSemaphore started;
    QSemaphore release;
    pool->start([&]() {
        started.release();
        release.acquire();
    });
    const auto cleanup = qScopeGuard([&]() {
        release.release();
        pool->waitForDone();
        pool->setMaxThreadCount(original_threads);
        RawHistogramProcessorGetInstance().SetRawFrameCallback(nullptr, nullptr);
    });
    QVERIFY(started.tryAcquire(1, 1000));
    auto packets = FramePackets();
    // GUI 暂停消费期间收到 1000 个完整帧，不能留下 1000 个携带数据的 Qt 事件。
    std::thread producer([&]() {
        for (uint16_t id = 1; id <= 1000; ++id) {
            OfferGammaFrame(parser, packets, id);
        }
    });
    producer.join();
    QCoreApplication::sendPostedEvents(&widget, QEvent::MetaCall);
    QCOMPARE(displayed.size(), size_t{0});
    QCOMPARE(gray_frames, 0);
    QTRY_COMPARE_WITH_TIMEOUT(displayed.size(), size_t{1}, 1000);
    QCOMPARE(displayed.back(), uint16_t{1000});
    QCOMPARE(gray_frames, 1);
    QCOMPARE(widget.GetImageWidget()->GetImage().size(), QSize(1, 120));
    QTest::qWait(160);
    QCOMPARE(displayed.size(), size_t{1}); // 不再逐步回放积压的 999 个历史帧。
    parser.SetPause(true);
    OfferGammaFrame(parser, packets, 1001);
    QTest::qWait(60);
    QCOMPARE(displayed.size(), size_t{1});
    parser.SendData();
    QCOMPARE(displayed.back(), uint16_t{1000}); // 暂停时仍可提取完整帧快照。
    parser.SetPause(false);
    OfferGammaFrame(parser, packets, 1002);
    QTRY_COMPARE_WITH_TIMEOUT(displayed.back(), uint16_t{1002}, 1000);
}

void Cs96GammaHistogramTest::SustainedStreamRefreshesAtBoundedRate()
{
    RawHistogramParser parser;
    RawDisplayWidget widget;
    widget.resize(1000, 600);
    widget.show();
    connect(&parser, &RawHistogramParser::SendFrameSig, &widget, &RawDisplayWidget::GetFrame);
    connect(&widget, &RawDisplayWidget::BinRangeChangedSig, &parser, &RawHistogramParser::SetBinRange);
    std::vector<uint16_t> displayed;
    connect(&parser, &RawHistogramParser::SendFrameSig, &widget,
            [&](const HistogramPackets &, DeviceType, uint16_t id) { displayed.push_back(id); });
    for (auto *worker : parser.GetProcessors()) {
        connect(worker, &RawHistogramParseImp::grayvec_full_frame_sig, widget.GetImageWidget(),
                &ImageWidget::UpdateFullFrame, Qt::QueuedConnection);
    }
    auto packets = FramePackets();
    OfferGammaFrame(parser, packets, 0);
    QTRY_COMPARE_WITH_TIMEOUT(displayed.size(), size_t{1}, 1000);
    for (int plot = 0; plot < 6; ++plot) {
        widget.GetImageWidget()->PlotShowSig(plot, {0, 119 - plot});
    }
    auto *plot = widget.findChild<RawCustomplot *>("line_show_wid0");
    QVERIFY(plot);
    plot->xAxis->setRange(50, 150);
    int heartbeats = 0;
    QTimer heartbeat;
    connect(&heartbeat, &QTimer::timeout, &widget, [&]() { ++heartbeats; });
    heartbeat.start(10);
    QElapsedTimer elapsed;
    elapsed.start();
    std::atomic<bool> finished{false};
    std::thread producer([&]() {
        for (uint16_t id = 1; id <= 100; ++id) {
            WriteLe16(packets[0][29].data() + 4094, id);
            OfferGammaFrame(parser, packets, id);
            std::this_thread::sleep_for(std::chrono::milliseconds(14)); // 约 71fps。
        }
        finished.store(true);
    });
    const auto cleanup = qScopeGuard([&]() {
        producer.join();
        RawHistogramProcessorGetInstance().SetRawFrameCallback(nullptr, nullptr);
    });
    QTest::qSleep(250); // 故意阻塞 GUI；恢复后应直接显示最新帧。
    QTRY_VERIFY_WITH_TIMEOUT(finished.load(), 5000);
    QTRY_COMPARE_WITH_TIMEOUT(displayed.back(), uint16_t{100}, 1000);
    QVERIFY(displayed.size() <= static_cast<size_t>(elapsed.elapsed() / 50 + 3));
    QVERIFY(displayed.size() < 60);
    QVERIFY(heartbeats > 20);
    QCOMPARE(plot->graph(0)->data()->at(511)->value, 100.0);
    QCOMPARE(plot->xAxis->range().lower, 50.0);
    QCOMPARE(plot->xAxis->range().upper, 150.0);
    QCOMPARE(widget.GetImageWidget()->GetImage().size(), QSize(1, 120));
}

void Cs96GammaHistogramTest::UdpDisplayAndSave_data()
{
    QTest::addColumn<int>("mode");
    QTest::addColumn<int>("save_count");
    QTest::newRow("continuous") << static_cast<int>(NodeWorkMode::KRawHistogram) << 2;
    QTest::newRow("continuous_save_one") << static_cast<int>(NodeWorkMode::KRawHistogram) << 1;
}

void Cs96GammaHistogramTest::UdpDisplayAndSave()
{
    QFETCH(int, mode);
    QFETCH(int, save_count);
    LidarInfo node{};
    node.model = LidarModel::KCS;
    node.hardware_version[0] = 'G';
    node.node_work_mode = static_cast<NodeWorkMode>(mode);
    node.ip = qToBigEndian(uint32_t{0x7F000001});
    node.is_connected = true;
    NodeSupervisorGetInstance().AddNode(node);
    RawHistogramParser parser;
    RawDisplayWidget widget;
    connect(&parser, &RawHistogramParser::SendFrameSig, &widget, &RawDisplayWidget::GetFrame);
    connect(&widget, &RawDisplayWidget::BinRangeChangedSig, &parser, &RawHistogramParser::SetBinRange);
    for (auto *worker : parser.GetProcessors()) {
        connect(worker, &RawHistogramParseImp::grayvec_full_frame_sig, widget.GetImageWidget(),
                &ImageWidget::UpdateFullFrame, Qt::QueuedConnection);
    }
    QSignalSpy received(&parser, &RawHistogramParser::SendFrameSig);
    auto &receiver = RawHistogramProcessorGetInstance();
    receiver.StopListening();
    receiver.SetSourceNode(node);
    QUdpSocket reservation;
    QVERIFY(reservation.bind(QHostAddress::LocalHost, quint16{0}));
    const auto port = reservation.localPort();
    reservation.close();
    const QString prefix = "gamma_histogram_test_" + QUuid::createUuid().toString(QUuid::Id128);
    const QDir root(QCoreApplication::applicationDirPath() + "/raw_data");
    const auto cleanup = qScopeGuard([&]() {
        receiver.StopListening();
        receiver.SetRawFrameCallback(nullptr, nullptr);
        QThreadPool::globalInstance()->waitForDone();
        for (const auto &folder : root.entryList({prefix + "*"}, QDir::Dirs | QDir::NoDotAndDotDot)) {
            QDir(root.filePath(folder)).removeRecursively();
        }
    });
    QVERIFY(receiver.StartListening(port));
    parser.SetSaveRawHistogram({save_count, prefix.toStdString()});
    QUdpSocket sender;
    // 截断到 Alpha 包长也必须拒绝，不能进入旧协议解析路径。
    const auto truncated = Packet(65535, 0);
    QCOMPARE(
        sender.writeDatagram(reinterpret_cast<const char *>(truncated.data()), 4009, QHostAddress::LocalHost, port),
        qint64{4009});
    auto send = [&](uint16_t id, uint16_t seq) {
        const auto bytes = Packet(id, seq);
        QCOMPARE(sender.writeDatagram(reinterpret_cast<const char *>(bytes.data()), bytes.size(),
                                      QHostAddress::LocalHost, port),
                 qint64{4105});
    };
    // Beta 的 24 包阈值不能提前触发 Gamma 的显示或保存。
    for (int seq = 29; seq >= 6; --seq) {
        send(65535, seq);
    }
    QTest::qWait(800); // 超过旧组帧路径的 700ms 超时，仍不能把缺包帧当成完整帧。
    QCOMPARE(received.count(), 0);
    // 超时的半帧已过期；重发完整帧，随后验证 65535 -> 0。
    for (uint16_t id : {uint16_t{65535}, uint16_t{0}}) {
        for (int seq = 29; seq >= 0; --seq) {
            send(id, seq);
            if (seq % 6 == 0) {
                QTest::qWait(1);
            }
        }
        QTRY_COMPARE_WITH_TIMEOUT(received.count(), id == 65535 ? 1 : 2, 3000);
    }
    QCOMPARE(widget.GetImageWidget()->GetImage().size(), QSize(1, 120));
    QStringList saved;
    auto files_ready = [&]() {
        saved.clear();
        for (const auto &folder : root.entryList({prefix + "*"}, QDir::Dirs | QDir::NoDotAndDotDot)) {
            const QDir directory(root.filePath(folder));
            for (const auto &name : directory.entryList({"*.txt"}, QDir::Files)) {
                QFile file(directory.filePath(name));
                if (file.open(QIODevice::ReadOnly) && file.readAll().contains("End of vector [0][119]")) {
                    saved.push_back(file.fileName());
                }
            }
        }
        return saved.size() == save_count;
    };
    QTRY_VERIFY_WITH_TIMEOUT(files_ready(), 5000);
    for (const auto &path : saved) {
        VerifySavedFrame(path, path.contains(QString("_gamma_%1_").arg(save_count)) ? 65535 : 0);
    }
    // 完整帧重发不能再次显示；保存数量达到要求后也不再追加。
    for (uint16_t seq = 0; seq < 30; ++seq) {
        send(0, seq);
    }
    QTest::qWait(100);
    QCOMPARE(received.count(), 2);
    // 下一完整帧仍能显示，但连续保存已达到指定数量，不应再落盘。
    for (uint16_t seq = 0; seq < 30; ++seq) {
        send(1, seq);
    }
    QTRY_COMPARE_WITH_TIMEOUT(received.count(), 3, 3000);
    QThreadPool::globalInstance()->waitForDone();
    QVERIFY(files_ready());
}

QTEST_MAIN(Cs96GammaHistogramTest)
#include "cs96_gamma_histogram_test.moc"
