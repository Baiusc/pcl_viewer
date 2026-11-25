#include "fl6532_range_lz4_test.h"

#include <QSignalSpy>
#include <QTest>

#include <algorithm>
#include <cstring>
#include <vector>

#include "business/fl6532_range_parser.h"
#include "range6532_protocol.h"

#ifdef USE_IOCP
#include "ys_net/session/io_context.h"
#endif

#include <lz4frame.h>

namespace ys_test {

using you_see::Fl6532RangeFrame;
using you_see::Fl6532RangeFramePtr;
using you_see::Fl6532RangeLz4Meta;
using you_see::Fl6532RangeProcessor;
using you_see::Fl6532RangeProcessorGetInstance;

namespace {

constexpr size_t kPixelLen = 278;
constexpr size_t kRoi0Groups = 32;
constexpr size_t kSlotsPerGroup = 4;
constexpr size_t kPixelsPerSlot = 192;
constexpr size_t kPushPixels = kSlotsPerGroup * kPixelsPerSlot; // 768

void WriteU16(uint8_t* p, uint16_t value)
{
    p[0] = static_cast<uint8_t>(value & 0xFF);
    p[1] = static_cast<uint8_t>((value >> 8) & 0xFF);
}

void WriteU32(uint8_t* p, uint32_t value)
{
    p[0] = static_cast<uint8_t>(value & 0xFF);
    p[1] = static_cast<uint8_t>((value >> 8) & 0xFF);
    p[2] = static_cast<uint8_t>((value >> 16) & 0xFF);
    p[3] = static_cast<uint8_t>((value >> 24) & 0xFF);
}

void AppendU16(QByteArray& out, uint16_t value)
{
    out.append(static_cast<char>(value & 0xFF));
    out.append(static_cast<char>((value >> 8) & 0xFF));
}

// 构造完整 ROI0 记录流(32 group × 4 slot × 192 pixel，每条 278 B)：
// 记录内部索引合法且 echo0 有效，算法 RangePipeline 可组出 128×192 帧。
std::vector<uint8_t> BuildRoi0Records(uint16_t frame_index)
{
    std::vector<uint8_t> records(kRoi0Groups * kPushPixels * kPixelLen, 0);
    size_t offset = 0;
    for (uint16_t group = 0; group < kRoi0Groups; ++group) {
        for (uint16_t slot = 0; slot < kSlotsPerGroup; ++slot) {
            for (uint16_t pixel = 0; pixel < kPixelsPerSlot; ++pixel) {
                uint8_t* record = records.data() + offset;
                WriteU16(record + 0x02, frame_index);  // frame_index
                WriteU16(record + 0x06, group);        // group_index
                WriteU16(record + 0x08, slot);         // slot_index
                WriteU16(record + 0x12, pixel);        // pixel_id
                record[0x22] = 1;                      // en_algo_echo_bin
                record[0x23] = 1;                      // peak_num(2 峰)
                // echo0 @0x3A：peakAux=100(有效)，dist_rp=25600(=1 m)
                record[0x3A + 2] = 100;
                WriteU32(record + 0x3A + 5, 25600);
                // echo bins @0x60：写入可辨识计数
                WriteU16(record + 0x60 + 4, 1234);
                offset += kPixelLen;
            }
        }
    }
    return records;
}

// 按 push 独立转置：trans[j * count + i] = orig[i * pixel_len + j]
void TransposePush(const uint8_t* records, size_t count, uint8_t* out)
{
    for (size_t i = 0; i < count; ++i) {
        const uint8_t* record = records + i * kPixelLen;
        for (size_t j = 0; j < kPixelLen; ++j) {
            out[j * count + i] = record[j];
        }
    }
}

std::vector<uint8_t> CompressLz4(const uint8_t* data, size_t size)
{
    const size_t bound = LZ4F_compressFrameBound(size, nullptr);
    std::vector<uint8_t> out(bound);
    const size_t written = LZ4F_compressFrame(out.data(), out.size(), data, size, nullptr);
    if (LZ4F_isError(written)) {
        return {};
    }
    out.resize(written);
    return out;
}

} // namespace

you_see::Fl6532RangeLz4Meta Fl6532RangeLz4Test::BuildRoi0Meta()
{
    you_see::Fl6532RangeLz4Meta meta{};
    meta.pixel_len = static_cast<uint16_t>(kPixelLen);
    meta.total_pixels = static_cast<uint32_t>(kRoi0Groups * kPushPixels);
    meta.push_cnt = static_cast<uint8_t>(kRoi0Groups);
    meta.group_cnt = static_cast<uint8_t>(kRoi0Groups);
    meta.roi_cnt = 1;
    meta.max_sub_frame_cnt = 1;
    meta.peak_num = 2;
    meta.group_row_unit = 12;
    meta.max_bin_cnt = 2048;
    meta.roi[0].width = 192;
    meta.roi[0].height = 128;
    meta.roi[0].push_pixels = static_cast<uint16_t>(kPushPixels);
    meta.roi[0].group_cnt = static_cast<uint8_t>(kRoi0Groups);
    meta.roi[0].group_start = 0;
    meta.roi[0].sub_frame_cnt = 1;
    meta.roi[0].pcl_unit = 2;
    meta.roi[0].range_max = 1920.0f;
    return meta;
}

QByteArray Fl6532RangeLz4Test::BuildLz4Packet(uint16_t frame_id, uint16_t seq,
    uint8_t frame_flag, uint8_t sub_frame_idx, const uint8_t* payload, size_t payload_len,
    const Fl6532RangeLz4Meta* meta, size_t padded_size)
{
    const size_t header_size = (frame_flag == 0) ? 86 : 14;
    const size_t total_size = (padded_size > 0) ? padded_size : header_size + payload_len;
    QByteArray packet;
    packet.append(static_cast<char>(0xAA)); // sof
    packet.append(static_cast<char>(1));    // version
    AppendU16(packet, frame_id);
    AppendU16(packet, seq);
    packet.append(static_cast<char>(frame_flag));
    packet.append(static_cast<char>(sub_frame_idx));
    AppendU16(packet, static_cast<uint16_t>(total_size)); // data_len(新协议固定包长)
    AppendU16(packet, static_cast<uint16_t>(payload_len)); // 有效负载长度
    AppendU16(packet, 0); // reserved
    if (meta != nullptr) {
        packet.append(reinterpret_cast<const char*>(meta), sizeof(*meta));
    }
    packet.append(reinterpret_cast<const char*>(payload), static_cast<int>(payload_len));
    // 新协议末包不足固定包长时尾部补 0
    while (static_cast<size_t>(packet.size()) < total_size) {
        packet.append('\0');
    }
    return packet;
}

void Fl6532RangeLz4Test::Send(const QByteArray& packet)
{
    auto* context = new you_see::IOContext();
    std::memcpy(context->buffer.get(), packet.constData(), static_cast<size_t>(packet.size()));
    context->buffer_size = static_cast<size_t>(packet.size());
    Fl6532RangeProcessorGetInstance().EnqueueContext(context);
}

bool Fl6532RangeLz4Test::WaitFrame(int timeout_ms)
{
    const int step_ms = 10;
    for (int waited = 0; waited < timeout_ms; waited += step_ms) {
        if (frame_count_.load(std::memory_order_relaxed) > 0) {
            return true;
        }
        QTest::qWait(step_ms);
    }
    return false;
}

Fl6532RangeFramePtr Fl6532RangeLz4Test::TakeFrame()
{
    frame_count_.store(0, std::memory_order_relaxed);
    std::lock_guard<std::mutex> lock(frame_mutex_);
    return std::move(last_frame_);
}

void Fl6532RangeLz4Test::initTestCase()
{
    Fl6532RangeProcessorGetInstance().Stop(); // 清理上一次状态(测试可重复运行)
    Fl6532RangeProcessorGetInstance().SetFrameCallback(
        [this](Fl6532RangeFramePtr frame) {
            {
                std::lock_guard<std::mutex> lock(frame_mutex_);
                last_frame_ = std::move(frame);
            }
            frame_count_.fetch_add(1, std::memory_order_relaxed);
        });
    Fl6532RangeProcessorGetInstance().Start();
}

void Fl6532RangeLz4Test::cleanupTestCase()
{
    // 先 Stop(join worker，确保不再有回调在途)再解绑，避免与回调竞争
    Fl6532RangeProcessorGetInstance().Stop();
    Fl6532RangeProcessorGetInstance().SetFrameCallback(nullptr);
}

void Fl6532RangeLz4Test::testLz4CompleteFrame()
{
    const uint8_t payload0[] = { 1, 2, 3 };
    const uint8_t payload1[] = { 4, 5 };
    const uint8_t payload2[] = { 6 };
    const Fl6532RangeLz4Meta meta = BuildRoi0Meta();

    Send(BuildLz4Packet(100, 0, 0, 0, payload0, sizeof(payload0), &meta));
    Send(BuildLz4Packet(100, 1, 1, 0, payload1, sizeof(payload1), nullptr));
    Send(BuildLz4Packet(100, 2, 2, 0, payload2, sizeof(payload2), nullptr));

    QVERIFY(WaitFrame());
    auto frame = TakeFrame();
    QVERIFY(frame != nullptr);
    QVERIFY(frame->lz4_compressed);
    QCOMPARE(frame->frame_cnt, static_cast<uint16_t>(100));
    QCOMPARE(frame->packet_count, 3u);
    QCOMPARE(frame->sub_frame_idx, static_cast<uint8_t>(0));
    QCOMPARE(frame->pixel_len, static_cast<uint16_t>(kPixelLen));
    QCOMPARE(frame->pixel_count, static_cast<uint64_t>(kRoi0Groups * kPushPixels));

    // 压缩流应为按 seq 升序拼接的负载
    const std::vector<uint8_t> expected{ 1, 2, 3, 4, 5, 6 };
    QCOMPARE(frame->compressed, expected);

    // 元信息应原样解析
    QCOMPARE(frame->lz4_meta.pixel_len, static_cast<uint16_t>(kPixelLen));
    QCOMPARE(frame->lz4_meta.roi_cnt, static_cast<uint8_t>(1));
    QCOMPARE(frame->lz4_meta.roi[0].width, static_cast<uint16_t>(192));
    QCOMPARE(frame->lz4_meta.roi[0].height, static_cast<uint16_t>(128));
    QCOMPARE(frame->lz4_meta.roi[0].push_pixels, static_cast<uint16_t>(kPushPixels));
    QCOMPARE(frame->lz4_meta.roi[0].group_cnt, static_cast<uint8_t>(kRoi0Groups));
}

void Fl6532RangeLz4Test::testLz4SequenceGapDiscardsFrame()
{
    const uint8_t payload[] = { 1 };
    const Fl6532RangeLz4Meta meta = BuildRoi0Meta();

    // 帧 200：START(seq 0) 后 seq 1 丢失，END(seq 2) 到达 → 整帧丢弃
    Send(BuildLz4Packet(200, 0, 0, 0, payload, sizeof(payload), &meta));
    Send(BuildLz4Packet(200, 2, 2, 0, payload, sizeof(payload), nullptr));
    QTest::qWait(300);
    QCOMPARE(frame_count_.load(std::memory_order_relaxed), 0);

    // 下一帧 201 正常，应输出
    Send(BuildLz4Packet(201, 0, 0, 0, payload, sizeof(payload), &meta));
    Send(BuildLz4Packet(201, 1, 2, 0, payload, sizeof(payload), nullptr));
    QVERIFY(WaitFrame());
    auto frame = TakeFrame();
    QVERIFY(frame != nullptr);
    QCOMPARE(frame->frame_cnt, static_cast<uint16_t>(201));
}

void Fl6532RangeLz4Test::testLz4OrphanPacketIgnored()
{
    const uint8_t payload[] = { 7 };
    // 无 START 直接来 NORMAL/END：应被忽略，不输出
    Send(BuildLz4Packet(300, 1, 1, 0, payload, sizeof(payload), nullptr));
    Send(BuildLz4Packet(300, 2, 2, 0, payload, sizeof(payload), nullptr));
    QTest::qWait(300);
    QCOMPARE(frame_count_.load(std::memory_order_relaxed), 0);
}

void Fl6532RangeLz4Test::testLz4NewStartDiscardsOldFrame()
{
    const uint8_t payload[] = { 9 };
    const Fl6532RangeLz4Meta meta = BuildRoi0Meta();

    // 旧帧 400 未收完，新 START(400) 覆盖后 END 应输出新帧
    Send(BuildLz4Packet(400, 0, 0, 0, payload, sizeof(payload), &meta));
    Send(BuildLz4Packet(400, 1, 1, 0, payload, sizeof(payload), nullptr));
    Send(BuildLz4Packet(400, 0, 0, 0, payload, sizeof(payload), &meta));
    Send(BuildLz4Packet(400, 1, 2, 0, payload, sizeof(payload), nullptr));

    QVERIFY(WaitFrame());
    auto frame = TakeFrame();
    QVERIFY(frame != nullptr);
    QCOMPARE(frame->frame_cnt, static_cast<uint16_t>(400));
    QCOMPARE(frame->packet_count, 2u);
    QCOMPARE(frame->compressed.size(), static_cast<size_t>(2));
}

void Fl6532RangeLz4Test::testLz4DatagramDetection()
{
    const uint8_t payload[] = { 1, 2 };
    const Fl6532RangeLz4Meta meta = BuildRoi0Meta();
    const QByteArray start_pkt = BuildLz4Packet(500, 0, 0, 0, payload, sizeof(payload), &meta);

    // 合法 START 包(86 B 头)应被分流判别命中
    auto* valid = new you_see::IOContext();
    std::memcpy(valid->buffer.get(), start_pkt.constData(), static_cast<size_t>(start_pkt.size()));
    valid->buffer_size = static_cast<size_t>(start_pkt.size());
    QVERIFY(Fl6532RangeProcessor::IsFl6532RangeContext(valid));
    QVERIFY(Fl6532RangeProcessor::IsFl6532RangeLz4Datagram(
        reinterpret_cast<const uint8_t*>(valid->buffer.get()), valid->buffer_size));
    delete valid;

    // 版本非法
    QByteArray bad_version = start_pkt;
    bad_version[1] = 2;
    QVERIFY(!Fl6532RangeProcessor::IsFl6532RangeLz4Datagram(
        reinterpret_cast<const uint8_t*>(bad_version.constData()),
        static_cast<size_t>(bad_version.size())));

    // payload_len 与长度不自洽
    QByteArray bad_len = start_pkt;
    bad_len[11] = static_cast<char>(0x7F);
    QVERIFY(!Fl6532RangeProcessor::IsFl6532RangeLz4Datagram(
        reinterpret_cast<const uint8_t*>(bad_len.constData()),
        static_cast<size_t>(bad_len.size())));

    // 过短
    const uint8_t short_buf[4] = { 0xAA, 0x01, 0x00, 0x00 };
    QVERIFY(!Fl6532RangeProcessor::IsFl6532RangeLz4Datagram(short_buf, sizeof(short_buf)));
}

void Fl6532RangeLz4Test::testLz4PaddedTailPacket()
{
    // 新协议：所有包固定 1400 B(避免 IP 分片)，末包不足时尾部补 0，
    // payload_len 为有效负载长度。接收端必须按 payload_len 截取，不得包含填充 0。
    const Fl6532RangeLz4Meta meta = BuildRoi0Meta();
    const uint8_t start_payload[] = { 1, 2, 3 };
    const uint8_t end_payload[] = { 4, 5 };

    Send(BuildLz4Packet(900, 0, 0, 0, start_payload, sizeof(start_payload), &meta, 1400));
    Send(BuildLz4Packet(900, 1, 2, 0, end_payload, sizeof(end_payload), nullptr, 1400));

    QVERIFY(WaitFrame());
    auto frame = TakeFrame();
    QVERIFY(frame != nullptr);
    QVERIFY(frame->lz4_compressed);
    QCOMPARE(frame->frame_cnt, static_cast<uint16_t>(900));
    QCOMPARE(frame->packet_count, 2u);
    // 压缩流只包含有效负载，尾部补 0 必须被剔除
    const std::vector<uint8_t> expected{ 1, 2, 3, 4, 5 };
    QCOMPARE(frame->compressed, expected);

    // 补 0 包应能通过分流判别
    const QByteArray padded = BuildLz4Packet(901, 0, 0, 0, start_payload,
        sizeof(start_payload), &meta, 1400);
    QVERIFY(Fl6532RangeProcessor::IsFl6532RangeLz4Datagram(
        reinterpret_cast<const uint8_t*>(padded.constData()),
        static_cast<size_t>(padded.size())));
}

void Fl6532RangeLz4Test::testFullRoi0FrameToPointCloud()
{
    Fl6532RangeProcessorGetInstance().Stop();
    Fl6532RangeProcessorGetInstance().SetFrameCallback(nullptr);

    ys_scene::Fl6532RangeParser parser;
    QSignalSpy spy(&parser, &ys_scene::Fl6532RangeParser::RangePointCloudReadySig);
    QVERIFY(spy.isValid());

    Fl6532RangeProcessorGetInstance().SetFrameCallback(
        [&parser](Fl6532RangeFramePtr frame) { parser.OnRangeFrame(std::move(frame)); });
    Fl6532RangeProcessorGetInstance().Start();

    // 1. 构造完整 ROI0 记录流并按 push 独立转置
    constexpr uint16_t kFrameId = 1234;
    const std::vector<uint8_t> records = BuildRoi0Records(kFrameId);
    std::vector<uint8_t> transposed(records.size());
    for (size_t group = 0; group < kRoi0Groups; ++group) {
        TransposePush(records.data() + group * kPushPixels * kPixelLen, kPushPixels,
            transposed.data() + group * kPushPixels * kPixelLen);
    }

    // 2. LZ4F 压缩整段转置流并切分为 UDP 包(START 包容量扣 72 B 元信息)
    const std::vector<uint8_t> compressed = CompressLz4(transposed.data(), transposed.size());
    QVERIFY(!compressed.empty());
    const Fl6532RangeLz4Meta meta = BuildRoi0Meta();

    size_t offset = 0;
    uint16_t seq = 0;
    while (offset < compressed.size()) {
        const bool is_first = (seq == 0);
        const size_t capacity = is_first ? 1436 : 1508;
        const size_t take = (std::min)(capacity, compressed.size() - offset);
        const bool is_last = (offset + take >= compressed.size());
        const uint8_t frame_flag = is_first ? 0 : (is_last ? 2 : 1);
        Send(BuildLz4Packet(kFrameId, seq, frame_flag, 0, compressed.data() + offset, take,
            is_first ? &meta : nullptr));
        offset += take;
        ++seq;
    }

    // 3. 等待算法输出点云帧(解压 + 逆转置 + RangePipeline 组帧)
    QTRY_VERIFY_WITH_TIMEOUT(spy.count() > 0, 15000);
    const QList<QVariant> args = spy.takeFirst();
    const auto cloud = args.at(0).value<ys_scene::CloudFramePtr>();
    QVERIFY(cloud != nullptr);
    QCOMPARE(cloud->data.size(), static_cast<size_t>(128 * 192));

    // 先 Stop(join worker，确保不再有回调引用 parser)再解绑并析构 parser
    Fl6532RangeProcessorGetInstance().Stop();
    Fl6532RangeProcessorGetInstance().SetFrameCallback(nullptr);
}

} // namespace ys_test

QTEST_GUILESS_MAIN(ys_test::Fl6532RangeLz4Test)
