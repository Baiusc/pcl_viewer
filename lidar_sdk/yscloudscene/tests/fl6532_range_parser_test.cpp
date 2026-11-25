#include "fl6532_range_parser_test.h"

#include <QTest>
#include <QSet>

#include <cstring>

#include "you_see/business/fl6532_range_processor.h"

#ifdef USE_IOCP
#include "ys_net/session/io_context.h"
#endif

namespace ys_test {

using you_see::Fl6532RangeFrame;
using you_see::Fl6532RangeFramePtr;
using you_see::Fl6532RangeProcessor;
using you_see::Fl6532RangeProcessorGetInstance;

void Fl6532RangeParserTest::initTestCase()
{
    Fl6532RangeProcessorGetInstance().Stop(); // 清理上一次状态(测试可重复运行)
    Fl6532RangeProcessorGetInstance().SetFrameCallback(
        [this](Fl6532RangeFramePtr frame) {
            {
                std::lock_guard<std::mutex> lock(frame_mutex_);
                frames_.push_back(std::move(frame));
            }
            frame_count_.fetch_add(1, std::memory_order_relaxed);
        });
    Fl6532RangeProcessorGetInstance().Start();
}

void Fl6532RangeParserTest::cleanupTestCase()
{
    // 先 Stop(join worker，确保不再有回调在途)再解绑，避免与回调竞争
    Fl6532RangeProcessorGetInstance().Stop();
    Fl6532RangeProcessorGetInstance().SetFrameCallback(nullptr);
}

QByteArray Fl6532RangeParserTest::BuildPacket(uint16_t frame_cnt, uint16_t data_sequence,
    uint8_t frame_flag, uint16_t dot_num, uint8_t roi, uint16_t width, uint16_t height)
{
    const size_t pixel_len = you_see::Fl6532RangeRawParser::pixel_len;
    QByteArray packet;
    // 6B fs_protocol_head_t
    packet.append(static_cast<char>(0x5A)); // head 0xA55A (LE)
    packet.append(static_cast<char>(0xA5));
    packet.append(static_cast<char>(2)); // protocol_ver_major
    packet.append(static_cast<char>(0x2E)); // data_type = CICL
    packet.append(static_cast<char>(0)); // lidar_module
    packet.append(static_cast<char>(0)); // reserved
    // 34B fs_lidar_pcl_echo_data_t
    packet.append(static_cast<char>(2)); // protocol_ver_minor
    packet.append(static_cast<char>(data_sequence & 0xFF)); // data_sequence LE
    packet.append(static_cast<char>((data_sequence >> 8) & 0xFF));
    packet.append(static_cast<char>(frame_cnt & 0xFF)); // frame_cnt LE
    packet.append(static_cast<char>((frame_cnt >> 8) & 0xFF));
    packet.append(static_cast<char>(dot_num & 0xFF)); // dot_num LE
    packet.append(static_cast<char>((dot_num >> 8) & 0xFF));
    packet.append(static_cast<char>(0x05)); // type FS_CICL_DATA
    packet.append(static_cast<char>(0)); // peak_type_2_1
    packet.append(static_cast<char>(frame_flag)); // frame_flag
    packet.append(static_cast<char>(width & 0xFF)); // width LE
    packet.append(static_cast<char>((width >> 8) & 0xFF));
    packet.append(static_cast<char>(height & 0xFF)); // height LE
    packet.append(static_cast<char>((height >> 8) & 0xFF));
    packet.append(static_cast<char>(0)); // max_bin_cnt LE
    packet.append(static_cast<char>(0));
    packet.append(static_cast<char>(0)); // ab_frame_idx
    packet.append(static_cast<char>(0)); // max_sub_frame_cnt
    packet.append(static_cast<char>(roi)); // roi_index
    packet.append(static_cast<char>(pixel_len & 0xFF)); // pixel_len LE
    packet.append(static_cast<char>((pixel_len >> 8) & 0xFF));
    packet.append(static_cast<char>(0)); // data_offset LE
    packet.append(static_cast<char>(0));
    packet.append(static_cast<char>(0)); // data_len LE
    packet.append(static_cast<char>(0));
    packet.append(static_cast<char>(0)); // peak_type_4_3
    for (int i = 0; i < 8; ++i) {
        packet.append(static_cast<char>(0)); // timestamp_base
    }
    // 像素载荷：可辨识字节 pattern
    for (uint16_t i = 0; i < dot_num; ++i) {
        const uint16_t pixel_id = static_cast<uint16_t>(data_sequence) * 3 + i;
        for (size_t b = 0; b < pixel_len; ++b) {
            packet.append(static_cast<char>((pixel_id + b) & 0xFF));
        }
    }
    return packet;
}

void Fl6532RangeParserTest::Send(const QByteArray& packet)
{
    // 与真实 IOCP 路径一致：分配 IOContext 填充接收缓冲后只投递指针
    auto* context = new you_see::IOContext();
    std::memcpy(context->buffer.get(), packet.constData(), static_cast<size_t>(packet.size()));
    context->buffer_size = static_cast<size_t>(packet.size());
    Fl6532RangeProcessorGetInstance().EnqueueContext(context);
}

bool Fl6532RangeParserTest::WaitFrame(int timeout_ms)
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

Fl6532RangeFramePtr Fl6532RangeParserTest::TakeFrame()
{
    std::lock_guard<std::mutex> lock(frame_mutex_);
    if (frames_.empty()) {
        return nullptr;
    }
    Fl6532RangeFramePtr frame = std::move(frames_.front());
    frames_.pop_front();
    frame_count_.fetch_sub(1, std::memory_order_relaxed);
    return frame;
}

void Fl6532RangeParserTest::testCompleteFrame()
{
    // START(5 像素) + 2×NORMAL(5 像素) + END(2 像素)：共 4 包 / 17 像素
    Send(BuildPacket(10, 0, 0, 5));
    Send(BuildPacket(10, 1, 1, 5));
    Send(BuildPacket(10, 2, 1, 5));
    Send(BuildPacket(10, 3, 2, 2));

    QVERIFY(WaitFrame());
    auto frame = TakeFrame();
    QVERIFY(frame != nullptr);
    QCOMPARE(frame->frame_cnt, static_cast<uint16_t>(10));
    QCOMPARE(frame->pixel_count, static_cast<uint64_t>(17));
    QCOMPARE(frame->packet_count, 4u);
    QCOMPARE(frame->pixels.size(), static_cast<size_t>(17) * 278);

    // 首包像素应为 START 包载荷(像素首字节 0)
    QCOMPARE(static_cast<uint8_t>(frame->pixels[0]), 0u);
    // 第 2 个像素首字节 = START 包第 2 像素 = 1
    QCOMPARE(static_cast<uint8_t>(frame->pixels[278]), 1u);
    // END 包(seq 3, dot 2)第 1 像素 id = 3×3+0 = 9
    const size_t end_first_pixel = frame->pixels.size() - 2 * 278;
    QCOMPARE(static_cast<uint8_t>(frame->pixels[end_first_pixel]), static_cast<uint8_t>(9u));
}

void Fl6532RangeParserTest::testSequenceGapDiscardsFrame()
{
    // 帧 20：START(seq 0) 后 seq 1 丢失，seq 2 到达 → 整帧应被丢弃
    Send(BuildPacket(20, 0, 0, 5));
    Send(BuildPacket(20, 2, 1, 5));
    Send(BuildPacket(20, 3, 2, 5));
    QTest::qWait(300);
    QCOMPARE(frame_count_.load(std::memory_order_relaxed), 0); // 丢包帧不应输出

    // 下一帧 21 正常，应输出
    Send(BuildPacket(21, 0, 0, 5));
    Send(BuildPacket(21, 1, 2, 5));
    QVERIFY(WaitFrame());
    auto frame = TakeFrame();
    QVERIFY(frame != nullptr);
    QCOMPARE(frame->frame_cnt, static_cast<uint16_t>(21));
}

void Fl6532RangeParserTest::testMultiplePacketsInOneCall()
{
    // 单个 IOContext 携带 START + END 两包
    Send(BuildPacket(40, 0, 0, 5) + BuildPacket(40, 1, 2, 3));
    QVERIFY(WaitFrame());
    auto frame = TakeFrame();
    QVERIFY(frame != nullptr);
    QCOMPARE(frame->frame_cnt, static_cast<uint16_t>(40));
    QCOMPARE(frame->pixel_count, static_cast<uint64_t>(8));
}

void Fl6532RangeParserTest::testOrphanPacketIgnored()
{
    // 无 START 直接来 NORMAL/END：应被忽略，不输出
    Send(BuildPacket(50, 0, 1, 5));
    Send(BuildPacket(50, 5, 2, 5));
    QTest::qWait(300);
    QCOMPARE(frame_count_.load(std::memory_order_relaxed), 0);
}

void Fl6532RangeParserTest::testMixedPacketSizes()
{
    // 实测三种包长混合组帧：1430(dot=5 满包)、874(dot=3)、596(dot=2)
    // START(1430) + NORMAL(874) + NORMAL(596) + NORMAL(1430) + END(596)
    Send(BuildPacket(70, 0, 0, 5));
    Send(BuildPacket(70, 1, 1, 3));
    Send(BuildPacket(70, 2, 1, 2));
    Send(BuildPacket(70, 3, 1, 5));
    Send(BuildPacket(70, 4, 2, 2));

    QVERIFY(WaitFrame());
    auto frame = TakeFrame();
    QVERIFY(frame != nullptr);
    QCOMPARE(frame->frame_cnt, static_cast<uint16_t>(70));
    QCOMPARE(frame->packet_count, 5u);
    QCOMPARE(frame->pixel_count, static_cast<uint64_t>(5 + 3 + 2 + 5 + 2));

    // 像素字节流应与各包 payload 拼接顺序一致
    QCOMPARE(static_cast<uint8_t>(frame->pixels[0]), 0u); // START 首像素
    QCOMPARE(static_cast<uint8_t>(frame->pixels[5 * 278]), static_cast<uint8_t>(3u)); // dot=3 包首像素 id=3
}

void Fl6532RangeParserTest::testConcurrentFrames()
{
    // 帧 80 与帧 81 的包交错投递(两帧大概率落在不同分片)，都应收齐
    Send(BuildPacket(80, 0, 0, 5));
    Send(BuildPacket(81, 0, 0, 5));
    Send(BuildPacket(80, 1, 1, 5));
    Send(BuildPacket(81, 1, 1, 5));
    Send(BuildPacket(80, 2, 2, 3));
    Send(BuildPacket(81, 2, 2, 3));

    // 两帧都应完成
    QVERIFY(WaitFrame());
    auto f1 = TakeFrame();
    QVERIFY(f1 != nullptr);
    QVERIFY(WaitFrame());
    auto f2 = TakeFrame();
    QVERIFY(f2 != nullptr);

    QSet<int> frames{ static_cast<int>(f1->frame_cnt), static_cast<int>(f2->frame_cnt) };
    QVERIFY(frames.contains(80));
    QVERIFY(frames.contains(81));
    // 每帧像素数 = 5+5+3 = 13
    QVERIFY(f1->pixel_count == 13 && f2->pixel_count == 13);
}

void Fl6532RangeParserTest::testIsFl6532RangeContext()
{
    const QByteArray pkt = BuildPacket(60, 0, 0, 5);

    auto* valid = new you_see::IOContext();
    std::memcpy(valid->buffer.get(), pkt.constData(), static_cast<size_t>(pkt.size()));
    valid->buffer_size = static_cast<size_t>(pkt.size());
    QVERIFY(Fl6532RangeProcessor::IsFl6532RangeContext(valid));
    delete valid;

    // 非法头
    auto* bad_head = new you_see::IOContext();
    std::memcpy(bad_head->buffer.get(), pkt.constData(), static_cast<size_t>(pkt.size()));
    bad_head->buffer_size = static_cast<size_t>(pkt.size());
    bad_head->buffer.get()[0] = 0x00;
    QVERIFY(!Fl6532RangeProcessor::IsFl6532RangeContext(bad_head));
    delete bad_head;

    // data_type 非 0x2E
    auto* bad_type = new you_see::IOContext();
    std::memcpy(bad_type->buffer.get(), pkt.constData(), static_cast<size_t>(pkt.size()));
    bad_type->buffer_size = static_cast<size_t>(pkt.size());
    bad_type->buffer.get()[3] = 0x21;
    QVERIFY(!Fl6532RangeProcessor::IsFl6532RangeContext(bad_type));
    delete bad_type;

    // 过短
    auto* short_ctx = new you_see::IOContext();
    std::memcpy(short_ctx->buffer.get(), pkt.constData(), 40);
    short_ctx->buffer_size = 40;
    QVERIFY(!Fl6532RangeProcessor::IsFl6532RangeContext(short_ctx));
    delete short_ctx;
}

} // namespace ys_test

QTEST_GUILESS_MAIN(ys_test::Fl6532RangeParserTest)
