#ifndef YS_TEST_FL6532_RANGE_LZ4_TEST_H_
#define YS_TEST_FL6532_RANGE_LZ4_TEST_H_

#include <QObject>
#include <atomic>
#include <memory>
#include <mutex>

#include "you_see/business/fl6532_range_processor.h"

namespace ys_test {

// FL6532 Range LZ4 压缩协议测试(板端 RANGE_TX_LZ4=1 链路)：
//  1. SDK Fl6532RangeProcessor 的 LZ4 组帧状态机：
//     START(带元信息)/NORMAL/END、seq 空洞丢弃、孤儿包、新 START 覆盖旧帧；
//  2. 完整 ROI0 帧：LZ4F 解压 -> 按 push 逆转置 -> range6532::RangePipeline
//     输出 128×192 有组织点云。
class Fl6532RangeLz4Test : public QObject {
    Q_OBJECT
private slots:
    void initTestCase();
    void cleanupTestCase();
    void testLz4CompleteFrame();
    void testLz4SequenceGapDiscardsFrame();
    void testLz4OrphanPacketIgnored();
    void testLz4NewStartDiscardsOldFrame();
    void testLz4DatagramDetection();
    void testLz4PaddedTailPacket();
    void testFullRoi0FrameToPointCloud();

private:
    // 构造 LZ4 帧元信息(ROI0: 192×128，每 push 768 像素)
    static you_see::Fl6532RangeLz4Meta BuildRoi0Meta();
    // 构造一个 LZ4 UDP 包(frame_flag=0 时携带 72 B 元信息)；
    // padded_size > 0 时按新协议补 0 到固定包长(末包 payload_len < 容量)
    static QByteArray BuildLz4Packet(uint16_t frame_id, uint16_t seq, uint8_t frame_flag,
        uint8_t sub_frame_idx, const uint8_t* payload, size_t payload_len,
        const you_see::Fl6532RangeLz4Meta* meta, size_t padded_size = 0);
    // 把字节包灌入新分配的 IOContext 并投递(投递后所有权归 SDK 处理器)
    static void Send(const QByteArray& packet);
    // 等待 SDK 帧回调产出帧
    bool WaitFrame(int timeout_ms = 3000);
    // 取回调收到的最近一帧(线程安全)
    you_see::Fl6532RangeFramePtr TakeFrame();

    std::atomic<int> frame_count_{ 0 };
    std::mutex frame_mutex_;
    you_see::Fl6532RangeFramePtr last_frame_;
};

} // namespace ys_test

#endif // YS_TEST_FL6532_RANGE_LZ4_TEST_H_
