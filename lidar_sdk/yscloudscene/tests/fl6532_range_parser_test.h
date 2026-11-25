#ifndef YS_TEST_FL6532_RANGE_PARSER_TEST_H_
#define YS_TEST_FL6532_RANGE_PARSER_TEST_H_

#include <QObject>
#include <atomic>
#include <deque>
#include <memory>
#include <mutex>

#include "you_see/business/fl6532_range_processor.h"

namespace ys_test {

// FL6532 RANGE(0x2E) SDK 组帧处理器测试：构造合成 0x2E 包写入 IOContext 缓冲后
// 投递给组帧队列(与真实 IOCP 路径一致：只投 IOContext* 指针)，验证 START/END
// 组帧、序号检测、半包缓冲与完整帧回调。
class Fl6532RangeParserTest : public QObject {
    Q_OBJECT
private slots:
    void initTestCase();
    void cleanupTestCase();
    // 正常帧：START + N×NORMAL + END 应输出一帧且像素字节顺序正确
    void testCompleteFrame();
    // 帧中序号空洞(丢包)：整帧丢弃，下一帧正常
    void testSequenceGapDiscardsFrame();
    // 单个 IOContext 携带多个连续包
    void testMultiplePacketsInOneCall();
    // 无 START 的孤立包应被忽略
    void testOrphanPacketIgnored();
    // 三种包长混合组帧：1430(dot=5)/874(dot=3)/596(dot=2)
    void testMixedPacketSizes();
    // 多帧并发交错投递：不同帧由不同分片并行组帧，互不干扰
    void testConcurrentFrames();
    // 分流判别
    void testIsFl6532RangeContext();

private:
    // 构造一个合成 0x2E 包：帧号/序号/标志/像素数可指定，载荷为可辨识字节
    QByteArray BuildPacket(uint16_t frame_cnt, uint16_t data_sequence, uint8_t frame_flag,
        uint16_t dot_num, uint8_t roi = 0, uint16_t width = 192, uint16_t height = 128);
    // 把字节包灌入新分配的 IOContext 并投递(投递后所有权归 SDK 处理器)
    void Send(const QByteArray& packet);
    // 等待回调产出帧
    bool WaitFrame(int timeout_ms = 3000);
    // 取回调收到的最近一帧(线程安全)
    you_see::Fl6532RangeFramePtr TakeFrame();

    std::atomic<int> frame_count_{ 0 };
    std::mutex frame_mutex_;
    // 帧队列：多帧并发完成时不能只保留最后一帧(否则先到帧被覆盖)
    std::deque<you_see::Fl6532RangeFramePtr> frames_;
};

} // namespace ys_test

#endif // YS_TEST_FL6532_RANGE_PARSER_TEST_H_
