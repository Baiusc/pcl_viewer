#ifndef YS_SCENE_FL6532_RANGE_PARSER_H_
#define YS_SCENE_FL6532_RANGE_PARSER_H_

#include <QObject>
#include <atomic>
#include <condition_variable>
#include <deque>
#include <memory>
#include <mutex>
#include <thread>

#include "you_see/business/fl6532_range_processor.h"
#include "range6532_pipeline.h"
#include "ys_scene_def.h"

// mf150_fl6532 RANGE 模式上位机消费层
//
// 接收/组帧在 SDK you_see::Fl6532RangeProcessor 内完成(接收线程只入队、组帧
// worker 主动轮询出队)。本类采用相同的两级流水模型，避免重处理阻塞组帧 worker：
//   SDK 组帧 worker --OnRangeFrame(仅入队)--> 待处理队列
//     --> 专用处理线程: LZ4F 解压 -> 按 push 逆转置 -> 喂算法 RangePipeline
//     --> 算法输出的有组织点云(128×192)经 Qt 信号跨线程投递给点云渲染层。
//
// 旧版未压缩 0x2E 帧(pixels 已是连续像素记录流)跳过解压/逆转置，直接喂算法。

namespace ys_scene {

using Fl6532RangeFrame = you_see::Fl6532RangeFrame;
using Fl6532RangeFramePtr = you_see::Fl6532RangeFramePtr;

class Fl6532RangeParser : public QObject {
    Q_OBJECT
public:
    explicit Fl6532RangeParser(QObject* parent = nullptr);
    ~Fl6532RangeParser() override;

public slots:
    // SDK Fl6532RangeProcessor 完整帧回调入口(组帧 worker 线程调用)：
    // 只把帧放入待处理队列后立即返回，解压/算法在专用处理线程执行
    void OnRangeFrame(Fl6532RangeFramePtr frame);

signals:
    // 完整帧到达(兼容旧接口，跨线程排队投递)
    void FrameReadySig(Fl6532RangeFramePtr frame);
    // 算法输出的有组织点云帧(ROI0: 128×192)
    void RangePointCloudReadySig(CloudFramePtr frame);

private:
    // 专用处理线程主循环
    void ProcessLoop();
    // 解压 + 逆转置 + 喂算法
    void ProcessFrame(const Fl6532RangeFramePtr& frame);
    // LZ4F 解压；expected_size 用于校验输出长度(转置流字节数)
    static bool DecompressLz4(const std::vector<uint8_t>& compressed, size_t expected_size,
                              std::vector<uint8_t>& out);
    // 按 push 逆转置：转置布局 trans[j * count + i] -> orig[i * pixel_len + j]
    static bool InverseTranspose(const std::vector<uint8_t>& trans,
                                 const you_see::Fl6532RangeLz4Meta& meta,
                                 std::vector<uint8_t>& out);
    // 将连续 278 B 像素记录流喂给算法
    void FeedPipeline(const uint8_t* records, size_t size, uint8_t roi_index);

private:
    range6532::RangePipeline pipeline_;

    // 待处理帧队列(SDK 组帧 worker 生产，专用处理线程消费)
    std::thread process_thread_;
    std::mutex pending_mutex_;
    std::condition_variable pending_cv_;
    std::deque<Fl6532RangeFramePtr> pending_frames_;
    std::atomic<bool> stop_{ false };
    uint64_t dropped_frames_{ 0 };
    // 积压上限：处理长期滞后时丢最旧帧，避免内存无界增长
    static constexpr size_t kMaxPendingFrames = 8;
};

} // namespace ys_scene

Q_DECLARE_METATYPE(ys_scene::Fl6532RangeFramePtr)
Q_DECLARE_METATYPE(ys_scene::CloudFramePtr)

#endif // YS_SCENE_FL6532_RANGE_PARSER_H_
