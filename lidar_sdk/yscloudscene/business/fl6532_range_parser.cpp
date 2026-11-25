#include "fl6532_range_parser.h"

#include "log/logging.h"

#include <lz4frame.h>

#include <algorithm>
#include <chrono>
#include <string>

namespace ys_scene {
namespace {

// 算法 intensity = selected peak_aux / 255，转回 8 bit 反射率
uint8_t ToReflectivity(float intensity)
{
    const float scaled = intensity * 255.0f;
    if (scaled <= 0.0f) {
        return 0;
    }
    if (scaled >= 255.0f) {
        return 255;
    }
    return static_cast<uint8_t>(scaled + 0.5f);
}

int64_t DurationMs(const std::chrono::steady_clock::time_point& begin,
    const std::chrono::steady_clock::time_point& end)
{
    return std::chrono::duration_cast<std::chrono::milliseconds>(end - begin).count();
}

} // namespace

Fl6532RangeParser::Fl6532RangeParser(QObject* parent)
    : QObject(parent)
    , pipeline_(range6532::EchoPolicy::PreferEcho0, false)
{
    qRegisterMetaType<Fl6532RangeFramePtr>("ys_scene::Fl6532RangeFramePtr");
    qRegisterMetaType<CloudFramePtr>("ys_scene::CloudFramePtr");

    // 算法回调在处理线程内同步触发；有组织点云转成上位机 CloudFrame 后
    // 经 Qt 队列投递到渲染层。
    pipeline_.setFrameCallback([this](range6532::PointCloudFrame&& frame) {
        auto cartesian_frame = std::make_shared<you_see::CloudFrame<you_see::CartesianPoint>>(
            frame.frameIndex, frame.validPointCount);
        cartesian_frame->data.reserve(frame.validPointCount);
        for (const auto& point : frame.points) {
            if (!point.valid) {
                continue;
            }
            you_see::CartesianPoint cp{};
            cp.x.f = point.x;
            cp.y.f = point.y;
            cp.z.f = point.z;
            cp.reflectivity = ToReflectivity(point.intensity);
            cartesian_frame->data.push_back(cp);
        }
        if (!cartesian_frame->data.empty()) {
            emit RangePointCloudReadySig(cartesian_frame);
        }
    });

    process_thread_ = std::thread(&Fl6532RangeParser::ProcessLoop, this);
}

Fl6532RangeParser::~Fl6532RangeParser()
{
    stop_.store(true);
    pending_cv_.notify_all();
    if (process_thread_.joinable()) {
        process_thread_.join();
    }
}

void Fl6532RangeParser::OnRangeFrame(Fl6532RangeFramePtr frame)
{
    // SDK 组帧 worker 线程调用：只入队立即返回，解压/算法在专用处理线程执行，
    // 避免重处理阻塞组帧 worker 的队列消费
    if (!frame || stop_.load()) {
        return;
    }
    emit FrameReadySig(frame); // 兼容旧接口转发(无消费者时开销可忽略)

    {
        std::lock_guard<std::mutex> lock(pending_mutex_);
        if (pending_frames_.size() >= kMaxPendingFrames) {
            pending_frames_.pop_front(); // 处理长期滞后时丢最旧帧
            ++dropped_frames_;
            if (dropped_frames_ % 8 == 1) {
                LOG_WARN("Fl6532Range processing lagging, dropped {} frames so far", dropped_frames_);
            }
        }
        pending_frames_.push_back(std::move(frame));
    }
    pending_cv_.notify_one();
}

void Fl6532RangeParser::ProcessLoop()
{
    while (true) {
        Fl6532RangeFramePtr frame;
        {
            std::unique_lock<std::mutex> lock(pending_mutex_);
            pending_cv_.wait(lock, [this] { return stop_.load() || !pending_frames_.empty(); });
            if (stop_.load() && pending_frames_.empty()) {
                break;
            }
            frame = std::move(pending_frames_.front());
            pending_frames_.pop_front();
        }
        ProcessFrame(frame);
    }
}

void Fl6532RangeParser::ProcessFrame(const Fl6532RangeFramePtr& frame)
{
    if (!frame) {
        return;
    }

    // 旧版未压缩 0x2E 帧：pixels 已是连续 278 B 像素记录流，直接喂算法
    if (!frame->lz4_compressed) {
        FeedPipeline(frame->pixels.data(), frame->pixels.size(), frame->roi_index);
        return;
    }

    // LZ4 压缩帧：解压 -> 长度校验 -> 按 push 逆转置 -> 按 ROI 喂算法
    const auto& meta = frame->lz4_meta;
    if (meta.pixel_len == 0 || meta.total_pixels == 0 || meta.roi_cnt == 0) {
        LOG_WARN("Fl6532Range LZ4 frame #{} invalid meta (pixel_len={}, total_pixels={}, roi_cnt={})",
            frame->frame_cnt, meta.pixel_len, meta.total_pixels, meta.roi_cnt);
        return;
    }
    const size_t expected_bytes = static_cast<size_t>(meta.total_pixels) * meta.pixel_len;

    const auto t0 = std::chrono::steady_clock::now();
    std::vector<uint8_t> transposed;
    if (!DecompressLz4(frame->compressed, expected_bytes, transposed)) {
        LOG_WARN("Fl6532Range LZ4 frame #{} decompress failed (compressed {} B, expect {} B)",
            frame->frame_cnt, frame->compressed.size(), expected_bytes);
        return;
    }
    const auto t1 = std::chrono::steady_clock::now();

    std::vector<uint8_t> records;
    if (!InverseTranspose(transposed, meta, records)) {
        LOG_WARN("Fl6532Range LZ4 frame #{} inverse transpose failed ({} B)", frame->frame_cnt,
            expected_bytes);
        return;
    }
    const auto t2 = std::chrono::steady_clock::now();

    size_t offset = 0;
    for (uint8_t roi = 0; roi < meta.roi_cnt &&
         roi < you_see::Fl6532RangeLz4RawParser::kMaxRoiCount; ++roi) {
        const auto& roi_info = meta.roi[roi];
        const size_t roi_bytes = static_cast<size_t>(roi_info.group_cnt) *
            roi_info.push_pixels * meta.pixel_len;
        if (roi_bytes == 0 || offset + roi_bytes > records.size()) {
            break;
        }
        FeedPipeline(records.data() + offset, roi_bytes, roi);
        offset += roi_bytes;
    }
    const auto t3 = std::chrono::steady_clock::now();

    size_t pending = 0;
    {
        std::lock_guard<std::mutex> lock(pending_mutex_);
        pending = pending_frames_.size();
    }
    LOG_INFO("Fl6532Range process frm={} sub={} decomp={}ms transpose={}ms pipeline={}ms "
        "pending={} dropped={}",
        frame->frame_cnt, frame->sub_frame_idx,
        DurationMs(t0, t1), DurationMs(t1, t2), DurationMs(t2, t3), pending, dropped_frames_);
}

bool Fl6532RangeParser::DecompressLz4(const std::vector<uint8_t>& compressed, size_t expected_size,
    std::vector<uint8_t>& out)
{
    out.clear();
    if (compressed.empty() || expected_size == 0) {
        return false;
    }

    LZ4F_dctx* dctx = nullptr;
    size_t ret = LZ4F_createDecompressionContext(&dctx, LZ4F_VERSION);
    if (LZ4F_isError(ret)) {
        return false;
    }

    out.resize(expected_size);
    size_t dst_pos = 0;
    const uint8_t* src_ptr = compressed.data();
    size_t src_remain = compressed.size();

    while (src_remain > 0 && dst_pos < out.size()) {
        size_t dst_size = out.size() - dst_pos;
        size_t src_size = src_remain;
        ret = LZ4F_decompress(dctx, out.data() + dst_pos, &dst_size, src_ptr, &src_size, nullptr);
        if (LZ4F_isError(ret)) {
            LZ4F_freeDecompressionContext(dctx);
            out.clear();
            return false;
        }
        dst_pos += dst_size;
        src_ptr += src_size;
        src_remain -= src_size;
        if (ret == 0) {
            break; // LZ4F 流正常结束
        }
        if (src_size == 0 && dst_size == 0) {
            break; // 防御异常流导致的死循环
        }
    }
    LZ4F_freeDecompressionContext(dctx);

    if (dst_pos != expected_size) {
        out.clear();
        return false;
    }
    return true;
}

bool Fl6532RangeParser::InverseTranspose(const std::vector<uint8_t>& trans,
    const you_see::Fl6532RangeLz4Meta& meta, std::vector<uint8_t>& out)
{
    const size_t pixel_len = meta.pixel_len;
    const size_t total = static_cast<size_t>(meta.total_pixels) * pixel_len;
    if (pixel_len == 0 || trans.size() != total) {
        return false;
    }
    out.resize(total);

    // 转置按 push 独立进行，push 顺序：ROI0 的 group_cnt 个 push，随后 ROI1...
    // 转置使用 64×64 tile：朴素逐字节转置的读侧以 count(768) 字节跨步，
    // 几乎每个读都是 cache miss；分块后读写局部性显著提升(单帧 747 万次搬运)
    constexpr size_t kTile = 64;
    size_t offset = 0;
    for (uint8_t roi = 0; roi < meta.roi_cnt &&
         roi < you_see::Fl6532RangeLz4RawParser::kMaxRoiCount; ++roi) {
        const size_t count = meta.roi[roi].push_pixels;
        const size_t push_bytes = count * pixel_len;
        for (uint8_t group = 0; group < meta.roi[roi].group_cnt; ++group) {
            if (count == 0 || offset + push_bytes > total) {
                return false; // 元信息与解压长度不自洽
            }
            const uint8_t* src = trans.data() + offset;
            uint8_t* dst = out.data() + offset;
            // trans[j * count + i] -> orig[i * pixel_len + j]
            for (size_t i0 = 0; i0 < count; i0 += kTile) {
                const size_t i1 = (std::min)(i0 + kTile, count);
                for (size_t j0 = 0; j0 < pixel_len; j0 += kTile) {
                    const size_t j1 = (std::min)(j0 + kTile, pixel_len);
                    for (size_t i = i0; i < i1; ++i) {
                        uint8_t* record = dst + i * pixel_len;
                        for (size_t j = j0; j < j1; ++j) {
                            record[j] = src[j * count + i];
                        }
                    }
                }
            }
            offset += push_bytes;
        }
    }
    return offset == total;
}

void Fl6532RangeParser::FeedPipeline(const uint8_t* records, size_t size, uint8_t roi_index)
{
    if (records == nullptr || size == 0) {
        return;
    }
    std::string error;
    if (!pipeline_.ingestRawPayload(records, size, roi_index, &error)) {
        LOG_WARN("Fl6532Range pipeline ingest failed (roi {}): {}", roi_index, error);
    }
}

} // namespace ys_scene
