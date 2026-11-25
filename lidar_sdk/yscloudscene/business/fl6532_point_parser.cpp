#include "fl6532_point_parser.h"

#include "log/logging.h"
#include "pointcloud_parser.h"

#include <cmath>
#include <set>

namespace ys_scene {

// 帧重组缓存
struct Fl6532PointParser::FrameData {
    uint16_t frame_cnt{ 0 };
    uint16_t width{ Fl6532RawParser::width };
    uint16_t height{ Fl6532RawParser::height };
    uint32_t total_pixels{ Fl6532RawParser::total_pixels };
    uint64_t timestamp_base{ 0 };
    uint32_t received_points{ 0 };
    std::set<uint16_t> sequences;
    std::map<uint16_t, std::vector<Fl6532PointOut>> packets; // data_sequence -> 点集
    std::chrono::steady_clock::time_point last_update{ std::chrono::steady_clock::now() };
};

Fl6532PointParser::Fl6532PointParser(QObject* parent)
    : QObject(parent)
{
}

Fl6532PointParser::~Fl6532PointParser()
{
}

void Fl6532PointParser::SetPointcloudParser(PointcloudParser* parser)
{
    pointcloud_parser_ = parser;
}

void Fl6532PointParser::ReceivePacketTrampoline(const uint32_t handle, void* src, uint32_t length, void* context)
{
    auto* self = static_cast<Fl6532PointParser*>(context);
    if (self) {
        self->ReceivePacket(reinterpret_cast<uint8_t*>(src), length);
    }
    (void)handle;
}

bool Fl6532PointParser::ParsePacket(const uint8_t* data, uint32_t length, Fl6532PacketHeader& header)
{
    header = Fl6532PacketHeader{};
    if (!data || length < Fl6532RawParser::packet_header_size) {
        return false;
    }
    // 满包 1462B 或残差包 166B（每个 group 拆 311 满包 + 1 残差包）
    if (length != Fl6532RawParser::udp_packet_size
        && length != Fl6532RawParser::tail_packet_size) {
        return false;
    }

    const auto* head = reinterpret_cast<const Fl6532ProtocolHead*>(data);
    if (head->head != Fl6532RawParser::kProtocolHead) {
        return false;
    }
    if (head->protocol_ver_major != Fl6532RawParser::kProtocolVerMajor) {
        return false;
    }
    if (head->data_type != Fl6532RawParser::kDataType) {
        return false;
    }

    const auto* echo = reinterpret_cast<const Fl6532EchoData*>(data + sizeof(Fl6532ProtocolHead));
    if (echo->protocol_ver_minor != Fl6532RawParser::kProtocolVerMinor) {
        return false;
    }
    if (echo->type != Fl6532RawParser::kPointTypeDisXyz) {
        return false;
    }
    if (echo->pixel_len != Fl6532RawParser::pixel_byte_size) {
        return false;
    }
    if (echo->dot_num == 0 || echo->dot_num > Fl6532RawParser::pixels_per_packet) {
        return false;
    }
    if (static_cast<size_t>(echo->data_offset) + echo->data_len > length) {
        return false;
    }

    header.valid = true;
    header.frame_cnt = echo->frame_cnt;
    header.data_sequence = echo->data_sequence;
    header.dot_num = echo->dot_num;
    header.frame_flag = echo->frame_flag;
    header.ab_frame_idx = echo->ab_frame_idx;
    header.width = echo->width;
    header.height = echo->height;
    header.timestamp_base = echo->timestamp_base;
    return true;
}

float Fl6532PointParser::CalcUnitScale(uint8_t ab_frame_idx)
{
    // pcl_unit = (ab_frame_idx >> 1) & 0x7F
    // N = ((pcl_unit >> 5) & 0x03) + (pcl_unit & 0x1F) * 0.03125
    uint8_t pcl_unit = static_cast<uint8_t>((ab_frame_idx >> 1) & 0x7F);
    float n = static_cast<float>((pcl_unit >> 5) & 0x03)
        + static_cast<float>(pcl_unit & 0x1F) * 0.03125f;
    if (n <= 0.0f) {
        n = 1.0f; // 除零保护
    }
    return n;
}

std::vector<Fl6532PointOut> Fl6532PointParser::ParsePoints(const uint8_t* data, uint32_t length,
    const Fl6532PacketHeader& header)
{
    std::vector<Fl6532PointOut> result;
    if (!header.valid || !data) {
        return result;
    }

    const uint8_t* base = data + Fl6532RawParser::packet_header_size;
    float n = CalcUnitScale(header.ab_frame_idx);

    result.reserve(header.dot_num);
    for (uint16_t i = 0; i < header.dot_num; ++i) {
        const uint8_t* p = base + static_cast<size_t>(i) * Fl6532RawParser::pixel_byte_size;
        const auto* dist_xyz = reinterpret_cast<const Fl6532DistXyzInfo*>(p);
        const auto* xyz = reinterpret_cast<const Fl6532XyzInfo*>(p + sizeof(Fl6532DistXyzInfo));

        Fl6532PointOut out{};
        out.timestamp_us = header.timestamp_base + dist_xyz->timestamp_offset;
        out.col = dist_xyz->col;
        out.row = dist_xyz->row;
        out.distance_cm = static_cast<float>(xyz->distance) / n; // 原始值 / N，单位 cm
        // x/y/z 为有符号 int16 编码，需符号扩展后再换算（否则负半轴坐标会被误读为超远点）
        out.x = static_cast<float>(static_cast<int16_t>(xyz->x)) / n / 100.0f; // cm -> m
        out.y = static_cast<float>(static_cast<int16_t>(xyz->y)) / n / 100.0f;
        out.z = static_cast<float>(static_cast<int16_t>(xyz->z)) / n / 100.0f;
        out.reflectivity = xyz->reflectivity;
        out.tag_label = xyz->tag_label;

        result.push_back(out);
    }
    return result;
}

CloudFramePtr Fl6532PointParser::FrameToCartesian(uint16_t frame_cnt,
    const std::vector<Fl6532PointOut>& points)
{
    auto cartesian_frame = std::make_shared<you_see::CloudFrame<you_see::CartesianPoint>>(
        frame_cnt, points.size());

    for (const auto& p : points) {
        // x/y/z 全部为 0 视为无效点；distance 字段可能为 0/冗余，不作为过滤依据
        if (std::abs(p.x) < 1e-6f && std::abs(p.y) < 1e-6f && std::abs(p.z) < 1e-6f) {
            continue;
        }

        you_see::CartesianPoint cp{};
        cp.x.f = p.x;
        cp.y.f = p.y;
        cp.z.f = p.z;
        cp.reflectivity = p.reflectivity;

        cartesian_frame->data.push_back(cp);
    }
    cartesian_frame->data.shrink_to_fit();
    return cartesian_frame;
}

std::vector<float> Fl6532PointParser::BuildDepthArray(const std::vector<Fl6532PointOut>& points,
    size_t width, size_t height)
{
    std::vector<float> depth(width * height, std::nanf(""));
    for (const auto& p : points) {
        if (p.row >= height || p.col >= width || p.distance_cm <= 0.0f) {
            continue;
        }
        // 距离 ≈ sqrt(x²+y²+z²)，单位 m
        depth[static_cast<size_t>(p.row) * width + p.col] = std::sqrt(p.x * p.x + p.y * p.y + p.z * p.z);
    }
    return depth;
}

void Fl6532PointParser::ReceivePacket(const uint8_t* data, uint32_t length)
{
    if (!data || length == 0) {
        return;
    }

    size_t offset = 0;
    // 满包 (1462B) 拆解
    while (length - offset >= Fl6532RawParser::udp_packet_size) {
        HandleSinglePacket(data + offset, Fl6532RawParser::udp_packet_size);
        offset += Fl6532RawParser::udp_packet_size;
    }
    // 残差包 (166B) 拆解
    while (length - offset == Fl6532RawParser::tail_packet_size) {
        HandleSinglePacket(data + offset, Fl6532RawParser::tail_packet_size);
        offset += Fl6532RawParser::tail_packet_size;
    }
    if (length - offset > 0) {
        LOG_WARN("Fl6532 unexpected trailing bytes: {0}", length - offset);
    }
}

void Fl6532PointParser::HandleSinglePacket(const uint8_t* data, uint32_t length)
{
    Fl6532PacketHeader header;
    if (!ParsePacket(data, length, header)) {
        return;
    }

    auto points = ParsePoints(data, length, header);
    if (points.empty()) {
        return;
    }

    auto now = std::chrono::steady_clock::now();
    std::lock_guard<std::mutex> lock(mutex_);

    // 清理超时的不完整帧
    for (auto it = frames_.begin(); it != frames_.end();) {
        if (now - it->second->last_update >= Fl6532RawParser::kFrameTimeout) {
            LOG_INFO("Fl6532 frame #{0} incomplete (received {1} points), DISCARDED",
                it->second->frame_cnt, it->second->received_points);
            it = frames_.erase(it);
        } else {
            ++it;
        }
    }

    auto& frame = frames_[header.frame_cnt];
    if (!frame) {
        frame = std::make_shared<FrameData>();
        frame->frame_cnt = header.frame_cnt;
        frame->width = header.width;
        frame->height = header.height;
        frame->timestamp_base = header.timestamp_base;
        frame->total_pixels = static_cast<uint32_t>(header.width) * header.height;
    }

    // 去重：同帧同 data_sequence 的重复包忽略
    if (frame->sequences.count(header.data_sequence) != 0) {
        return;
    }

    // 丢包检测：帧内 data_sequence 应连续递增；序号回绕（变小）视为新子帧开始，不告警
    if (!frame->sequences.empty()) {
        uint16_t last_seq = *frame->sequences.rbegin();
        if (header.data_sequence > static_cast<uint16_t>(last_seq + 1)) {
            LOG_WARN("Fl6532 frame #{0} packet sequence gap: {1} -> {2}",
                header.frame_cnt, last_seq, header.data_sequence);
        }
    }

    frame->sequences.insert(header.data_sequence);
    frame->packets.emplace(header.data_sequence, std::move(points));
    frame->received_points += header.dot_num;
    frame->last_update = now;

    // 完整帧判定：累计点数达到 width*height 视为完成
    if (frame->received_points >= frame->total_pixels) {
        DispatchCompleteFrame(frame);
        frames_.erase(header.frame_cnt);
        return;
    }
    // 收到帧结束标志但点数不足：缺失不超过一个满包时输出部分帧，否则丢弃
    if (header.frame_flag == Fl6532RawParser::kFrameFlagEnd) {
        if (frame->received_points >= frame->total_pixels - Fl6532RawParser::pixels_per_packet) {
            LOG_WARN("Fl6532 frame #{0} complete with minor loss: {1}/{2}",
                frame->frame_cnt, frame->received_points, frame->total_pixels);
            DispatchCompleteFrame(frame);
        } else {
            LOG_WARN("Fl6532 frame #{0} ended with only {1}/{2} points, DISCARDED",
                frame->frame_cnt, frame->received_points, frame->total_pixels);
        }
        frames_.erase(header.frame_cnt);
    }
}

void Fl6532PointParser::DispatchCompleteFrame(const std::shared_ptr<FrameData>& frame)
{
    // 按 data_sequence 顺序组装完整帧点集
    std::vector<Fl6532PointOut> all_points;
    all_points.reserve(frame->received_points);
    for (auto& item : frame->packets) {
        auto& seq_points = item.second;
        all_points.insert(all_points.end(), seq_points.begin(), seq_points.end());
    }

    auto cartesian_frame = FrameToCartesian(frame->frame_cnt, all_points);

    ++frames_count_;
    LOG_INFO("Fl6532 frame #{0}: received {1}/{2} pts, rendered {3} pts",
        frame->frame_cnt, frame->received_points, frame->total_pixels,
        cartesian_frame->data.size());
    LogFrameDiagnostics(all_points, frame->received_points, cartesian_frame->data.size());

    if (pointcloud_parser_ && cartesian_frame && !cartesian_frame->data.empty()) {
        pointcloud_parser_->AddCartesianFrame(cartesian_frame);
    }
}

void Fl6532PointParser::LogFrameDiagnostics(const std::vector<Fl6532PointOut>& points,
    size_t received, size_t rendered) const
{
    // 每 10 帧输出一次统计，用于诊断视场缺失、无效点过滤、符号编码等问题
    if (frames_count_ % 10 != 0) {
        return;
    }

    size_t distance_zero = 0;   // distance 原始值换算后为 0 的点数（协议冗余字段，不作为无效标记）
    size_t xyz_zero = 0;        // x/y/z 全部为 0 的点数（真正的无效点）
    size_t neg_x = 0;           // 符号扩展后 x<0 的点数
    size_t neg_y = 0;           // 符号扩展后 y<0 的点数
    size_t far_points = 0;      // |x/y/z| > 100m 的点数，疑似无符号数据被误符号扩展
    float min_x = 0.0f, max_x = 0.0f, min_y = 0.0f, max_y = 0.0f, min_z = 0.0f, max_z = 0.0f;
    uint16_t min_col = 0, max_col = 0, min_row = 0, max_row = 0;
    bool first = true;

    for (const auto& p : points) {
        if (p.distance_cm <= 0.0f) {
            ++distance_zero;
        }
        if (std::abs(p.x) < 1e-6f && std::abs(p.y) < 1e-6f && std::abs(p.z) < 1e-6f) {
            ++xyz_zero;
        }
        if (p.x < 0.0f) {
            ++neg_x;
        }
        if (p.y < 0.0f) {
            ++neg_y;
        }
        if (std::abs(p.x) > 100.0f || std::abs(p.y) > 100.0f || std::abs(p.z) > 100.0f) {
            ++far_points;
        }

        // 仅统计有效点（非 x/y/z 全 0）的坐标/阵列范围
        if (std::abs(p.x) < 1e-6f && std::abs(p.y) < 1e-6f && std::abs(p.z) < 1e-6f) {
            continue;
        }
        if (first) {
            min_x = max_x = p.x;
            min_y = max_y = p.y;
            min_z = max_z = p.z;
            min_col = max_col = p.col;
            min_row = max_row = p.row;
            first = false;
        } else {
            if (p.x < min_x) min_x = p.x;
            if (p.x > max_x) max_x = p.x;
            if (p.y < min_y) min_y = p.y;
            if (p.y > max_y) max_y = p.y;
            if (p.z < min_z) min_z = p.z;
            if (p.z > max_z) max_z = p.z;
            if (p.col < min_col) min_col = p.col;
            if (p.col > max_col) max_col = p.col;
            if (p.row < min_row) min_row = p.row;
            if (p.row > max_row) max_row = p.row;
        }
    }

    LOG_INFO("Fl6532 diag: valid {0}/{1}, dist_zero {2}, xyz_zero {3}, "
        "neg_x {4} neg_y {5}, far>100m {6}, "
        "x[{7:.2f},{8:.2f}]m y[{9:.2f},{10:.2f}]m z[{11:.2f},{12:.2f}]m, "
        "col[{13},{14}] row[{15},{16}]",
        rendered, received, distance_zero, xyz_zero,
        neg_x, neg_y, far_points,
        min_x, max_x, min_y, max_y, min_z, max_z,
        min_col, max_col, min_row, max_row);
}

} // namespace ys_scene
