#include "cs96_gamma_raw.h"

#include <algorithm>
#include <filesystem>
#include <fstream>
#include <iomanip>

namespace ys_scene {
namespace {
uint32_t ReadLittleEndian(const uint8_t *data, size_t bytes)
{
    uint32_t value = 0;
    for (size_t i = 0; i < bytes; ++i) {
        value |= static_cast<uint32_t>(data[i]) << (8 * i);
    }
    return value;
}

bool IsNewer(uint16_t candidate, uint16_t reference)
{
    const uint16_t distance = static_cast<uint16_t>(candidate - reference);
    return distance != 0 && distance < 0x8000;
}
} // namespace

void Cs96GammaRawAssembler::Reset(const you_see::LidarInfo &node)
{
    // 1945B 仅在 CS 系列的原始点云模式识别，不能路由到真实点云解析器。
    enabled_ = node.model == you_see::LidarModel::KCS && node.node_work_mode == you_see::NodeWorkMode::KOriginCloud;
    device_ip_ = node.ip;
    newest_frame_.reset();
    last_delivered_frame_.reset();
    pending_frames_.clear();
}

Cs96GammaRawAssembler::FramePtr Cs96GammaRawAssembler::Feed(uint32_t device_ip, const uint8_t *data, size_t size)
{
    using namespace CS96GammaRawParser;
    if (!enabled_ || device_ip != device_ip_ || !data || size != udp_packet_size || data[0] != 0xAA ||
        ReadLittleEndian(data + 7, 2) != slot_data_size) {
        return nullptr;
    }

    const auto frame_id = static_cast<uint16_t>(ReadLittleEndian(data + 1, 2));
    const auto slot = static_cast<uint16_t>(ReadLittleEndian(data + 3, 2));
    const uint8_t *value = data + origin_header_size;
    if (slot >= slot_counts || (ReadLittleEndian(value, 4) & 0xFFF) != slot) {
        return nullptr;
    }
    // seq_num 是驱动批次下标：一个数据报已包含完整 SLOT，不按 seq_num 切片或拼接。
    if ((last_delivered_frame_ && !IsNewer(frame_id, *last_delivered_frame_)) ||
        (newest_frame_ && !IsNewer(frame_id, *newest_frame_) &&
         static_cast<uint16_t>(*newest_frame_ - frame_id) >= kMaxPendingFrames)) {
        return nullptr;
    }
    if (!newest_frame_ || IsNewer(frame_id, *newest_frame_)) {
        newest_frame_ = frame_id;
        for (auto it = pending_frames_.begin(); it != pending_frames_.end();) {
            if (static_cast<uint16_t>(frame_id - it->first) >= kMaxPendingFrames) {
                it = pending_frames_.erase(it);
            }
            else {
                ++it;
            }
        }
    }

    auto &frame = pending_frames_[frame_id];
    if (!frame) {
        frame = std::make_shared<you_see::CloudFrame<uint8_t>>(frame_id, frame_data_size);
        frame->data.resize(frame_data_size);
    }
    if (!frame->received_slots.insert(slot).second) {
        return nullptr;
    }
    std::copy_n(value, slot_data_size, frame->data.data() + slot * slot_data_size);
    ++frame->receive_times;
    frame->last_update = std::chrono::steady_clock::now();
    if (frame->received_slots.size() != slot_counts) {
        return nullptr;
    }

    auto complete = frame;
    last_delivered_frame_ = frame_id;
    pending_frames_.erase(frame_id);
    return complete;
}

bool SaveCS96GammaRawFrameSync(const uint8_t *data, size_t data_size, size_t frame_number, uint64_t ms_timestamp,
                               const std::string &file_path)
{
    using namespace CS96GammaRawParser;
    if (!data || data_size != frame_data_size) {
        return false;
    }
    for (size_t slot = 0; slot < slot_counts; ++slot) {
        if ((ReadLittleEndian(data + slot * slot_data_size, 4) & 0xFFF) != slot) {
            return false;
        }
    }

    namespace fs = std::filesystem;
    const fs::path text_path(file_path);
    fs::path binary_path = text_path;
    binary_path.replace_extension(".bin");
    if (binary_path == text_path) {
        return false;
    }
    std::error_code error;
    if (!text_path.parent_path().empty()) {
        fs::create_directories(text_path.parent_path(), error);
        if (error) {
            return false;
        }
    }

    std::ofstream binary(binary_path, std::ios::binary);
    if (!binary) {
        return false;
    }
    binary.write(reinterpret_cast<const char *>(data), static_cast<std::streamsize>(data_size));
    binary.close();
    if (!binary) {
        return false;
    }

    std::ofstream file(text_path);
    if (!file) {
        return false;
    }
    file << "frame_id: " << frame_number << '\n';
    file << "timestamp: " << ms_timestamp << '\n';
    file << "cols: " << slot_counts << "  rows: " << pixels_per_slot << '\n';
    file << "echo_fields: ft0 et0 pk0 ft1 et1 pk1\n\n";
    constexpr int column_width = 18;
    constexpr int pixel_width = 3;

    for (size_t slot = 0; slot < slot_counts; ++slot) {
        const uint8_t *value = data + slot * slot_data_size;
        const uint32_t word = ReadLittleEndian(value, 4);
        file << "start of col [" << slot << "]\n";
        file << "slot_number: " << (word & 0xFFF) << '\n';
        file << "mirror_angle: " << (word >> 12) << '\n';
        file << "relative_time_us: " << ReadLittleEndian(value + 4, 3) << '\n';
        file << "fpga_frame_id: " << static_cast<unsigned>(value[14]) << '\n';
        file << "face_id: " << (value[14] % 4) << '\n';
        file << "              " << std::setw(pixel_width) << "id";
        for (const char *field : {"ft0", "et0", "pk0", "ft1", "et1", "pk1"}) {
            file << ' ' << std::setw(column_width) << field;
        }
        file << '\n';
        for (size_t pixel = 0; pixel < pixels_per_slot; ++pixel) {
            file << "pixel_number: " << std::setw(pixel_width) << pixel;
            for (size_t echo = 0; echo < echoes; ++echo) {
                const uint8_t *raw = value + slot_header_size + pixel * pixel_byte_size + echo * 8;
                file << ' ' << std::setw(column_width) << ReadLittleEndian(raw, 3)
                     << ' ' << std::setw(column_width) << ReadLittleEndian(raw + 3, 3)
                     << ' ' << std::setw(column_width) << ReadLittleEndian(raw + 6, 2);
            }
            file << '\n';
        }
        file << "end of slot [" << slot << "]\n\n";
    }
    file.close();
    return static_cast<bool>(file);
}

} // namespace ys_scene
