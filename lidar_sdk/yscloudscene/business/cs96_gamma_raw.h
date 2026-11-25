#ifndef YS_SCENE_CS96_GAMMA_RAW_H_
#define YS_SCENE_CS96_GAMMA_RAW_H_

#include "ys_lidar_type.h"

#include <optional>
#include <string>
#include <unordered_map>

namespace ys_scene {
namespace CS96GammaRawParser {
constexpr size_t origin_header_size = 9;
constexpr size_t slot_header_size = 16;
constexpr size_t pixels_per_slot = 120;
constexpr size_t pixel_byte_size = 16;
constexpr size_t echoes = 2;
constexpr size_t slot_counts = 800;
constexpr size_t slot_data_size = slot_header_size + pixels_per_slot * pixel_byte_size;
constexpr size_t udp_packet_size = origin_header_size + slot_data_size;
constexpr size_t frame_data_size = slot_counts * slot_data_size;
} // namespace CS96GammaRawParser

// 单线程使用；调用方按连接重置，UDP 头和 SLOT 均按小端读取。
class Cs96GammaRawAssembler
{
  public:
    using FramePtr = std::shared_ptr<you_see::CloudFrame<uint8_t>>;
    void Reset(const you_see::LidarInfo &node = {});
    FramePtr Feed(uint32_t device_ip, const uint8_t *data, size_t size);
  private:
    static constexpr uint16_t kMaxPendingFrames = 4;
    bool enabled_{false};
    uint32_t device_ip_{0};
    std::optional<uint16_t> newest_frame_;
    std::optional<uint16_t> last_delivered_frame_;
    std::unordered_map<uint16_t, FramePtr> pending_frames_;
};

// TXT 保留两回波 ft/et/pk 原始码值；同名 BIN 为按槽号排列的 800 个完整 FPGA SLOT。
bool SaveCS96GammaRawFrameSync(const uint8_t *data, size_t data_size, size_t frame_number, uint64_t ms_timestamp,
                               const std::string &file_path);

} // namespace ys_scene

#endif // YS_SCENE_CS96_GAMMA_RAW_H_
