#ifndef YS_SCENE_CS96_GAMMA_HISTOGRAM_H_
#define YS_SCENE_CS96_GAMMA_HISTOGRAM_H_

#include "ys_lidar_validate_type.h"

#include <string>

namespace ys_scene {

using HistogramPackets = std::unordered_map<uint16_t, std::unordered_map<uint16_t, std::vector<uint8_t>>>;

// 仅接受 slot 0 的 30 个完整载荷；输出保持 FPGA 上传像素顺序。
std::vector<uint16_t> DecodeGammaHistogram(const HistogramPackets &packets);
bool SaveGammaHistogram(const HistogramPackets &packets, uint16_t frame_id, const std::string &text_path);

} // namespace ys_scene

#endif // YS_SCENE_CS96_GAMMA_HISTOGRAM_H_
