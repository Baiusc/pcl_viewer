#include "cs96_gamma_histogram.h"

#include <filesystem>
#include <fstream>

namespace ys_scene {

std::vector<uint16_t> DecodeGammaHistogram(const HistogramPackets &packets)
{
    using namespace you_see::RawParser;
    const auto slot = packets.find(0);
    if (packets.size() != 1 || slot == packets.end() || slot->second.size() != kFragmentCountsCs96Gamma) {
        return {};
    }
    std::vector<uint16_t> bins;
    bins.reserve(kRawRowsCs96Gamma * kPixelPointsCs96Gamma);
    for (uint16_t seq = 0; seq < kFragmentCountsCs96Gamma; ++seq) {
        const auto packet = slot->second.find(seq);
        if (packet == slot->second.end() || packet->second.size() != kRawPayloadSizeCs96Gamma) {
            return {};
        }
        const auto &data = packet->second;
        for (size_t i = 0; i < data.size(); i += 2) {
            bins.push_back(static_cast<uint16_t>(data[i] | (static_cast<uint16_t>(data[i + 1]) << 8)));
        }
    }
    return bins;
}

bool SaveGammaHistogram(const HistogramPackets &packets, uint16_t frame_id, const std::string &text_path)
{
    using namespace you_see::RawParser;
    const auto bins = DecodeGammaHistogram(packets);
    if (bins.empty()) {
        return false;
    }
    namespace fs = std::filesystem;
    const fs::path path(text_path);
    fs::path binary_path = path;
    binary_path.replace_extension(".bin");
    if (path == binary_path) {
        return false;
    }
    std::error_code error;
    if (!path.parent_path().empty()) {
        fs::create_directories(path.parent_path(), error);
        if (error) {
            return false;
        }
    }
    std::ofstream binary(binary_path, std::ios::binary);
    std::ofstream text(path);
    if (!binary || !text) {
        return false;
    }
    for (uint16_t seq = 0; seq < kFragmentCountsCs96Gamma; ++seq) {
        const auto &payload = packets.at(0).at(seq);
        binary.write(reinterpret_cast<const char *>(payload.data()), static_cast<std::streamsize>(payload.size()));
    }
    text << "device: cs96_gamma\nframe_id: " << frame_id << "\ncols: 1  rows: 120  bins: 512\n";
    for (size_t pixel = 0; pixel < kRawRowsCs96Gamma; ++pixel) {
        text << "Start of vector [0][" << pixel << "] (Device: cs96_gamma)\n";
        for (size_t bin = 0; bin < kPixelPointsCs96Gamma; ++bin) {
            text << bins[pixel * kPixelPointsCs96Gamma + bin] << ' ';
        }
        text << "\nEnd of vector [0][" << pixel << "]\n\n";
    }
    binary.close();
    text.close();
    return static_cast<bool>(binary) && static_cast<bool>(text);
}

} // namespace ys_scene
