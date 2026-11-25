#include "cs96_raw_save.h"
#include "mf150_calibration_runner.h"
#include "ys_lidar_validate_type.h"

#include <filesystem>
#include <fstream>
#include <iomanip>
#include <vector>

// ===== Beta: 编码器角度刻度表 (与 origin_point_parser_imp.cpp 共用) =====
static constexpr float kBetaEncoderAngle[] = {
    0,       1.506,   3.034,   4.477,   5.968,   7.522,   9.005,   10.511,
    12.019,  13.492,  15.026,  16.532,  18.038,  19.567,  21.092,  22.53,
    24.097,  25.566,  27.05,   28.546,  30.091,  31.591,  33.087,  34.588,
    36.107,  37.61,   39.12,   40.605,  42.108,  43.651,  45.183,  46.659,
    48.164,  49.654,  51.167,  52.64,   54.164,  55.675,  57.19,   58.728,
    60.237,  61.732,  63.259,  64.725,  66.226,  67.735,  69.231,  70.737,
    72.27,   73.743,  75.256,  76.835,  78.27,   79.78,   81.325,  82.856,
    84.341,  85.829,  87.366,  88.868,  90.459,  91.903,  93.385,  94.857,
    96.346,  97.854,  99.358,  100.921, 102.343, 103.902, 105.408, 106.904,
    108.456, 109.967, 111.459, 112.987, 114.503, 116.008, 117.543, 119.007,
    120.474, 121.977, 123.484, 124.999, 126.467, 128.024, 129.546, 130.969,
    132.448, 134.004, 135.509, 137.024, 138.58,  140.075, 141.562, 143.056,
    144.582, 146.032, 147.555, 149.039, 150.554, 152.013, 153.535, 155.018,
    156.528, 158.006, 159.497, 161.052, 162.552, 164.051, 165.569, 167.075,
    168.571, 170.089, 171.573, 173.061, 174.538, 176.012, 177.575, 179.043,
    180.533, 182.065, 183.512, 185.031, 186.517, 188.025, 189.467, 191.002,
    192.523, 194.008, 195.518, 197.019, 198.502, 199.977, 201.496, 202.96,
    204.504, 205.957, 207.464, 208.941, 210.439, 211.966, 213.402, 214.942,
    216.505, 217.917, 219.389, 220.883, 222.359, 223.889, 225.342, 226.864,
    228.363, 229.854, 231.38,  232.83,  234.331, 235.814, 237.325, 238.806,
    240.298, 241.793, 243.28,  244.785, 246.281, 247.79,  249.294, 250.763,
    252.211, 253.761, 255.222, 256.77,  258.213, 259.744, 261.176, 262.764,
    264.277, 265.718, 267.199, 268.727, 270.191, 271.664, 273.147, 274.695,
    276.112, 277.614, 279.108, 280.585, 282.137, 283.559, 285.041, 286.551,
    288.127, 289.574, 291.042, 292.552, 294.078, 295.623, 297.058, 298.638,
    300.048, 301.535, 302.991, 304.526, 305.984, 307.501, 308.975, 310.456,
    312.004, 313.457, 314.987, 316.464, 317.998, 319.498, 321.037, 322.518,
    323.985, 325.485, 327.038, 328.488, 329.977, 331.506, 333.016, 334.43,
    335.954, 337.465, 338.966, 340.488, 341.993, 343.497, 344.968, 346.456,
    347.948, 349.451, 350.96,  352.537, 354.01,  355.483, 356.984, 358.474,
    360,
};
static constexpr size_t kBetaEncoderAngleCnt = sizeof(kBetaEncoderAngle) / sizeof(kBetaEncoderAngle[0]);

namespace ys_scene {
using namespace you_see;

bool SaveCS96RawFrameSync(const uint8_t* data, size_t data_size,
    size_t frame_number, uint64_t ms_timestamp,
    const std::string& file_path)
{
    if (!data || data_size == 0) {
        return false;
    }

    namespace fs = std::filesystem;
    fs::path fp(file_path);
    auto parent = fp.parent_path();
    if (!parent.empty() && !fs::exists(parent)) {
        fs::create_directories(parent);
    }

    size_t slot_size = data_size / CS96BetaRawParser::slot_counts;
    if (slot_size < CS96BetaRawParser::slot_header_size) {
        return false;
    }

    // 构建 grid: [slot][pixel], 像素倒序排列
    std::vector<std::vector<Mf150RawPixel>> grid(
        CS96BetaRawParser::slot_counts,
        std::vector<Mf150RawPixel>(CS96BetaRawParser::pixels_per_slot));

    for (size_t slot = 0; slot < CS96BetaRawParser::slot_counts; ++slot) {
        const uint8_t* slot_data = data + slot * slot_size;

        // 解析 16B slot头: word0(4B) + word1(4B) + reserved_high(8B)
        uint32_t w0 = static_cast<uint32_t>(slot_data[0])
            | (static_cast<uint32_t>(slot_data[1]) << 8)
            | (static_cast<uint32_t>(slot_data[2]) << 16)
            | (static_cast<uint32_t>(slot_data[3]) << 24);

        // 像素数据从 offset 16 开始, 倒序: pixel[95]在前, pixel[0]在后
        const uint8_t* pixel_base = slot_data + CS96BetaRawParser::slot_header_size;

        for (size_t i = 0; i < CS96BetaRawParser::pixels_per_slot; ++i) {
            const uint8_t* pixel_data = pixel_base + i * CS96BetaRawParser::pixel_byte_size;

            auto& cell = grid[slot][i];
            cell.valid = true;

            // 第1重回波: tof 3B + trailing 3B + peak 2B
            uint32_t tof0 = static_cast<uint32_t>(pixel_data[0])
                | (static_cast<uint32_t>(pixel_data[1]) << 8)
                | (static_cast<uint32_t>(pixel_data[2]) << 16);
            uint32_t trail0 = static_cast<uint32_t>(pixel_data[3])
                | (static_cast<uint32_t>(pixel_data[4]) << 8)
                | (static_cast<uint32_t>(pixel_data[5]) << 16);
            uint16_t peak0 = static_cast<uint16_t>(pixel_data[6])
                | (static_cast<uint16_t>(pixel_data[7]) << 8);

            cell.tof[0] = tof0;
            cell.pulse_width[0] = trail0;
            cell.peak[0] = peak0;

            if (tof0 == 0) {
                cell.valid = false;
            }

            // 第2重回波: tof 3B + trailing 3B + peak 2B
            uint32_t tof1 = static_cast<uint32_t>(pixel_data[8])
                | (static_cast<uint32_t>(pixel_data[9]) << 8)
                | (static_cast<uint32_t>(pixel_data[10]) << 16);
            uint32_t trail1 = static_cast<uint32_t>(pixel_data[11])
                | (static_cast<uint32_t>(pixel_data[12]) << 8)
                | (static_cast<uint32_t>(pixel_data[13]) << 16);
            uint16_t peak1 = static_cast<uint16_t>(pixel_data[14])
                | (static_cast<uint16_t>(pixel_data[15]) << 8);

            cell.tof[1] = tof1;
            cell.pulse_width[1] = trail1;
            cell.peak[1] = peak1;

            // 第3重回波: 填假数据 (16)
            cell.tof[2] = CS96BetaRawParser::kFpgaInvalidPixelValue;
            cell.pulse_width[2] = CS96BetaRawParser::kFpgaInvalidPixelValue;
            cell.peak[2] = static_cast<uint16_t>(CS96BetaRawParser::kFpgaInvalidPixelValue);
        }

        (void)w0;
    }

    std::ofstream file(file_path);
    if (!file.is_open()) {
        return false;
    }

    int column_width = 18;

    file << "frame_id: " << frame_number << std::endl;
    file << "timestamp: " << ms_timestamp << std::endl;
    file << "cols: " << CS96BetaRawParser::slot_counts
         << "  rows: " << CS96BetaRawParser::pixels_per_slot << std::endl;
    file << std::endl;

    for (size_t slot = 0; slot < CS96BetaRawParser::slot_counts; ++slot) {
        const uint8_t* slot_data = data + slot * slot_size;

        // 从 word0 提取 mirror_angle (20bit: 低12位=real_angle, 高8位=scale)
        uint32_t w0 = static_cast<uint32_t>(slot_data[0])
            | (static_cast<uint32_t>(slot_data[1]) << 8)
            | (static_cast<uint32_t>(slot_data[2]) << 16)
            | (static_cast<uint32_t>(slot_data[3]) << 24);
        uint32_t mirror_angle = (w0 >> 12) & 0xFFFFF;

        uint16_t real_angle = mirror_angle & 0xFFF;
        uint8_t scale = static_cast<uint8_t>((mirror_angle >> 12) & 0xFF);
        float integer_scale = (scale < kBetaEncoderAngleCnt)
            ? kBetaEncoderAngle[scale] : 0.0f;
        float computed_angle = (integer_scale + 0.0009f * real_angle) / 360.0f * 61440.0f;

        file << "start of col [" << slot << "]" << std::endl;
        file << "rotate mirror angle: " << static_cast<int>(computed_angle) << std::endl;

        for (size_t row = 0; row < CS96BetaRawParser::pixels_per_slot; ++row) {
            const auto& cell = grid[slot][row];
            file << "pixel_number: " << row;

            if (cell.valid) {
                for (int echo = 0; echo < 3; ++echo) {
                    file << " " << std::setw(column_width) << cell.tof[echo]
                         << " " << std::setw(column_width) << cell.pulse_width[echo]
                         << " " << std::setw(column_width) << cell.peak[echo];
                }
            } else {
                for (int echo = 0; echo < 3; ++echo) {
                    file << " " << std::setw(column_width) << 0
                         << " " << std::setw(column_width) << 0
                         << " " << std::setw(column_width) << 0;
                }
            }
            file << std::endl;
        }

        file << "end of slot [" << slot << "]" << std::endl;
        file << std::endl;
    }

    file.close();
    return true;
}

} // namespace ys_scene
