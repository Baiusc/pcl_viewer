#include "mf150_calibration_runner.h"

#include "calib_dist_builder.h"
#include "ys_lidar_validate_type.h"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <memory>
#include <sstream>

namespace {

constexpr double kPi = 3.14159265358979323846;
constexpr double kLightSpeed = 299792458e-9;
constexpr int kURow = 75;

// 单个像素网格 -> 算法库 SslPolarPoint(extern "C" 全局类型，与旧版几何换算保持一致)
SslPolarPoint PixelToSslPoint(const ys_scene::Mf150CalibPixel& cell,
    uint32_t row, uint32_t col)
{
    SslPolarPoint pt{};
    pt.row_idx = static_cast<uint16_t>(row);
    pt.col_idx = static_cast<uint16_t>(col);
    pt.echo_idx = 0;

    float theta_deg = static_cast<float>((static_cast<int>(row) - kURow) / 152.0 * 50.0);
    float omega_deg = static_cast<float>(col / 360.0 * 120.0 - 60.0);
    pt.theta_rad = theta_deg * static_cast<float>(kPi / 180.0);
    pt.omega_rad = omega_deg * static_cast<float>(kPi / 180.0);

    if (!cell.valid || cell.tof == 0) {
        return pt;
    }
    pt.r = static_cast<float>(cell.tof / 1000.0 * kLightSpeed / 2.0);
    pt.peak = cell.peak;
    pt.intensity = static_cast<float>(cell.peak);
    return pt;
}

// 把一帧校准网格打包为算法库 SslCalibFrame(逐像素 SslPolarPoint)
algo_pc_calib::SslCalibFrame GridToSslFrame(const ys_scene::Mf150CalibGrid& grid, float dist_real)
{
    using algo_pc_calib::SslCalibFrame;
    SslCalibFrame frame;
    frame.true_dist = dist_real;
    frame.points.reserve(grid.size() * (grid.empty() ? 0 : grid[0].size()));

    for (size_t row = 0; row < grid.size(); ++row) {
        for (size_t col = 0; col < grid[row].size(); ++col) {
            const auto& cell = grid[row][col];
            if (!cell.valid || cell.tof == 0) {
                continue;
            }
            frame.points.push_back(PixelToSslPoint(cell, static_cast<uint32_t>(row),
                static_cast<uint32_t>(col)));
        }
    }
    return frame;
}

// 把单点表(SslDistCalibSingle)转成行优先 offset 表
void CopySingleTableToMap(const SslDistCalibSingle& single_table,
    std::vector<std::vector<float>>& out_map)
{
    out_map.assign(SSL_MAX_ROWS, std::vector<float>(SSL_MAX_COLS, 0.0f));
    for (int row = 0; row < SSL_MAX_ROWS; ++row) {
        for (int col = 0; col < SSL_MAX_COLS; ++col) {
            out_map[static_cast<size_t>(row)][static_cast<size_t>(col)] =
                single_table.base_offset[row][col];
        }
    }
}

} // namespace

namespace ys_scene {
using namespace you_see;

bool RunMf150Calibration(const Mf150CalibGrid& grid,
    int echo_index,
    float dist_real,
    const std::string& bin_path,
    CalibStatusFn status_cb,
    std::vector<std::vector<float>>& out_map)
{
    if (grid.empty() || grid[0].empty()) {
        return false;
    }
    (void)echo_index; // 新版建表 API 直接使用有效像素点，echo 抽取由上层完成

    // 注意：CalibDistBuilder 的 Data 内嵌约 7.5MB 的标定表结构，必须堆分配
    auto builder = std::make_unique<algo_pc_calib::CalibDistBuilder>();
    builder->m_data.list_frame.push_back(GridToSslFrame(grid, dist_real));
    builder->m_data.min_valid_dist = 0.05f;
    builder->m_data.max_valid_dist = 40.0f;
    if (status_cb) {
        builder->m_data.cb = [&status_cb](int step_idx, int total_steps,
                                         const std::string& step_name,
                                         const std::string& msg) {
            status_cb(step_idx, total_steps, step_name, msg);
        };
    }

    if (!builder->doBuildSinglePoint()) {
        if (status_cb) {
            status_cb(0, 1, "Error", builder->m_data.msg);
        }
        return false;
    }

    CopySingleTableToMap(builder->m_data.single_table, out_map);

    // 落盘(旧 BIN 格式，兼容启动时加载)
    if (!bin_path.empty() && !SaveCalibMapToFile(out_map, bin_path)) {
        if (status_cb) {
            status_cb(0, 1, "Error", "标定表文件写入失败: " + bin_path);
        }
        return false;
    }
    return true;
}

bool RunMf150MultiCalibration(
    const std::vector<std::pair<float, Mf150CalibGrid>>& samples,
    int echo_index,
    const std::string& lut_path,
    CalibStatusFn status_cb,
    SslDistCalibMulti& out_table)
{
    if (samples.empty()) {
        return false;
    }
    (void)echo_index; // 上层已按 echo 抽取为单回波网格

    auto builder = std::make_unique<algo_pc_calib::CalibDistBuilder>();
    for (const auto& [dist_real, grid] : samples) {
        if (grid.empty() || grid[0].empty()) {
            continue;
        }
        builder->m_data.list_frame.push_back(GridToSslFrame(grid, dist_real));
    }
    if (builder->m_data.list_frame.empty()) {
        return false;
    }

    builder->m_data.min_valid_dist = 0.05f;
    builder->m_data.max_valid_dist = 50.0f;
    if (status_cb) {
        builder->m_data.cb = [&status_cb](int step_idx, int total_steps,
                                         const std::string& step_name,
                                         const std::string& msg) {
            status_cb(step_idx, total_steps, step_name, msg);
        };
    }

    if (!builder->doBuildMultiPoint()) {
        if (status_cb) {
            status_cb(0, 1, "Error", builder->m_data.msg);
        }
        return false;
    }

    out_table = builder->m_data.multi_table;

    if (!lut_path.empty() && !SaveMultiCalibTableToFile(out_table, lut_path)) {
        if (status_cb) {
            status_cb(0, 1, "Error", "多点 LUT 表文件写入失败: " + lut_path);
        }
        return false;
    }
    return true;
}

bool SaveCalibMapToFile(const std::vector<std::vector<float>>& out_map,
    const std::string& bin_path)
{
    if (out_map.empty() || out_map[0].empty()) {
        return false;
    }
    namespace fs = std::filesystem;
    fs::path fp(bin_path);
    auto parent = fp.parent_path();
    if (!parent.empty() && !fs::exists(parent)) {
        std::error_code ec;
        fs::create_directories(parent, ec);
    }

    std::ofstream ofs(bin_path, std::ios::binary | std::ios::trunc);
    if (!ofs) {
        return false;
    }
    const uint32_t num_rows = static_cast<uint32_t>(out_map.size());
    const uint32_t num_cols = static_cast<uint32_t>(out_map[0].size());
    ofs.write(reinterpret_cast<const char*>(&num_rows), sizeof(num_rows));
    ofs.write(reinterpret_cast<const char*>(&num_cols), sizeof(num_cols));
    const float zero = 0.0f;
    for (const auto& row : out_map) {
        const size_t write_bytes = sizeof(float) * std::min(row.size(), size_t(num_cols));
        if (write_bytes > 0) {
            ofs.write(reinterpret_cast<const char*>(row.data()),
                static_cast<std::streamsize>(write_bytes));
        }
        for (size_t c = row.size(); c < num_cols; ++c) {
            ofs.write(reinterpret_cast<const char*>(&zero), sizeof(zero));
        }
    }
    ofs.flush();
    return ofs.good();
}

bool LoadCalibMapFromFile(const std::string& bin_path,
    std::vector<std::vector<float>>& out_map)
{
    std::ifstream ifs(bin_path, std::ios::binary);
    if (!ifs) {
        return false;
    }
    uint32_t num_rows = 0;
    uint32_t num_cols = 0;
    ifs.read(reinterpret_cast<char*>(&num_rows), sizeof(num_rows));
    ifs.read(reinterpret_cast<char*>(&num_cols), sizeof(num_cols));
    if (!ifs || num_rows == 0 || num_cols == 0 || num_rows > 4096 || num_cols > 4096) {
        return false;
    }

    out_map.assign(num_rows, std::vector<float>(num_cols, 0.0f));
    for (uint32_t r = 0; r < num_rows; ++r) {
        ifs.read(reinterpret_cast<char*>(out_map[r].data()),
            static_cast<std::streamsize>(sizeof(float) * num_cols));
        if (!ifs) {
            out_map.clear();
            return false;
        }
    }
    return !out_map.empty();
}

bool SaveMultiCalibTableToFile(const SslDistCalibMulti& table,
    const std::string& path)
{
    if (path.empty()) {
        return false;
    }
    namespace fs = std::filesystem;
    fs::path fp(path);
    auto parent = fp.parent_path();
    if (!parent.empty() && !fs::exists(parent)) {
        std::error_code ec;
        fs::create_directories(parent, ec);
    }

    std::ofstream ofs(path, std::ios::binary | std::ios::trunc);
    if (!ofs) {
        return false;
    }
    // 结构体直写，与算法侧 AlgoCalib_InitMultiTableFromBin 的读取格式一致
    ofs.write(reinterpret_cast<const char*>(&table), sizeof(table));
    ofs.flush();
    return ofs.good();
}

bool LoadMultiCalibTableFromFile(const std::string& path,
    SslDistCalibMulti& out_table)
{
    std::ifstream ifs(path, std::ios::binary);
    if (!ifs) {
        return false;
    }
    ifs.read(reinterpret_cast<char*>(&out_table), sizeof(out_table));
    return ifs.good() && ifs.gcount() == static_cast<std::streamsize>(sizeof(out_table));
}

bool SaveMf150RawFrameSync(const uint8_t* data, size_t data_size,
    size_t frame_number, uint64_t ms_timestamp,
    const std::string& file_path)
{
    if (!data || data_size < Mf150RawParser::frame_data_size) {
        return false;
    }

    std::vector<std::vector<Mf150RawPixel>> grid;
    ParseMf150RawFrame(data, data_size, grid);

    return SaveMf150RawFrameToFile(grid, frame_number, ms_timestamp, file_path);
}

void ParseMf150RawFrame(const uint8_t* data, size_t data_size,
    std::vector<std::vector<Mf150RawPixel>>& out_grid)
{
    out_grid.assign(Mf150RawParser::rows, std::vector<Mf150RawPixel>(Mf150RawParser::cols));

    if (!data || data_size < Mf150RawParser::frame_data_size) {
        return;
    }

    size_t total_packets = data_size
        / Mf150RawParser::pixel_byte_size / Mf150RawParser::pixels_per_packet;

    for (size_t pkt = 0; pkt < total_packets; ++pkt) {
        const uint8_t* pkt_data = data
            + pkt * Mf150RawParser::pixel_byte_size * Mf150RawParser::pixels_per_packet;

        for (size_t pix = 0; pix < Mf150RawParser::pixels_per_packet; ++pix) {
            const uint8_t* pixel = pkt_data + pix * Mf150RawParser::pixel_byte_size;

            uint32_t coord = static_cast<uint32_t>(pixel[0])
                | (static_cast<uint32_t>(pixel[1]) << 8)
                | (static_cast<uint32_t>(pixel[2]) << 16);
            uint16_t row = static_cast<uint16_t>(coord & 0x1FF);
            uint16_t col = static_cast<uint16_t>((coord >> 9) & 0x1FF);

            if (row >= Mf150RawParser::rows || col >= Mf150RawParser::cols) {
                continue;
            }

            auto& cell = out_grid[row][col];
            cell.valid = true;

            for (int echo = 0; echo < static_cast<int>(Mf150RawParser::echoes); ++echo) {
                const uint8_t* echo_base = pixel + 3 + echo * 8;

                uint32_t leading = static_cast<uint32_t>(echo_base[0])
                    | (static_cast<uint32_t>(echo_base[1]) << 8)
                    | (static_cast<uint32_t>(echo_base[2]) << 16);

                uint32_t trailing = static_cast<uint32_t>(echo_base[3])
                    | (static_cast<uint32_t>(echo_base[4]) << 8)
                    | (static_cast<uint32_t>(echo_base[5]) << 16);

                uint16_t peak = static_cast<uint16_t>(echo_base[6])
                    | (static_cast<uint16_t>(echo_base[7]) << 8);

                cell.tof[echo] = leading;
                if (leading == Mf150RawParser::kFpgaInvalidPixelValue
                    || trailing == Mf150RawParser::kFpgaInvalidPixelValue) {
                    cell.pulse_width[echo] = Mf150RawParser::kFpgaInvalidPixelValue;
                } else {
                    cell.pulse_width[echo] = trailing - leading;
                }
                cell.peak[echo] = peak;
            }
        }
    }
}

bool SaveMf150RawFrameToFile(const std::vector<std::vector<Mf150RawPixel>>& grid,
    size_t frame_number, uint64_t ms_timestamp,
    const std::string& file_path)
{
    namespace fs = std::filesystem;
    fs::path fp(file_path);
    auto parent = fp.parent_path();
    if (!parent.empty() && !fs::exists(parent)) {
        fs::create_directories(parent);
    }

    std::ofstream file(file_path);
    if (!file.is_open()) {
        return false;
    }

    int angle_step = 100;
    int column_width = 18;

    file << "frame_id: " << frame_number << std::endl;
    file << "timestamp: " << ms_timestamp << std::endl;
    file << "cols: " << Mf150RawParser::cols << "  rows: " << Mf150RawParser::rows << std::endl;
    file << "angle_per_col: " << angle_step << std::endl;
    file << std::endl;

    for (size_t col = 0; col < Mf150RawParser::cols; ++col) {
        file << "start of col [" << col << "], angle: " << (col * angle_step) << std::endl;

        for (size_t row = 0; row < Mf150RawParser::rows; ++row) {
            const auto& cell = grid[row][col];
            file << "row: " << std::setw(4) << row;

            if (cell.valid) {
                for (int echo = 0; echo < static_cast<int>(Mf150RawParser::echoes); ++echo) {
                    file << " " << std::setw(column_width) << cell.tof[echo]
                         << " " << std::setw(column_width) << cell.pulse_width[echo]
                         << " " << std::setw(column_width) << cell.peak[echo];
                }
            } else {
                for (int echo = 0; echo < static_cast<int>(Mf150RawParser::echoes); ++echo) {
                    file << " " << std::setw(column_width) << 0
                         << " " << std::setw(column_width) << 0
                         << " " << std::setw(column_width) << 0;
                }
            }
            file << std::endl;
        }

        file << "end of col [" << col << "]" << std::endl;
        file << std::endl;
    }

    file.close();
    return true;
}

Mf150CalibGrid ExtractMf150CalibGrid(const std::vector<std::vector<Mf150RawPixel>>& grid, int echo_index)
{
    size_t rows = grid.size();
    size_t cols = rows > 0 ? grid[0].size() : 0;

    Mf150CalibGrid calib_grid(rows, std::vector<Mf150CalibPixel>(cols));

    for (size_t row = 0; row < rows; ++row) {
        for (size_t col = 0; col < cols; ++col) {
            const auto& src = grid[row][col];
            if (!src.valid || src.tof[echo_index] == 0) {
                continue;
            }

            auto& dst = calib_grid[row][col];
            dst.valid = true;
            dst.tof = src.tof[echo_index];
            dst.peak = src.peak[echo_index];
            dst.pulse_width = src.pulse_width[echo_index];
        }
    }

    return calib_grid;
}

} // namespace ys_scene
