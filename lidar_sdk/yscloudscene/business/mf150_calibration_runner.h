#ifndef YS_SCENE_MF150_CALIBRATION_RUNNER_H_
#define YS_SCENE_MF150_CALIBRATION_RUNNER_H_

#include "calib_dist_types.h" // SslDistCalibMulti(每像素距离 LUT)/SslDistCalibSingle(单点 base offset)

#include <cstdint>
#include <functional>
#include <string>
#include <vector>

namespace ys_scene {

struct Mf150CalibPixel {
    bool valid{false};
    uint32_t tof{};
    uint16_t peak{};
    uint32_t pulse_width{};
};

using Mf150CalibGrid = std::vector<std::vector<Mf150CalibPixel>>;

struct Mf150RawPixel {
    bool valid{false};
    uint32_t tof[3]{};
    uint32_t pulse_width[3]{};
    uint16_t peak[3]{};
};

using CalibStatusFn = std::function<void(int step_idx, int total_steps,
    const std::string& step_name, const std::string& msg)>;

// 单点距离标定：基于一帧像素网格(校准网格 152×360)构建每像素 base offset 表。
// 采用算法统一库(FL6031)新版建表 API(algo_pc_calib::CalibDistBuilder)，输出 out_map
// 语义不变(行优先 offset 表，offset = 真实距离 - 测量均值)，并把表以旧 BIN 格式
// (u32 rows + u32 cols + float 行优先平铺)写入 bin_path，向后兼容已有标定文件。
bool RunMf150Calibration(const Mf150CalibGrid& grid,
    int echo_index,
    float dist_real,
    const std::string& bin_path,
    CalibStatusFn status_cb,
    std::vector<std::vector<float>>& out_map);

// 多点距离标定：基于多帧像素网格(每帧附带真实距离，至少 2 个距离点)构建
// 每像素一维查表插值(LUT)模型 SslDistCalibMulti，并按算法侧约定的结构直写格式
// 落盘(lut_path，与 AlgoCalib_InitMultiTableFromBin 读取格式一致)。
bool RunMf150MultiCalibration(
    const std::vector<std::pair<float, Mf150CalibGrid>>& samples,
    int echo_index,
    const std::string& lut_path,
    CalibStatusFn status_cb,
    SslDistCalibMulti& out_table);

// 单点标定表文件读写(旧 BIN 格式：u32 rows + u32 cols + float 行优先平铺)
bool SaveCalibMapToFile(const std::vector<std::vector<float>>& out_map,
    const std::string& bin_path);
bool LoadCalibMapFromFile(const std::string& bin_path,
    std::vector<std::vector<float>>& out_map);

// 多点 LUT 表文件读写(结构体直写，大小 sizeof(SslDistCalibMulti))
bool SaveMultiCalibTableToFile(const SslDistCalibMulti& table,
    const std::string& path);
bool LoadMultiCalibTableFromFile(const std::string& path,
    SslDistCalibMulti& out_table);

bool SaveMf150RawFrameSync(const uint8_t* data, size_t data_size,
    size_t frame_number, uint64_t ms_timestamp,
    const std::string& file_path);

void ParseMf150RawFrame(const uint8_t* data, size_t data_size,
    std::vector<std::vector<Mf150RawPixel>>& out_grid);

Mf150CalibGrid ExtractMf150CalibGrid(const std::vector<std::vector<Mf150RawPixel>>& grid, int echo_index);

bool SaveMf150RawFrameToFile(const std::vector<std::vector<Mf150RawPixel>>& grid,
    size_t frame_number, uint64_t ms_timestamp,
    const std::string& file_path);

} // namespace ys_scene

#endif // YS_SCENE_MF150_CALIBRATION_RUNNER_H_
