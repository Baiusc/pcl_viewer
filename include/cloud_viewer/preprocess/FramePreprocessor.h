/**
 * @brief 可选预处理（标定）模块接口
 *
 * 默认 Lite 运行不自动应用标定；只有显式配置并成功加载标定文件时才处理。
 * 6532 的已重建 CSV 不应二次调用 legacy 标定。
 */
#pragma once

#include <cstdint>
#include <filesystem>
#include <string>

#include "point_type.h"

namespace cloud_viewer
{

struct PreprocessConfig
{
    std::filesystem::path calibrationBin;
    std::uint8_t roi = 0;
    algo_lidar::Range6532LegacyModel model = algo_lidar::Range6532LegacyModel::Tail3LnFixed;
};

class IFramePreprocessor
{
public:
    virtual ~IFramePreprocessor() = default;

    virtual std::string id() const = 0;

    virtual bool configure(const PreprocessConfig &config, std::string &error) = 0;

    virtual bool process(algo_lidar::PointMatrix &points, std::string &error) = 0;
};

/// FL6532 legacy raw 数据重建：包装现有 applyRange6532LegacyCalibration。
class Legacy6532Calibration : public IFramePreprocessor
{
public:
    std::string id() const override { return "legacy_6532"; }

    bool configure(const PreprocessConfig &config, std::string &error) override;

    bool process(algo_lidar::PointMatrix &points, std::string &error) override;

    bool ready() const { return ready_; }

private:
    PreprocessConfig config_;
    algo_lidar::Range6532LegacyCalibrationBin calibration_;
    bool ready_ = false;
};

} // namespace cloud_viewer
