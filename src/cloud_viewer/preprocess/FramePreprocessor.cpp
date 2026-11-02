#include "cloud_viewer/preprocess/FramePreprocessor.h"

namespace cloud_viewer
{

bool Legacy6532Calibration::configure(const PreprocessConfig &config, std::string &error)
{
    ready_ = false;

    if (config.calibrationBin.empty())
    {
        error = "未指定 6532 标定文件（calib_data.bin）";
        return false;
    }
    if (!std::filesystem::exists(config.calibrationBin))
    {
        error = "标定文件不存在: " + config.calibrationBin.string();
        return false;
    }

    if (!algo_lidar::loadRange6532LegacyCalibrationBin(config.calibrationBin.string(),
                                                       calibration_))
    {
        error = "标定文件解析失败（布局或版本不匹配）: " + config.calibrationBin.string();
        return false;
    }

    config_ = config;
    ready_ = true;
    return true;
}

bool Legacy6532Calibration::process(algo_lidar::PointMatrix &points, std::string &error)
{
    if (!ready_)
    {
        error = "标定模块未配置";
        return false;
    }
    if (points.empty())
    {
        error = "点矩阵为空";
        return false;
    }

    algo_lidar::Range6532LegacyApplyOptions options;
    options.model = config_.model;
    algo_lidar::applyRange6532LegacyCalibration(points, calibration_, config_.roi, options);
    return true;
}

} // namespace cloud_viewer
