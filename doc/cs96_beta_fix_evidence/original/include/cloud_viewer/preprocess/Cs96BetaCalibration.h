/**
 * @brief CS96 Beta 原始点云标定（距离 / 水平角 / 俯仰角 / 反射率）
 *
 * 只作用于 work_mode=1 的原始点云；work_mode=2 由雷达端直接输出 xyz，无需本标定。
 *
 * 标定数据目录包含：
 *   ys_calibration.ini      omega_delta[96]、d_delta[96]（cm）、a[96]、theta 参数、面起始角
 *   spot_slope_map.csv      Face,Angle_FPGA,Diff —— FPGA 刻度到真实水平角（每面 80 点）
 *   reflectivity_map.csv    pixel_target,distance,reflectance_0..4 —— 反射率查表
 *
 * 计算公式（与 you_see_arm 一致）：
 *   r     = kLightSpeed * (ft >> 1) * a[pixel] + d_delta[pixel] * 0.01
 *   theta = (line - u) * (theta_max - theta_min) / lines_v + theta_tilt
 *   face  = scale 所在面（刻度 8/68/128/188）
 *   hc    = encoder_angle[scale] + 0.0009 * real_angle - starting_angle[face]
 *   omega = hc * 2 + omega0 + omega_delta[pixel]
 */
#pragma once

#include <array>
#include <cstdint>
#include <filesystem>
#include <map>
#include <string>
#include <vector>

#include "cloud_viewer/core/CloudFrame.h"

namespace cloud_viewer
{

class Cs96BetaCalibration
{
public:
    struct ReflectivityRow
    {
        int pixel = 0;
        float distance = 0.0f;
        std::array<float, 5> reference{};
    };

    Cs96BetaCalibration();

    bool load(const std::filesystem::path &directory, std::string &error);
    bool loaded() const { return loaded_; }
    const std::filesystem::path &directory() const { return directory_; }

    /// 由原始点云的一回波计算笛卡尔坐标与反射率。
    /// face 为该帧对应的转镜面号（frame_index % 4），每面相差 90°。
    void apply(int pixel, std::uint32_t ft, std::uint32_t mirrorAngle, std::uint16_t slot,
               std::uint16_t peak, int face, algo_lidar::PointData &point) const;

    /// 反射率查表（0~100）；无法查到返回 -1。
    float reflectivity(int pixel, float distance, std::uint16_t peak) const;

    static int faceByScale(std::uint32_t mirrorAngle);

private:
    bool parseIni(const std::filesystem::path &path, std::string &error);
    bool parseEncoderAngle(const std::filesystem::path &path, std::string &error);
    bool parseSpotSlope(const std::filesystem::path &path, std::string &error);
    bool parseReflectivity(const std::filesystem::path &path, std::string &error);

    bool loaded_ = false;
    std::filesystem::path directory_;

    int linesV_ = 96;
    int linesH_ = 800;
    int u_ = 47;
    float omega0_ = -60.0f;
    float thetaMin_ = -10.0f;
    float thetaMax_ = 10.0f;
    float thetaTilt_ = -2.5f;

    std::vector<float> omegaDelta_;   // 96
    std::vector<float> dDelta_;       // 96，单位 cm
    std::vector<float> aFactor_;      // 96
    std::vector<float> theta_;        // 96，单位度
    std::array<float, 4> startingAngle_{};
    std::vector<float> encoderAngle_; // 240

    std::vector<ReflectivityRow> reflectivityRows_;
    std::map<int, std::vector<ReflectivityRow>> reflectivityByPixel_;
};

} // namespace cloud_viewer
