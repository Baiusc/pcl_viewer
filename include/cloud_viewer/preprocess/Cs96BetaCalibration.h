/**
 * @brief CS96 Beta 原始点云标定（距离 / 水平角 / 俯仰角 / 反射率）
 *
 * 只作用于 work_mode=1 的原始点云；work_mode=2 由雷达端直接输出 xyz，无需本标定。
 *
 * 标定数据目录包含：
 *   ys_calibration.ini      omega_delta[96]、d_delta[96]（cm）、a[96]、theta 参数、面起始角
 *   ys_encoder_angle.csv    scale 0~240 -> 0~360 度整圈编码器表
 *   reflectivity_map.csv    pixel_target,distance,reflectance_0..4 —— 反射率查表
 *
 * 计算公式（与 you_see_arm 一致）：
 *   r     = kLightSpeed * (ft >> 1) * a[pixel] + d_delta[pixel] * 0.01
 *   theta = (line - u) * (theta_max - theta_min) / lines_v + theta_tilt
 *   face  = 接收层按帧起始 PA 判定并锁定的面号
 *   hc    = encoder_angle[scale] + 0.0009 * real_angle - starting_angle[face]
 *   omega = wrap180(hc) * 2 + omega0 + face * 90 + omega_delta[pixel]
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
    /// face 由独立的 PA 判面层传入，每面相差 90°；本类不负责判面。
    void apply(int pixel, std::uint32_t ft, std::uint32_t mirrorAngle, std::uint16_t slot,
               std::uint16_t peak, int face, algo_lidar::PointData &point) const;

    /// 反射率查表（0~100）；无法查到返回 -1。
    float reflectivity(int pixel, float distance, std::uint16_t peak) const;

    const std::array<float, 4> &startingAngles() const { return startingAngle_; }

    /// PA 绝对角（0~360°）。
    float paAngle(int scale, int realAngle) const;

    /// 由 PA 角唯一判定转镜面号（0~3）；无法唯一判定返回 -1。
    int faceByPa(float paAngleDeg) const;

private:
    bool parseIni(const std::filesystem::path &path, std::string &error);
    bool parseEncoderAngle(const std::filesystem::path &path, std::string &error);
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
    std::array<float, 4> standardEncoder_{};
    std::array<float, 4> sideAngularOffset_{};
    std::vector<float> encoderAngle_; // 241，含 scale=240 边界

    std::vector<ReflectivityRow> reflectivityRows_;
    std::map<int, std::vector<ReflectivityRow>> reflectivityByPixel_;
};

} // namespace cloud_viewer
