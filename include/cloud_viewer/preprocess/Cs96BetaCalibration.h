/**
 * @brief CS96 Beta 原始点云标定（距离 / 水平角 / 俯仰角 / 反射率）
 *
 * 只作用于 work_mode=1 的原始点云；work_mode=2 由雷达端直接输出 xyz，无需本标定。
 *
 * 标定数据有两种布局，均由 load() 自动识别：
 *   1) 单文件聚合表 cs96beta_calibration.csv
 *      由 tools/pack_cs96beta_calibration.py 把下列多份文件打包而成，
 *      列格式 section,key,index,value0..value5。
 *   2) 传统目录布局（兼容旧标定包）
 *      ys_calibration.ini      omega_delta[96]、d_delta[96]（cm）、a[96]、theta 参数、面起始角
 *      ys_encoder_angle.csv    scale 0~240 -> 0~360 度整圈编码器表
 *      reflectivity_map.csv    pixel_target,distance,reflectance_0..4 —— 反射率查表
 *      spot_slope_map.csv      Face,Angle_FPGA,Diff —— 按面角度曲线（暂未参与解算）
 *
 * 计算公式（与 you_see_arm 一致）：
 *   r     = kLightSpeed * (ft >> 1) * a[pixel] + d_delta[pixel] * 0.01
 *   theta = (line - u) * (theta_max - theta_min) / lines_v + theta_tilt
 *   face  = 接收层按帧起始 PA 判定并锁定的面号
 *   hc    = encoder_angle[scale] + 0.0009 * real_angle - starting_angle[face]
 *   omega = wrap180(hc) * 2 + omega0 + omega_delta[pixel]
 *
 * 注意：单面视场只有约 120°，omega 落在 [-60, 60]，不含面间 90° 偏移。
 * 关闭标定时由 BetaUdpStream 走纯几何分支（不调用本类的 apply），
 * 距离不做 a/d 修正、水平角按 slot 线性铺开、反射率直接用峰值。
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

    /// 加载标定数据。
    /// path 为单文件 CSV（tools/pack_cs96beta_calibration.py 生成）时读聚合表；
    /// path 为目录时回退读取传统的 ini + 多 csv 布局。
    bool load(const std::filesystem::path &path, std::string &error);
    bool loaded() const { return loaded_; }
    const std::filesystem::path &directory() const { return directory_; }

    /// 由原始点云的一回波计算笛卡尔坐标与反射率（全标定路径）。
    /// face 由独立的 PA 判面层传入，每面相差 90°；本类不负责判面。
    void apply(int pixel, std::uint32_t ft, std::uint32_t mirrorAngle, std::uint16_t slot,
               std::uint16_t peak, int face, algo_lidar::PointData &point) const;

    /// 纯几何路径：不读任何标定表，供“关闭标定”与回归对照使用。
    /// r 直接由光速换算（无 a/d 修正）、omega 按 slot 线性铺开、反射率取峰值。
    static void applyRaw(int pixel, std::uint32_t ft, std::uint16_t slot, std::uint16_t peak,
                         int face, algo_lidar::PointData &point);

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
    bool parseSpotSlope(const std::filesystem::path &path, std::string &error);
    /// 读取 tools/pack_cs96beta_calibration.py 生成的单文件聚合表。
    bool parseUnifiedCsv(const std::filesystem::path &path, std::string &error);
    /// 校验尺寸并派生面起始角、逐线俯仰角。
    bool finalizeGeometry(std::string &error);

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

    /// 按面的角度旋转曲线（spot_slope_map）：face -> (FPGA 角, 修正量)。
    /// 目前只做保存与访问，未参与 apply()，便于后续对齐下位机 get_beta_spot_slope()。
    std::array<std::vector<std::pair<float, float>>, 4> spotSlope_;

    std::vector<ReflectivityRow> reflectivityRows_;
    std::map<int, std::vector<ReflectivityRow>> reflectivityByPixel_;
};

} // namespace cloud_viewer
