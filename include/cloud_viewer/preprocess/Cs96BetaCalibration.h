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
 *      ys_calibration.ini      omega_delta[96]、d_delta[96]（m）、a[96]、theta 参数、面起始角
 *      ys_encoder_angle.csv    scale 0~240 -> 0~360 度整圈编码器表
 *      reflectivity_map.csv    pixel_target,distance,reflectance_0..4 —— 反射率查表
 *      spot_slope_map.csv      Face,Angle_FPGA,Diff —— 按面角度曲线（暂未参与解算）
 *
 * 计算公式（与 you_see_arm 一致）：
 *   r     = kLightSpeed * (ft >> 1) * a[pixel] + d_delta[pixel]
 *   theta = (line - u) * (theta_max - theta_min) / lines_v + theta_tilt
 *   face  = 接收层按帧起始 PA 判定并锁定的面号
 *   hc    = encoder_angle[scale] + 0.0009 * real_angle - 本面起点角
 *   omega = wrap180(hc) * 2 + omega0 + omega_delta[pixel]
 *
 * encoder_angle 支持两种布局，load() 时按表值范围自动判定（见 anchorPerFace()）：
 *   1) 绝对表（旧）：scale 0~240 -> 0~360° 码盘机械角，单调递增；
 *      本面起点角 = startingAngle[face]（ini 的 angle_face_N + side_angular_offset 细调）。
 *   2) 每面锚定表（20261117 起，标定端 anchor_per_face=true）：每面从锚点（默认 -30°机械/-60°视场）
 *      重新起算，表值可负、回绕行可超过 360°，因此四个面段互不连续；
 *      本面起点角 = 该面起始刻度在表里的值 tableStart[face]（细调已折算进表值）。
 *   两种布局下 hc 的物理含义相同（相对本面起爆点的机械转角），故 omega 公式不变。
 *   绝对角 = 表值 + dialBase[face]（仅每面锚定表需要，用于 paAngle() 还原 PA 判面）。
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

    /// 原始点云解算的四项标定开关，与 you_see_arm 中四份可缺失的标定数据一一对应。
    /// 关闭某项就等于该数据源缺失时设备端的降级行为，其余项不受影响。
    struct DecodeOptions
    {
        /// a[] 距离系数 + d_delta[] 零点：对应 [distance_factor] / [zero_distance_offset]。
        bool distanceCorrection = true;
        /// omega_delta[] 逐线水平角：对应 [horizontal_angular_offset]。
        bool lineOmegaDelta = true;
        /// FPGA 旋转角曲线：对应 spot_slope_map（缺失时 get_beta_spot_slope 原样返回）。
        bool spotSlope = true;
        /// 反射率查表：对应 reflectivity_map（缺失时退回 get_intensity_by_peak）。
        bool reflectivityMap = true;

        /// 水平角解算方案：false = Beta（查表 + 面起始角），true = Alpha（编码盘差分 + 帧中点基准）。
        /// 这不是"修正开关"，而是两代固件对同一物理量的两套算法，仅供对照观察。
        /// Alpha 方案依赖 spot_slope_map（设备端是必经环节），spotSlope 关闭时不修正。
        bool alphaHorizontalAngle = false;

        bool any() const
        {
            return distanceCorrection || lineOmegaDelta || spotSlope || reflectivityMap;
        }

        static DecodeOptions allOff()
        {
            DecodeOptions options;
            options.distanceCorrection = false;
            options.lineOmegaDelta = false;
            options.spotSlope = false;
            options.reflectivityMap = false;
            return options;
        }
    };

    Cs96BetaCalibration();

    /// 加载标定数据。
    /// path 为单文件 CSV（tools/pack_cs96beta_calibration.py 生成）时读聚合表；
    /// path 为目录时回退读取传统的 ini + 多 csv 布局。
    bool load(const std::filesystem::path &path, std::string &error);
    bool loaded() const { return loaded_; }
    const std::filesystem::path &directory() const { return directory_; }

    /// 把当前标定写回单文件聚合表，格式与 tools/pack_cs96beta_calibration.py 一致。
    /// 用于「选一个目录标定包，一键更新 cs96beta_calibration.csv」。
    /// 只导出解算实际用到的 section；vcsel_index / light_intensity 不参与几何，不写回。
    ///
    /// base 用于「目录里缺某一份数据」时继承旧表：例如新标定包没有
    /// reflectivity_map.csv，此时沿用 base 的反射率，避免覆盖后反射率查表失效。
    bool exportUnifiedCsv(const std::filesystem::path &path, std::string &error,
                          const Cs96BetaCalibration *base = nullptr) const;

    /// 标定表记录的规格：垂直线数 / 每帧 slot 数。
    /// 96×800（IMX459）与 120×800（SL100）的标定表不能混用，连接时需比对。
    int lineCount() const { return linesV_; }
    int slotCount() const { return linesH_; }

    /// 俯仰中心线 u（SPAD 中心线）：供无法套用整张标定的兜底解算使用。
    /// Gamma（120 线）的 u 仍是 47，不能按 (lines-1)/2 推导。
    int thetaCenterLine() const { return u_; }

    /// 由原始点云的一回波计算笛卡尔坐标与反射率。
    /// face 由独立的 PA 判面层传入，每面相差 90°；本类不负责判面。
    /// 结构参数（编码器表、面起始角、omega0、theta、sat_peak_max）始终生效，
    /// 只有 options 里显式关闭的标定项才降级。
    void apply(int pixel, std::uint32_t ft, std::uint32_t mirrorAngle, std::uint16_t slot,
               std::uint16_t peak, int face, const DecodeOptions &options,
               algo_lidar::PointData &point) const;

    /// 反射率查表（0~100）；无法查到返回 -1。
    float reflectivity(int pixel, float distance, std::uint16_t peak) const;

    /// FPGA 旋转角修正（度）：复刻 calibRotateAngle::repairRotateAngle。
    /// 曲线缺失或不含该面时原样返回。
    float spotSlopeCorrection(float angleDeg, int face) const;

    const std::array<float, 4> &startingAngles() const { return startingAngle_; }

    /// 编码器表是否为"每面锚定"布局。
    /// load() 时自动判定：表里出现负数或 >360° 的回绕行 → true（即标定端 anchor_per_face=true）。
    /// 该布局下每个面的表段都从 anchorStartMechDeg()（默认 -30°）起算，四个面段互不连续。
    bool anchorPerFace() const { return anchorPerFace_; }

    /// 手动覆盖自动判定（调试/对照用）。会同步重算 tableStart / dialBase 派生量。
    void setAnchorPerFace(bool on);

    /// 每面锚定表使用的机械锚点（度），必须与标定端的 anchor_start_mech_angle 一致，默认 -30°。
    float anchorStartMechDeg() const { return anchorStartMechDeg_; }
    void setAnchorStartMechDeg(float deg);

    /// 是否加载了每格实测耗时（time_intervals.txt）。
    /// false 时 apply() 的细角度退回名义间隔 kNominalIntervalUs。
    bool hasTimeIntervals() const { return !timeIntervals_.empty(); }

    /// PA 绝对角（0~360°）。
    float paAngle(int scale, int realAngle) const;

    /// 由 PA 角唯一判定转镜面号（0~3）；无法唯一判定返回 -1。
    int faceByPa(float paAngleDeg) const;

private:
    /// 由 startingAngle_ / standardEncoder_ / encoderAngle_ 派生每面锚定表的两个量：
    ///   tableStart[face] = 该面起始刻度在表里的值（每面锚定表下即本面起爆点）
    ///   dialBase[face]   = 绝对角 - 表值，用于 paAngle() 还原 PA 判面用的码盘绝对角
    void refreshAnchorDerived();

    /// 刻度落在哪个面段（按 standardEncoder_ 分段，环形取最近的前一个起点）。
    int faceOfScale(int scale) const;

    bool parseIni(const std::filesystem::path &path, std::string &error);
    bool parseEncoderAngle(const std::filesystem::path &path, std::string &error);
    /// 读取 time_intervals.txt：每个粗刻度到下一个刻度的实测耗时（us，240 项）。
    /// 可接受 239 行（末格用均值补）、241 行（首行是 FPGA t0，自动丢弃）。
    bool parseTimeIntervals(const std::filesystem::path &path, std::string &error);
    /// 第 scale 格的名义/实测耗时（us）；缺表时返回 kNominalIntervalUs。
    float timeIntervalUs(int scale) const;
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
    /// ys_calibration.ini 的 sat_peak_max，反射率峰值归一化的上限。
    std::uint16_t satPeakMax_ = 1612;
    /// ys_calibration.ini 的 sat_peak_min，低于该峰值判为无效回波（设备端同规则）。
    std::uint16_t satPeakMin_ = 0;
    /// ys_calibration.ini 的 dist_max（米）：有效点的距离上限，不要再硬编码 200。
    float distMax_ = 200.0f;
    float thetaMin_ = -10.0f;
    float thetaMax_ = 10.0f;
    float thetaTilt_ = -2.5f;

    std::vector<float> omegaDelta_;   // 96
    std::vector<float> dDelta_;       // 96，单位 m
    std::vector<float> aFactor_;      // 96
    std::vector<float> theta_;        // 96，单位度
    std::array<float, 4> startingAngle_{};
    std::array<float, 4> standardEncoder_{};
    std::array<float, 4> sideAngularOffset_{};
    std::vector<float> encoderAngle_; // 241，含 scale=240 边界
    /// 编码器表布局：true = 每面锚定表（表值为表面局部角，见 anchorPerFace()）。
    bool anchorPerFace_ = false;
    /// 每面锚定表的机械锚点（度），标定端 anchor_start_mech_angle 的镜像。
    float anchorStartMechDeg_ = -30.0f;
    /// 每面起始刻度在表里的值（每面锚定表下 = 本面起爆点，细调已折算在内）。
    std::array<float, 4> tableStart_{};
    /// 绝对角与表值之差：绝对角 = encoderAngle_[scale] + dialBase_[face]。
    std::array<float, 4> dialBase_{};

    /// 每个粗刻度到下一个刻度的实测耗时（us，240 项，scale 0..239）。
    /// 来自 time_intervals.txt；为空时 apply() 退回 kNominalIntervalUs。
    std::vector<float> timeIntervals_;

    /// 按面的角度旋转曲线（spot_slope_map）：face -> (FPGA 角, 修正量)。
    /// 目前只做保存与访问，未参与 apply()，便于后续对齐下位机 get_beta_spot_slope()。
    std::array<std::vector<std::pair<float, float>>, 4> spotSlope_;

    std::vector<ReflectivityRow> reflectivityRows_;

    /// 聚合 CSV 里本类不解析的 section 原始行（vcsel_index / light_intensity 等）。
    /// 重新导出时原样写回，否则「用目录更新一次」就会把这些段落整段丢掉。
    std::vector<std::string> passthroughRows_;
    std::map<int, std::vector<ReflectivityRow>> reflectivityByPixel_;
};

} // namespace cloud_viewer
