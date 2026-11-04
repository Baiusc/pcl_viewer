/**
 * @brief CS96 Beta 原始点云四项标定开关的对照测试。
 *
 * 对应 you_see_arm 的四份可缺失标定数据：
 *   1. [distance_factor]/[zero_distance_offset]  -> distanceCorrection
 *   2. [horizontal_angular_offset]               -> lineOmegaDelta
 *   3. spot_slope_map                            -> spotSlope
 *   4. reflectivity_map                          -> reflectivityMap
 *
 * 结构参数（编码器表、面起始角、omega0、俯仰角）在任何开关下都保持生效。
 */

#include "cloud_viewer/preprocess/Cs96BetaCalibration.h"

#include <cmath>
#include <cstdio>
#include <filesystem>
#include <string>

namespace
{

using cloud_viewer::Cs96BetaCalibration;
using Options = Cs96BetaCalibration::DecodeOptions;

constexpr float kLightSpeed = 299792458.0f * 1e-12f; // 米/皮秒
constexpr std::uint16_t kSatPeakMax = 1612;
constexpr std::uint16_t kMinPeak = 51;

std::filesystem::path findCalibration()
{
#ifdef CV_TEST_DATA_DIR
    const std::filesystem::path direct =
        std::filesystem::path(CV_TEST_DATA_DIR) / "param" / "cs96beta_calibration.csv";
    std::error_code directError;
    if (std::filesystem::exists(direct, directError))
    {
        return direct;
    }
#endif
    const char *candidates[] = {
        "data/param/cs96beta_calibration.csv",
        "../data/param/cs96beta_calibration.csv",
        "../../data/param/cs96beta_calibration.csv",
        "../../../data/param/cs96beta_calibration.csv",
    };
    for (const char *candidate : candidates)
    {
        std::error_code ec;
        if (std::filesystem::exists(candidate, ec))
        {
            return candidate;
        }
    }
    return {};
}

} // namespace

int main()
{
    const auto source = findCalibration();
    if (source.empty())
    {
        std::printf("CALIB_TEST_SKIP 未找到 cs96beta_calibration.csv\n");
        return 0;
    }

    Cs96BetaCalibration calibration;
    std::string error;
    if (!calibration.load(source, error))
    {
        std::printf("CALIB_TEST_FAIL 加载失败: %s\n", error.c_str());
        return 1;
    }

    int failures = 0;
    auto check = [&failures](const char *name, bool ok, const std::string &detail)
    {
        std::printf("  %-26s %s  %s\n", name, ok ? "PASS" : "FAIL", detail.c_str());
        if (!ok)
        {
            ++failures;
        }
    };

    // 面起始角必须与设备实测值一致（聚合 CSV 与目录布局共用同一算法）。
    const auto &starts = calibration.startingAngles();
    check("start_face0", std::fabs(starts[0] - 357.0f) < 0.05f, std::to_string(starts[0]));
    check("start_face1", std::fabs(starts[1] - 87.4336f) < 0.05f, std::to_string(starts[1]));

    // 固定一条回波：line=32、ft=1000、slot=400、peak=800、face=0。
    // mirrorAngle 取 scale=238、realAngle=0，即恰好落在该面起点。
    const int line = 32;
    const std::uint32_t ft = 60000;
    const std::uint16_t slot = 400;
    const std::uint16_t peak = 800;
    const std::uint32_t mirrorAngle = (238u << 12);

    auto decode = [&](const Options &options)
    {
        algo_lidar::PointData point{};
        calibration.apply(line, ft, mirrorAngle, slot, peak, 0, options, point);
        return point;
    };

    const Options allOff = Options::allOff();
    Options lineOnly = allOff;
    lineOnly.lineOmegaDelta = true;
    Options spotOnly = allOff;
    spotOnly.spotSlope = true;
    const Options allOn{};

    const algo_lidar::PointData off = decode(allOff);
    const algo_lidar::PointData linePoint = decode(lineOnly);
    const algo_lidar::PointData spotPoint = decode(spotOnly);
    const algo_lidar::PointData on = decode(allOn);

    // 距离：关闭 distanceCorrection 等价于 a=1、d=0。
    const float expectedR = kLightSpeed * static_cast<float>(ft >> 1);
    check("off_r", std::fabs(off.r - expectedR) < 1e-4f,
          std::to_string(off.r) + " vs " + std::to_string(expectedR));
    check("on_r_differs", std::fabs((on.r - off.r) - (-5.8716f)) < 1e-3f,
          std::to_string(on.r) + " vs off " + std::to_string(off.r));

    check("distance_on_off_valid", off.is_valid && on.is_valid, "both valid");
    algo_lidar::PointData tooNear{};
    calibration.apply(line, 1000, mirrorAngle, slot, peak, 0, allOn, tooNear);
    check("negative_range_invalid", !tooNear.is_valid, std::to_string(tooNear.r));

    // 水平角：关闭时仍走编码器链，只是不加 omega_delta 与旋转角修正。
    check("off_omega_in_fov", off.slot_ang >= -61.0f && off.slot_ang <= 61.0f,
          std::to_string(off.slot_ang));
    // omega_delta 实测从 -0.002 线性降到 -1.279（96 线），单线量级约 1°。
    check("line_delta_small", std::fabs(linePoint.slot_ang - off.slot_ang) < 2.0f &&
                                  linePoint.slot_ang != off.slot_ang,
          std::to_string(linePoint.slot_ang - off.slot_ang));
    check("spot_delta_small", std::fabs(spotPoint.slot_ang - off.slot_ang) < 2.0f &&
                                  std::fabs(spotPoint.slot_ang - off.slot_ang) > 1e-3f,
          std::to_string(spotPoint.slot_ang - off.slot_ang));

    // 叠加顺序：hc*2 + omega0 -> spot slope -> omega_delta。
    const float expectedOnOmega = spotPoint.slot_ang + (linePoint.slot_ang - off.slot_ang);
    check("omega_order", std::fabs(on.slot_ang - expectedOnOmega) < 1e-3f,
          std::to_string(on.slot_ang) + " vs " + std::to_string(expectedOnOmega));

    // 坐标约定：x+ 雷达正前、y+ 雷达自身右侧、z+ 向上。
    // omega 随 slot 由 -60° 增到 +60°，故 y = +r·cosθ·sinω 让 slot0 落在左侧，
    // 与"左侧目标先扫到"一致。设备内部 y+ 向左，真实点云在 BetaUdpStream 侧取反。
    const float thetaRad = off.theta * (3.14159265358979323846f / 180.0f);
    const float horizontal = off.r * std::cos(thetaRad);
    const float axisTol = 1e-4f * (1.0f + std::fabs(horizontal));
    check("axis_x_front", std::fabs(off.x - horizontal * std::cos(off.omega_rad)) < axisTol,
          std::to_string(off.x));
    check("axis_y_right", std::fabs(off.y - horizontal * std::sin(off.omega_rad)) < axisTol,
          std::to_string(off.y));
    check("axis_z_up",
          std::fabs(off.z - off.r * std::sin(thetaRad)) < 1e-4f * (1.0f + std::fabs(off.r)),
          std::to_string(off.z));

    // 反射率：关闭 reflectivityMap 时走 get_intensity_by_peak，
    //   reflect = (peak - 51) / (sat_peak_max - 51)，换算到 0~100。
    const float expectedReflect =
        100.0f * static_cast<float>(peak - kMinPeak) / static_cast<float>(kSatPeakMax - kMinPeak);
    check("off_reflect", std::fabs(off.reflect - expectedReflect) < 1e-2f,
          std::to_string(off.reflect) + " vs " + std::to_string(expectedReflect));
    check("off_intensity", std::fabs(static_cast<int>(off.intensity) -
                                     static_cast<int>(expectedReflect * 2.55f)) <= 1,
          std::to_string(static_cast<int>(off.intensity)));
    check("on_reflect_differs", std::fabs(on.reflect - off.reflect) > 1e-3f,
          std::to_string(on.reflect) + " vs off " + std::to_string(off.reflect));

    const bool pass = failures == 0;
    std::printf("BETA_CALIB_TEST_%s failures=%d\n", pass ? "PASS" : "FAIL", failures);
    return pass ? 0 : 1;
}
