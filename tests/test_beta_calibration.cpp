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

    // 面起始角来自标定表（angle_face + side_angular_offset），具体数值随标定版本变化，
    // 这里只校验结构关系：4 面依次相差 90°，避免每次换表都要改断言。
    const auto &starts = calibration.startingAngles();
    const auto wrapDelta = [](float delta)
    {
        while (delta > 180.0f) delta -= 360.0f;
        while (delta < -180.0f) delta += 360.0f;
        return delta;
    };
    check("start_face0_finite", std::isfinite(starts[0]), std::to_string(starts[0]));
    for (int face = 1; face < 4; ++face)
    {
        const float expected = 90.0f * static_cast<float>(face);
        const float actual = wrapDelta(starts[face] - starts[0]);
        // 180°/270° 会被折到 ±180 内，比较时对差值再折一次。
        check((std::string("start_spacing_") + std::to_string(face)).c_str(),
              std::fabs(wrapDelta(actual - expected)) < 2.0f,
              std::to_string(actual) + " vs " + std::to_string(expected));
    }

    // 固定一条回波：line=32、ft=60000、slot=400、peak=800、face=0。
    // mirrorAngle 取 scale=30、realAngle=0：落在第 0 面区间内（面内约 11.8°），
    // 换标定表时只需确认该 scale 仍属于 face 0。
    const int line = 32;
    const std::uint32_t ft = 60000;
    const std::uint16_t slot = 400;
    const std::uint16_t peak = 800;
    const std::uint32_t mirrorAngle = (30u << 12);

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
    // 转镜雷达的 [zero_distance_offset] 单位是米，不加任何换算（line=32 的标定值）。
    check("on_r_differs", std::fabs((on.r - off.r) - (-5.8525f)) < 1e-3f,
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
