/**
 * @brief CS96 Beta 标定路径与纯几何路径的对照测试。
 *
 * 关键约束：
 *   1. 单文件聚合 CSV 必须能被加载，且解析出的面起始角与目录布局一致；
 *   2. applyRaw() 完全不依赖标定表（光速距离、slot 线性水平角、峰值反射率）；
 *   3. 同一回波在“启用标定”与“不启用标定”下必须得到不同结果。
 */

#include "cloud_viewer/preprocess/Cs96BetaCalibration.h"

#include <cmath>
#include <cstdio>
#include <filesystem>
#include <string>

namespace
{

constexpr float kLightSpeed = 299792458.0f * 1e-12f; // 米/皮秒

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

    cloud_viewer::Cs96BetaCalibration calibration;
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
    check("start_face0", std::fabs(starts[0] - 356.984f) < 0.05f, std::to_string(starts[0]));
    check("start_face1", std::fabs(starts[1] - 87.3658f) < 0.05f, std::to_string(starts[1]));

    // 固定一条回波：line=32、ft=1000、slot=400、peak=800、face=0。
    // mirrorAngle 取 scale=238、realAngle=0，即恰好落在该面起点。
    const int line = 32;
    const std::uint32_t ft = 1000;
    const std::uint16_t slot = 400;
    const std::uint16_t peak = 800;
    const std::uint32_t mirrorAngle = (238u << 12);

    algo_lidar::PointData raw{};
    algo_lidar::PointData calibrated{};
    cloud_viewer::Cs96BetaCalibration::applyRaw(line, ft, slot, peak, 0, raw);
    calibration.apply(line, ft, mirrorAngle, slot, peak, 0, calibrated);

    // 纯几何：r 只由光速决定，omega 只由 slot 决定，反射率就是峰值。
    const float expectedR = kLightSpeed * static_cast<float>(ft >> 1);
    const float expectedOmega = -60.0f + 120.0f * slot / 800.0f;
    check("raw_r", std::fabs(raw.r - expectedR) < 1e-3f,
          std::to_string(raw.r) + " vs " + std::to_string(expectedR));
    check("raw_omega", std::fabs(raw.slot_ang - expectedOmega) < 1e-3f,
          std::to_string(raw.slot_ang) + " vs " + std::to_string(expectedOmega));
    check("raw_reflect", std::fabs(raw.reflect - static_cast<float>(peak)) < 1e-3f,
          std::to_string(raw.reflect));

    // 标定路径必须引入差异：距离含 d_delta 零点偏移，水平角走编码器表。
    check("calibrated_r_differs", std::fabs(calibrated.r - raw.r) > 1e-3f,
          std::to_string(calibrated.r) + " vs raw " + std::to_string(raw.r));
    check("calibrated_omega_differs", std::fabs(calibrated.slot_ang - raw.slot_ang) > 1.0f,
          std::to_string(calibrated.slot_ang) + " vs raw " + std::to_string(raw.slot_ang));
    // 单面视场约束：标定后的水平角仍应落在 [-60, 60] 附近。
    check("calibrated_omega_range",
          calibrated.slot_ang >= -61.0f && calibrated.slot_ang <= 61.0f,
          std::to_string(calibrated.slot_ang));

    const bool pass = failures == 0;
    std::printf("BETA_CALIB_TEST_%s failures=%d\n", pass ? "PASS" : "FAIL", failures);
    return pass ? 0 : 1;
}
