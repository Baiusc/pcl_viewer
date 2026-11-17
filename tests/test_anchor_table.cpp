/**
 * @brief 编码器表两种布局（绝对角 / 每面锚定）的加载与解算对照测试。
 *
 * 标定端（LightSpotSlope / measure_light_spot）自 20261117 起支持 anchor_per_face：
 * 每个面段从固定锚点（机械 -30° / 视场 -60°）重新起算，细调 side_angular_offset 已折进表值，
 * 于是表里会出现负数、第 240 行回绕行会超过 360°。本测试：
 *   1. 现造一份这样的目录标定包（ini + ys_encoder_angle.csv），验证自动判定为每面锚定；
 *   2. 验证起爆刻度处 omega = omega0（-60°）、走 20 格后 omega = 0°；
 *   3. 验证 paAngle() 仍能还原成码盘绝对角、faceByPa() 判面正确；
 *   4. 若仓库自带旧版绝对角聚合表，再确认它被判成"绝对表"。
 */

#include "cloud_viewer/preprocess/Cs96BetaCalibration.h"

#include <cmath>
#include <cstdio>
#include <filesystem>
#include <fstream>
#include <string>
#include <vector>

namespace
{

using cloud_viewer::Cs96BetaCalibration;
using Options = Cs96BetaCalibration::DecodeOptions;

constexpr int kScales = 240;
constexpr int kFaces = 4;
constexpr float kSpacing = 1.5f;        // 码盘机械刻度间隔
constexpr float kAnchorMech = -30.0f;   // 标定端的 anchor_start_mech_angle
constexpr int kStartTicks[kFaces] = {22, 82, 142, 202};
constexpr int kSideOffsets[kFaces] = {22, 0, 33, 33};
constexpr int kLitBars = 40;

int gFailures = 0;

void check(const char *name, bool ok, const std::string &detail)
{
    std::printf("  %-26s %s  %s\n", name, ok ? "PASS" : "FAIL", detail.c_str());
    if (!ok)
    {
        ++gFailures;
    }
}

/// 按标定端的规则生成表：每面首根光条锚定为 -30°-细调，其后 1.5°/格，未发光刻度按 1.5° 递推。
std::vector<float> buildPerFaceTable()
{
    std::vector<float> table(kScales, 0.0f);
    std::vector<bool> measured(kScales, false);

    for (int face = 0; face < kFaces; ++face)
    {
        const float side = static_cast<float>(kSideOffsets[face]) * kSpacing / 256.0f;
        const float anchor = kAnchorMech - side;
        for (int k = 0; k < kLitBars; ++k)
        {
            const int tick = (kStartTicks[face] + k) % kScales;
            table[tick] = anchor + kSpacing * static_cast<float>(k);
            measured[tick] = true;
        }
    }

    // 未实测刻度：从最大的实测刻度之后逐格递推（与标定端 unlit_fill_step=1.5 一致）
    int seed = -1;
    for (int i = 0; i < kScales; ++i)
        if (measured[i]) seed = i;
    float cur = table[seed];
    for (int k = 1; k <= kScales; ++k)
    {
        const int tick = (seed + k) % kScales;
        if (measured[tick])
        {
            cur = table[tick];
        }
        else
        {
            cur += kSpacing;
            table[tick] = cur;
        }
    }
    return table;
}

bool writePerFacePackage(const std::filesystem::path &dir, std::string &error)
{
    std::error_code ec;
    std::filesystem::remove_all(dir, ec);
    std::filesystem::create_directories(dir, ec);
    if (ec)
    {
        error = "无法创建临时目录: " + ec.message();
        return false;
    }

    {
        std::ofstream ini(dir / "ys_calibration.ini");
        if (!ini.is_open())
        {
            error = "无法写 ys_calibration.ini";
            return false;
        }
        ini << "[standard_encoder]\n";
        for (int face = 0; face < kFaces; ++face)
            ini << "angle_face_" << (face + 1) << " = \"" << kStartTicks[face] << "\"\n";
        ini << "\n[side_angular_offset]\n";
        for (int face = 0; face < kFaces; ++face)
            ini << "side" << face << "_angular_offset = \"" << kSideOffsets[face] << "\"\n";
    }

    {
        const std::vector<float> table = buildPerFaceTable();
        std::ofstream csv(dir / "ys_encoder_angle.csv");
        if (!csv.is_open())
        {
            error = "无法写 ys_encoder_angle.csv";
            return false;
        }
        csv << "scale,angle\n";
        char buffer[64];
        for (int i = 0; i < kScales; ++i)
        {
            std::snprintf(buffer, sizeof(buffer), "%d,%.9f\n", i, table[i]);
            csv << buffer;
        }
        // 回绕行：刻度 240 就是刻度 0，直接复制第 0 行（标定端不再 +360°）
        std::snprintf(buffer, sizeof(buffer), "%d,%.9f\n", kScales, table[0]);
        csv << buffer;
    }
    return true;
}

/// 旧版绝对角聚合表（仓库自带则用之，确认不会被误判成每面锚定）
std::filesystem::path findAbsoluteCsv()
{
#ifdef CV_TEST_DATA_DIR
    const std::filesystem::path direct =
        std::filesystem::path(CV_TEST_DATA_DIR) / "param" / "cs96beta_calibration.csv";
    std::error_code ec;
    if (std::filesystem::exists(direct, ec))
    {
        return direct;
    }
#endif
    return {};
}

} // namespace

int main()
{
    const std::filesystem::path dir =
        std::filesystem::temp_directory_path() / "cv_anchor_table_test";
    std::string error;
    if (!writePerFacePackage(dir, error))
    {
        std::printf("ANCHOR_TEST_FAIL %s\n", error.c_str());
        return 1;
    }

    Cs96BetaCalibration perFace;
    if (!perFace.load(dir, error))
    {
        std::printf("ANCHOR_TEST_FAIL 加载每面锚定包失败: %s\n", error.c_str());
        return 1;
    }

    check("detect_per_face", perFace.anchorPerFace(), "表值含负数");

    // 固定回波：line=32、ft=60000、slot=400；第 4 面起爆刻度 202（第 3 面，从 0 计）
    const int line = 32;
    const std::uint32_t ft = 60000;
    const std::uint16_t slot = 400;
    const std::uint16_t peak = 800;
    const int face = 3;

    auto decode = [&](int scale, int realAngle)
    {
        algo_lidar::PointData point{};
        const std::uint32_t mirrorAngle =
            (static_cast<std::uint32_t>(scale) << 12) | static_cast<std::uint32_t>(realAngle & 0xFFF);
        perFace.apply(line, ft, mirrorAngle, slot, peak, face, Options::allOff(), point);
        return point;
    };

    // 起爆刻度：hc = 0 -> omega = omega0 = -60°，坐标另按左手系取反 -> +60°
    const algo_lidar::PointData atStart = decode(kStartTicks[face], 0);
    check("omega_at_start", std::fabs(std::fabs(atStart.slot_ang) - 60.0f) < 1e-3f,
          std::to_string(atStart.slot_ang));

    // 起爆后 20 格：hc = 30° -> omega = 0°
    const algo_lidar::PointData plus20 = decode(kStartTicks[face] + 20, 0);
    check("omega_plus20", std::fabs(plus20.slot_ang) < 1e-3f, std::to_string(plus20.slot_ang));

    // PA 判面：paAngle 应还原为码盘绝对角 202/240*360 = 303°
    const float pa = perFace.paAngle(kStartTicks[face], 0);
    check("pa_restored", std::fabs(pa - 303.0f) < 0.5f, std::to_string(pa));
    check("face_by_pa", perFace.faceByPa(pa) == face, std::to_string(perFace.faceByPa(pa)));

    // 旧版绝对角表不应被判成每面锚定
    const std::filesystem::path absoluteCsv = findAbsoluteCsv();
    if (!absoluteCsv.empty())
    {
        Cs96BetaCalibration absolute;
        if (absolute.load(absoluteCsv, error))
        {
            check("detect_absolute", !absolute.anchorPerFace(), "旧表 0~360 单调");
        }
        else
        {
            std::printf("  (跳过旧表校验: %s)\n", error.c_str());
        }
    }

    std::error_code ec;
    std::filesystem::remove_all(dir, ec);
    std::printf("RESULT failures=%d\n", gFailures);
    return gFailures ? 1 : 0;
}
