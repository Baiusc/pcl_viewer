#include "cloud_viewer/preprocess/Cs96BetaCalibration.h"

#include <algorithm>
#include <cctype>
#include <cmath>
#include <cstdlib>
#include <fstream>
#include <limits>
#include <sstream>

namespace cloud_viewer
{
namespace
{

constexpr float kLightSpeed = 299792458.0f * 1e-12f; // 米/皮秒
constexpr float kDegToRad = 3.14159265358979323846f / 180.0f;
// ys_decode_cs96_beta.c: CS96_BETA_PA_COUNTS_PER_REVOLUTION。
constexpr float kPaCountsPerRevolution = 240.0f;
constexpr int kFaces = 4;

// 纯几何路径使用的光学标称值（非逐设备标定），与 ys_calibration.ini 出厂默认一致。
constexpr int kLinesV = 96;
constexpr int kSlotsPerFace = 800;
constexpr int kNominalPixelCenter = 47;
constexpr float kNominalFov = 120.0f;      // 单面水平视场（度）
constexpr float kNominalOmega0 = -60.0f;   // 水平视场左边界
constexpr float kNominalThetaFov = 20.0f;  // 俯仰视场（度）
constexpr float kNominalThetaTilt = -2.5f; // 俯仰倾斜（度）

std::string trim(const std::string &text)
{
    const auto begin = text.find_first_not_of(" \t\r\n");
    if (begin == std::string::npos)
    {
        return {};
    }
    const auto end = text.find_last_not_of(" \t\r\n");
    return text.substr(begin, end - begin + 1);
}

/// 去掉 ini 的行尾注释（分号后）。
std::string stripComment(const std::string &text)
{
    const auto semicolon = text.find(';');
    auto value = trim(semicolon == std::string::npos ? text : text.substr(0, semicolon));
    if (value.size() >= 2 && value.front() == '"' && value.back() == '"')
        value = value.substr(1, value.size() - 2);
    return value;
}

/// 按逗号拆分一行 CSV，支持双引号包裹的字段。
std::vector<std::string> splitCsv(const std::string &line)
{
    std::vector<std::string> fields;
    std::string current;
    bool quoted = false;
    for (std::size_t i = 0; i < line.size(); ++i)
    {
        const char ch = line[i];
        if (ch == '"')
        {
            // 连续两个引号表示字段内的一个引号。
            if (quoted && i + 1 < line.size() && line[i + 1] == '"')
            {
                current.push_back('"');
                ++i;
            }
            else
            {
                quoted = !quoted;
            }
        }
        else if (ch == ',' && !quoted)
        {
            fields.push_back(current);
            current.clear();
        }
        else
        {
            current.push_back(ch);
        }
    }
    fields.push_back(current);
    return fields;
}

} // namespace

Cs96BetaCalibration::Cs96BetaCalibration()
{
    omegaDelta_.assign(linesV_, 0.0f);
    dDelta_.assign(linesV_, 0.0f);
    aFactor_.assign(linesV_, 1.0f);
    theta_.assign(linesV_, 0.0f);
    encoderAngle_.assign(kFaces * 60, 0.0f);
}

bool Cs96BetaCalibration::load(const std::filesystem::path &path, std::string &error)
{
    *this = Cs96BetaCalibration();
    directory_ = path;
    error.clear();

    std::error_code ec;
    if (std::filesystem::is_directory(path, ec))
    {
        // 传统布局：ini + 多个 csv（保留兼容旧标定包）。
        if (!parseIni(path / "ys_calibration.ini", error))
        {
            return false;
        }
        // spot_slope_map 不是整圈编码器表，缺表直接报告，避免静默使用错误几何。
        if (!parseEncoderAngle(path / "ys_encoder_angle.csv", error)) return false;
        parseReflectivity(path / "reflectivity_map.csv", error);
        parseSpotSlope(path / "spot_slope_map.csv", error);
    }
    else
    {
        // 单文件聚合表（tools/pack_cs96beta_calibration.py 生成）。
        if (!parseUnifiedCsv(path, error)) return false;
    }

    return finalizeGeometry(error);
}

bool Cs96BetaCalibration::finalizeGeometry(std::string &error)
{
    if (linesV_ != 96 || linesH_ != 800)
    {
        error = "CS96 Beta 标定尺寸必须为 96x800";
        return false;
    }

    // 面起始角：设备实测 face0~3 = 357.00 / 87.43 / 177.54 / 267.03，
    // 对应编码器 scale 238 / 58 / 118 / 178，即 (238 + face*60) % 240。
    // 仓库里的 standard_encoder 公式在本设备上与其他面不吻合，改用整圈表插值（误差 < 0.6°）。
    for (int face = 0; face < kFaces; ++face)
    {
        const int scale = (238 + face * 60) % 240;
        if (scale < static_cast<int>(encoderAngle_.size()))
        {
            startingAngle_[face] = encoderAngle_[scale];
        }
        else
        {
            startingAngle_[face] = (standardEncoder_[face] * 256.0f +
                                    sideAngularOffset_[face]) / 256.0f * 360.0f /
                                   kPaCountsPerRevolution;
        }
    }

    // 俯仰角：与下位机 CalculateTheta 一致（像素倒序映射）。
    const float step = (thetaMax_ - thetaMin_) / static_cast<float>(linesV_);
    for (int i = linesV_ - 1, j = 0; i >= 0; --i, ++j)
    {
        theta_[j] = static_cast<float>(i - u_) * step + thetaTilt_;
    }

    loaded_ = true;
    return true;
}

bool Cs96BetaCalibration::parseIni(const std::filesystem::path &path, std::string &error)
{
    std::ifstream input(path);
    if (!input.is_open())
    {
        error = "无法打开 " + path.string();
        return false;
    }

    std::array<bool, kFaces> encoderPresent{}, offsetPresent{};
    std::string section;
    std::string line;
    while (std::getline(input, line))
    {
        line = trim(line);
        if (line.empty() || line[0] == ';' || line[0] == '#')
        {
            continue;
        }
        if (line.front() == '[' && line.back() == ']')
        {
            section = line.substr(1, line.size() - 2);
            continue;
        }

        const auto equal = line.find('=');
        if (equal == std::string::npos)
        {
            continue;
        }
        const std::string key = trim(line.substr(0, equal));
        const std::string value = stripComment(line.substr(equal + 1));
        if (value.empty())
        {
            continue;
        }

        const int index = [&]() -> int
        {
            std::string digits;
            for (char ch : key)
            {
                if (std::isdigit(static_cast<unsigned char>(ch)))
                {
                    digits.push_back(ch);
                }
            }
            return digits.empty() ? -1 : std::atoi(digits.c_str());
        }();

        if (section == "parameter")
        {
            if (key == "lines_v") linesV_ = std::atoi(value.c_str());
            else if (key == "lines_h") linesH_ = std::atoi(value.c_str());
            else if (key == "u") u_ = std::atoi(value.c_str());
            else if (key == "omega0") omega0_ = std::stof(value);
            else if (key == "theta_min") thetaMin_ = std::stof(value);
            else if (key == "theta_max") thetaMax_ = std::stof(value);
            else if (key == "theta_tilt") thetaTilt_ = std::stof(value);
        }
        else if (section == "horizontal_angular_offset" && index >= 0 && index < 96)
        {
            omegaDelta_[index] = std::stof(value);
        }
        else if (section == "zero_distance_offset" && index >= 0 && index < 96)
        {
            dDelta_[index] = std::stof(value); // 单位 cm
        }
        else if (section == "distance_factor" && index >= 0 && index < 96)
        {
            aFactor_[index] = std::stof(value);
        }
        else if (section == "side_angular_offset" && index >= 0 && index < kFaces)
        {
            sideAngularOffset_[index] = std::stof(value);
            offsetPresent[index] = true;
        }
        else if (section == "standard_encoder")
        {
            if (key.rfind("angle_face_", 0) == 0 && index >= 1 && index <= kFaces)
            {
                standardEncoder_[index - 1] = std::stof(value);
                encoderPresent[index - 1] = true;
            }
        }
    }

    if (linesV_ != 96 || linesH_ != 800)
    {
        error = "CS96 Beta 标定尺寸必须为 96x800";
        return false;
    }
    for (int face = 0; face < kFaces; ++face)
    {
        if (!encoderPresent[face] || !offsetPresent[face] ||
            !std::isfinite(standardEncoder_[face]) || standardEncoder_[face] < 0 ||
            standardEncoder_[face] > kPaCountsPerRevolution ||
            !std::isfinite(sideAngularOffset_[face]))
        {
            error = "缺少面参数或 standard_encoder 不是 Beta PA 计数 (0~240)";
            return false;
        }
    }
    return true;
}

bool Cs96BetaCalibration::parseEncoderAngle(const std::filesystem::path &path, std::string &error)
{
    std::ifstream input(path);
    if (!input.is_open())
    {
        error = "无法打开 " + path.string();
        return false;
    }

    std::vector<float> angles(241, std::numeric_limits<float>::quiet_NaN());
    std::string line;
    while (std::getline(input, line))
    {
        std::stringstream row(line);
        int scale;
        char comma;
        float angle;
        if (!(row >> scale >> comma >> angle) || comma != ',') continue;
        if (scale < 0 || scale > 240 || !std::isfinite(angle) ||
            angle < 0.0f || angle > 360.0f || std::isfinite(angles[scale]))
        {
            error = "编码器角度表含无效或重复 scale";
            return false;
        }
        angles[scale] = angle;
    }
    if (std::any_of(angles.begin(), angles.end(), [](float a) { return !std::isfinite(a); }))
    {
        error = "编码器角度表必须包含 scale 0~240";
        return false;
    }
    encoderAngle_ = std::move(angles);
    return true;
}

bool Cs96BetaCalibration::parseSpotSlope(const std::filesystem::path &path, std::string &error)
{
    std::ifstream input(path);
    if (!input.is_open())
    {
        // spot_slope 为可选项，缺失不影响基础几何。
        error.clear();
        return false;
    }

    std::string line;
    bool headerSkipped = false;
    while (std::getline(input, line))
    {
        line = trim(line);
        if (line.empty())
        {
            continue;
        }
        if (!headerSkipped)
        {
            headerSkipped = true;
            if (line.find("Face") != std::string::npos)
            {
                continue;
            }
        }

        std::stringstream row(line);
        std::string faceText, angleText, diffText;
        if (!std::getline(row, faceText, ',') || !std::getline(row, angleText, ',') ||
            !std::getline(row, diffText, ','))
        {
            continue;
        }
        faceText = trim(faceText);
        if (!faceText.empty() && (faceText.front() == 'F' || faceText.front() == 'f'))
        {
            faceText.erase(0, 1);
        }
        if (faceText.empty() || !std::isdigit(static_cast<unsigned char>(faceText.front())))
        {
            continue;
        }
        const int face = std::atoi(faceText.c_str()) - 1;
        if (face < 0 || face >= kFaces)
        {
            continue;
        }
        spotSlope_[face].emplace_back(std::strtof(angleText.c_str(), nullptr),
                                      std::strtof(diffText.c_str(), nullptr));
    }
    return true;
}

bool Cs96BetaCalibration::parseUnifiedCsv(const std::filesystem::path &path, std::string &error)
{
    std::ifstream input(path);
    if (!input.is_open())
    {
        error = "无法打开 " + path.string();
        return false;
    }

    // encoder 表必须覆盖 scale 0~240，缺失会让面起始角整体错位。
    std::vector<float> angles(241, std::numeric_limits<float>::quiet_NaN());
    bool headerSkipped = false;
    std::string line;
    while (std::getline(input, line))
    {
        if (!line.empty() && line.back() == '\r')
        {
            line.pop_back();
        }
        if (line.empty())
        {
            continue;
        }

        const auto fields = splitCsv(line);
        if (fields.size() < 4)
        {
            continue;
        }
        if (!headerSkipped)
        {
            headerSkipped = true;
            if (trim(fields[0]) == "section")
            {
                continue;
            }
        }

        const std::string section = trim(fields[0]);
        const std::string key = trim(fields[1]);
        const std::string indexText = trim(fields[2]);
        const std::string value0 = trim(fields[3]);
        if (value0.empty())
        {
            continue;
        }

        if (section == "parameter")
        {
            if (key == "lines_v") linesV_ = std::atoi(value0.c_str());
            else if (key == "lines_h") linesH_ = std::atoi(value0.c_str());
            else if (key == "u") u_ = std::atoi(value0.c_str());
            else if (key == "omega0") omega0_ = std::strtof(value0.c_str(), nullptr);
            else if (key == "theta_min") thetaMin_ = std::strtof(value0.c_str(), nullptr);
            else if (key == "theta_max") thetaMax_ = std::strtof(value0.c_str(), nullptr);
            else if (key == "theta_tilt") thetaTilt_ = std::strtof(value0.c_str(), nullptr);
        }
        else if (section == "horizontal_angular_offset" || section == "zero_distance_offset" ||
                 section == "distance_factor")
        {
            if (indexText.empty())
            {
                continue;
            }
            const int index = std::atoi(indexText.c_str());
            const float value = std::strtof(value0.c_str(), nullptr);
            // 各表长度固定为 linesV_，逐表判断以兼容 CSV 中段落的任意顺序。
            if (section == "horizontal_angular_offset" &&
                index >= 0 && index < static_cast<int>(omegaDelta_.size()))
            {
                omegaDelta_[index] = value;
            }
            else if (section == "zero_distance_offset" &&
                     index >= 0 && index < static_cast<int>(dDelta_.size()))
            {
                dDelta_[index] = value;
            }
            else if (section == "distance_factor" &&
                     index >= 0 && index < static_cast<int>(aFactor_.size()))
            {
                aFactor_[index] = value;
            }
        }
        else if (section == "side_angular_offset")
        {
            const int index = std::atoi(indexText.c_str());
            if (index >= 0 && index < kFaces)
            {
                sideAngularOffset_[index] = std::strtof(value0.c_str(), nullptr);
            }
        }
        else if (section == "standard_encoder")
        {
            // key 形如 angle_face_N，index 已是 N-1。
            const int index = std::atoi(indexText.c_str());
            if (index >= 0 && index < kFaces)
            {
                standardEncoder_[index] = std::strtof(value0.c_str(), nullptr);
            }
        }
        else if (section == "encoder_angle")
        {
            const int scale = std::atoi(indexText.c_str());
            if (scale < 0 || scale > 240 || std::isfinite(angles[scale]))
            {
                error = "编码器角度表含越界或重复 scale: " + indexText;
                return false;
            }
            angles[scale] = std::strtof(value0.c_str(), nullptr);
        }
        else if (section == "reflectivity")
        {
            ReflectivityRow row;
            row.pixel = std::atoi(indexText.c_str());
            row.distance = std::strtof(value0.c_str(), nullptr);
            for (int i = 0; i < 5 && 4 + i < static_cast<int>(fields.size()); ++i)
            {
                const std::string &text = fields[4 + i];
                if (!trim(text).empty())
                {
                    row.reference[i] = std::strtof(text.c_str(), nullptr);
                }
            }
            reflectivityRows_.push_back(row);
        }
        else if (section == "spot_slope")
        {
            const int face = std::atoi(indexText.c_str()) - 1;
            if (face >= 0 && face < kFaces && fields.size() > 4 && !trim(fields[4]).empty())
            {
                spotSlope_[face].emplace_back(std::strtof(value0.c_str(), nullptr),
                                              std::strtof(fields[4].c_str(), nullptr));
            }
        }
        // vcsel_index / light_intensity 等仅保存于 CSV，暂不参与几何。
    }

    if (std::any_of(angles.begin(), angles.end(), [](float a) { return !std::isfinite(a); }))
    {
        error = "编码器角度表必须包含 scale 0~240: " + path.string();
        return false;
    }
    encoderAngle_ = std::move(angles);

    // 反射率按像素建索引并按距离升序，与旧的 parseReflectivity 行为一致。
    reflectivityByPixel_.clear();
    for (const auto &row : reflectivityRows_)
    {
        reflectivityByPixel_[row.pixel].push_back(row);
    }
    for (auto &entry : reflectivityByPixel_)
    {
        std::sort(entry.second.begin(), entry.second.end(),
                  [](const ReflectivityRow &a, const ReflectivityRow &b)
                  { return a.distance < b.distance; });
    }
    return true;
}

bool Cs96BetaCalibration::parseReflectivity(const std::filesystem::path &path, std::string &error)
{
    std::ifstream input(path);
    if (!input.is_open())
    {
        error = "无法打开 " + path.string();
        return false;
    }

    reflectivityRows_.clear();
    reflectivityByPixel_.clear();

    std::string line;
    bool headerSkipped = false;
    while (std::getline(input, line))
    {
        line = trim(line);
        if (line.empty())
        {
            continue;
        }
        if (!headerSkipped)
        {
            headerSkipped = true;
            if (line.find("pixel") != std::string::npos)
            {
                continue;
            }
        }

        std::stringstream stream(line);
        std::string field;
        std::vector<float> values;
        while (std::getline(stream, field, ','))
        {
            values.push_back(static_cast<float>(std::atof(trim(field).c_str())));
        }
        if (values.size() < 7)
        {
            continue;
        }

        ReflectivityRow row;
        row.pixel = static_cast<int>(values[0]);
        row.distance = values[1];
        for (int i = 0; i < 5; ++i)
        {
            row.reference[i] = values[2 + i];
        }
        reflectivityRows_.push_back(row);
    }

    for (const auto &row : reflectivityRows_)
    {
        reflectivityByPixel_[row.pixel].push_back(row);
    }
    for (auto &entry : reflectivityByPixel_)
    {
        std::sort(entry.second.begin(), entry.second.end(),
                  [](const ReflectivityRow &a, const ReflectivityRow &b)
                  { return a.distance < b.distance; });
    }
    return true;
}

float Cs96BetaCalibration::reflectivity(int pixel, float distance, std::uint16_t peak) const
{
    if (reflectivityByPixel_.empty())
    {
        return -1.0f;
    }

    // 找最近的像素标定点。
    auto it = reflectivityByPixel_.lower_bound(pixel);
    if (it == reflectivityByPixel_.end())
    {
        --it;
    }
    else if (it != reflectivityByPixel_.begin())
    {
        const auto previous = std::prev(it);
        if (std::abs(previous->first - pixel) < std::abs(it->first - pixel))
        {
            it = previous;
        }
    }

    const auto &rows = it->second;
    if (rows.empty())
    {
        return -1.0f;
    }

    // 按距离插值参考回波。
    std::array<float, 5> reference = rows.front().reference;
    if (distance <= rows.front().distance)
    {
        reference = rows.front().reference;
    }
    else if (distance >= rows.back().distance)
    {
        reference = rows.back().reference;
    }
    else
    {
        for (std::size_t i = 1; i < rows.size(); ++i)
        {
            if (distance <= rows[i].distance)
            {
                const float span = rows[i].distance - rows[i - 1].distance;
                const float ratio = span > 0.0f ? (distance - rows[i - 1].distance) / span : 0.0f;
                for (int k = 0; k < 5; ++k)
                {
                    reference[k] = rows[i - 1].reference[k] * (1.0f - ratio) +
                                   rows[i].reference[k] * ratio;
                }
                break;
            }
        }
    }

    // peak 在 5 个反射率基准之间的位置（0~4）。
    if (peak <= reference[0])
    {
        return 0.0f;
    }
    for (int k = 1; k < 5; ++k)
    {
        if (peak <= reference[k])
        {
            const float span = reference[k] - reference[k - 1];
            const float ratio = span > 0.0f ? (peak - reference[k - 1]) / span : 0.0f;
            return (static_cast<float>(k - 1) + ratio) / 4.0f * 100.0f;
        }
    }
    return 100.0f;
}

float Cs96BetaCalibration::paAngle(int scale, int realAngle) const
{
    float angle = (scale >= 0 && scale < static_cast<int>(encoderAngle_.size()))
                      ? encoderAngle_[static_cast<std::size_t>(scale)]
                      : static_cast<float>(scale) * (360.0f / kPaCountsPerRevolution);
    angle += 0.0009f * static_cast<float>(realAngle);
    while (angle < 0.0f) angle += 360.0f;
    while (angle >= 360.0f) angle -= 360.0f;
    return angle;
}

int Cs96BetaCalibration::faceByPa(float paAngleDeg) const
{
    // 每面 40 计数（60°），前后各 5 计数（7.5°）容差；面间隔 60 计数（90°）。
    constexpr float kRange = 40.0f * (360.0f / kPaCountsPerRevolution);
    constexpr float kTolerance = 5.0f * (360.0f / kPaCountsPerRevolution);

    int matched = -1;
    for (int face = 0; face < kFaces; ++face)
    {
        float delta = paAngleDeg - startingAngle_[static_cast<std::size_t>(face)];
        while (delta > 180.0f) delta -= 360.0f;
        while (delta < -180.0f) delta += 360.0f;

        if (delta >= -kTolerance && delta <= kRange + kTolerance)
        {
            if (matched >= 0)
            {
                return -1; // 区间重叠时不猜测
            }
            matched = face;
        }
    }
    return matched;
}

void Cs96BetaCalibration::applyRaw(int pixel, std::uint32_t ft, std::uint16_t slot,
                                   std::uint16_t peak, int face, algo_lidar::PointData &point)
{
    point.pixel_idx = static_cast<std::uint16_t>(pixel);
    point.slot_idx = slot;
    point.peak = peak;
    point.select_idx = 0;
    point.timestamp = 0;
    point.is_tail_pt = false;
    point.list_pulse.clear();

    // 距离：直接用光速换算，不做 a_factor 距离系数，也不做 d_delta 零点偏移。
    const float r = (ft > 16) ? kLightSpeed * static_cast<float>(ft >> 1) : 0.0f;
    // 俯仰：光学设计标称值（视场 ±10°、中心像素 47、倾斜 -2.5°），非逐设备标定。
    const float theta = static_cast<float>((pixel - kNominalPixelCenter) *
                                               (kNominalThetaFov / kLinesV) +
                                           kNominalThetaTilt) *
                        kDegToRad;
    // 水平角：按 slot 线性铺满单面 120° 视场，不查编码器表，不加逐线修正。
    const float omegaDeg = kNominalOmega0 + kNominalFov * slot / kSlotsPerFace;
    const float omega = omegaDeg * kDegToRad;

    point.r = r;
    point.theta = theta / kDegToRad;
    point.omega_rad = omega;
    point.slot_ang = omegaDeg;
    point.intensity = static_cast<std::uint8_t>(peak > 1638 ? 255 : (peak * 255u) / 1638u);
    // 反射率：不查 reflectivity_map，直接用峰值强度。
    point.reflect = static_cast<float>(peak);
    point.is_valid = face >= 0 && r > 0.1f && r < 200.0f;

    point.x = point.y = point.z = std::numeric_limits<float>::quiet_NaN();
    if (point.is_valid)
    {
        const float horizontal = r * std::cos(theta);
        point.x = horizontal * std::cos(omega);
        point.y = -horizontal * std::sin(omega);
        point.z = r * std::sin(theta);
    }
}

void Cs96BetaCalibration::apply(int pixel, std::uint32_t ft, std::uint32_t mirrorAngle,
                                std::uint16_t slot, std::uint16_t peak, int face,
                                algo_lidar::PointData &point) const
{
    point.pixel_idx = static_cast<std::uint16_t>(pixel);
    point.slot_idx = slot;
    point.peak = peak;
    point.select_idx = 0;
    point.timestamp = 0;
    point.is_tail_pt = false;
    point.list_pulse.clear();

    point.x = point.y = point.z = std::numeric_limits<float>::quiet_NaN();
    point.r = 0.0f;
    point.is_valid = false;
    if (ft <= 16 || face < 0 || face >= kFaces)
    {
        point.is_valid = false;
        return;
    }

    const int index = std::clamp(pixel, 0, linesV_ - 1);
    const float r = kLightSpeed * static_cast<float>(ft >> 1) * aFactor_[index] +
                    dDelta_[index] * 0.01f; // d_delta 单位 cm

    const int scale = static_cast<int>((mirrorAngle >> 12) & 0xFF);
    const int realAngle = static_cast<int>(mirrorAngle & 0xFFF);
    const int faceIndex = face;
    if (slot != 0 && scale >= static_cast<int>(encoderAngle_.size())) return;
    // slot0 为同步标记，严格保留设备的 omega0 -> *2 + omega0 分支。
    float hc = omega0_;
    if (slot != 0)
    {
        hc = encoderAngle_[scale] + 0.0009f * static_cast<float>(realAngle) -
             startingAngle_[faceIndex];
        if (hc > 180.0f) hc -= 360.0f;
        else if (hc < -180.0f) hc += 360.0f;
    }

    // 与下位机一致：hc * 2 + omega0，再加逐线水平角修正。
    // 单面视场只有约 120°，slot_ang 必须落在 [-60, 60]；面间 90° 偏移不能加进来，
    // 否则 4 个面会被摊成 360° 的圆环（这正是之前"转圈圈"的根因）。
    const float omegaDeg = hc * 2.0f + omega0_ + omegaDelta_[index];
    const float omega = omegaDeg * kDegToRad;
    const float theta = theta_[index] * kDegToRad;

    point.r = r;
    point.theta = theta_[index];
    point.omega_rad = omega;
    point.slot_ang = omegaDeg;
    point.x = r * std::cos(theta) * std::cos(omega);
    // CS96 Beta 的 Y 轴方向与常规右手系相反。
    point.y = -r * std::cos(theta) * std::sin(omega);
    point.z = r * std::sin(theta);
    point.intensity = static_cast<std::uint8_t>(peak > 1638 ? 255 : (peak * 255u) / 1638u);

    const float reflected = reflectivity(index, r, peak);
    point.reflect = reflected >= 0.0f ? reflected : static_cast<float>(peak);
    point.is_valid = r > 0.1f && r < 200.0f;
}

} // namespace cloud_viewer
