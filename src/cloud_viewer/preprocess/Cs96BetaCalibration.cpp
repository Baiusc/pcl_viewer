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

// ys_algo_calib_reflectivity.c: #define MIN_PEAK 51
constexpr std::uint16_t kMinPeak = 51;

// calib_reflectivity/include/calc_intensity.h: _reflectances = {0.1, 0.3, 0.5, 0.7, 0.9}
// 反射率表存的是 5 档参考峰值，插值结果是这 5 个固定反射率之间的值。
constexpr float kReflectanceLevels[5] = {0.1f, 0.3f, 0.5f, 0.7f, 0.9f};

// include/Spline.h: linearExtrapolate() 的外推锚点
constexpr float kExtrapolateMinPeak = 0.0f;
constexpr float kExtrapolateMinReflect = 0.0f;
constexpr float kExtrapolateMaxPeak = 1600.0f;
constexpr float kExtrapolateMaxReflect = 1.0f;

/// 复刻 ys_algo_calib_reflectivity.c 的 get_reflect_by_peak()：返回 0~1。
float reflectByPeak(std::uint16_t peak, std::uint16_t maxPeak)
{
    if (peak <= kMinPeak)
    {
        return 0.0f;
    }
    if (peak >= maxPeak)
    {
        return 1.0f;
    }
    return static_cast<float>(peak - kMinPeak) / static_cast<float>(maxPeak - kMinPeak);
}

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

    // 与设备端 PA 公式一致，不能用编码器表采样点代替面起始参数。
    for (int face = 0; face < kFaces; ++face)
    {
        startingAngle_[face] = (standardEncoder_[face] * 256.0f + sideAngularOffset_[face]) /
                               256.0f * 360.0f / kPaCountsPerRevolution;
    }


    // 俯仰角：与下位机 CalculateTheta 一致（像素倒序映射）。
    const float step = (thetaMax_ - thetaMin_) / static_cast<float>(linesV_);
    for (int i = linesV_ - 1, j = 0; i >= 0; --i, ++j)
    {
        theta_[j] = static_cast<float>(i - u_) * step + thetaTilt_;
    }

    // FPGA 旋转角曲线按角度升序排列，保证插值/外推的单调前提。
    for (auto &curve : spotSlope_)
    {
        std::sort(curve.begin(), curve.end(),
                  [](const std::pair<float, float> &a, const std::pair<float, float> &b)
                  { return a.first < b.first; });
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
            else if (key == "sat_peak_max") satPeakMax_ = static_cast<std::uint16_t>(std::atoi(value.c_str()));
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
            dDelta_[index] = std::stof(value); // 单位 m
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
            else if (key == "sat_peak_max") satPeakMax_ = static_cast<std::uint16_t>(std::atoi(value0.c_str()));
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

    // 复刻 CalibReflect::calcIntensity()：把 peak 对 5 个参考峰值做线性插值，
    // 得到 _reflectances = {0.1, 0.3, 0.5, 0.7, 0.9} 之间的值；越界按 linearExtrapolate()
    // 的锚点 (0, 0) 与 (1600, 1) 外推并钳到 [0, 1]。
    float reflect = 0.0f;
    if (peak <= reference[0])
    {
        const float span = reference[0] - kExtrapolateMinPeak;
        const float ratio = span > 0.0f ? (peak - kExtrapolateMinPeak) / span : 0.0f;
        reflect = kExtrapolateMinReflect + ratio * (kReflectanceLevels[0] - kExtrapolateMinReflect);
    }
    else if (peak >= reference[4])
    {
        const float span = kExtrapolateMaxPeak - reference[4];
        const float ratio = span > 0.0f ? (peak - reference[4]) / span : 0.0f;
        reflect = kReflectanceLevels[4] + ratio * (kExtrapolateMaxReflect - kReflectanceLevels[4]);
    }
    else
    {
        for (int k = 1; k < 5; ++k)
        {
            if (peak <= reference[k])
            {
                const float span = reference[k] - reference[k - 1];
                const float ratio = span > 0.0f ? (peak - reference[k - 1]) / span : 0.0f;
                reflect = kReflectanceLevels[k - 1] +
                          ratio * (kReflectanceLevels[k] - kReflectanceLevels[k - 1]);
                break;
            }
        }
    }
    return std::clamp(reflect, 0.0f, 1.0f) * 100.0f;
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

void Cs96BetaCalibration::apply(int pixel, std::uint32_t ft, std::uint32_t mirrorAngle,
                                std::uint16_t slot, std::uint16_t peak, int face,
                                const DecodeOptions &options, algo_lidar::PointData &point) const
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

    // 距离 —— ys_decode_common.c: calculate_distance()
    //   r = c·(ft >> 1)·a[pixel] + d_delta[pixel]
    // 关闭 distanceCorrection 等价于 a = 1、d_delta = 0。
    float r = kLightSpeed * static_cast<float>(ft >> 1);
    if (options.distanceCorrection)
    {
        r = r * aFactor_[index] + dDelta_[index]; // 设备端直接按米相加（ys_decode_common.c）
    }

    // 水平角 —— ys_algo_horizontal_corners.c
    //   hc    = encoder[scale] + 0.0009·real - starting[face]，wrap 到 ±180
    //   omega = hc·2 + omega0
    // 编码器表与面起始角属结构参数（PA 实测、非逐线修正），任何开关下都保留。
    const int scale = static_cast<int>((mirrorAngle >> 12) & 0xFF);
    const int realAngle = static_cast<int>(mirrorAngle & 0xFFF);
    if (slot != 0 && scale >= static_cast<int>(encoderAngle_.size())) return;
    // slot0 为同步标记，严格保留设备的 omega0 -> *2 + omega0 分支。
    float hc = omega0_;
    if (slot != 0)
    {
        hc = encoderAngle_[scale] + 0.0009f * static_cast<float>(realAngle) - startingAngle_[face];
        if (hc > 180.0f) hc -= 360.0f;
        else if (hc < -180.0f) hc += 360.0f;
    }
    // 单面视场只有约 120°，面间 90° 偏移不能加进来，否则 4 个面会被摊成 360°。
    float omegaDeg = hc * 2.0f + omega0_;

    // FPGA 旋转角标定 —— ys_algo_horizontal_corners.c 在同一位置调用
    // get_beta_spot_slope()，入参正是"已乘 2 加 omega0"的水平角。
    // spot_slope_map 缺失时设备端原样返回，等价于关闭本项。
    if (slot != 0 && options.spotSlope)
    {
        omegaDeg = spotSlopeCorrection(omegaDeg, face);
    }

    // 逐线（VCSEL 通道）水平角修正 —— [horizontal_angular_offset]。
    // 这是设备端 deal_orgin_point_cloud_data() 里唯一一处单独叠加的标定量：
    //   omega = horizontal_corner + g_omega_delta[pixel]
    if (options.lineOmegaDelta)
    {
        omegaDeg += omegaDelta_[index];
    }

    const float omega = omegaDeg * kDegToRad;
    const float theta = theta_[index] * kDegToRad;

    point.r = r;
    point.theta = theta_[index];
    point.omega_rad = omega;
    point.slot_ang = omegaDeg;
    point.x = r * std::cos(theta) * std::cos(omega);
    point.y = r * std::cos(theta) * std::sin(omega);
    point.z = r * std::sin(theta);

    // 反射率 —— 有表时复刻 CalibReflect::calcIntensity()：
    //   用 5 个参考峰值对 peak 做线性插值，得到 0.1~0.9 的反射率。
    // 关闭标定或该像素查不到时，退回 get_reflect() 的 handle == NULL 分支。
    float reflectNorm = -1.0f;
    if (options.reflectivityMap)
    {
        const float mapped = reflectivity(index, r, peak); // 0~100
        if (mapped >= 0.0f)
        {
            reflectNorm = mapped * 0.01f;
        }
    }
    if (reflectNorm < 0.0f)
    {
        reflectNorm = reflectByPeak(peak, satPeakMax_);
    }
    point.reflect = reflectNorm * 100.0f;
    point.intensity = static_cast<std::uint8_t>(std::clamp(reflectNorm * 255.0f, 0.0f, 255.0f));
    point.is_valid = r > 0.1f && r < 200.0f;
}

float Cs96BetaCalibration::spotSlopeCorrection(float angleDeg, int face) const
{
    if (face < 0 || face >= kFaces)
    {
        return angleDeg;
    }
    const auto &curve = spotSlope_[face];
    // 与 repairRotateAngle() 一致：数据不足 2 点、查表失败都原样返回。
    if (curve.size() < 2)
    {
        return angleDeg;
    }

    float diff = 0.0f;
    if (angleDeg <= curve.front().first)
    {
        const float x1 = curve.front().first;
        const float y1 = curve.front().second;
        const float x2 = curve[1].first;
        const float y2 = curve[1].second;
        const float slope = (x2 != x1) ? (y2 - y1) / (x2 - x1) : 0.0f;
        diff = y1 + slope * (angleDeg - x1);
    }
    else if (angleDeg >= curve.back().first)
    {
        const std::size_t n = curve.size();
        const float x1 = curve[n - 2].first;
        const float y1 = curve[n - 2].second;
        const float x2 = curve[n - 1].first;
        const float y2 = curve[n - 1].second;
        const float slope = (x2 != x1) ? (y2 - y1) / (x2 - x1) : 0.0f;
        diff = y2 + slope * (angleDeg - x2);
    }
    else
    {
        for (std::size_t i = 1; i < curve.size(); ++i)
        {
            if (angleDeg <= curve[i].first)
            {
                const float span = curve[i].first - curve[i - 1].first;
                const float ratio = span > 0.0f ? (angleDeg - curve[i - 1].first) / span : 0.0f;
                diff = curve[i - 1].second + ratio * (curve[i].second - curve[i - 1].second);
                break;
            }
        }
    }
    return angleDeg + diff;
}

} // namespace cloud_viewer
