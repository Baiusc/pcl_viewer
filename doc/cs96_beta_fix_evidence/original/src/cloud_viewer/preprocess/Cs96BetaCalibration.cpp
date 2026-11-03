#include "cloud_viewer/preprocess/Cs96BetaCalibration.h"

#include <algorithm>
#include <cctype>
#include <cmath>
#include <cstdlib>
#include <fstream>
#include <sstream>

namespace cloud_viewer
{
namespace
{

constexpr float kLightSpeed = 299792458.0f * 1e-12f; // 米/皮秒
constexpr float kDegToRad = 3.14159265358979323846f / 180.0f;
constexpr float kFaceSpan = 60.0f;   // 每面 60 个刻度
constexpr int kFaces = 4;

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
    return trim(semicolon == std::string::npos ? text : text.substr(0, semicolon));
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

bool Cs96BetaCalibration::load(const std::filesystem::path &directory, std::string &error)
{
    directory_ = directory;
    loaded_ = false;

    if (!parseIni(directory / "ys_calibration.ini", error))
    {
        return false;
    }

    // 整圈编码器角度表（scale 0~240 -> 0~360°），设备端 g_encoder_angle 即来自此表。
    if (!parseEncoderAngle(directory / "ys_encoder_angle.csv", error))
    {
        error.clear();
        // 没有整圈表时退回 spot_slope 标定表。
        parseSpotSlope(directory / "spot_slope_map.csv", error);
        error.clear();
    }

    // 每面覆盖 90°：以该面起始 scale 的角度作为基准。
    for (int face = 0; face < kFaces; ++face)
    {
        const int scale = face * 60;
        if (scale < static_cast<int>(encoderAngle_.size()))
        {
            startingAngle_[face] = encoderAngle_[scale];
        }
    }

    parseReflectivity(directory / "reflectivity_map.csv", error);

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
        else if (section == "horizontal_angular_offset" && index >= 0 && index < linesV_)
        {
            omegaDelta_[index] = std::stof(value);
        }
        else if (section == "zero_distance_offset" && index >= 0 && index < linesV_)
        {
            dDelta_[index] = std::stof(value); // 单位 cm
        }
        else if (section == "distance_factor" && index >= 0 && index < linesV_)
        {
            aFactor_[index] = std::stof(value);
        }
        else if (section == "standard_encoder")
        {
            if (key.rfind("angle_face_", 0) == 0 && index >= 1 && index <= kFaces)
            {
                startingAngle_[index - 1] = static_cast<float>(std::atof(value.c_str()));
            }
        }
    }

    omegaDelta_.resize(linesV_, 0.0f);
    dDelta_.resize(linesV_, 0.0f);
    aFactor_.resize(linesV_, 1.0f);
    theta_.resize(linesV_, 0.0f);
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

    std::vector<float> angles;
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
            if (line.find("scale") != std::string::npos)
            {
                continue;
            }
        }
        const auto comma = line.find(',');
        if (comma == std::string::npos)
        {
            continue;
        }
        angles.push_back(static_cast<float>(std::atof(trim(line.substr(comma + 1)).c_str())));
    }

    if (angles.size() < 241)
    {
        error = "编码器角度表行数不足: " + std::to_string(angles.size());
        return false;
    }
    // scale 0~239；第 240 行是 360°，用于插值边界。
    encoderAngle_.assign(angles.begin(), angles.begin() + 240);
    return true;
}

bool Cs96BetaCalibration::parseSpotSlope(const std::filesystem::path &path, std::string &error)
{
    std::ifstream input(path);
    if (!input.is_open())
    {
        error = "无法打开 " + path.string();
        return false;
    }

    std::array<std::vector<float>, kFaces> angles;
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

        std::stringstream stream(line);
        std::string faceText;
        std::string angleText;
        std::string diffText;
        if (!std::getline(stream, faceText, ',') || !std::getline(stream, angleText, ','))
        {
            continue;
        }
        const std::string face = trim(faceText);
        if (face.size() < 2)
        {
            continue;
        }
        const int faceIndex = face[1] - '1';
        if (faceIndex < 0 || faceIndex >= kFaces)
        {
            continue;
        }
        angles[faceIndex].push_back(std::stof(trim(angleText)));
    }

    for (int face = 0; face < kFaces; ++face)
    {
        auto &list = angles[face];
        if (list.size() < 2)
        {
            continue;
        }
        startingAngle_[face] = list.front();

        // 每面 60 个 scale，按表内点数线性插值到 encoderAngle_。
        const int begin = static_cast<int>(face * kFaceSpan);
        for (int scale = 0; scale < static_cast<int>(kFaceSpan); ++scale)
        {
            const float t = static_cast<float>(scale) /
                            static_cast<float>(kFaceSpan - 1);
            const float position = t * static_cast<float>(list.size() - 1);
            const int low = static_cast<int>(std::floor(position));
            const int high = std::min(low + 1, static_cast<int>(list.size() - 1));
            const float ratio = position - static_cast<float>(low);
            encoderAngle_[begin + scale] = list[low] * (1.0f - ratio) + list[high] * ratio;
        }
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

int Cs96BetaCalibration::faceByScale(std::uint32_t mirrorAngle)
{
    const int scale = static_cast<int>((mirrorAngle >> 12) & 0xFF);
    if (scale >= 9 && scale < 69) return 0;
    if (scale >= 69 && scale < 129) return 1;
    if (scale >= 129 && scale < 189) return 2;
    return 3;
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

    if (ft <= 16)
    {
        point.is_valid = false;
        return;
    }

    const int index = std::clamp(pixel, 0, linesV_ - 1);
    const float r = kLightSpeed * static_cast<float>(ft >> 1) * aFactor_[index] +
                    dDelta_[index] * 0.01f; // d_delta 单位 cm

    const int scale = static_cast<int>((mirrorAngle >> 12) & 0xFF);
    const int realAngle = static_cast<int>(mirrorAngle & 0xFFF);
    const int faceIndex = (face >= 0 && face < kFaces) ? face : faceByScale(mirrorAngle);

    float angle = (scale >= 0 && scale < static_cast<int>(encoderAngle_.size()))
                      ? encoderAngle_[scale]
                      : 0.0f;
    float hc = angle + 0.0009f * static_cast<float>(realAngle) - startingAngle_[faceIndex];
    if (hc > 180.0f) hc -= 360.0f;
    else if (hc < -180.0f) hc += 360.0f;

    // 与下位机一致：hc * 2 + omega0，再加面间 90° 偏移与逐线水平角修正。
    const float omegaDeg = hc * 2.0f + omega0_ + static_cast<float>(faceIndex) * 90.0f +
                           omegaDelta_[index];
    const float omega = omegaDeg * kDegToRad;
    const float theta = theta_[index] * kDegToRad;

    point.r = r;
    point.theta = theta_[index];
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
