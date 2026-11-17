#include "cloud_viewer/preprocess/Cs96BetaCalibration.h"

#include <algorithm>
#include <atomic>
#include <cctype>
#include <cmath>
#include <cstdlib>
#include <fstream>
#include <iomanip>
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

// 码盘一圈 240 个粗刻度（scale 0..239）；encoderAngle_ 有 241 项（含 240 边界）。
constexpr int kScalesPerRev = 240;
// time_intervals.txt 缺失时的名义刻度耗时（us）。理论值 1000/240/2.5≈1666.7，
// 现场实测普遍在 1700 上下，沿用旧实现里的 1701 作为兜底。
constexpr float kNominalIntervalUs = 1701.0f;

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

        // FPGA 旋转角曲线有两种命名：旧的 spot_slope_map.csv，新的 angle_calib_YYYYMMDD.csv。
        std::filesystem::path spotPath = path / "spot_slope_map.csv";
        std::error_code spotEc;
        if (!std::filesystem::exists(spotPath, spotEc))
        {
            for (const auto &entry : std::filesystem::directory_iterator(path, spotEc))
            {
                const std::string name = entry.path().filename().string();
                if (entry.path().extension() == ".csv" && name.rfind("angle_calib", 0) == 0)
                {
                    spotPath = entry.path();
                    break;
                }
            }
        }
        parseSpotSlope(spotPath, error);

        // 可选：FPGA 实测的每格耗时（us）。缺失时 apply() 用 kNominalIntervalUs，
        // 因此解析失败不阻断加载，只影响细角度的折算精度。
        std::string timingError;
        parseTimeIntervals(path / "time_intervals.txt", timingError);
    }
    else
    {
        // 单文件聚合表（tools/pack_cs96beta_calibration.py 生成）。
        // 失败时 error 内是可直接定位的原因，由调用方 logUi 打印到控制台。
        if (!parseUnifiedCsv(path, error)) return false;
    }

    return finalizeGeometry(error);
}

bool Cs96BetaCalibration::finalizeGeometry(std::string &error)
{
    // 96×800（IMX459）与 120×800（SL100）共用同一套解算，只拒绝明显非法的尺寸。
    if (linesV_ <= 0 || linesV_ > 512 || linesH_ <= 0 || linesH_ > 16384)
    {
        error = "标定尺寸非法: " + std::to_string(linesV_) + "x" + std::to_string(linesH_);
        return false;
    }

    // 逐线数组按标定表声明的线数对齐（旧表 96，新表 120）。
    omegaDelta_.resize(static_cast<std::size_t>(linesV_), 0.0f);
    dDelta_.resize(static_cast<std::size_t>(linesV_), 0.0f);
    aFactor_.resize(static_cast<std::size_t>(linesV_), 1.0f);
    theta_.resize(static_cast<std::size_t>(linesV_), 0.0f);

    // 与设备端 PA 公式一致，不能用编码器表采样点代替面起始参数。
    // for (int face = 0; face < kFaces; ++face)
    // {
    //     startingAngle_[face] = (standardEncoder_[face] * 256.0f + sideAngularOffset_[face]) /
    //                            256.0f * 360.0f / kPaCountsPerRevolution;
    // }
    // 拆分上式  便于调试
    // sideAngularOffset_[1] += 20; 
    // sideAngularOffset_[2] += 40;
    for (int face = 0; face < kFaces; ++face)
    {
        const float stdEncode_ = standardEncoder_[face];    // 第 0-3 面转镜的起始发光粗刻度 取值范围 0-240
        const float sideOffset = sideAngularOffset_[face]; // 第 0-3 面的水平角相对于第 0 面的误差细角度 取值范围 0-255 单位为 1.5°/256
        const float roughAngle = stdEncode_ / 240.f * 360.f; // 4面起始发光角 粗角度
        const float offsetAngle = sideOffset / 256.f * 1.5f; // 4面零位角 细角度
        startingAngle_[face] = roughAngle + offsetAngle;
    }
    // 编码器表布局判定（20261117）：标定端 anchor_per_face=true 时，每个面段都从锚点
    // （默认 -30° 机械）重新起算，表值可负、第 240 行回绕行可超过 360°，据此自动识别。
    anchorPerFace_ = std::any_of(encoderAngle_.begin(), encoderAngle_.end(),
                                 [](float a)
                                 { return a < -0.5f || a > 360.5f; });
    refreshAnchorDerived();

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
            else if (key == "sat_peak_min") satPeakMin_ = static_cast<std::uint16_t>(std::atoi(value.c_str()));
            else if (key == "dist_max") distMax_ = std::stof(value);
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
            // 转镜雷达（Alpha/Beta/Gamma）的 d_delta 单位就是米，不要换算。
            // 只有固态雷达（6532/6031）的距离标定表才是厘米。
            dDelta_[index] = std::stof(value);
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

    // 传统目录布局（ini + 多 csv）仅用于旧的 96×800 标定包；
    // 120×800 请使用 tools/pack_cs96beta_calibration.py 生成的聚合 CSV。
    if (linesV_ != 96 || linesH_ != 800)
    {
        error = "目录布局标定仅支持 96x800，当前为 " + std::to_string(linesV_) + "x" +
                std::to_string(linesH_) + "；请改用聚合 CSV";
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

void Cs96BetaCalibration::refreshAnchorDerived()
{
    tableStart_.fill(0.0f);
    dialBase_.fill(0.0f);
    if (encoderAngle_.empty())
    {
        return;
    }

    const int last = static_cast<int>(encoderAngle_.size()) - 1;
    for (int face = 0; face < kFaces; ++face)
    {
        const auto index = static_cast<std::size_t>(face);
        int tick = static_cast<int>(std::lround(standardEncoder_[index]));
        tick = std::clamp(tick, 0, last);
        // 每面锚定表：本面起爆点就是起始刻度在表里的值（细调已折算进表值）
        tableStart_[index] = encoderAngle_[static_cast<std::size_t>(tick)];
        // 绝对角 = 表值 + dialBase；锚点是机械角常量（默认 -30°），
        // 与 ini 的起始角（码盘绝对角系）相差正好一个锚点。
        dialBase_[index] = startingAngle_[index] - anchorStartMechDeg_;
    }
}

int Cs96BetaCalibration::faceOfScale(int scale) const
{
    // 按 standardEncoder 分段，环形取"最近的前一个面起点"。
    int bestFace = 0;
    int bestDelta = kScalesPerRev;
    for (int face = 0; face < kFaces; ++face)
    {
        const int start = static_cast<int>(
            std::lround(standardEncoder_[static_cast<std::size_t>(face)]));
        int delta = (scale - start) % kScalesPerRev;
        if (delta < 0)
        {
            delta += kScalesPerRev;
        }
        if (delta < bestDelta)
        {
            bestDelta = delta;
            bestFace = face;
        }
    }
    return bestFace;
}

void Cs96BetaCalibration::setAnchorPerFace(bool on)
{
    anchorPerFace_ = on;
    refreshAnchorDerived();
}

void Cs96BetaCalibration::setAnchorStartMechDeg(float deg)
{
    anchorStartMechDeg_ = deg;
    refreshAnchorDerived();
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
        // 每面锚定表（20261117）允许负数与 >360° 的回绕行，这里只排除明显非法的量级；
        // 表布局由 finalizeGeometry() 按值的范围自动判定。
        if (scale < 0 || scale > 240 || !std::isfinite(angle) ||
            angle < -720.0f || angle > 720.0f || std::isfinite(angles[scale]))
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

bool Cs96BetaCalibration::parseTimeIntervals(const std::filesystem::path &path,
                                             std::string &error)
{
    std::error_code ec;
    if (!std::filesystem::exists(path, ec))
    {
        error.clear();
        return false; // 可选文件：缺失时用名义间隔，不算错误
    }

    std::ifstream input(path);
    if (!input.is_open())
    {
        error = "无法打开 " + path.string();
        return false;
    }

    std::vector<float> values;
    std::string line;
    while (std::getline(input, line))
    {
        const std::string text = trim(line);
        if (text.empty() || text[0] == '#')
        {
            continue;
        }
        char *end = nullptr;
        const float value = std::strtof(text.c_str(), &end);
        if (end == text.c_str() || !std::isfinite(value) || value <= 0.0f)
        {
            continue;
        }
        values.push_back(value);
    }

    if (values.empty())
    {
        error = path.string() + " 里没有可用的刻度耗时";
        return false;
    }

    // 241 行 = 1 个 FPGA t0 + 240 个间隔；t0 远小于其余值，判定后丢掉。
    if (values.size() == static_cast<std::size_t>(kScalesPerRev) + 1)
    {
        std::vector<float> rest(values.begin() + 1, values.end());
        std::nth_element(rest.begin(), rest.begin() + rest.size() / 2, rest.end());
        if (values.front() < rest[rest.size() / 2] * 0.5f)
        {
            values.erase(values.begin());
        }
    }

    // 239 行：末格（scale 239 → 0）缺失，用其余均值补齐，避免整格退回名义值。
    if (values.size() + 1 == static_cast<std::size_t>(kScalesPerRev))
    {
        float sum = 0.0f;
        for (float value : values)
        {
            sum += value;
        }
        values.push_back(sum / static_cast<float>(values.size()));
    }

    if (values.size() != static_cast<std::size_t>(kScalesPerRev))
    {
        error = path.string() + " 需要 " + std::to_string(kScalesPerRev) +
                " 个刻度耗时（也接受 239/241 行），实际 " + std::to_string(values.size());
        return false;
    }

    timeIntervals_ = std::move(values);
    error.clear();
    return true;
}

float Cs96BetaCalibration::timeIntervalUs(int scale) const
{
    if (scale >= 0 && scale < static_cast<int>(timeIntervals_.size()))
    {
        const float value = timeIntervals_[static_cast<std::size_t>(scale)];
        if (value > 1.0f)
        {
            return value;
        }
    }
    return kNominalIntervalUs;
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
    std::size_t lineNo = 0;
    while (std::getline(input, line))
    {
        ++lineNo;
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
            if (key == "lines_v")
            {
                linesV_ = std::atoi(value0.c_str());
                // 逐线表在 lines_v 之后才出现，先按声明线数扩容，避免 120 线的数据被丢弃。
                if (linesV_ > 0 && linesV_ <= 512)
                {
                    const auto count = static_cast<std::size_t>(linesV_);
                    omegaDelta_.resize(count, 0.0f);
                    dDelta_.resize(count, 0.0f);
                    aFactor_.resize(count, 1.0f);
                    theta_.resize(count, 0.0f);
                }
            }
            else if (key == "lines_h") linesH_ = std::atoi(value0.c_str());
            else if (key == "u") u_ = std::atoi(value0.c_str());
            else if (key == "omega0") omega0_ = std::strtof(value0.c_str(), nullptr);
            else if (key == "sat_peak_max") satPeakMax_ = static_cast<std::uint16_t>(std::atoi(value0.c_str()));
            else if (key == "sat_peak_min") satPeakMin_ = static_cast<std::uint16_t>(std::atoi(value0.c_str()));
            else if (key == "dist_max") distMax_ = std::strtof(value0.c_str(), nullptr);
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
                // 与 ini 路径一致：转镜雷达的 d_delta 单位是米。
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
                error = "编码器角度表第 " + std::to_string(lineNo) + " 行 scale 越界或重复: " +
                        indexText;
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
        else if (section == "time_interval")
        {
            // 每格实测耗时（us），apply() 用它把 realAngle 折算成角度。
            const int index = std::atoi(indexText.c_str());
            if (index >= 0 && index < kScalesPerRev)
            {
                if (timeIntervals_.size() != static_cast<std::size_t>(kScalesPerRev))
                {
                    timeIntervals_.assign(static_cast<std::size_t>(kScalesPerRev), 0.0f);
                }
                timeIntervals_[static_cast<std::size_t>(index)] =
                    std::strtof(value0.c_str(), nullptr);
            }
        }
        else
        {
            // vcsel_index / light_intensity 等暂不参与几何，但要原样保留，
            // 否则用目录重算聚合表时会把这几段整段删掉。
            passthroughRows_.push_back(line);
        }
    }

    if (std::any_of(angles.begin(), angles.end(), [](float a) { return !std::isfinite(a); }))
    {
        // 列出缺失的 scale，避免只报“缺表”却不知道缺在哪一段。
        std::ostringstream missing;
        int missingCount = 0;
        for (std::size_t i = 0; i < angles.size(); ++i)
        {
            if (std::isfinite(angles[i]))
            {
                continue;
            }
            ++missingCount;
            if (missingCount <= 8)
            {
                missing << (missingCount > 1 ? "," : "") << i;
            }
        }
        if (missingCount > 8)
        {
            missing << ",...";
        }
        error = "编码器角度表缺少 " + std::to_string(missingCount) + " 个 scale（" + missing.str() +
                "），必须覆盖 0~240: " + path.string();
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
    if (anchorPerFace_ && !encoderAngle_.empty())
    {
        // 每面锚定表是表面局部角，判面用的 PA 需要还原成码盘绝对角
        angle += dialBase_[static_cast<std::size_t>(faceOfScale(scale))];
    }
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
    if (slot > 1)
    {
        int kdebug = 1;
    }
    // 调试开关：CV_BETA_SLOT_TRACE=1 时限流打印 slot/pixel 序列
    // （每 2000 次调用一行，约每秒 400 行，不阻塞接收线程）。
    // 实测设备推流的首个 slot 是 2，slot 0/1 从未进入过本函数。
    if (0)
    {
        static std::atomic<unsigned long> traceCount{0};
        if ((traceCount.fetch_add(1, std::memory_order_relaxed) % 2000) == 0)
        {
            std::fprintf(stderr, "[slot] slot=%u pixel=%d\n",
                         static_cast<unsigned>(slot), pixel);
            std::fflush(stderr);
        }
    }
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
    if (options.distanceCorrection || 1)  // debug 可选 || 1 一直开
    {
        r = r * aFactor_[index] + dDelta_[index]; // 设备端直接按米相加（ys_decode_common.c）
    }

    // 水平角 —— ys_algo_horizontal_corners.c
    //   hc    = encoder[scale] + 0.0009·real - starting[face]，wrap 到 ±180
    //   omega = hc·2 + omega0
    // 编码器表与面起始角属结构参数（PA 实测、非逐线修正），任何开关下都保留。
    const int scale = static_cast<int>((mirrorAngle >> 12) & 0xFF); // 8位 0-256 0-240(避免末尾取不到)
    const int realAngle = static_cast<int>(mirrorAngle & 0xFFF); // 0-4095 0-1700 12位
    if (slot != 0 && scale >= static_cast<int>(encoderAngle_.size())) return;

    float omegaDeg = 0.0f;
    if (options.alphaHorizontalAngle)
    {
        // ── Alpha 方案（ys_algo_horizontal_corners.c:80-100）────────────────
        //   code  = 绝对编码（1/65536 圈）
        //   mid   = 该面起始编码 + 400 slot × 13.6 + code_delay(7)   ← 帧中点
        //   diff  = code - mid，wrap 到 ±半圈
        //   omega = diff × (360/65536) × 2       （不再加 omega0：alpha 以帧中点为 0）
        constexpr float kMaxEncoder = 65536.0f;
        constexpr float kCodePerSlot = 13.6f;
        constexpr float kHalfFrameCode = 400.0f * kCodePerSlot; // 5440 = 半帧
        constexpr float kCodeDelay = 7.0f;                      // 光斑短半轴补偿

        const float absDeg =
            encoderAngle_[scale] + 0.0009f * static_cast<float>(realAngle);
        const float absCode = absDeg / 360.0f * kMaxEncoder;
        const float midCode =
            startingAngle_[face] / 360.0f * kMaxEncoder + kHalfFrameCode + kCodeDelay;

        float diff = absCode - midCode;
        while (diff > kMaxEncoder * 0.5f) diff -= kMaxEncoder;
        while (diff < -kMaxEncoder * 0.5f) diff += kMaxEncoder;

        omegaDeg = diff * (360.0f / kMaxEncoder) * 2.0f;
        if (slot == 0)
        {
            omegaDeg = omega0_;
        }
        else if (options.spotSlope)
        {
            // alpha 里 get_spot_slope() 是必经环节，无表时原样返回。
            omegaDeg = spotSlopeCorrection(omegaDeg, face);
        }
    }
    else
    {
        // ── Beta 方案（ys_algo_horizontal_corners.c:111-127）────────────────
        // slot0 为同步标记，严格保留设备的 omega0 -> *2 + omega0 分支。
        float hc = omega0_;
        if (slot != 0)
        {
            // 码盘机械粗刻度角度（查表，消除刻度不均匀）
            const float encoderPart = encoderAngle_[scale];
            // 下一格角度。必须先判边界再取下标：encoderAngle_ 只有 241 项，
            // scale=240 时取 [241] 是越界（原实现把边界判断写在了取值之后）。
            const int lastScale = static_cast<int>(encoderAngle_.size()) - 1;
            const float nominalStep = 360.0f / static_cast<float>(kScalesPerRev);
            float angle_next = (scale < lastScale) ? encoderAngle_[scale + 1] : encoderPart + nominalStep;
            // 兜底：每面锚定表在面边界处会跳变（下一面段重新从锚点起算），
            // 第 240 行回绕行也可能比末行大 360°，两者都不能当成本格角差，退化为名义一格。
            const float step = angle_next - encoderPart;
            if (!(step > 0.0f) || step > 3.0f * nominalStep)
            {
                angle_next = encoderPart + nominalStep;
            }
            const float angle_spacing = angle_next - encoderPart;

            // 细角度：realAngle 是本格内的计时计数，除以"本格实际耗时"再乘本格角度差。
            // 原来固定除以 1701us，转速或该格耗时有偏差时整格都会被推歪。
            const float interval = timeIntervalUs(scale);
            const float interval_prev = timeIntervalUs(scale - 1);
            const float interval_next = timeIntervalUs(scale + 1);
            
            // float realPart = realAngle / 1701.f * angle_spacing;  // planB 计算细角度时，不去查interval_time.txt时间表，按1701固定值来
            float realPart = realAngle / interval * angle_spacing; // planA 计算细角度时，查interval_time.txt时间表，按实际耗时来

            const float fineLimit = angle_spacing * 1.05f;
            // 细角度不应超过一格：超了说明该格计时/角度表不可信，钳到边界。
            if (realPart < -fineLimit || realPart > fineLimit)
            {
                const float raw = realPart; // 保存越界前的值
                realPart = std::clamp(raw, -fineLimit, fineLimit);

                static int lastScale = std::numeric_limits<int>::min(); // 保证首帧必打
                if ( 0 && scale != lastScale) // 仅 debug 时打印
                {
                    lastScale = scale;
                    std::fprintf(stderr,
                                 "[clamp]: scale=%d, realAngle=%d, interval=%.3f, interval_prev=%.3f, "
                                 "interval_next=%.3f, angle_spacing=%.3f\n",
                                 scale, realAngle, interval, interval_prev, interval_next, angle_spacing);
                }
            }
            realPart = std::clamp(realPart, -fineLimit, fineLimit);
            // 本面起爆点：绝对表用 ini 的起始角；每面锚定表的细调已折进表值，直接取表中该刻度的值
            const float startPart = anchorPerFace_
                                        ? tableStart_[static_cast<std::size_t>(face)]
                                        : startingAngle_[static_cast<std::size_t>(face)];

            hc = encoderPart + realPart - startPart;
            // wrap 到 [-180, 180]
            if (hc > 180.0f)
            {
                hc -= 360.0f;
            }
            else if (hc < -180.0f)
            {
                hc += 360.0f; // 尚未转过起始角，等价于绕一整圈后到达
            }
        }
        // 机械旋转角转换成光学扫描角
        omegaDeg = hc * 2.0f + omega0_;

        // FPGA 旋转角标定 —— ys_algo_horizontal_corners.c 在同一位置调用
        // get_beta_spot_slope()，入参正是"已乘 2 加 omega0"的水平角。
        // spot_slope_map 缺失时设备端原样返回，等价于关闭本项。
        if (slot != 0 && options.spotSlope)
        {
            // omegaDeg = spotSlopeCorrection(omegaDeg, face);
        }
    }

    // 逐线（VCSEL 通道）水平角修正 —— [horizontal_angular_offset]。
    // 这是设备端 deal_orgin_point_cloud_data() 里唯一一处单独叠加的标定量：
    //   omega = horizontal_corner + g_omega_delta[pixel]
    if (options.lineOmegaDelta)
    {
        omegaDeg += omegaDelta_[index];
    }

    // ── 原始点云走左手系：x+ 前、y+ 右、z+ 上 ──────────────────────────────
    // VTK 渲染器是右手系。若直接按 y = r·cosθ·sinω 生成，画面里 y+ 会落在左边，
    // 于是 slot0 显示在右、slot799 显示在左 —— 方向与真实点云一致但和需求相反。
    // 这里把水平角取反（等价于 y 取反，x 因为 cos 是偶函数而不变），得到左手系的视觉：
    //   slot0   → y > 0 → 显示在最左
    //   slot799 → y < 0 → 显示在最右
    // 扫描方向随之变为顺时针，且 slot_ang 与坐标符号保持一致（着色梯度不打架）。
    //
    // 真实点云走 BetaUdpStream::parseCartesian，直接用设备 xyz，不经过这里，
    // 因此保持原有右手系不变。兜底路径 applyFallbackNoTable 有同样的取反。
    omegaDeg = -omegaDeg;

    const float omega = omegaDeg * kDegToRad;
    const float theta = theta_[index] * kDegToRad;

    point.r = r;
    point.theta = theta_[index];
    point.omega_rad = omega;
    point.slot_ang = omegaDeg;
    // 坐标约定：x+ 雷达正前方、y+ 雷达自身右侧、z+ 向上（左手系外观）。
    // 注意 omegaDeg 已在上面取反，因此 slot 递增时 y 从正变负：
    // slot0 侧 y>0、slot799 侧 y<0，在右手系俯视图里就是 slot0 在左、slot799 在右。

    point.x = r * std::cos(theta) * std::cos(omega);
    point.y = r * std::cos(theta) * std::sin(omega);
    point.z = r * std::sin(theta);

    // CV_ANGLE_PROBE=1：各打一次首末 slot 的角度与坐标，用来确认左手系方向。
    // getenv 结果缓存成 static：本函数每秒被调用 76 万次，不能每次都查环境变量。
    static const bool angleProbe = std::getenv("CV_ANGLE_PROBE") != nullptr;
    if (angleProbe && (slot == 2 || slot == 799))
    {
        static bool done2 = false;
        static bool done799 = false;
        bool &done = (slot == 2) ? done2 : done799;
        if (!done)
        {
            done = true;
            std::fprintf(stderr, "[probe] slot=%u omega=%.2f x=%.3f y=%.3f\n",
                         static_cast<unsigned>(slot), omegaDeg, point.x, point.y);
            std::fflush(stderr);
        }
    }

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
    // 距离上限与峰值下限都来自标定表（dist_max / sat_peak_min），不要写死 200。
    point.is_valid = r >= 0.5f && r <= distMax_ && peak >= satPeakMin_ && peak <= satPeakMax_;
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
    if(abs(diff) > 10.f)
    {
        int dd_debug = 1;
    }
    return angleDeg + diff;
}

bool Cs96BetaCalibration::exportUnifiedCsv(const std::filesystem::path &path, std::string &error,
                                           const Cs96BetaCalibration *base) const
{
    if (!loaded_)
    {
        error = "标定未加载，无法导出";
        return false;
    }

    std::ofstream out(path);
    if (!out.is_open())
    {
        error = "无法写入 " + path.string();
        return false;
    }

    const auto number = [](float value)
    {
        std::ostringstream text;
        // 9 位有效数字足够覆盖标定值（角度/系数），又不会写出一长串尾数。
        text << std::setprecision(9) << value;
        return text.str();
    };

    out << "section,key,index,value0,value1,value2,value3,value4,value5\n";

    const auto parameter = [&](const char *key, const std::string &value)
    {
        out << "parameter," << key << ",," << value << ",,,,,\n";
    };
    parameter("lines_v", std::to_string(linesV_));
    parameter("lines_h", std::to_string(linesH_));
    parameter("u", std::to_string(u_));
    parameter("omega0", number(omega0_));
    parameter("sat_peak_max", std::to_string(satPeakMax_));
    parameter("sat_peak_min", std::to_string(satPeakMin_));
    parameter("dist_max", number(distMax_));
    parameter("theta_min", number(thetaMin_));
    parameter("theta_max", number(thetaMax_));
    parameter("theta_tilt", number(thetaTilt_));

    // 面起始角由 standard_encoder(计数) + side_angular_offset 派生，
    // 所以写回这两项、不写 starting_angle，重新加载时 finalizeGeometry 会重算。
    for (int face = 0; face < kFaces; ++face)
    {
        out << "side_angular_offset,," << face << "," << number(sideAngularOffset_[face])
            << ",,,,,\n";
    }
    for (int face = 0; face < kFaces; ++face)
    {
        out << "standard_encoder,angle_face_" << (face + 1) << "," << face << ","
            << number(standardEncoder_[face]) << ",,,,,\n";
    }

    for (std::size_t i = 0; i < omegaDelta_.size(); ++i)
    {
        out << "horizontal_angular_offset,," << i << "," << number(omegaDelta_[i]) << ",,,,,\n";
    }
    for (std::size_t i = 0; i < dDelta_.size(); ++i)
    {
        out << "zero_distance_offset,," << i << "," << number(dDelta_[i]) << ",,,,,\n";
    }
    for (std::size_t i = 0; i < aFactor_.size(); ++i)
    {
        out << "distance_factor,," << i << "," << number(aFactor_[i]) << ",,,,,\n";
    }
    for (std::size_t i = 0; i < encoderAngle_.size(); ++i)
    {
        out << "encoder_angle,," << i << "," << number(encoderAngle_[i]) << ",,,,,\n";
    }

    // 目录标定包常常没有 reflectivity_map.csv / 角度曲线；此时沿用旧表，
    // 否则一次「更新」会把反射率查表和 FPGA 旋转角修正整段清空。
    const std::vector<ReflectivityRow> &reflectRows =
        (!reflectivityRows_.empty() || base == nullptr) ? reflectivityRows_
                                                        : base->reflectivityRows_;
    for (const auto &row : reflectRows)
    {
        out << "reflectivity,," << row.pixel << "," << number(row.distance);
        for (float reference : row.reference)
        {
            out << "," << number(reference);
        }
        out << "\n";
    }

    bool ownSpotSlope = false;
    for (const auto &curve : spotSlope_)
    {
        if (!curve.empty())
        {
            ownSpotSlope = true;
            break;
        }
    }
    const auto &spotCurves = (ownSpotSlope || base == nullptr) ? spotSlope_ : base->spotSlope_;
    for (int face = 0; face < kFaces; ++face)
    {
        for (const auto &entry : spotCurves[static_cast<std::size_t>(face)])
        {
            out << "spot_slope,," << (face + 1) << "," << number(entry.first) << ","
                << number(entry.second) << ",,,,\n";
        }
    }

    // 每格实测耗时（us）：更新标定时从目录读入并写回，apply() 用它折算细角度。
    const auto &timing =
        (!timeIntervals_.empty() || base == nullptr) ? timeIntervals_ : base->timeIntervals_;
    for (std::size_t i = 0; i < timing.size(); ++i)
    {
        out << "time_interval,," << i << "," << number(timing[i]) << ",,,,,\n";
    }

    // 本类不解析的 section（vcsel_index / light_intensity 等）原样写回。
    // 本次解析到的优先，否则沿用 base 的；都没有就什么都不写。
    const auto &passthrough =
        (!passthroughRows_.empty() || base == nullptr) ? passthroughRows_ : base->passthroughRows_;
    for (const auto &row : passthrough)
    {
        out << row << "\n";
    }

    out.flush();
    if (!out.good())
    {
        error = "写入失败: " + path.string();
        return false;
    }
    return true;
}

} // namespace cloud_viewer
