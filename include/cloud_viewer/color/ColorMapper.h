/**
 * @brief 标量场 -> RGB 的颜色映射
 *
 * 复用现有 PclHelperB::hsv2rgb 与 addCloudI 的映射规则（低值蓝、高值红），
 * 但抽离为纯计算类，不依赖任何 Viewer 对象。
 */
#pragma once

#include <array>
#include <cstdint>
#include <vector>

#include "cloud_viewer/core/CloudFrame.h"

namespace cloud_viewer
{

struct ColorRange
{
    float minValue = 0.0f;
    float maxValue = 1.0f;
    std::size_t finiteCount = 0;
    bool valid = false;
};

struct ColorSettings
{
    enum class Mode
    {
        Continuous,
        Categorical,
        Solid
    };

    ColorSource source = ColorSource::Intensity;
    Mode mode = Mode::Continuous;
    bool autoRange = true;
    float minValue = 0.0f;
    float maxValue = 1.0f;
    std::array<std::uint8_t, 3> solidColor{200, 200, 200};
    std::array<std::uint8_t, 3> invalidColor{80, 80, 80};
};

struct ColorArray
{
    std::vector<std::array<std::uint8_t, 3>> colors;
    ColorRange range;
};

class ColorMapper
{
public:
    /// 统计有限有效值的范围。autoRange=false 时直接返回给定区间。
    static ColorRange computeRange(const ScalarField &field, bool autoRange,
                                   float fixedMin, float fixedMax);

    /// 生成与 field 等长的颜色数组；缺失/NaN 使用 invalidColor。
    static ColorArray map(const ScalarField &field, const ColorSettings &settings,
                          const ColorRange *rangeOverride = nullptr);

    /// 0~1 的 HSV 转 0~1 的 RGB（与现有 PclHelperB::hsv2rgb 一致）。
    static void hsv2rgb(float h, float s, float v, float &r, float &g, float &b);

    /// 回波序号/布尔/簇 id 等离散字段使用分类色。
    static bool isCategorical(ColorSource source);
};

} // namespace cloud_viewer
