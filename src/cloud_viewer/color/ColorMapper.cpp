#include "cloud_viewer/color/ColorMapper.h"

#include <algorithm>
#include <cmath>
#include <limits>

namespace cloud_viewer
{
namespace
{

constexpr std::array<std::array<std::uint8_t, 3>, 8> kCategoricalPalette = {{
    {{0, 0, 255}},
    {{0, 255, 0}},
    {{255, 255, 0}},
    {{255, 0, 0}},
    {{122, 0, 191}},
    {{0, 225, 225}},
    {{233, 84, 32}},
    {{255, 130, 147}},
}};

constexpr std::array<std::array<std::uint8_t, 3>, 3> kEchoPalette = {{
    {{255, 60, 60}},
    {{60, 220, 60}},
    {{80, 140, 255}},
}};

} // namespace

void ColorMapper::hsv2rgb(float h, float s, float v, float &r, float &g, float &b)
{
    if (s <= 0.0f)
    {
        r = g = b = v;
        return;
    }
    h = std::fmod(h * 360.0f, 360.0f) / 60.0f;
    const int i = static_cast<int>(std::floor(h));
    const float f = h - static_cast<float>(i);
    const float x = v * (1.0f - s);
    const float y = v * (1.0f - s * f);
    const float z = v * (1.0f - s * (1.0f - f));

    switch (i)
    {
    case 0:
        r = v;
        g = z;
        b = x;
        break;
    case 1:
        r = y;
        g = v;
        b = x;
        break;
    case 2:
        r = x;
        g = v;
        b = z;
        break;
    case 3:
        r = x;
        g = y;
        b = v;
        break;
    case 4:
        r = z;
        g = x;
        b = v;
        break;
    default:
        r = v;
        g = x;
        b = y;
        break;
    }
}

bool ColorMapper::isCategorical(ColorSource source)
{
    return source == ColorSource::EchoIndex ||
           source == ColorSource::IsTailPoint ||
           source == ColorSource::IsValid ||
           source == ColorSource::ClusterId;
}

ColorRange ColorMapper::computeRange(const ScalarField &field, bool autoRange,
                                     float fixedMin, float fixedMax)
{
    ColorRange range;
    if (!autoRange)
    {
        range.minValue = fixedMin;
        range.maxValue = fixedMax;
        range.valid = fixedMin <= fixedMax;
        range.finiteCount = field.size();
        return range;
    }

    float minValue = std::numeric_limits<float>::max();
    float maxValue = std::numeric_limits<float>::lowest();
    std::size_t count = 0;

    for (std::size_t i = 0; i < field.size(); ++i)
    {
        if (field.valid[i] == 0 || !std::isfinite(field.values[i]))
        {
            continue;
        }
        minValue = std::min(minValue, field.values[i]);
        maxValue = std::max(maxValue, field.values[i]);
        ++count;
    }

    if (count == 0)
    {
        return range;
    }

    range.minValue = minValue;
    range.maxValue = maxValue;
    range.finiteCount = count;
    range.valid = true;
    return range;
}

ColorArray ColorMapper::map(const ScalarField &field, const ColorSettings &settings,
                            const ColorRange *rangeOverride)
{
    ColorArray result;
    result.colors.resize(field.size());

    if (settings.mode == ColorSettings::Mode::Solid || field.empty())
    {
        std::fill(result.colors.begin(), result.colors.end(), settings.solidColor);
        return result;
    }

    const ColorRange range = rangeOverride ? *rangeOverride
                                           : computeRange(field, settings.autoRange,
                                                          settings.minValue, settings.maxValue);
    result.range = range;

    const bool categorical = (settings.mode == ColorSettings::Mode::Categorical) ||
                             isCategorical(settings.source);

    const float span = range.maxValue - range.minValue;
    const bool constant = std::abs(span) < 1e-12f;

    for (std::size_t i = 0; i < field.size(); ++i)
    {
        if (field.valid[i] == 0 || !std::isfinite(field.values[i]))
        {
            result.colors[i] = settings.invalidColor;
            continue;
        }

        if (categorical)
        {
            if (settings.source == ColorSource::EchoIndex)
            {
                const int index = static_cast<int>(std::lround(field.values[i]));
                result.colors[i] = kEchoPalette[static_cast<std::size_t>(
                    std::clamp(index, 0, 2))];
            }
            else if (settings.source == ColorSource::IsTailPoint ||
                     settings.source == ColorSource::IsValid)
            {
                const bool on = field.values[i] > 0.5f;
                result.colors[i] = on ? std::array<std::uint8_t, 3>{{255, 90, 90}}
                                      : std::array<std::uint8_t, 3>{{70, 70, 70}};
            }
            else
            {
                const auto key = static_cast<std::size_t>(
                    std::llround(field.values[i]));
                result.colors[i] = kCategoricalPalette[key % kCategoricalPalette.size()];
            }
            continue;
        }

        float t = constant ? 0.5f : (field.values[i] - range.minValue) / span;
        t = std::clamp(t, 0.0f, 1.0f);

        const float hue = (1.0f - t) * (2.0f / 3.0f);
        float r = 0.0f;
        float g = 0.0f;
        float b = 0.0f;
        hsv2rgb(hue, 0.8f, 0.8f, r, g, b);

        result.colors[i] = {{
            static_cast<std::uint8_t>(std::lround(std::clamp(r, 0.0f, 1.0f) * 255.0f)),
            static_cast<std::uint8_t>(std::lround(std::clamp(g, 0.0f, 1.0f) * 255.0f)),
            static_cast<std::uint8_t>(std::lround(std::clamp(b, 0.0f, 1.0f) * 255.0f)),
        }};
    }

    return result;
}

} // namespace cloud_viewer
