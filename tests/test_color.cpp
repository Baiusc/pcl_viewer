#include <cmath>
#include <cstdlib>
#include <iostream>
#include <limits>

#include "cloud_viewer/color/ColorMapper.h"

namespace
{

int g_failures = 0;

void check(bool condition, const char *message)
{
    if (!condition)
    {
        std::cerr << "[FAIL] " << message << "\n";
        ++g_failures;
    }
}

} // namespace

int main()
{
    using namespace cloud_viewer;

    ScalarField field;
    field.resize(5);
    field.values = {0.0f, 5.0f, 10.0f, std::numeric_limits<float>::quiet_NaN(), 20.0f};
    field.valid = {1, 1, 1, 0, 0};

    ColorSettings settings;
    settings.source = ColorSource::Range;
    settings.mode = ColorSettings::Mode::Continuous;
    settings.autoRange = true;
    settings.invalidColor = {{80, 80, 80}};

    const ColorArray colors = ColorMapper::map(field, settings);
    check(colors.range.valid, "自动范围应有效");
    check(std::abs(colors.range.minValue - 0.0f) < 1e-6f, "最小值为 0");
    check(std::abs(colors.range.maxValue - 10.0f) < 1e-6f, "最大值应忽略无效值");

    // 低值蓝、高值红：只检查通道相对关系，避免依赖精确色板。
    check(colors.colors[0][2] > colors.colors[0][0], "最小值应为蓝色分量占优");
    check(colors.colors[2][0] > colors.colors[2][2], "最大值应为红色分量占优");
    check(colors.colors[3] == settings.invalidColor, "NaN/缺失应使用 invalidColor");
    check(colors.colors[4] == settings.invalidColor, "无效位应使用 invalidColor");

    // 常量范围使用色带中点，不应崩溃。
    ScalarField constant;
    constant.resize(2);
    constant.values = {3.0f, 3.0f};
    constant.valid = {1, 1};
    const ColorArray constantColors = ColorMapper::map(constant, settings);
    check(constantColors.colors.size() == 2, "常量范围应生成颜色");

    // 分类字段：回波序号使用固定三色且不因范围变化。
    ScalarField echo;
    echo.resize(3);
    echo.values = {0.0f, 1.0f, 2.0f};
    echo.valid = {1, 1, 1};
    ColorSettings echoSettings;
    echoSettings.source = ColorSource::EchoIndex;
    const ColorArray echoColors = ColorMapper::map(echo, echoSettings);
    check(echoColors.colors[0] != echoColors.colors[1], "回波 0/1 颜色应不同");
    check(echoColors.colors[1] != echoColors.colors[2], "回波 1/2 颜色应不同");

    // 固定色模式忽略标量值。
    ColorSettings solid;
    solid.mode = ColorSettings::Mode::Solid;
    solid.solidColor = {{12, 34, 56}};
    const ColorArray solidColors = ColorMapper::map(field, solid);
    check(solidColors.colors[0] == solid.solidColor, "固定色模式应全部使用固定色");

    if (g_failures == 0)
    {
        std::cout << "[PASS] color mapper tests\n";
        return EXIT_SUCCESS;
    }
    return EXIT_FAILURE;
}
