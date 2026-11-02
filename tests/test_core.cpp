#include <cstdlib>
#include <iostream>

#include "cloud_viewer/core/CloudFrame.h"
#include "cloud_viewer/render/CloudRenderBridge.h"

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

algo_lidar::PointData makePoint(bool valid, float x, float y, float z,
                                std::uint8_t intensity, std::uint32_t timestamp)
{
    algo_lidar::PointData point{};
    point.is_valid = valid;
    point.x = x;
    point.y = y;
    point.z = z;
    point.r = std::sqrt(x * x + y * y + z * z);
    point.intensity = intensity;
    point.timestamp = timestamp;
    point.peak = 1000;
    point.reflect = 0.5f;
    point.theta = 1.0f;
    point.slot_ang = 2.0f;
    point.select_idx = 0;
    return point;
}

} // namespace

int main()
{
    algo_lidar::PointMatrix matrix(2, std::vector<algo_lidar::PointData>(3));
    matrix[0][0] = makePoint(false, 0.0f, 0.0f, 0.0f, 0, 0);
    matrix[0][1] = makePoint(true, 1.0f, 0.0f, 0.0f, 10, 0);
    matrix[0][2] = makePoint(true, 2.0f, 0.0f, 0.0f, 20, 0);
    matrix[1][0] = makePoint(true, 3.0f, 0.0f, 0.0f, 30, 0);
    matrix[1][1] = makePoint(true, 4.0f, 0.0f, 0.0f, 40, 0);
    matrix[1][2] = makePoint(true, 5.0f, 0.0f, 0.0f, 50, 0);

    cloud_viewer::FrameMetadata metadata;
    metadata.model = cloud_viewer::RadarModel::Generic;

    auto frame = cloud_viewer::makeFrame(matrix, metadata, 1);
    check(frame->metadata.totalSamples == 6, "totalSamples 应为 6");
    check(frame->metadata.validSamples == 5, "validSamples 应为 5");
    check(frame->metadata.invalidSamples == 1, "invalidSamples 应为 1");
    check(frame->availability.has(cloud_viewer::ColorSource::Range), "Range 字段应可用");
    check(!frame->availability.has(cloud_viewer::ColorSource::Timestamp),
          "全零时间戳不应标记为可用");

    auto renderable = cloud_viewer::CloudRenderBridge::prepare(frame);
    check(renderable->displayed == 5, "显示点数应为 5");
    check(renderable->displayToSource.size() == 5, "来源映射数量应为 5");
    check(renderable->displayToSource[0].matrixRow == 0 &&
              renderable->displayToSource[0].matrixCol == 1,
          "首个显示点应映射到 (0,1)");
    check(renderable->displayToSource[4].matrixRow == 1 &&
              renderable->displayToSource[4].matrixCol == 2,
          "末个显示点应映射到 (1,2)");

    // 确定性抽样：原始索引步长保留，映射仍可回溯。
    cloud_viewer::RenderRequest sampleRequest;
    sampleRequest.maxPoints = 2;
    auto sampled = cloud_viewer::CloudRenderBridge::prepare(frame, sampleRequest);
    check(sampled->displayed == 2, "抽样后显示点数应为 2");
    check(sampled->displayToSource.size() == sampled->displayed,
          "抽样后映射数量应与显示点数一致");

    // 非零时间戳应使字段可用，且与“值为零”区分。
    matrix[0][2].timestamp = 12345;
    auto frameWithTime = cloud_viewer::makeFrame(matrix, metadata, 2);
    check(frameWithTime->availability.has(cloud_viewer::ColorSource::Timestamp),
          "存在非零时间戳时 Timestamp 字段应可用");

    // 切换颜色来源不能修改原始 PointData。
    const float originalIntensity = static_cast<float>(matrix[1][1].intensity);
    auto renderable2 = cloud_viewer::CloudRenderBridge::prepare(frameWithTime);
    check(static_cast<float>(frameWithTime->points[1][1].intensity) == originalIntensity,
          "转换后原始 PointData 不应被修改");
    check(renderable2->scalars.count(cloud_viewer::ColorSource::Intensity) == 1,
          "显示云应携带强度标量场");

    if (g_failures == 0)
    {
        std::cout << "[PASS] core frame mapping tests\n";
        return EXIT_SUCCESS;
    }
    return EXIT_FAILURE;
}
