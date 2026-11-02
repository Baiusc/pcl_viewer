/**
 * @brief PointMatrix -> PCL 显示云 的转换桥
 *
 * 原则：PCL 云只是显示/搜索派生缓存，PointMatrix 才是业务真值。
 * 每个显示点都保留回到原始帧、矩阵行列与选中回波的引用。
 */
#pragma once

#include <cstdint>
#include <map>
#include <memory>
#include <vector>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include "cloud_viewer/core/CloudFrame.h"

namespace cloud_viewer
{

struct SourcePointRef
{
    FrameId frameId = 0;
    int matrixRow = 0;
    int matrixCol = 0;
    std::uint8_t selectedEcho = 0;
};

struct RenderRequest
{
    bool skipInvalid = true;   ///< 是否跳过无效点
    std::size_t maxPoints = 0; ///< 0 表示不抽样；否则确定性原始索引抽样
};

struct RenderableCloud
{
    using Ptr = std::shared_ptr<RenderableCloud>;

    pcl::PointCloud<pcl::PointXYZRGB>::Ptr geometry;
    std::vector<SourcePointRef> displayToSource;
    std::map<ColorSource, ScalarField> scalars;

    bool hasBounds = false;
    pcl::PointXYZRGB minPoint;
    pcl::PointXYZRGB maxPoint;

    std::uint64_t frameRevision = 0;
    std::uint64_t renderRevision = 0;
    std::size_t sourceTotal = 0;
    std::size_t displayed = 0;
};

class CloudRenderBridge
{
public:
    /// 将一帧转换为显示云。不做颜色映射，颜色由 ColorMapper 填充。
    static RenderableCloud::Ptr prepare(const FrameHandle &frame,
                                        const RenderRequest &request = {});
};

} // namespace cloud_viewer
