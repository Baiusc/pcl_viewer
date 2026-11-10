#include "cloud_viewer/render/CloudRenderBridge.h"

#include <cmath>
#include <limits>

namespace cloud_viewer
{

RenderableCloud::Ptr CloudRenderBridge::prepare(const FrameHandle &frame,
                                                const RenderRequest &request)
{
    auto renderable = std::make_shared<RenderableCloud>();
    renderable->geometry.reset(new pcl::PointCloud<pcl::PointXYZRGB>());

    if (!frame)
    {
        return renderable;
    }

    const algo_lidar::PointMatrix &matrix = frame->points;
    const std::size_t rows = matrix.size();
    const std::size_t cols = rows > 0 ? matrix.front().size() : 0;

    renderable->frameRevision = frame->revision;
    renderable->renderRevision = frame->revision;
    renderable->sourceTotal = rows * cols;
    renderable->scalars = frame->scalarFields;

    // 只有抽样时才需要先统计可显示点数。
    // 一帧是 96×800 = 76800 点，多遍历一遍并不便宜，
    // 不抽样时直接按上限预留、单遍填充。
    const bool subsample = request.maxPoints > 0;
    std::size_t candidateCount = 0;
    std::size_t step = 1;
    if (subsample)
    {
        for (std::size_t r = 0; r < rows; ++r)
        {
            for (std::size_t c = 0; c < cols; ++c)
            {
                const algo_lidar::PointData &point = matrix[r][c];
                if (request.skipInvalid && !point.is_valid)
                {
                    continue;
                }
                if (!std::isfinite(point.x) || !std::isfinite(point.y) || !std::isfinite(point.z))
                {
                    continue;
                }
                ++candidateCount;
            }
        }
        if (candidateCount > request.maxPoints)
        {
            step = (candidateCount + request.maxPoints - 1) / request.maxPoints;
            if (step == 0)
            {
                step = 1;
            }
        }
    }

    const std::size_t reserveCount =
        subsample ? std::min(candidateCount, request.maxPoints) : rows * cols;
    renderable->geometry->resize(reserveCount);
    renderable->displayToSource.reserve(reserveCount);

    std::size_t seen = 0;
    std::size_t written = 0;
    bool hasBounds = false;
    float minX = std::numeric_limits<float>::max();
    float minY = std::numeric_limits<float>::max();
    float minZ = std::numeric_limits<float>::max();
    float maxX = std::numeric_limits<float>::lowest();
    float maxY = std::numeric_limits<float>::lowest();
    float maxZ = std::numeric_limits<float>::lowest();

    for (std::size_t r = 0; r < rows; ++r)
    {
        for (std::size_t c = 0; c < cols; ++c)
        {
            const algo_lidar::PointData &point = matrix[r][c];
            if (request.skipInvalid && !point.is_valid)
            {
                continue;
            }
            if (!std::isfinite(point.x) || !std::isfinite(point.y) || !std::isfinite(point.z))
            {
                continue;
            }

            const std::size_t ordinal = seen++;
            if (step > 1 && (ordinal % step) != 0)
            {
                continue;
            }

            // 直接按下标写，避免 push_back 的每次容量检查（30 万点时很可观）。
            pcl::PointXYZRGB &displayPoint = renderable->geometry->points[written++];
            displayPoint.x = point.x;
            displayPoint.y = point.y;
            displayPoint.z = point.z;
            displayPoint.r = 200;
            displayPoint.g = 200;
            displayPoint.b = 200;

            renderable->displayToSource.push_back(
                SourcePointRef{frame->id, static_cast<int>(r), static_cast<int>(c), point.select_idx});

            if (!hasBounds)
            {
                minX = maxX = point.x;
                minY = maxY = point.y;
                minZ = maxZ = point.z;
                hasBounds = true;
            }
            else
            {
                minX = std::min(minX, point.x);
                minY = std::min(minY, point.y);
                minZ = std::min(minZ, point.z);
                maxX = std::max(maxX, point.x);
                maxY = std::max(maxY, point.y);
                maxZ = std::max(maxZ, point.z);
            }
        }
    }

    // 收缩到实际写入的点数（预留时按上限分配过）。
    renderable->geometry->resize(written);
    renderable->geometry->width = static_cast<std::uint32_t>(renderable->geometry->size());
    renderable->geometry->height = 1;
    renderable->geometry->is_dense = true;

    renderable->displayed = renderable->geometry->size();
    renderable->hasBounds = hasBounds;
    if (hasBounds)
    {
        renderable->minPoint.x = minX;
        renderable->minPoint.y = minY;
        renderable->minPoint.z = minZ;
        renderable->maxPoint.x = maxX;
        renderable->maxPoint.y = maxY;
        renderable->maxPoint.z = maxZ;
    }

    return renderable;
}

} // namespace cloud_viewer
