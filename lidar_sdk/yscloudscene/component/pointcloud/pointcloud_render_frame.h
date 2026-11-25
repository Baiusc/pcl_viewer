#pragma once

#include "component/color_types.hpp"
#include "ys_lidar_type.h"

#include <memory>
#include <atomic>
#include <cstdint>
#include <vector>

#include <QMetaType>
#include <QVector3D>

namespace ys_scene {

// Process-wide submission order, independent of wrapping hardware mirror IDs.
[[nodiscard]] inline std::uint64_t NextPointcloudRenderSequence() noexcept
{
    static std::atomic<std::uint64_t> sequence{0};
    return sequence.fetch_add(1, std::memory_order_relaxed) + 1;
}

struct PointcloudRenderFrame
{
	using Points = std::vector<you_see::CartesianPoint>;

	std::shared_ptr<const Points> points;
    std::vector<Rgb> colors;

	QVector3D bounds_min;
	QVector3D bounds_max;
    double bounds_diagonal{0.0};

	int frame_id{0};
    std::uint64_t sequence{0};

	[[nodiscard]] bool IsValid() const noexcept
    {
            return points != nullptr
				&& colors.size() == points->size()
				&& frame_id >= 0;
    }
};

using PointcloudRenderFramePtr =
	std::shared_ptr<const PointcloudRenderFrame>;

} // namespace ys_scene

Q_DECLARE_METATYPE(ys_scene::PointcloudRenderFramePtr)
