#include "mf150_origin_pick_provider.h"
#include "business/origin_point_parser_imp.h"
#include "ys_lidar_validate_type.h"
#include <algorithm>
#include <cmath>

namespace ys_scene {

Mf150OriginPickProvider::Mf150OriginPickProvider(
    std::shared_ptr<Mf150PixelGrid> grid,
    std::shared_ptr<std::vector<you_see::CartesianPoint>> cp_vec,
    std::shared_ptr<std::vector<uint32_t>> cp_to_flat,
    int echo_index)
    : grid_(std::move(grid))
    , cp_vec_(std::move(cp_vec))
    , cp_to_flat_(std::move(cp_to_flat))
    , echo_index_(echo_index)
{
}

size_t Mf150OriginPickProvider::PointCount() const
{
    return cp_vec_ ? cp_vec_->size() : 0;
}

QVector3D Mf150OriginPickProvider::GetPointPos(size_t index) const
{
    if (!cp_vec_ || index >= cp_vec_->size()) {
        return {};
    }
    const auto& cp = (*cp_vec_)[index];
    return {cp.x.f, cp.y.f, cp.z.f};
}

uint8_t Mf150OriginPickProvider::GetReflectivity(size_t index) const
{
    if (!cp_vec_ || index >= cp_vec_->size()) {
        return 0;
    }
    return (*cp_vec_)[index].reflectivity;
}

PickedPointInfo Mf150OriginPickProvider::GetPointInfo(size_t index) const
{
    PickedPointInfo info{};

    if (!cp_vec_ || index >= cp_vec_->size()) {
        return info;
    }

    const auto& cp = (*cp_vec_)[index];
    info.x = cp.x.f;
    info.y = cp.y.f;
    info.z = cp.z.f;
    info.reflectivity = cp.reflectivity;
    info.point_index = static_cast<int>(index);
    info.rho = std::sqrt(cp.x.f * cp.x.f + cp.y.f * cp.y.f + cp.z.f * cp.z.f);

    if (!cp_to_flat_ || !grid_ || index >= cp_to_flat_->size()) {
        return info;
    }

    uint32_t flat = (*cp_to_flat_)[index];
    info.row = static_cast<uint16_t>((flat >> 16) & 0xFFFF);
    info.col = static_cast<uint16_t>(flat & 0xFFFF);

    if (info.row >= grid_->size()) {
        return info;
    }

    const auto& row_data = (*grid_)[info.row];
    if (info.col >= row_data.size()) {
        return info;
    }

    const auto& cell = row_data[info.col];

    constexpr int kURow = 75;

    info.theta_deg = static_cast<float>((static_cast<int>(info.row) - kURow) / 152.0 * 50.0);
    info.omega_deg = static_cast<float>(info.col / 360.0 * 120.0 - 60.0);

    for (int e = 0; e < 3; ++e) {
        auto& echo = info.echoes[e];

        if (!cell.valid || cell.tof[e] == 0
            || cell.tof[e] == Mf150RawParser::kFpgaInvalidPixelValue) {
            echo.valid = false;
            continue;
        }

        echo.valid = true;
        echo.leading_tof = cell.tof[e];
        echo.pulse_width = cell.pulse_width[e];
        echo.trailing_tof = cell.pulse_width[e] == Mf150RawParser::kFpgaInvalidPixelValue
            ? cell.tof[e]
            : cell.tof[e] + cell.pulse_width[e];
        echo.peak = cell.peak[e];
    }

    std::sort(info.echoes, info.echoes + 3,
        [](const PickedPointInfo::EchoDetail& a, const PickedPointInfo::EchoDetail& b) {
            if (!a.valid) return false;
            if (!b.valid) return true;
            return a.peak > b.peak;
        });

    return info;
}

} // namespace ys_scene
