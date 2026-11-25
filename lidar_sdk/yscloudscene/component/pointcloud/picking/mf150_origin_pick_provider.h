#pragma once

#include "pick_data_provider.h"
#include "include/ys_lidar_type.h"
#include "business/origin_point_parser_imp.h"
#include <memory>
#include <vector>
#include <cstdint>

namespace ys_scene {

using Mf150PixelGrid = std::vector<std::vector<Mf150RawPixel>>;

class Mf150OriginPickProvider : public PickDataProvider
{
public:
    Mf150OriginPickProvider(std::shared_ptr<Mf150PixelGrid> grid,
                            std::shared_ptr<std::vector<you_see::CartesianPoint>> cp_vec,
                            std::shared_ptr<std::vector<uint32_t>> cp_to_flat,
                            int echo_index);

    size_t    PointCount() const override;
    QVector3D GetPointPos(size_t index) const override;
    uint8_t   GetReflectivity(size_t index) const override;
    PickedPointInfo GetPointInfo(size_t index) const override;

private:
    std::shared_ptr<Mf150PixelGrid> grid_;
    std::shared_ptr<std::vector<you_see::CartesianPoint>> cp_vec_;
    std::shared_ptr<std::vector<uint32_t>> cp_to_flat_;
    int echo_index_;
};

} // namespace ys_scene
