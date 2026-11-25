#pragma once

#include "picked_point_info.h"
#include <QVector3D>
#include <cstdint>

namespace ys_scene {

class PickDataProvider
{
public:
    virtual ~PickDataProvider() = default;

    virtual size_t PointCount() const = 0;
    virtual QVector3D GetPointPos(size_t index) const = 0;
    virtual uint8_t GetReflectivity(size_t index) const = 0;
    virtual PickedPointInfo GetPointInfo(size_t index) const = 0;
};

} // namespace ys_scene
