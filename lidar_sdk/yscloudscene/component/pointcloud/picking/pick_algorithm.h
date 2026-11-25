#pragma once

#include "picked_point_info.h"
#include <QPoint>
#include <QSize>
#include <QMatrix4x4>
#include <QVector3D>

namespace ys_scene {

class PickDataProvider;

class PickAlgorithm
{
public:
    static int FindNearest(QPoint screen_click,
                           QSize viewport,
                           const QVector3D& pick_point_3d,
                           const QMatrix4x4& proj,
                           const QMatrix4x4& view,
                           PickDataProvider* provider,
                           int pick_tolerance = 3);
};

} // namespace ys_scene
