#include "pick_algorithm.h"
#include "pick_data_provider.h"
#include <cmath>
#include <limits>

namespace ys_scene {

int PickAlgorithm::FindNearest(QPoint screen_click,
                               QSize viewport,
                               const QVector3D& pick_point_3d,
                               const QMatrix4x4& proj,
                               const QMatrix4x4& view,
                               PickDataProvider* provider,
                               int pick_tolerance)
{
    if (!provider) {
        return -1;
    }

    size_t count = provider->PointCount();
    if (count == 0) {
        return -1;
    }

    int nearest_index = -1;
    double nearest_dist2 = std::numeric_limits<double>::max();

    QRect vp_rect(0, 0, viewport.width(), viewport.height());

    for (size_t i = 0; i < count; ++i) {
        QVector3D pos = provider->GetPointPos(i);

        QVector3D screen_pos = pos.project(view, proj, vp_rect);

        double dx = screen_pos.x() - screen_click.x();
        double dy = screen_pos.y() - screen_click.y();

        if (std::abs(dx) > pick_tolerance || std::abs(dy) > pick_tolerance) {
            continue;
        }

        QVector3D diff = pick_point_3d - pos;
        double dist2 = diff.x() * diff.x() + diff.y() * diff.y() + diff.z() * diff.z();

        if (dist2 < nearest_dist2) {
            nearest_dist2 = dist2;
            nearest_index = static_cast<int>(i);
        }
    }

    return nearest_index;
}

} // namespace ys_scene
