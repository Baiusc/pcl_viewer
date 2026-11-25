#include "single_pick_base.h"

namespace ys_scene {

SinglePickBase::SinglePickBase(QObject* parent)
    : QObject(parent)
{
}

void SinglePickBase::SetProvider(std::shared_ptr<PickDataProvider> provider)
{
    provider_ = std::move(provider);
}

void SinglePickBase::ExecutePick(QPoint screen_pos,
                                 QSize viewport,
                                 const QVector3D& pick_point_3d,
                                 const QMatrix4x4& proj,
                                 const QMatrix4x4& view)
{
    if (!provider_) {
        return;
    }

    int idx = PickAlgorithm::FindNearest(screen_pos, viewport, pick_point_3d, proj, view, provider_.get());
    if (idx < 0) {
        return;
    }

    PickedPointInfo info = provider_->GetPointInfo(static_cast<size_t>(idx));
    HandleResult(info);
}

QVector3D SinglePickBase::AnnotationPos() const
{
    return {};
}

QString SinglePickBase::AnnotationLabel() const
{
    return {};
}

const AnnotationList& SinglePickBase::GetAnnotations() const
{
    static const AnnotationList empty;
    return empty;
}

int SinglePickBase::FindNearestAnnotation(QPoint screen_gl, QSize viewport,
                                           const QMatrix4x4& proj,
                                           const QMatrix4x4& view) const
{
    QRect vp(0, 0, viewport.width(), viewport.height());
    const auto& annotations = GetAnnotations();
    int nearest = -1;
    double min_dist2 = 64.0;

    for (int i = 0; i < annotations.size(); ++i) {
        QVector3D sp = annotations[i].pos.project(view, proj, vp);
        double dx = sp.x() - screen_gl.x();
        double dy = sp.y() - screen_gl.y();
        double d2 = dx * dx + dy * dy;

        if (d2 < min_dist2) {
            min_dist2 = d2;
            nearest = i;
        }
    }

    return nearest;
}

} // namespace ys_scene
