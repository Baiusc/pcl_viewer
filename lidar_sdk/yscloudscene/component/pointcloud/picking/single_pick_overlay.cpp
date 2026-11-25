#include "single_pick_overlay.h"

namespace ys_scene {

SinglePickOverlay::SinglePickOverlay(QObject* parent)
    : SinglePickBase(parent)
{
}

void SinglePickOverlay::HandleResult(const PickedPointInfo& info)
{
    Annotation ann;
    ann.pos   = QVector3D(info.x, info.y, info.z);
    ann.info  = info;
    ann.index = static_cast<int>(annotations_.size()) + 1;
    ann.label = QString("#%1 [%2,%3,%4] S%5 P%6")
        .arg(ann.index)
        .arg(info.x, 0, 'f', 2)
        .arg(info.y, 0, 'f', 2)
        .arg(info.z, 0, 'f', 2)
        .arg(info.col)
        .arg(info.row);

    annotations_.append(ann);
    emit PointPicked(info);
}

void SinglePickOverlay::ClearSelection()
{
    if (annotations_.isEmpty()) {
        return;
    }

    annotations_.clear();
    emit SelectionCleared();
}

bool SinglePickOverlay::HasSelection() const
{
    return !annotations_.isEmpty();
}

void SinglePickOverlay::RemoveAnnotation(int index)
{
    if (index < 0 || index >= annotations_.size()) {
        return;
    }

    annotations_.removeAt(index);

    for (int i = 0; i < annotations_.size(); ++i) {
        auto& ann = annotations_[i];
        ann.index = i + 1;
        ann.label = QString("#%1 [%2,%3,%4] S%5 P%6")
            .arg(ann.index)
            .arg(ann.info.x, 0, 'f', 2)
            .arg(ann.info.y, 0, 'f', 2)
            .arg(ann.info.z, 0, 'f', 2)
            .arg(ann.info.col)
            .arg(ann.info.row);
    }

    emit AnnotationRemoved(index);

    if (annotations_.isEmpty()) {
        emit SelectionCleared();
    }
}

const AnnotationList& SinglePickOverlay::GetAnnotations() const
{
    return annotations_;
}

QVector3D SinglePickOverlay::AnnotationPos() const
{
    if (annotations_.isEmpty()) {
        return {};
    }
    return annotations_.last().pos;
}

QString SinglePickOverlay::AnnotationLabel() const
{
    if (annotations_.isEmpty()) {
        return {};
    }
    return annotations_.last().label;
}

} // namespace ys_scene
