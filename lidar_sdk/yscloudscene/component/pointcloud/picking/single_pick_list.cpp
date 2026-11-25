#include "single_pick_list.h"

namespace ys_scene {

SinglePickList::SinglePickList(QObject* parent)
    : SinglePickBase(parent)
{
}

void SinglePickList::HandleResult(const PickedPointInfo& info)
{
    list_.append(info);

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

void SinglePickList::ClearSelection()
{
    if (list_.isEmpty()) {
        return;
    }

    list_.clear();
    annotations_.clear();
    emit SelectionCleared();
}

bool SinglePickList::HasSelection() const
{
    return !list_.isEmpty();
}

void SinglePickList::RemoveAnnotation(int index)
{
    if (index < 0 || index >= list_.size()) {
        return;
    }

    list_.removeAt(index);
    annotations_.removeAt(index);

    emit PointRemoved(index);

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

    if (list_.isEmpty()) {
        emit SelectionCleared();
    }
}

const QList<PickedPointInfo>& SinglePickList::GetList() const
{
    return list_;
}

const AnnotationList& SinglePickList::GetAnnotations() const
{
    return annotations_;
}

} // namespace ys_scene
