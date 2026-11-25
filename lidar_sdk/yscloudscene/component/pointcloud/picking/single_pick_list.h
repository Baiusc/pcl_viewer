#pragma once

#include "single_pick_base.h"
#include "picked_point_info.h"
#include <QList>

namespace ys_scene {

class SinglePickList : public SinglePickBase
{
    Q_OBJECT

public:
    explicit SinglePickList(QObject* parent = nullptr);

    void ClearSelection() override;
    bool HasSelection() const override;

    void RemoveAnnotation(int index) override;
    const AnnotationList& GetAnnotations() const override;

    const QList<PickedPointInfo>& GetList() const;

signals:
    void PointRemoved(int index);

protected:
    void HandleResult(const PickedPointInfo& info) override;

private:
    QList<PickedPointInfo> list_;
    AnnotationList annotations_;
};

} // namespace ys_scene
