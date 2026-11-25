#pragma once

#include "single_pick_base.h"
#include "picked_point_info.h"
#include <QVector3D>
#include <QString>
#include <QList>

namespace ys_scene {

class SinglePickOverlay : public SinglePickBase
{
    Q_OBJECT

public:
    explicit SinglePickOverlay(QObject* parent = nullptr);

    void ClearSelection() override;
    bool HasSelection() const override;

    void RemoveAnnotation(int index) override;
    const AnnotationList& GetAnnotations() const override;

    QVector3D AnnotationPos() const override;
    QString   AnnotationLabel() const override;

signals:
    void AnnotationRemoved(int index);

protected:
    void HandleResult(const PickedPointInfo& info) override;

private:
    AnnotationList annotations_;
};

} // namespace ys_scene
