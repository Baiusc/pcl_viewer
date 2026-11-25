#pragma once

#include "picked_point_info.h"
#include "pick_data_provider.h"
#include "pick_algorithm.h"
#include <QObject>
#include <QPoint>
#include <QSize>
#include <QMatrix4x4>
#include <QVector3D>
#include <QString>
#include <QList>
#include <memory>

namespace ys_scene {

struct Annotation
{
    QVector3D pos;
    QString label;
    int index;
    PickedPointInfo info;
};

using AnnotationList = QList<Annotation>;

class SinglePickBase : public QObject
{
    Q_OBJECT

public:
    explicit SinglePickBase(QObject* parent = nullptr);
    ~SinglePickBase() override = default;

    void SetProvider(std::shared_ptr<PickDataProvider> provider);

    virtual void ExecutePick(QPoint screen_pos,
                     QSize viewport,
                     const QVector3D& pick_point_3d,
                     const QMatrix4x4& proj,
                     const QMatrix4x4& view);

    virtual void ClearSelection() = 0;
    virtual bool HasSelection() const = 0;

    virtual const AnnotationList& GetAnnotations() const;

    int  FindNearestAnnotation(QPoint screen_gl, QSize viewport,
                               const QMatrix4x4& proj, const QMatrix4x4& view) const;
    virtual void RemoveAnnotation(int index) = 0;

protected:
    virtual QVector3D AnnotationPos() const;
    virtual QString   AnnotationLabel() const;

signals:
    void PointPicked(const PickedPointInfo& info);
    void SelectionCleared();

protected:
    virtual void HandleResult(const PickedPointInfo& info) = 0;
    std::shared_ptr<PickDataProvider> provider_;
};

} // namespace ys_scene
