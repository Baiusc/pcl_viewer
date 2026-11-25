#pragma once

#include "single_pick_base.h"
#include "picked_point_info.h"
#include <QVector3D>

namespace ys_scene {

class RoiPickEngine : public SinglePickBase
{
    Q_OBJECT

public:
    enum class State {
        WaitingFirst,
        WaitingSecond,
        Full,
        AdjustingCorner1,
        AdjustingCorner2,
    };

    explicit RoiPickEngine(QObject* parent = nullptr);

    void ExecutePick(QPoint screen_pos,
                     QSize viewport,
                     const QVector3D& pick_point_3d,
                     const QMatrix4x4& proj,
                     const QMatrix4x4& view) override;

    void ClearSelection() override;
    bool HasSelection() const override;
    const AnnotationList& GetAnnotations() const override;

    void RemoveAnnotation(int index) override;

    State GetState() const;
    bool HasRoi() const;
    void GetRoiRect(int& row, int& col, int& width, int& height) const;

    void BuildRoiVertices(std::vector<float>& out_vertices) const;

signals:
    void RoiDefined(int row, int col, int width, int height);
    void RoiCleared();
    void RoiStateChanged();

protected:
    void HandleResult(const PickedPointInfo& info) override;

private:
    void ComputeRoiFromCorners();
    void UpdateAnnotations();
    QVector3D ComputeCorner3D(int row, int col, float r) const;

    State state_{State::WaitingFirst};
    PickedPointInfo corner1_;
    PickedPointInfo corner2_;

    int roi_row_{0};
    int roi_col_{0};
    int roi_width_{0};
    int roi_height_{0};

    mutable AnnotationList annotations_;
};

} // namespace ys_scene
