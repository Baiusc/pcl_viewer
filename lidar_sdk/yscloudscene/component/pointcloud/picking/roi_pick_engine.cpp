#include "roi_pick_engine.h"
#include "pick_algorithm.h"
#include <algorithm>
#include <cmath>

namespace ys_scene {

static constexpr int kURow = 75;
static constexpr double kPi = 3.14159265358979323846;

RoiPickEngine::RoiPickEngine(QObject* parent)
    : SinglePickBase(parent)
{
}

void RoiPickEngine::ExecutePick(QPoint screen_pos,
                                QSize viewport,
                                const QVector3D& pick_point_3d,
                                const QMatrix4x4& proj,
                                const QMatrix4x4& view)
{
    if (state_ == State::Full) {
        int near_idx = FindNearestAnnotation(screen_pos, viewport, proj, view);
        if (near_idx == 0) {
            state_ = State::AdjustingCorner1;
            emit RoiStateChanged();
            return;
        }
        if (near_idx == 1) {
            state_ = State::AdjustingCorner2;
            emit RoiStateChanged();
            return;
        }
        ClearSelection();
        state_ = State::WaitingFirst;
        emit RoiCleared();
        emit RoiStateChanged();
    }

    SinglePickBase::ExecutePick(screen_pos, viewport, pick_point_3d, proj, view);
}

void RoiPickEngine::HandleResult(const PickedPointInfo& info)
{
    switch (state_) {
    case State::WaitingFirst:
        corner1_ = info;
        state_ = State::WaitingSecond;
        UpdateAnnotations();
        break;

    case State::WaitingSecond:
        corner2_ = info;
        state_ = State::Full;
        UpdateAnnotations();
        ComputeRoiFromCorners();
        emit RoiDefined(roi_row_, roi_col_, roi_width_, roi_height_);
        break;

    case State::AdjustingCorner1:
        corner1_ = info;
        state_ = State::Full;
        UpdateAnnotations();
        ComputeRoiFromCorners();
        emit RoiDefined(roi_row_, roi_col_, roi_width_, roi_height_);
        break;

    case State::AdjustingCorner2:
        corner2_ = info;
        state_ = State::Full;
        UpdateAnnotations();
        ComputeRoiFromCorners();
        emit RoiDefined(roi_row_, roi_col_, roi_width_, roi_height_);
        break;

    default:
        break;
    }
}

void RoiPickEngine::ClearSelection()
{
    corner1_ = PickedPointInfo{};
    corner2_ = PickedPointInfo{};
    roi_row_ = 0;
    roi_col_ = 0;
    roi_width_ = 0;
    roi_height_ = 0;
    annotations_.clear();
    state_ = State::WaitingFirst;
    emit RoiCleared();
    emit RoiStateChanged();
}

bool RoiPickEngine::HasSelection() const
{
    return state_ == State::Full;
}

const AnnotationList& RoiPickEngine::GetAnnotations() const
{
    return annotations_;
}

void RoiPickEngine::RemoveAnnotation(int index)
{
    if (index < 0 || index >= annotations_.size()) {
        return;
    }
    ClearSelection();
}

RoiPickEngine::State RoiPickEngine::GetState() const
{
    return state_;
}

bool RoiPickEngine::HasRoi() const
{
    return state_ == State::Full && roi_width_ > 0 && roi_height_ > 0;
}

void RoiPickEngine::GetRoiRect(int& row, int& col, int& width, int& height) const
{
    row = roi_row_;
    col = roi_col_;
    width = roi_width_;
    height = roi_height_;
}

void RoiPickEngine::BuildRoiVertices(std::vector<float>& out_vertices) const
{
    out_vertices.clear();

    if (state_ != State::Full) {
        return;
    }

    float r1 = corner1_.rho;
    float r2 = corner2_.rho;
    float r_avg = (r1 + r2) * 0.5f;

    int rows[2] = { std::min(static_cast<int>(corner1_.row), static_cast<int>(corner2_.row)),
                     std::max(static_cast<int>(corner1_.row), static_cast<int>(corner2_.row)) };
    int cols[2] = { std::min(static_cast<int>(corner1_.col), static_cast<int>(corner2_.col)),
                     std::max(static_cast<int>(corner1_.col), static_cast<int>(corner2_.col)) };

    QVector3D c00 = ComputeCorner3D(rows[0], cols[0], r_avg);
    QVector3D c01 = ComputeCorner3D(rows[0], cols[1], r_avg);
    QVector3D c10 = ComputeCorner3D(rows[1], cols[0], r_avg);
    QVector3D c11 = ComputeCorner3D(rows[1], cols[1], r_avg);

    float green[] = { 0.0f, 1.0f, 0.0f };

    auto add_vertex = [&](const QVector3D& pos, const float* color) {
        out_vertices.push_back(pos.x());
        out_vertices.push_back(pos.y());
        out_vertices.push_back(pos.z());
        out_vertices.push_back(color[0]);
        out_vertices.push_back(color[1]);
        out_vertices.push_back(color[2]);
    };

    add_vertex(c00, green);
    add_vertex(c01, green);

    add_vertex(c01, green);
    add_vertex(c11, green);

    add_vertex(c11, green);
    add_vertex(c10, green);

    add_vertex(c10, green);
    add_vertex(c00, green);
}

void RoiPickEngine::ComputeRoiFromCorners()
{
    int r1 = static_cast<int>(corner1_.row);
    int c1 = static_cast<int>(corner1_.col);
    int r2 = static_cast<int>(corner2_.row);
    int c2 = static_cast<int>(corner2_.col);

    roi_row_ = std::min(r1, r2);
    roi_col_ = std::min(c1, c2);
    roi_width_ = std::abs(c2 - c1) + 1;
    roi_height_ = std::abs(r2 - r1) + 1;
}

void RoiPickEngine::UpdateAnnotations()
{
    annotations_.clear();

    {
        Annotation ann;
        ann.pos = QVector3D(corner1_.x, corner1_.y, corner1_.z);
        ann.info = corner1_;
        ann.index = 1;
        ann.label = QString("[%1,%2]").arg(corner1_.row).arg(corner1_.col);
        annotations_.append(ann);
    }

    {
        Annotation ann;
        ann.pos = QVector3D(corner2_.x, corner2_.y, corner2_.z);
        ann.info = corner2_;
        ann.index = 2;
        ann.label = QString("[%1,%2]").arg(corner2_.row).arg(corner2_.col);
        annotations_.append(ann);
    }
}

QVector3D RoiPickEngine::ComputeCorner3D(int row, int col, float r) const
{
    float theta = static_cast<float>((row - kURow) / 152.0 * 50.0 * kPi / 180.0);
    float omega = static_cast<float>((col / 360.0 * 120.0 - 60.0) * kPi / 180.0);

    float cos_theta = cosf(theta);
    return QVector3D(
        r * cos_theta * cosf(omega),
        r * cos_theta * sinf(omega),
        r * sinf(theta)
    );
}

} // namespace ys_scene
