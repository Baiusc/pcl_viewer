#include "cloud_viewer/render/PclVtkRenderer.h"

#include <algorithm>

#include <vtkActor.h>
#include <vtkCamera.h>
#include <vtkCellArray.h>
#include <vtkInteractorStyleTrackballCamera.h>
#include <vtkPointData.h>
#include <vtkPointPicker.h>
#include <vtkPoints.h>
#include <vtkPolyData.h>
#include <vtkPolyDataMapper.h>
#include <vtkProperty.h>
#include <vtkRenderer.h>
#include <vtkRenderWindow.h>
#include <vtkRenderWindowInteractor.h>
#include <vtkSphereSource.h>
#include <vtkUnsignedCharArray.h>

namespace cloud_viewer
{

bool PclVtkRenderer::initialize(int width, int height)
{
    renderer_ = vtkSmartPointer<vtkRenderer>::New();
    renderer_->SetBackground(37.0 / 255.0, 37.0 / 255.0, 37.0 / 255.0);

    window_ = vtkSmartPointer<vtkRenderWindow>::New();
    window_->SetWindowName("cloud_viewer Lite");
    window_->AddRenderer(renderer_);
    window_->SetSize(width, height);

    interactor_ = vtkSmartPointer<vtkRenderWindowInteractor>::New();
    interactor_->SetRenderWindow(window_);

    auto style = vtkSmartPointer<vtkInteractorStyleTrackballCamera>::New();
    interactor_->SetInteractorStyle(style);

    return true;
}

void PclVtkRenderer::upsert(SceneObjectId id, const RenderableCloud::Ptr &cloud,
                            const ColorArray &colors)
{
    if (!renderer_ || !cloud || !cloud->geometry)
    {
        return;
    }

    const auto &points = cloud->geometry->points;
    const std::size_t count = points.size();
    if (count == 0)
    {
        remove(id);
        return;
    }

    auto vtkPointsData = vtkSmartPointer<vtkPoints>::New();
    vtkPointsData->SetDataTypeToFloat();
    vtkPointsData->SetNumberOfPoints(static_cast<vtkIdType>(count));

    auto cells = vtkSmartPointer<vtkCellArray>::New();
    cells->Allocate(static_cast<vtkIdType>(count));

    auto rgb = vtkSmartPointer<vtkUnsignedCharArray>::New();
    rgb->SetName("Colors");
    rgb->SetNumberOfComponents(3);
    rgb->SetNumberOfTuples(static_cast<vtkIdType>(count));

    for (std::size_t i = 0; i < count; ++i)
    {
        vtkPointsData->SetPoint(static_cast<vtkIdType>(i), points[i].x, points[i].y, points[i].z);
        cells->InsertNextCell(1);
        cells->InsertCellPoint(static_cast<vtkIdType>(i));

        const std::array<std::uint8_t, 3> color =
            (i < colors.colors.size()) ? colors.colors[i]
                                       : std::array<std::uint8_t, 3>{{200, 200, 200}};
        rgb->SetTypedTuple(static_cast<vtkIdType>(i),
                           reinterpret_cast<const unsigned char *>(color.data()));
    }

    auto polyData = vtkSmartPointer<vtkPolyData>::New();
    polyData->SetPoints(vtkPointsData);
    polyData->SetVerts(cells);
    polyData->GetPointData()->SetScalars(rgb);

    auto mapper = vtkSmartPointer<vtkPolyDataMapper>::New();
    mapper->SetInputData(polyData);
    mapper->ScalarVisibilityOn();
    mapper->SetColorModeToDirectScalars();

    auto actor = vtkSmartPointer<vtkActor>::New();
    actor->SetMapper(mapper);
    actor->GetProperty()->SetPointSize(pointSize_);

    remove(id);

    renderer_->AddActor(actor);
    actors_[id] = actor;
    actorToObject_[actor] = id;
}

void PclVtkRenderer::remove(SceneObjectId id)
{
    const auto it = actors_.find(id);
    if (it == actors_.end())
    {
        return;
    }
    if (renderer_ && it->second)
    {
        renderer_->RemoveActor(it->second);
        actorToObject_.erase(it->second);
    }
    actors_.erase(it);
}

void PclVtkRenderer::removeAll()
{
    for (auto &entry : actors_)
    {
        if (renderer_ && entry.second)
        {
            renderer_->RemoveActor(entry.second);
        }
    }
    actors_.clear();
    actorToObject_.clear();
    clearHighlight();
}

void PclVtkRenderer::setVisible(SceneObjectId id, bool visible)
{
    const auto it = actors_.find(id);
    if (it != actors_.end() && it->second)
    {
        it->second->SetVisibility(visible ? 1 : 0);
    }
}

void PclVtkRenderer::setPointSize(float size)
{
    pointSize_ = std::clamp(size, 1.0f, 10.0f);
    applyPointSize();
}

void PclVtkRenderer::applyPointSize()
{
    for (auto &entry : actors_)
    {
        if (entry.second)
        {
            entry.second->GetProperty()->SetPointSize(pointSize_);
        }
    }
    if (highlightActor_)
    {
        // 高亮球大小与点大小无关。
    }
}

void PclVtkRenderer::setBackground(double r, double g, double b)
{
    if (renderer_)
    {
        renderer_->SetBackground(r, g, b);
    }
}

void PclVtkRenderer::resetCamera()
{
    if (!renderer_)
    {
        return;
    }

    renderer_->ResetCamera();
    renderer_->ResetCameraClippingRange();

    // 默认给一个斜视角，避免平面/曲面点云被看成一条线。
    if (auto *camera = renderer_->GetActiveCamera())
    {
        camera->Azimuth(35.0);
        camera->Elevation(20.0);
        camera->OrthogonalizeViewUp();
        renderer_->ResetCameraClippingRange();
    }
}

void PclVtkRenderer::render()
{
    if (window_)
    {
        window_->Render();
    }
}

bool PclVtkRenderer::pick(int displayX, int displayY, SceneObjectId &objectId,
                          vtkIdType &pointId, double world[3]) const
{
    if (!renderer_)
    {
        return false;
    }

    auto picker = vtkSmartPointer<vtkPointPicker>::New();
    picker->SetTolerance(0.01);
    const int picked = picker->Pick(static_cast<double>(displayX),
                                    static_cast<double>(displayY), 0.0, renderer_);
    if (picked == 0)
    {
        return false;
    }

    const auto it = actorToObject_.find(picker->GetActor());
    if (it == actorToObject_.end())
    {
        return false;
    }

    objectId = it->second;
    pointId = picker->GetPointId();
    if (pointId < 0)
    {
        return false;
    }

    picker->GetPickPosition(world);
    return true;
}

void PclVtkRenderer::highlightPoint(const double world[3])
{
    if (!renderer_)
    {
        return;
    }

    clearHighlight();

    auto sphere = vtkSmartPointer<vtkSphereSource>::New();
    sphere->SetCenter(world[0], world[1], world[2]);
    sphere->SetRadius(0.06);
    sphere->SetThetaResolution(12);
    sphere->SetPhiResolution(12);

    auto mapper = vtkSmartPointer<vtkPolyDataMapper>::New();
    mapper->SetInputConnection(sphere->GetOutputPort());

    auto actor = vtkSmartPointer<vtkActor>::New();
    actor->SetMapper(mapper);
    actor->GetProperty()->SetColor(1.0, 0.15, 0.15);

    renderer_->AddActor(actor);
    highlightActor_ = actor;
}

void PclVtkRenderer::clearHighlight()
{
    if (highlightActor_ && renderer_)
    {
        renderer_->RemoveActor(highlightActor_);
    }
    highlightActor_ = nullptr;
}

} // namespace cloud_viewer
