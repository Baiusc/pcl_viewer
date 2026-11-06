#include "cloud_viewer/render/PclVtkRenderer.h"

#include <algorithm>

#include <vtkActor.h>
#include <vtkCamera.h>
#include <vtkCellArray.h>
#include <vtkFloatArray.h>
#include <vtkIdTypeArray.h>
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
    const auto count = static_cast<vtkIdType>(points.size());
    if (count == 0)
    {
        remove(id);
        return;
    }

    ActorSlot &slot = actors_[id];
    if (!slot.actor)
    {
        slot.coords = vtkSmartPointer<vtkFloatArray>::New();
        slot.coords->SetName("Points");
        slot.coords->SetNumberOfComponents(3);

        slot.points = vtkSmartPointer<vtkPoints>::New();
        slot.points->SetDataTypeToFloat();
        slot.points->SetData(slot.coords);

        slot.vertOffsets = vtkSmartPointer<vtkIdTypeArray>::New();
        slot.vertConn = vtkSmartPointer<vtkIdTypeArray>::New();
        slot.verts = vtkSmartPointer<vtkCellArray>::New();

        slot.colors = vtkSmartPointer<vtkUnsignedCharArray>::New();
        slot.colors->SetName("Colors");
        slot.colors->SetNumberOfComponents(3);

        slot.polyData = vtkSmartPointer<vtkPolyData>::New();
        slot.polyData->SetPoints(slot.points);
        slot.polyData->GetPointData()->SetScalars(slot.colors);

        slot.mapper = vtkSmartPointer<vtkPolyDataMapper>::New();
        slot.mapper->SetInputData(slot.polyData);
        slot.mapper->ScalarVisibilityOn();
        slot.mapper->SetColorModeToDirectScalars();

        slot.actor = vtkSmartPointer<vtkActor>::New();
        slot.actor->SetMapper(slot.mapper);
        slot.actor->GetProperty()->SetPointSize(pointSize_);

        renderer_->AddActor(slot.actor);
        actorToObject_[slot.actor] = id;
    }

    // 容量不足才扩容（预留 25%），避免每帧重新分配。
    if (count > slot.capacity)
    {
        const vtkIdType capacity = count + count / 4 + 1024;
        slot.coords->SetNumberOfTuples(capacity);
        slot.colors->SetNumberOfTuples(capacity);

        // 每点一个 vertex cell：offsets = 0,1,2,...,n；connectivity = 0,1,...,n-1。
        // 预生成到容量上限，之后按 count 截断即可复用，不需要每帧重建拓扑。
        slot.vertOffsets->SetNumberOfTuples(capacity + 1);
        vtkIdType *offsets = slot.vertOffsets->GetPointer(0);
        for (vtkIdType i = 0; i <= capacity; ++i)
        {
            offsets[i] = i;
        }

        slot.vertConn->SetNumberOfTuples(capacity);
        vtkIdType *conn = slot.vertConn->GetPointer(0);
        for (vtkIdType i = 0; i < capacity; ++i)
        {
            conn[i] = i;
        }

        slot.capacity = capacity;
    }

    // 坐标：直接写底层数组，省掉 76800 次 SetPoint 虚调用。
    slot.coords->SetNumberOfTuples(count);
    float *dst = static_cast<float *>(slot.coords->GetVoidPointer(0));
    for (vtkIdType i = 0; i < count; ++i)
    {
        const auto &p = points[static_cast<std::size_t>(i)];
        const vtkIdType base = 3 * i;
        dst[base + 0] = p.x;
        dst[base + 1] = p.y;
        dst[base + 2] = p.z;
    }

    // 颜色：同样直接写标量数组。
    slot.colors->SetNumberOfTuples(count);
    auto *rgb = static_cast<unsigned char *>(slot.colors->GetVoidPointer(0));
    const std::size_t colorCount = colors.colors.size();
    for (vtkIdType i = 0; i < count; ++i)
    {
        const std::size_t index = static_cast<std::size_t>(i);
        const std::array<std::uint8_t, 3> color =
            (index < colorCount) ? colors.colors[index]
                                 : std::array<std::uint8_t, 3>{{200, 200, 200}};
        const vtkIdType base = 3 * i;
        rgb[base + 0] = color[0];
        rgb[base + 1] = color[1];
        rgb[base + 2] = color[2];
    }

    // 拓扑：数组内容已预生成，截断到 count 即可。
    slot.vertOffsets->SetNumberOfTuples(count + 1);
    slot.vertConn->SetNumberOfTuples(count);
    slot.verts->SetData(slot.vertOffsets, slot.vertConn);
    slot.polyData->SetVerts(slot.verts);

    slot.coords->Modified();
    slot.colors->Modified();
    slot.points->Modified();
    slot.polyData->Modified();
}

void PclVtkRenderer::remove(SceneObjectId id)
{
    const auto it = actors_.find(id);
    if (it == actors_.end())
    {
        return;
    }
    if (renderer_ && it->second.actor)
    {
        renderer_->RemoveActor(it->second.actor);
        actorToObject_.erase(it->second.actor);
    }
    actors_.erase(it);
}

void PclVtkRenderer::removeAll()
{
    for (auto &entry : actors_)
    {
        if (renderer_ && entry.second.actor)
        {
            renderer_->RemoveActor(entry.second.actor);
        }
    }
    actors_.clear();
    actorToObject_.clear();
    clearHighlight();
}

void PclVtkRenderer::setVisible(SceneObjectId id, bool visible)
{
    const auto it = actors_.find(id);
    if (it != actors_.end() && it->second.actor)
    {
        it->second.visible = visible;
        it->second.actor->SetVisibility(visible ? 1 : 0);
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
        if (entry.second.actor)
        {
            entry.second.actor->GetProperty()->SetPointSize(pointSize_);
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
