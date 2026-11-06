/**
 * @brief VTK 渲染器：负责 Actor 生命周期、相机与拾取
 *
 * 本层只持有显示派生数据（RenderableCloud），不持有 PointMatrix 真值。
 */
#pragma once

#include <array>
#include <map>

#include <vtkSmartPointer.h>

#include "cloud_viewer/color/ColorMapper.h"
#include "cloud_viewer/render/CloudRenderBridge.h"

class vtkActor;
class vtkCellArray;
class vtkFloatArray;
class vtkIdTypeArray;
class vtkPoints;
class vtkPolyData;
class vtkPolyDataMapper;
class vtkRenderer;
class vtkRenderWindow;
class vtkRenderWindowInteractor;
class vtkUnsignedCharArray;

namespace cloud_viewer
{

struct RenderSettings
{
    float pointSize = 2.0f;
    std::array<double, 3> background{{37.0 / 255.0, 37.0 / 255.0, 37.0 / 255.0}};
};

/// 一个实体的持久渲染槽位。
/// 点云每秒更新 20 次，重建 vtkPoints/CellArray/Mapper 会让交互掉帧，
/// 所以这里缓存整条管线，每帧只覆盖底层数组。
struct ActorSlot
{
    vtkSmartPointer<vtkActor> actor;
    vtkSmartPointer<vtkPolyDataMapper> mapper;
    vtkSmartPointer<vtkPolyData> polyData;
    vtkSmartPointer<vtkPoints> points;
    vtkSmartPointer<vtkFloatArray> coords;   // xyz 3 分量，直接写指针
    vtkSmartPointer<vtkCellArray> verts;     // 每点一个 vertex cell
    vtkSmartPointer<vtkIdTypeArray> vertOffsets;
    vtkSmartPointer<vtkIdTypeArray> vertConn;
    vtkSmartPointer<vtkUnsignedCharArray> colors;
    vtkIdType capacity = 0;    // coords/colors/verts 已分配容量
    bool visible = true;
};

class PclVtkRenderer
{
public:
    bool initialize(int width = 1280, int height = 800);

    /// 新增或更新一个实体对应的显示云。
    void upsert(SceneObjectId id, const RenderableCloud::Ptr &cloud, const ColorArray &colors);

    void remove(SceneObjectId id);

    /// 清除全部点云 Actor（用于重新导入）。
    void removeAll();

    void setVisible(SceneObjectId id, bool visible);

    void setPointSize(float size);

    void setBackground(double r, double g, double b);

    void resetCamera();

    void render();

    /// 屏幕坐标拾取。返回 true 时填充实体、显示点索引与三维坐标。
    bool pick(int displayX, int displayY, SceneObjectId &objectId,
              vtkIdType &pointId, double world[3]) const;

    /// 用独立红色小球高亮选中点，不修改点云原始颜色。
    void highlightPoint(const double world[3]);

    void clearHighlight();

    vtkRenderer *renderer() const { return renderer_; }
    vtkRenderWindow *window() const { return window_; }
    vtkRenderWindowInteractor *interactor() const { return interactor_; }

private:
    void applyPointSize();

    vtkSmartPointer<vtkRenderer> renderer_;
    vtkSmartPointer<vtkRenderWindow> window_;
    vtkSmartPointer<vtkRenderWindowInteractor> interactor_;

    std::map<SceneObjectId, ActorSlot> actors_;
    std::map<vtkActor *, SceneObjectId> actorToObject_;

    vtkSmartPointer<vtkActor> highlightActor_;
    float pointSize_ = 2.0f;
};

} // namespace cloud_viewer
