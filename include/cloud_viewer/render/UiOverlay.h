/**
 * @brief 轻量 2D 按钮层
 *
 * 不引入 Qt/ImGui：用 VTK 2D Actor 绘制按钮矩形与文本，
 * 通过归一化视口坐标做命中检测。命中按钮时不再传给三维相机。
 */
#pragma once

#include <cstddef>
#include <functional>
#include <string>
#include <vector>

#include <vtkSmartPointer.h>

class vtkActor2D;
class vtkRenderer;
class vtkTextActor;

namespace cloud_viewer
{

struct UiButton
{
    double x = 0.0;
    double y = 0.0;
    double width = 0.0;
    double height = 0.0;
    std::string label;
    std::function<void()> onClick;

    vtkSmartPointer<vtkActor2D> background;
    vtkSmartPointer<vtkTextActor> text;
};

class UiOverlay
{
public:
    void initialize(vtkRenderer *renderer);

    void addButton(const std::string &label, double x, double y, double width, double height,
                   std::function<void()> onClick);

    /// 返回 true 表示点击被 UI 消费，调用方不应再旋转/拾取。
    bool handleClick(int displayX, int displayY, int windowWidth, int windowHeight);

    void setLabel(std::size_t index, const std::string &label);

    std::size_t size() const { return buttons_.size(); }

private:
    vtkRenderer *renderer_ = nullptr;
    std::vector<UiButton> buttons_;
};

} // namespace cloud_viewer
