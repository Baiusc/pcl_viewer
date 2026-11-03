#include "cloud_viewer/render/UiOverlay.h"

#include <cstdlib>
#include <iostream>

#include <vtkActor2D.h>
#include <vtkCellArray.h>
#include <vtkCoordinate.h>
#include <vtkPoints.h>
#include <vtkPolyData.h>
#include <vtkPolyDataMapper2D.h>
#include <vtkProperty.h>
#include <vtkProperty2D.h>
#include <vtkRenderer.h>
#include <vtkTextActor.h>
#include <vtkTextProperty.h>

namespace cloud_viewer
{

void UiOverlay::initialize(vtkRenderer *renderer)
{
    renderer_ = renderer;
    buttons_.clear();
}

void UiOverlay::addButton(const std::string &label, double x, double y, double width,
                          double height, std::function<void()> onClick)
{
    if (!renderer_)
    {
        return;
    }

    UiButton button;
    button.x = x;
    button.y = y;
    button.width = width;
    button.height = height;
    button.label = label;
    button.onClick = std::move(onClick);

    // 背景矩形（归一化视口坐标）。
    auto points = vtkSmartPointer<vtkPoints>::New();
    points->InsertNextPoint(x, y, 0.0);
    points->InsertNextPoint(x + width, y, 0.0);
    points->InsertNextPoint(x + width, y + height, 0.0);
    points->InsertNextPoint(x, y + height, 0.0);

    auto cells = vtkSmartPointer<vtkCellArray>::New();
    cells->InsertNextCell(4);
    for (vtkIdType i = 0; i < 4; ++i)
    {
        cells->InsertCellPoint(i);
    }

    auto polyData = vtkSmartPointer<vtkPolyData>::New();
    polyData->SetPoints(points);
    polyData->SetPolys(cells);

    auto coordinate = vtkSmartPointer<vtkCoordinate>::New();
    coordinate->SetCoordinateSystemToNormalizedViewport();

    auto mapper = vtkSmartPointer<vtkPolyDataMapper2D>::New();
    mapper->SetInputData(polyData);
    mapper->SetTransformCoordinate(coordinate);

    auto background = vtkSmartPointer<vtkActor2D>::New();
    background->SetMapper(mapper);
    background->GetProperty()->SetColor(0.16, 0.19, 0.24);
    background->GetProperty()->SetOpacity(0.92);
    renderer_->AddActor2D(background);
    button.background = background;

    auto text = vtkSmartPointer<vtkTextActor>::New();
    text->GetTextProperty()->SetFontSize(12);
    text->GetTextProperty()->SetColor(0.95, 0.95, 0.95);
    text->GetTextProperty()->SetJustificationToCentered();
    text->GetTextProperty()->SetVerticalJustificationToCentered();
    text->GetTextProperty()->SetBold(1);
    text->GetPositionCoordinate()->SetCoordinateSystemToNormalizedViewport();
    text->SetPosition(x + width * 0.5, y + height * 0.5);
    text->SetInput(label.c_str());
    renderer_->AddActor2D(text);
    button.text = text;

    buttons_.push_back(std::move(button));
}

bool UiOverlay::handleClick(int displayX, int displayY, int windowWidth, int windowHeight)
{
    if (windowWidth <= 0 || windowHeight <= 0)
    {
        return false;
    }

    const double nx = static_cast<double>(displayX) / static_cast<double>(windowWidth);
    const double ny = static_cast<double>(displayY) / static_cast<double>(windowHeight);

    for (auto &button : buttons_)
    {
        if (nx >= button.x && nx <= button.x + button.width &&
            ny >= button.y && ny <= button.y + button.height)
        {
            std::cout << "[ui] click \"" << button.label << "\" at (" << displayX << ","
                      << displayY << ")" << std::endl;
            if (button.onClick)
            {
                button.onClick();
            }
            return true;
        }
    }

    // 未命中按钮默认不打印，避免拖动/旋转时刷屏；排查点击偏移时设 CV_UI_DEBUG=1。
    const char *debug = std::getenv("CV_UI_DEBUG");
    if (debug != nullptr && debug[0] == '1')
    {
        std::cout << "[ui] click at (" << displayX << "," << displayY
                  << ") missed all buttons" << std::endl;
    }
    return false;
}

void UiOverlay::setLabel(std::size_t index, const std::string &label)
{
    if (index >= buttons_.size())
    {
        return;
    }
    buttons_[index].label = label;
    if (buttons_[index].text)
    {
        buttons_[index].text->SetInput(label.c_str());
    }
}

void UiOverlay::removeButtonsFrom(std::size_t index)
{
    if (index >= buttons_.size())
    {
        return;
    }
    for (auto it = buttons_.begin() + static_cast<std::ptrdiff_t>(index); it != buttons_.end(); ++it)
    {
        if (renderer_ != nullptr)
        {
            if (it->background)
            {
                renderer_->RemoveActor2D(it->background);
            }
            if (it->text)
            {
                renderer_->RemoveActor2D(it->text);
            }
        }
    }
    buttons_.erase(buttons_.begin() + static_cast<std::ptrdiff_t>(index), buttons_.end());
}

} // namespace cloud_viewer
