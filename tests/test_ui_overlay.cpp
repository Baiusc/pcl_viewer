#include <iostream>

#include <vtkRenderer.h>
#include <vtkSmartPointer.h>

#include "cloud_viewer/render/UiOverlay.h"

int main()
{
    auto renderer = vtkSmartPointer<vtkRenderer>::New();

    cloud_viewer::UiOverlay ui;
    ui.initialize(renderer);

    int hits = 0;
    ui.addButton("Prev", 0.10, 0.90, 0.20, 0.05, [&hits] { ++hits; });

    // 命中按钮：nx=0.12, ny=0.94。
    if (!ui.handleClick(120, 940, 1000, 1000) || hits != 1)
    {
        std::cerr << "UI_OVERLAY_TEST_FAIL hit branch\n";
        return 1;
    }

    // 未命中：应返回 false 且不触发回调。
    if (ui.handleClick(500, 100, 1000, 1000) || hits != 1)
    {
        std::cerr << "UI_OVERLAY_TEST_FAIL miss branch\n";
        return 2;
    }

    std::cout << "UI_OVERLAY_TEST_PASS hits=" << hits << std::endl;
    return 0;
}
