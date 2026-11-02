/**
 * @brief 应用控制器：数据源切换、帧切换、着色、选点与 UI 状态
 */
#pragma once

#include <filesystem>
#include <map>
#include <string>
#include <vector>

#include <vtkSmartPointer.h>

#include "cloud_viewer/color/ColorMapper.h"
#include "cloud_viewer/core/CloudFrame.h"
#include "cloud_viewer/io/LoaderTypes.h"
#include "cloud_viewer/render/CloudRenderBridge.h"
#include "cloud_viewer/render/PclVtkRenderer.h"
#include "cloud_viewer/render/UiOverlay.h"
#include "cloud_viewer/scene/SceneGraph.h"

class vtkTextActor;

namespace cloud_viewer
{

struct LoadedFile
{
    std::filesystem::path path;
    ProbeResult probe;
    const IPointCloudLoader *loader = nullptr;
    FileFrameIndex index;
};

class ViewerApp
{
public:
    /// fps > 0 时启用 mock 定时推帧（模拟在线数据源）；fps = 0 为纯离线交互。
    /// screenshotPath 非空时渲染一帧并保存 PNG，随后退出（用于无头验收）。
    int run(const std::vector<std::string> &files, double fps = 0.0,
            const std::string &screenshotPath = "");

    /// Shift + 左键触发：拾取并显示原始点信息。
    void selectAt(int displayX, int displayY);
    void handleKey(const std::string &key);
    void stepFrame(int delta);
    void nextColorSource();

    /// 按钮命中检测；返回 true 表示点击已被 UI 消费。
    bool handleUiClick(int displayX, int displayY);

    /// 通过对话框选择并导入文件/目录。
    void importFiles(const std::filesystem::path &path);
    void importFiles(const std::vector<std::filesystem::path> &paths);

private:
    void openFilesDialog();
    void openDirectoryDialog();
    void clearScene();
    void toggleCurrentVisibility();

    bool loadFiles(const std::vector<std::string> &files);
    bool loadFrame(std::size_t fileIndex, std::size_t frameIndex);
    FrameHandle acquireFrame(LoadedFile &file, const FrameDescriptor &descriptor);
    void applyColor();
    void rebuildCurrentObject();
    void updateHud();
    void updateInfoPanel(const std::string &text);
    void setupInteraction();

    SceneGraph scene_;
    PclVtkRenderer renderer_;
    RenderSettings renderSettings_;

    std::vector<LoadedFile> files_;
    std::size_t currentFile_ = 0;
    std::size_t currentFrame_ = 0;

    FrameHandle currentFrameHandle_;
    RenderableCloud::Ptr currentRenderable_;
    SceneObjectId currentCloudId_ = 0;

    ColorSource colorSource_ = ColorSource::Intensity;
    ColorSettings colorSettings_;
    ColorRange lastRange_;
    float pointSize_ = 2.0f;

    std::map<FrameId, FrameHandle> cache_;
    std::size_t cacheBudget_ = 8;

    UiOverlay ui_;
    std::size_t colorButtonIndex_ = 0;

    vtkSmartPointer<vtkTextActor> hudActor_;
    vtkSmartPointer<vtkTextActor> infoActor_;
    std::string infoText_;
    bool frozen_ = false;
};

} // namespace cloud_viewer
