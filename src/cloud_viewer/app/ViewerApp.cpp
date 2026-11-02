#include "cloud_viewer/app/ViewerApp.h"

#include <algorithm>
#include <cctype>
#include <cmath>
#include <iostream>
#include <sstream>

#include <vtkAxesActor.h>
#include <vtkCallbackCommand.h>
#include <vtkInteractorStyleTrackballCamera.h>
#include <vtkObjectFactory.h>
#include <vtkOrientationMarkerWidget.h>
#include <vtkPNGWriter.h>
#include <vtkRenderer.h>
#include <vtkRenderWindow.h>
#include <vtkRenderWindowInteractor.h>
#include <vtkTextActor.h>
#include <vtkTextProperty.h>
#include <vtkWindowToImageFilter.h>

#include "cloud_viewer/io/Loaders.h"
#include "cloud_viewer/platform/FileDialog.h"

namespace cloud_viewer
{
namespace
{

/// Shift + 左键执行拾取，其余情况保持默认旋转/平移。
class SelectionStyle : public vtkInteractorStyleTrackballCamera
{
public:
    static SelectionStyle *New();
    vtkTypeMacro(SelectionStyle, vtkInteractorStyleTrackballCamera);

    void SetApp(ViewerApp *app) { app_ = app; }

    void OnLeftButtonDown() override
    {
        if (this->Interactor && app_)
        {
            int position[2] = {0, 0};
            this->Interactor->GetEventPosition(position);

            // UI 按钮优先于三维交互。
            if (app_->handleUiClick(position[0], position[1]))
            {
                return;
            }
            if (this->Interactor->GetShiftKey())
            {
                app_->selectAt(position[0], position[1]);
                return;
            }
        }
        vtkInteractorStyleTrackballCamera::OnLeftButtonDown();
    }

private:
    ViewerApp *app_ = nullptr;
};

vtkStandardNewMacro(SelectionStyle);

std::string formatFloat(float value, int precision = 3)
{
    std::ostringstream stream;
    stream.setf(std::ios::fixed);
    stream.precision(precision);
    stream << value;
    return stream.str();
}

} // namespace

void ViewerApp::setupInteraction()
{
    auto *interactor = renderer_.interactor();

    auto style = vtkSmartPointer<SelectionStyle>::New();
    style->SetApp(this);
    interactor->SetInteractorStyle(style);

    auto keyCallback = vtkSmartPointer<vtkCallbackCommand>::New();
    keyCallback->SetCallback([](vtkObject *caller, unsigned long, void *clientData, void *)
                             {
                                 auto *app = static_cast<ViewerApp *>(clientData);
                                 auto *iren = static_cast<vtkRenderWindowInteractor *>(caller);
                                 const char *key = iren->GetKeySym();
                                 if (app && key)
                                 {
                                     app->handleKey(key);
                                 }
                             });
    keyCallback->SetClientData(this);
    interactor->AddObserver(vtkCommand::KeyPressEvent, keyCallback);

    hudActor_ = vtkSmartPointer<vtkTextActor>::New();
    hudActor_->GetTextProperty()->SetFontSize(14);
    hudActor_->GetTextProperty()->SetColor(0.92, 0.92, 0.92);
    hudActor_->GetTextProperty()->SetBackgroundColor(0.0, 0.0, 0.0);
    hudActor_->GetTextProperty()->SetBackgroundOpacity(0.35);
    hudActor_->GetPositionCoordinate()->SetCoordinateSystemToNormalizedViewport();
    hudActor_->SetPosition(0.01, 0.68);
    renderer_.renderer()->AddActor2D(hudActor_);

    infoActor_ = vtkSmartPointer<vtkTextActor>::New();
    infoActor_->GetTextProperty()->SetFontSize(13);
    infoActor_->GetTextProperty()->SetColor(1.0, 0.85, 0.4);
    infoActor_->GetTextProperty()->SetBackgroundColor(0.0, 0.0, 0.0);
    infoActor_->GetTextProperty()->SetBackgroundOpacity(0.35);
    infoActor_->GetPositionCoordinate()->SetCoordinateSystemToNormalizedViewport();
    infoActor_->SetPosition(0.60, 0.55);
    renderer_.renderer()->AddActor2D(infoActor_);

    // 顶部工具栏：文件导入为第一入口，键盘仅为快捷方式。
    ui_.initialize(renderer_.renderer());
    constexpr double kButtonY = 0.925;
    constexpr double kButtonHeight = 0.052;

    ui_.addButton("Open Files", 0.010, kButtonY, 0.105, kButtonHeight,
                  [this] { openFilesDialog(); });
    ui_.addButton("Open Folder", 0.122, kButtonY, 0.108, kButtonHeight,
                  [this] { openDirectoryDialog(); });
    ui_.addButton("Prev", 0.238, kButtonY, 0.075, kButtonHeight,
                  [this] { stepFrame(-1); renderer_.render(); });
    ui_.addButton("Next", 0.320, kButtonY, 0.075, kButtonHeight,
                  [this] { stepFrame(1); renderer_.render(); });

    colorButtonIndex_ = ui_.size();
    ui_.addButton("Color: intensity", 0.402, kButtonY, 0.150, kButtonHeight,
                  [this] { nextColorSource(); renderer_.render(); });

    ui_.addButton("Reset", 0.560, kButtonY, 0.075, kButtonHeight,
                  [this]
                  {
                      renderer_.resetCamera();
                      renderer_.render();
                  });
    ui_.addButton("Hide", 0.643, kButtonY, 0.070, kButtonHeight,
                  [this] { toggleCurrentVisibility(); });
}

bool ViewerApp::handleUiClick(int displayX, int displayY)
{
    int *size = renderer_.window() ? renderer_.window()->GetSize() : nullptr;
    if (!size)
    {
        return false;
    }
    return ui_.handleClick(displayX, displayY, size[0], size[1]);
}

void ViewerApp::clearScene()
{
    scene_.clear();
    renderer_.removeAll();
    cache_.clear();
    files_.clear();

    currentCloudId_ = 0;
    currentFile_ = 0;
    currentFrame_ = 0;
    currentFrameHandle_.reset();
    currentRenderable_.reset();

    infoText_.clear();
    updateInfoPanel("");
}

void ViewerApp::importFiles(const std::filesystem::path &path)
{
    importFiles(std::vector<std::filesystem::path>{path});
}

void ViewerApp::importFiles(const std::vector<std::filesystem::path> &paths)
{
    if (paths.empty())
    {
        return;
    }

    clearScene();

    std::vector<std::string> list;
    list.reserve(paths.size());
    for (const auto &path : paths)
    {
        list.push_back(path.string());
    }

    if (!loadFiles(list))
    {
        updateHud();
        renderer_.render();
        return;
    }

    if (loadFrame(0, 0))
    {
        renderer_.resetCamera();
    }
    updateHud();
    renderer_.render();
}

void ViewerApp::openFilesDialog()
{
    if (!FileDialogService::available())
    {
        std::cerr << "[cloud_viewer] 未找到原生文件对话框后端（Linux 需要 zenity 或 kdialog）。\n";
        return;
    }

    const auto files = FileDialogService::openFiles();
    if (!files.empty())
    {
        importFiles(files);
    }
}

void ViewerApp::openDirectoryDialog()
{
    const auto directory = FileDialogService::chooseDirectory();
    if (!directory)
    {
        return;
    }

    std::vector<std::filesystem::path> files;
    std::error_code error;
    for (const auto &entry : std::filesystem::directory_iterator(*directory, error))
    {
        if (!entry.is_regular_file())
        {
            continue;
        }
        std::string extension = entry.path().extension().string();
        std::transform(extension.begin(), extension.end(), extension.begin(),
                       [](unsigned char ch) { return static_cast<char>(std::tolower(ch)); });
        if (extension == ".csv" || extension == ".txt" || extension == ".pcd")
        {
            files.push_back(entry.path());
        }
    }
    std::sort(files.begin(), files.end());

    if (files.empty())
    {
        std::cerr << "[cloud_viewer] 目录中没有 csv/txt/pcd 文件: " << directory->string() << "\n";
        return;
    }
    importFiles(files);
}

void ViewerApp::toggleCurrentVisibility()
{
    if (currentCloudId_ == 0)
    {
        return;
    }
    if (const SceneObject *object = scene_.find(currentCloudId_))
    {
        const bool visible = !object->visible;
        scene_.setVisible(currentCloudId_, visible);
        renderer_.setVisible(currentCloudId_, visible);
        updateHud();
        renderer_.render();
    }
}

bool ViewerApp::loadFiles(const std::vector<std::string> &files)
{
    LoaderRegistry &registry = LoaderRegistry::instance();
    LoadOptions options;

    for (const auto &file : files)
    {
        const std::filesystem::path path(file);
        ProbeResult probe;
        const IPointCloudLoader *loader = registry.selectLoader(path, options, probe);
        if (!loader)
        {
            std::cerr << "[cloud_viewer] 跳过无法识别的文件: " << file << "\n";
            continue;
        }

        FileFrameIndex index;
        std::string error;
        if (!registry.buildIndex(path, options, index, error))
        {
            std::cerr << "[cloud_viewer] 无法建立帧索引: " << file << " (" << error << ")\n";
            continue;
        }

        LoadedFile loaded;
        loaded.path = path;
        loaded.probe = probe;
        loaded.loader = loader;
        loaded.index = std::move(index);
        files_.push_back(std::move(loaded));
    }

    return !files_.empty();
}

FrameHandle ViewerApp::acquireFrame(LoadedFile &file, const FrameDescriptor &descriptor)
{
    const auto cached = cache_.find(descriptor.id);
    if (cached != cache_.end())
    {
        return cached->second;
    }

    LoadOptions options;
    std::string error;
    FrameHandle frame = LoaderRegistry::instance().decode(descriptor, options, file.loader, error);
    if (!frame)
    {
        std::cerr << "[cloud_viewer] 解码帧失败: " << error << "\n";
        return nullptr;
    }

    if (cache_.size() >= cacheBudget_)
    {
        // 简化 LRU：保留当前帧，清空其余缓存。
        auto keep = cache_.find(currentFrameHandle_ ? currentFrameHandle_->id : 0);
        if (keep != cache_.end())
        {
            FrameHandle kept = keep->second;
            cache_.clear();
            cache_[kept->id] = kept;
        }
        else
        {
            cache_.clear();
        }
    }

    cache_[descriptor.id] = frame;
    return frame;
}

bool ViewerApp::loadFrame(std::size_t fileIndex, std::size_t frameIndex)
{
    if (fileIndex >= files_.size())
    {
        return false;
    }
    LoadedFile &file = files_[fileIndex];
    if (file.index.frames.empty())
    {
        return false;
    }

    frameIndex = std::min(frameIndex, file.index.frames.size() - 1);
    const FrameDescriptor &descriptor = file.index.frames[frameIndex];
    FrameHandle frame = acquireFrame(file, descriptor);
    if (!frame)
    {
        return false;
    }

    currentFile_ = fileIndex;
    currentFrame_ = frameIndex;
    currentFrameHandle_ = frame;

    if (currentCloudId_ != 0)
    {
        renderer_.remove(currentCloudId_);
        scene_.remove(currentCloudId_);
        currentCloudId_ = 0;
    }

    RenderRequest request;
    currentRenderable_ = CloudRenderBridge::prepare(frame, request);

    SceneObject object;
    object.name = file.path.filename().string() + " #" + std::to_string(descriptor.frameNumber);
    object.kind = SceneObjectKind::Cloud;
    object.frame = frame;
    object.renderable = currentRenderable_;
    currentCloudId_ = scene_.add(0, std::move(object));

    applyColor();
    return true;
}

void ViewerApp::applyColor()
{
    if (!currentFrameHandle_ || !currentRenderable_ || currentCloudId_ == 0)
    {
        return;
    }

    ColorSettings settings;
    settings.source = colorSource_;
    settings.mode = ColorMapper::isCategorical(colorSource_)
                        ? ColorSettings::Mode::Categorical
                        : ColorSettings::Mode::Continuous;

    ColorArray colors;
    const auto field = currentFrameHandle_->scalarFields.find(colorSource_);
    if (colorSource_ == ColorSource::SolidColor || field == currentFrameHandle_->scalarFields.end())
    {
        settings.mode = ColorSettings::Mode::Solid;
        colors = ColorMapper::map(ScalarField{}, settings);
        // 空标量场时按显示点数生成固定色。
        colors.colors.assign(currentRenderable_->displayed, settings.solidColor);
        lastRange_ = ColorRange{};
    }
    else
    {
        colors = ColorMapper::map(field->second, settings);
        lastRange_ = colors.range;
        // 标量场按原始矩阵组织，需按显示点重排。
        std::vector<std::array<std::uint8_t, 3>> displayColors;
        displayColors.reserve(currentRenderable_->displayed);
        const std::size_t cols = static_cast<std::size_t>(
            currentFrameHandle_->metadata.cols > 0 ? currentFrameHandle_->metadata.cols : 0);
        for (const auto &ref : currentRenderable_->displayToSource)
        {
            const std::size_t index =
                static_cast<std::size_t>(ref.matrixRow) * cols +
                static_cast<std::size_t>(ref.matrixCol);
            if (index < colors.colors.size())
            {
                displayColors.push_back(colors.colors[index]);
            }
            else
            {
                displayColors.push_back(settings.invalidColor);
            }
        }
        colors.colors = std::move(displayColors);
    }

    renderer_.upsert(currentCloudId_, currentRenderable_, colors);
    updateHud();
}

void ViewerApp::nextColorSource()
{
    static const std::vector<ColorSource> order = {
        ColorSource::Range,
        ColorSource::Intensity,
        ColorSource::Reflectivity,
        ColorSource::Peak,
        ColorSource::X,
        ColorSource::Y,
        ColorSource::Z,
        ColorSource::Theta,
        ColorSource::SlotAngle,
        ColorSource::EchoIndex,
        ColorSource::IsTailPoint,
        ColorSource::IsValid,
        ColorSource::Timestamp,
        ColorSource::SolidColor};

    if (!currentFrameHandle_)
    {
        return;
    }

    std::size_t start = 0;
    for (std::size_t i = 0; i < order.size(); ++i)
    {
        if (order[i] == colorSource_)
        {
            start = i;
            break;
        }
    }

    for (std::size_t step = 1; step <= order.size(); ++step)
    {
        const ColorSource candidate = order[(start + step) % order.size()];
        if (candidate == ColorSource::SolidColor ||
            currentFrameHandle_->availability.has(candidate))
        {
            colorSource_ = candidate;
            applyColor();
            return;
        }
    }
}

void ViewerApp::stepFrame(int delta)
{
    if (files_.empty() || frozen_)
    {
        return;
    }
    LoadedFile &file = files_[currentFile_];
    if (file.index.frames.empty())
    {
        return;
    }

    const int count = static_cast<int>(file.index.frames.size());
    int next = static_cast<int>(currentFrame_) + delta;
    next = (next % count + count) % count;

    loadFrame(currentFile_, static_cast<std::size_t>(next));
    renderer_.render();
}

void ViewerApp::handleKey(const std::string &key)
{
    if (key == "Right")
    {
        stepFrame(1);
    }
    else if (key == "Left")
    {
        stepFrame(-1);
    }
    else if (key == "Tab")
    {
        if (files_.size() > 1)
        {
            loadFrame((currentFile_ + 1) % files_.size(), 0);
            renderer_.render();
        }
    }
    else if (key == "c" || key == "C")
    {
        nextColorSource();
        renderer_.render();
    }
    else if (key == "r" || key == "R")
    {
        renderer_.resetCamera();
        renderer_.render();
    }
    else if (key == "n" || key == "N")
    {
        if (const SceneObject *object = scene_.find(currentCloudId_))
        {
            const bool visible = !object->visible;
            scene_.setVisible(currentCloudId_, visible);
            renderer_.setVisible(currentCloudId_, visible);
            renderer_.render();
        }
    }
    else if (key == "plus" || key == "equal")
    {
        pointSize_ = std::min(10.0f, pointSize_ + 1.0f);
        renderer_.setPointSize(pointSize_);
        renderer_.render();
    }
    else if (key == "minus")
    {
        pointSize_ = std::max(1.0f, pointSize_ - 1.0f);
        renderer_.setPointSize(pointSize_);
        renderer_.render();
    }
    else if (key == "space")
    {
        frozen_ = !frozen_;
        updateHud();
        renderer_.render();
    }
    else if (key == "Escape" || key == "q" || key == "Q")
    {
        renderer_.interactor()->TerminateApp();
    }
}

void ViewerApp::selectAt(int displayX, int displayY)
{
    SceneObjectId objectId = 0;
    vtkIdType pointId = -1;
    double world[3] = {0.0, 0.0, 0.0};

    if (!renderer_.pick(displayX, displayY, objectId, pointId, world))
    {
        return;
    }

    const SceneObject *object = scene_.find(objectId);
    if (!object || !object->renderable || !object->frame)
    {
        return;
    }
    if (pointId < 0 ||
        static_cast<std::size_t>(pointId) >= object->renderable->displayToSource.size())
    {
        return;
    }

    const SourcePointRef ref = object->renderable->displayToSource[static_cast<std::size_t>(pointId)];
    const auto &matrix = object->frame->points;
    if (ref.matrixRow < 0 || ref.matrixCol < 0 ||
        static_cast<std::size_t>(ref.matrixRow) >= matrix.size() ||
        ref.matrixCol >= static_cast<int>(matrix[ref.matrixRow].size()))
    {
        return;
    }

    const algo_lidar::PointData &point = matrix[ref.matrixRow][ref.matrixCol];

    std::ostringstream text;
    text << "Picked point\n";
    text << "file: " << object->frame->metadata.sourcePath.filename().string() << "\n";
    text << "frame: " << object->frame->metadata.frameNumber
         << "  row/col: " << ref.matrixRow << "/" << ref.matrixCol << "\n";
    text << "xyz: " << formatFloat(point.x) << ", " << formatFloat(point.y)
         << ", " << formatFloat(point.z) << " m\n";
    text << "r: " << formatFloat(point.r) << " m\n";
    text << "theta/slot_ang: " << formatFloat(point.theta) << " / "
         << formatFloat(point.slot_ang) << " deg\n";
    text << "intensity: " << static_cast<int>(point.intensity)
         << "  reflect: " << formatFloat(point.reflect) << "\n";
    text << "peak: " << point.peak << "  select_idx: " << static_cast<int>(point.select_idx) << "\n";
    text << "pixel/slot: " << point.pixel_idx << "/" << point.slot_idx << "\n";
    text << "timestamp: " << point.timestamp << "\n";
    text << "valid: " << (point.is_valid ? "yes" : "no")
         << "  tail: " << (point.is_tail_pt ? "yes" : "no") << "\n";
    text << "echoes:";
    for (const auto &pulse : point.list_pulse)
    {
        text << " [tof=" << formatFloat(pulse.tof, 2)
             << " width=" << formatFloat(pulse.width, 2)
             << " peak=" << formatFloat(pulse.peak, 1) << "]";
    }

    infoText_ = text.str();
    updateInfoPanel(infoText_);
    renderer_.highlightPoint(world);
    renderer_.render();

    std::cout << infoText_ << std::endl;
}

void ViewerApp::updateInfoPanel(const std::string &text)
{
    if (infoActor_)
    {
        infoActor_->SetInput(text.c_str());
    }
}

void ViewerApp::updateHud()
{
    if (!hudActor_)
    {
        return;
    }

    std::ostringstream text;
    if (files_.empty())
    {
        text << "No point cloud loaded\n";
        text << "Click \"Open Files\" or \"Open Folder\" to import CSV / TXT / PCD\n";
        text << "Keys: C color  R reset  +/- size  Shift+LMB pick\n";
    }
    else
    {
        if (currentFile_ < files_.size())
        {
            const LoadedFile &file = files_[currentFile_];
            text << "File: " << file.path.filename().string()
                 << " (" << (currentFile_ + 1) << "/" << files_.size() << ")"
                 << "  Frame: " << (currentFrame_ + 1) << "/" << file.index.frames.size() << "\n";
        }
        text << "Color: " << colorSourceDisplayName(colorSource_);
        if (lastRange_.valid)
        {
            text << "  [" << formatFloat(lastRange_.minValue, 3) << ", "
                 << formatFloat(lastRange_.maxValue, 3) << "] n=" << lastRange_.finiteCount;
        }
        text << "\n";
        if (currentRenderable_)
        {
            text << "Points: " << currentRenderable_->displayed << "/"
                 << currentRenderable_->sourceTotal
                 << "  LOD: " << ((currentRenderable_->displayed < currentRenderable_->sourceTotal)
                                       ? "on"
                                       : "off")
                 << "\n";
        }
        text << "Keys: L/R frame  Tab file  C color  N hide  R reset  +/- size  Space "
             << (frozen_ ? "unfreeze" : "freeze")
             << "\nShift+LMB pick point";
    }
    hudActor_->SetInput(text.str().c_str());

    if (colorButtonIndex_ < ui_.size())
    {
        ui_.setLabel(colorButtonIndex_, std::string("Color: ") + colorSourceName(colorSource_));
    }
}

int ViewerApp::run(const std::vector<std::string> &files, double fps,
                   const std::string &screenshotPath)
{
    if (!renderer_.initialize(1280, 800))
    {
        return 11;
    }

    setupInteraction();

    // 允许空场景启动：用户通过顶部按钮导入文件。
    if (!files.empty())
    {
        if (!loadFiles(files))
        {
            std::cerr << "[cloud_viewer] 初始文件加载失败，可点击 Open Files 重新选择。\n";
        }
        else if (loadFrame(0, 0))
        {
            renderer_.resetCamera();
        }
    }

    auto *interactor = renderer_.interactor();
    interactor->Initialize();

    // 左下角坐标轴。
    auto axes = vtkSmartPointer<vtkAxesActor>::New();
    auto marker = vtkSmartPointer<vtkOrientationMarkerWidget>::New();
    marker->SetOrientationMarker(axes);
    marker->SetInteractor(interactor);
    marker->SetViewport(0.0, 0.0, 0.15, 0.15);
    marker->SetEnabled(1);
    marker->InteractiveOn();

    if (fps > 0.0)
    {
        auto timerCallback = vtkSmartPointer<vtkCallbackCommand>::New();
        timerCallback->SetCallback([](vtkObject *, unsigned long, void *clientData, void *)
                                   {
                                       auto *app = static_cast<ViewerApp *>(clientData);
                                       if (app)
                                       {
                                           app->stepFrame(1);
                                       }
                                   });
        timerCallback->SetClientData(this);
        interactor->AddObserver(vtkCommand::TimerEvent, timerCallback);
        interactor->CreateRepeatingTimer(1000.0 / fps);
    }

    updateHud();
    renderer_.render();

    if (!screenshotPath.empty())
    {
        renderer_.window()->SetOffScreenRendering(1);
        renderer_.render();

        auto grabber = vtkSmartPointer<vtkWindowToImageFilter>::New();
        grabber->SetInput(renderer_.window());
        grabber->SetInputBufferTypeToRGB();
        grabber->ReadFrontBufferOff();
        grabber->Update();

        auto writer = vtkSmartPointer<vtkPNGWriter>::New();
        writer->SetFileName(screenshotPath.c_str());
        writer->SetInputConnection(grabber->GetOutputPort());
        writer->Write();

        std::cout << "[cloud_viewer] 截图已保存: " << screenshotPath << "\n";
        return 0;
    }

    interactor->Start();
    return 0;
}

} // namespace cloud_viewer
