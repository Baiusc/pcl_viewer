#include "cloud_viewer/app/ViewerApp.h"

#include <algorithm>
#include <cctype>
#include <chrono>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <fstream>
#include <iostream>
#include <sstream>
#include <thread>

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
#include "cloud_viewer/preprocess/OnlineCalibration.h"
#include "cloud_viewer/platform/FileDialog.h"
#include "cloud_viewer/platform/AppSettings.h"
#include "cloud_viewer/platform/NetworkSetup.h"
#include "cloud_viewer/platform/RadarLauncher.h"

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

std::uint64_t nowMs()
{
    return static_cast<std::uint64_t>(
        std::chrono::duration_cast<std::chrono::milliseconds>(
            std::chrono::steady_clock::now().time_since_epoch())
            .count());
}

std::string shellQuote(const std::string &text)
{
    std::string out = "'";
    for (char ch : text)
    {
        if (ch == '\'')
        {
            out += "'\\''";
        }
        else
        {
            out.push_back(ch);
        }
    }
    out += "'";
    return out;
}

std::string faceMaskLabel(std::uint8_t mask)
{
    if ((mask & 0x0F) == 0x0F)
    {
        return "All";
    }
    std::string text;
    for (int index = 0; index < 4; ++index)
    {
        if ((mask & (1u << index)) != 0)
        {
            if (!text.empty())
            {
                text += "+";
            }
            text += "F" + std::to_string(index + 1);
        }
    }
    return text.empty() ? "All" : text;
}

/// 统一按钮动作日志，便于在控制台还原“点了什么、做了什么、结果如何”。
void logUi(const std::string &action, const std::string &detail, bool ok = true)
{
    std::cout << "[ui] " << action << (ok ? " OK" : " FAILED");
    if (!detail.empty())
    {
        std::cout << " | " << detail;
    }
    std::cout << std::endl;
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
                      logUi("Reset", "相机已重置到当前点云包络");
                  });
    ui_.addButton("Hide", 0.643, kButtonY, 0.070, kButtonHeight,
                  [this] { toggleCurrentVisibility(); });

    connectButtonIndex_ = ui_.size();
    ui_.addButton("Connect Radar", 0.720, kButtonY, 0.130, kButtonHeight,
                  [this] { toggleRadarConnection(); });
    searchButtonIndex_ = ui_.size();
    ui_.addButton("Search Device", 0.858, kButtonY, 0.136, kButtonHeight,
                  [this] { searchDevices(); });
    modeButtonIndex_ = ui_.size();
    ui_.addButton("Mode: Cloud", 0.010, 0.863, 0.18, kButtonHeight,
                  [this] { switchRadarMode(); });
    roiButtonIndex_ = ui_.size();
    ui_.addButton("ROI: 0", 0.200, 0.863, 0.13, kButtonHeight,
                  [this] { switchRadarRoi(); });
    ui_.addButton("Select Calib", 0.340, 0.863, 0.19, kButtonHeight,
                  [this] { loadLiveCalibration(); });
    calibrationButtonIndex_ = ui_.size();
    ui_.addButton("App Cal: OFF", 0.540, 0.863, 0.26, kButtonHeight,
                  [this] { toggleLiveCalibration(); });
    faceButtonIndex_ = ui_.size();
    ui_.addButton("Faces: All", 0.808, 0.863, 0.185, kButtonHeight,
                  [this] { cycleBetaFace(); });

    // 动态设备按钮从该索引开始，刷新时只移除这一段，避免误删固定按钮。
    firstDeviceButtonIndex_ = ui_.size();
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
        logUi("Import", "没有可导入的文件", false);
        return;
    }

    logUi("Import", "加载 " + std::to_string(paths.size()) + " 个文件，首个: " +
                        paths.front().filename().string());

    clearScene();

    std::vector<std::string> list;
    list.reserve(paths.size());
    for (const auto &path : paths)
    {
        list.push_back(path.string());
    }

    if (!loadFiles(list))
    {
        logUi("Import", "文件解析/索引失败", false);
        updateHud();
        renderer_.render();
        return;
    }

    if (loadFrame(0, 0))
    {
        renderer_.resetCamera();
        logUi("Import", "成功加载 " + std::to_string(files_.size()) + " 个文件，显示首个文件第 0 帧");
    }
    else
    {
        logUi("Import", "索引建立成功，但首帧解码失败", false);
    }
    updateHud();
    renderer_.render();
}

void ViewerApp::openFilesDialog()
{
    logUi("Open Files", "打开原生文件选择对话框");
    if (!FileDialogService::available())
    {
        logUi("Open Files", "未找到原生对话框后端（Linux 需要 zenity 或 kdialog）", false);
        return;
    }

    const auto files = FileDialogService::openFiles();
    if (files.empty())
    {
        logUi("Open Files", "用户取消或未选择文件", false);
        return;
    }

    logUi("Open Files", "已选择 " + std::to_string(files.size()) + " 个文件");
    importFiles(files);
}

void ViewerApp::openDirectoryDialog()
{
    logUi("Open Folder", "打开目录选择对话框");
    const auto directory = FileDialogService::chooseDirectory();
    if (!directory)
    {
        logUi("Open Folder", "用户取消目录选择", false);
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
        logUi("Open Folder", "目录中没有 csv/txt/pcd 文件: " + directory->string(), false);
        return;
    }
    logUi("Open Folder", directory->string() + " 下找到 " + std::to_string(files.size()) + " 个文件");
    importFiles(files);
}

void ViewerApp::toggleCurrentVisibility()
{
    if (currentCloudId_ == 0)
    {
        logUi("Hide", "当前没有可隐藏的点云", false);
        return;
    }
    if (const SceneObject *object = scene_.find(currentCloudId_))
    {
        const bool visible = !object->visible;
        scene_.setVisible(currentCloudId_, visible);
        renderer_.setVisible(currentCloudId_, visible);
        logUi("Hide", "cloudId=" + std::to_string(currentCloudId_) +
                          " visible=" + (visible ? "true" : "false"));
        updateHud();
        renderer_.render();
    }
    else
    {
        logUi("Hide", "场景树中找不到 cloudId=" + std::to_string(currentCloudId_), false);
    }
}

ViewerApp::~ViewerApp()
{
    // 接收回调引用本对象，必须在 mutex、pending frame 等成员析构前退出线程。
    if (discovery_) discovery_->stop();
    if (betaStream_) betaStream_->stop();
    if (radar_) radar_->stop();
}

void ViewerApp::connectBeta(const std::string &ip)
{
    // 用广播中的 work_mode 初始化当前模式，避免 HUD 与实际包格式不一致。
    betaWorkMode_ = 2;
    for (const auto &device : discoveredDevices_)
    {
        if (device.ip == ip)
        {
            betaWorkMode_ = device.workMode;
        }
    }

    if (radar_)
    {
        radar_->stop();
        radar_.reset();
    }
    if (betaStream_)
    {
        betaStream_->stop();
        betaStream_.reset();
    }

    betaStream_ = std::make_unique<BetaUdpStream>();
    BetaUdpStream::Config config;
    config.radarIp = ip;
    if (const char *port = std::getenv("CV_BETA_PORT"))
    {
        const int value = std::atoi(port);
        if (value > 0 && value < 65536)
        {
            config.localPort = static_cast<std::uint16_t>(value);
        }
    }

    auto sink = [this](algo_lidar::PointMatrix &&matrix, std::uint64_t frameNumber)
    {
        const int detected = betaStream_ ? betaStream_->stats().detectedMode : 2;
        FrameMetadata metadata;
        // 原始合成为 96x3200，真实点云为 96x800，直方图为 96x512。
        metadata.rows = static_cast<int>(matrix.size());
        metadata.cols = matrix.empty() ? 0 : static_cast<int>(matrix.front().size());
        metadata.model = RadarModel::BetaRotating;
        metadata.roiId = 0;
        metadata.frameNumber = frameNumber;
        metadata.timestampUnit = "frame";
        metadata.algorithmId = detected == 1 ? "cs96_beta_origin"
                              : detected == 0 ? "cs96_beta_histogram"
                                              : "cs96_beta_cartesian";
        metadata.calibrationState = detected == 2 ? "device_xyz" : "device_raw";

        auto frame = makeFrame(std::move(matrix), metadata);
        std::lock_guard<std::mutex> lock(radarMutex_);
        pendingRadarFrame_ = frame;
    };

    // 原始点云标定表（距离 / 水平角 / 俯仰角 / 反射率）。
    // 默认读取聚合成单文件的 CSV；目录布局作为兼容回退。
    if (!betaCalibration_)
    {
        betaCalibration_ = std::make_shared<Cs96BetaCalibration>();
        std::filesystem::path source = "data/param/cs96beta_calibration.csv";
        if (const char *override = std::getenv("CV_BETA_CALIB"))
        {
            source = override;
        }
        std::error_code ec;
        if (!std::filesystem::exists(source, ec))
        {
            const std::filesystem::path legacy = "data/param/cs96beta_calib_data";
            if (std::filesystem::is_directory(legacy, ec))
            {
                logUi("Calib", "未找到 " + source.string() + "，回退目录布局 " + legacy.string());
                source = legacy;
            }
        }
        std::string calibrationError;
        if (betaCalibration_->load(source, calibrationError))
        {
            const auto &starts = betaCalibration_->startingAngles();
            std::ostringstream detail;
            detail << "CS96 Beta 标定已加载: " << source.string()
                   << " start=" << starts[0] << "," << starts[1] << "," << starts[2] << "," << starts[3];
            logUi("Calib", detail.str());
        }
        else
        {
            logUi("Calib", "CS96 Beta 标定加载失败: " + calibrationError, false);
            betaCalibration_.reset();
        }
    }

    // CV_BETA_CALIB_ENABLED=0 可在启动时关闭标定，便于对比纯几何结果。
    if (const char *enabled = std::getenv("CV_BETA_CALIB_ENABLED"))
    {
        betaCalibrationEnabled_ = std::string(enabled) != "0";
    }
    // 自动化验证：CV_BETA_FACE=1..4 只显示单面；CV_BETA_FACE=ALL 显示全部。
    if (const char *faceEnv = std::getenv("CV_BETA_FACE"))
    {
        const std::string value(faceEnv);
        if (value == "ALL" || value == "all")
        {
            betaFaceMask_ = 0x0F;
        }
        else
        {
            const int number = std::atoi(faceEnv);
            if (number >= 1 && number <= 4)
            {
                betaFaceMask_ = static_cast<std::uint8_t>(1u << (number - 1));
            }
        }
    }

    betaStream_->setCalibration(betaCalibration_);
    betaStream_->setCalibrationEnabled(betaCalibrationEnabled_);
    betaStream_->setFaceMask(betaFaceMask_);

    std::string error;
    if (!betaStream_->open(config, sink, error))
    {
        logUi("Connect Device", "CS96 Beta 监听失败: " + error, false);
        radarState_ = RadarState::Error;
        std::lock_guard<std::mutex> lock(radarMessageMutex_);
        radarMessage_ = "CS96 Beta 监听失败: " + error;
        updateHud();
        renderer_.render();
        return;
    }

    radarConnected_ = true;
    radarState_ = RadarState::Waiting;
    radarWaitStartMs_ = nowMs();
    autoStartAttempted_ = true; // Beta 为被动接收，不做 SSH 拉起
    {
        std::lock_guard<std::mutex> lock(radarMessageMutex_);
        radarMessage_.clear();
    }
    logUi("Connect Device", "CS96 Beta 监听 UDP " + std::to_string(config.localPort) +
                               "，等待 " + ip + " 推流");
    updateHud();
    renderer_.render();
}

void ViewerApp::searchDevices()
{
    logUi("Search Device", "监听 UDP 55000 广播，最多 2.2 秒");
    if (!discovery_)
    {
        discovery_ = std::make_unique<DeviceDiscovery>();
    }
    discovery_->start(55000);

    // 雷达广播周期约 1 秒，取两个周期以覆盖丢包。
    std::this_thread::sleep_for(std::chrono::milliseconds(2200));
    discoveredDevices_ = discovery_->snapshot();
    discovery_->stop();

    if (discoveredDevices_.empty())
    {
        logUi("Search Device",
              "未发现设备：确认雷达已上电、与本机同网段且允许 UDP 55000 广播", false);
    }
    else
    {
        std::ostringstream detail;
        for (const auto &device : discoveredDevices_)
        {
            detail << "[" << device.model << " " << device.ip << " " << device.serialNumber << "] ";
        }
        logUi("Search Device", "发现 " + std::to_string(discoveredDevices_.size()) +
                                   " 台: " + detail.str());
    }

    rebuildDeviceButtons();
    updateHud();
    renderer_.render();
}

void ViewerApp::rebuildDeviceButtons()
{
    // 旧设备按钮从 firstDeviceButtonIndex_ 起，刷新时整体移除后重建。
    ui_.removeButtonsFrom(firstDeviceButtonIndex_);
    firstDeviceButtonIndex_ = ui_.size();

    constexpr std::size_t kMaxDeviceButtons = 4;
    const std::size_t count = std::min(discoveredDevices_.size(), kMaxDeviceButtons);
    for (std::size_t i = 0; i < count; ++i)
    {
        const DiscoveredDevice &device = discoveredDevices_[i];
        std::string label = device.model + " " + device.ip;
        if (device.lineCount > 0)
        {
            label += " " + std::to_string(device.lineCount) + "x" +
                     std::to_string(device.slotsPerFrame);
        }
        ui_.addButton(label, 0.010 + 0.245 * static_cast<double>(i), 0.800, 0.235, 0.052,
                      [this, i] { connectDiscoveredDevice(i); });
    }

    if (discoveredDevices_.size() > kMaxDeviceButtons)
    {
        logUi("Search Device", "设备过多，仅显示前 " +
                                   std::to_string(kMaxDeviceButtons) + " 台", false);
    }
}

void ViewerApp::connectDiscoveredDevice(std::size_t index)
{
    if (index >= discoveredDevices_.size())
    {
        logUi("Connect Device", "设备索引失效，请重新搜索", false);
        return;
    }

    const DiscoveredDevice device = discoveredDevices_[index];
    logUi("Connect Device", device.model + " " + device.ip + " serial=" + device.serialNumber);

    if (device.model == "CS96 Beta" || device.model == "CS96")
    {
        connectBeta(device.ip);
        return;
    }

    radarIpOverride_ = device.ip;
    connectRadar();
}

void ViewerApp::connectRadar()
{
    // 重试时必须先释放旧 socket，否则 Error 状态下 radar_ 仍 running，
    // 直接 return 会导致点击 Retry 毫无反应。
    if (radar_)
    {
        radar_->stop();
        radar_.reset();
    }

    // 雷达向本机 .100 单播；该地址通常是临时添加的，电脑重启后会丢失，
    // 导致 viewer 收不到任何包而一直停在 Waiting。连接前先检测并自动补齐。
    const char *envIface = std::getenv("CV_RADAR_IFACE");
    const std::string iface = (envIface != nullptr && envIface[0] != '\0') ? envIface : "enp4s0";
    const char *envCidr = std::getenv("CV_RADAR_CIDR");
    const std::string cidr = (envCidr != nullptr && envCidr[0] != '\0') ? envCidr : "192.168.137.100/24";
    const char *envLocalIp = std::getenv("CV_RADAR_LOCAL_IP");
    const std::string localIp = (envLocalIp != nullptr && envLocalIp[0] != '\0') ? envLocalIp : "192.168.137.100";

    const NetworkSetupResult network = ensureLocalAddress(iface, cidr, localIp);
    if (!network.configured)
    {
        std::cerr << "[cloud_viewer] " << network.message << "\n";
        radarState_ = RadarState::Error;
        {
            std::lock_guard<std::mutex> lock(radarMessageMutex_);
            radarMessage_ = network.message;
        }
        updateHud();
        renderer_.render();
        return;
    }
    if (!network.hasAddress)
    {
        std::cout << "[cloud_viewer] " << network.message << "\n";
    }

    radar_ = std::make_unique<UdpRadarStream>();
    UdpRadarConfig config; // 192.168.137.200:6001 -> 本地 UDP 6002
    config.mode = liveMode_;
    config.roi = liveRoi_;

    // 允许通过环境变量覆盖，便于真机换 IP 与本地回环测试。
    try
    {
        if (const char *ip = std::getenv("CV_RADAR_IP"))
        {
            config.radarIp = ip;
        }
        if (const char *port = std::getenv("CV_RADAR_PORT"))
        {
            const int value=std::stoi(port);
            if(value<1 || value>65535) throw std::out_of_range("CV_RADAR_PORT");
            config.radarPort=static_cast<std::uint16_t>(value);
        }
        if (const char *local = std::getenv("CV_LOCAL_PORT"))
        {
            const int value=std::stoi(local);
            if(value<1 || value>65535) throw std::out_of_range("CV_LOCAL_PORT");
            config.localPort=static_cast<std::uint16_t>(value);
        }
    }
    catch (...)
    {
        // 非法环境变量保持默认值。
    }

    // 搜索结果中选中的设备优先于环境变量与默认 IP。
    if (!radarIpOverride_.empty())
    {
        config.radarIp = radarIpOverride_;
    }

    std::string error;

    auto sink = [this, config](algo_lidar::PointMatrix &&matrix, std::uint64_t timestampUs, int roi)
    {
        FrameMetadata metadata;
        metadata.model = roi < 0 ? RadarModel::Generic : RadarModel::Fl6532;
        metadata.roiId = roi;
        metadata.frameNumber = timestampUs;
        metadata.timestampUnit = config.mode == RadarOutputMode::Range ? "raw" : "us";
        metadata.algorithmId = config.mode == RadarOutputMode::Range ? "range6532_raw" : "device_cloud";
        metadata.calibrationState = config.mode == RadarOutputMode::Range ? "raw_range" : "device_unknown";

        auto frame = makeFrame(std::move(matrix), metadata);
        std::lock_guard<std::mutex> lock(radarMutex_);
        pendingRadarFrame_ = frame;
    };

    if (!radar_->open(config, sink, error))
    {
        std::cerr << "[cloud_viewer] 雷达连接失败: " << error << "\n";
        radar_.reset();
        radarConnected_ = false;
        radarState_ = RadarState::Error;
        std::lock_guard<std::mutex> lock(radarMessageMutex_);
        radarMessage_ = "UDP 监听失败: " + error;
    }
    else
    {
        radarConnected_ = true;
        radarState_ = RadarState::Waiting;
        radarWaitStartMs_ = nowMs();
        autoStartAttempted_ = false;
        {
            std::lock_guard<std::mutex> lock(radarMessageMutex_);
            radarMessage_.clear();
        }
        logUi("Connect Radar", "已监听 UDP " + std::to_string(config.localPort) +
                                   "，等待 " + config.radarIp + ":" + std::to_string(config.radarPort));
    }

    if (radarState_ == RadarState::Error)
    {
        radarState_ = RadarState::Error;
    }

    updateHud();
    renderer_.render();
}

void ViewerApp::disconnectRadar()
{
    if (betaStream_)
    {
        betaStream_->stop();
        betaStream_.reset();
    }
    if (radar_)
    {
        radar_->stop();
        radar_.reset();
    }
    radarConnected_ = false;
    radarState_ = RadarState::Disconnected;
    autoStartAttempted_ = false;
    lastRawLiveFrame_.reset();
    {
        std::lock_guard<std::mutex> lock(radarMutex_);
        pendingRadarFrame_.reset();
    }
    updateHud();
    renderer_.render();
}

void ViewerApp::toggleRadarConnection()
{
    const auto stateName = [](RadarState value) -> const char *
    {
        switch (value)
        {
        case RadarState::Disconnected:
            return "Disconnected";
        case RadarState::Waiting:
            return "Waiting";
        case RadarState::Live:
            return "Live";
        case RadarState::Error:
            return "Error";
        }
        return "Unknown";
    };

    if (modeFuture_.valid())
    {
        logUi("Connect Radar", "模式切换进行中，忽略本次点击", false);
        return;
    }

    const auto state = radarState_.load();
    if (state == RadarState::Waiting || state == RadarState::Live)
    {
        logUi("Disconnect", std::string("state=") + stateName(state));
        disconnectRadar();
    }
    else
    {
        logUi("Connect Radar", std::string("state=") + stateName(state));
        connectRadar();
    }
}

std::string ViewerApp::betaSshPassword()
{
    if (const char *value = std::getenv("CV_BETA_SSH_PASS"))
    {
        if (value[0] != '\0')
        {
            return value;
        }
    }

    const char *home = std::getenv("HOME");
    if (home == nullptr || home[0] == '\0')
    {
        return {};
    }

    std::ifstream input(std::string(home) + "/.config/cloud_viewer/beta_ssh.conf");
    std::string line;
    while (std::getline(input, line))
    {
        const auto equal = line.find('=');
        if (equal == std::string::npos)
        {
            continue;
        }
        if (line.substr(0, equal) == "BETA_SSH_PASS")
        {
            return line.substr(equal + 1);
        }
    }
    return {};
}

bool ViewerApp::applyBetaWorkMode(const std::string &ip, int mode, std::string &message)
{
    const std::string password = betaSshPassword();
    if (password.empty())
    {
        message = "未配置 Beta SSH 密码（CV_BETA_SSH_PASS 或 ~/.config/cloud_viewer/beta_ssh.conf）";
        return false;
    }

    // 通过脚本文件传给设备执行，避免多层引号转义问题。
    const std::string scriptPath = "/tmp/cv_beta_mode.sh";
    {
        std::ofstream script(scriptPath, std::ios::trunc);
        if (!script.is_open())
        {
            message = "无法创建本地切换脚本";
            return false;
        }
        // 后台进程放进子 shell 并脱离会话，避免 SSH 等待其 stdout 而挂住。
        script << "#!/bin/sh\n"
               << "sed -i 's/^work_mode.*/work_mode                      = \"" << mode
               << "\" ;工作模式/' /etc/ys_conf.ini\n"
               << "kill -9 $(ps | grep you_see | grep -v grep | awk '{print $1}') 2>/dev/null\n"
               << "sleep 1\n"
               << "setsid sh -c 'cd /root && exec ./you_see >>/tmp/you_stdout.log 2>&1' "
                  "</dev/null >/dev/null 2>&1 &\n"
               << "sleep 2\n"
               << "echo SWITCH_OK\n"
               << "exit 0\n";
    }

    const std::string command =
        "timeout 25 sshpass -p " + shellQuote(password) +
        " ssh -o StrictHostKeyChecking=no -o UserKnownHostsFile=/dev/null -o ConnectTimeout=6 "
        "-o ServerAliveInterval=3 -o ServerAliveCountMax=2 " +
        "root@" + ip + " sh -s < " + scriptPath + " 2>&1";

    FILE *pipe = popen(command.c_str(), "r");
    if (pipe == nullptr)
    {
        message = "无法执行 SSH 命令";
        std::remove(scriptPath.c_str());
        return false;
    }

    std::string output;
    char buffer[512];
    while (std::fgets(buffer, sizeof(buffer), pipe) != nullptr)
    {
        output += buffer;
    }
    const int rc = pclose(pipe);
    std::remove(scriptPath.c_str());

    // 设备端 setsid 启动后 SSH 可能因 fd 未关闭而超时，以脚本回显为准。
    (void)rc;
    if (output.find("SWITCH_OK") != std::string::npos)
    {
        message = output;
        return true;
    }
    message = "SSH 切换失败: " + output.substr(0, 200);
    return false;
}

void ViewerApp::cycleBetaFace()
{
    if (!betaStream_)
    {
        logUi("Faces", "当前不是 CS96 Beta，面选择无效", false);
        return;
    }

    // 常用组合循环：All -> F1 -> F3 -> F1+F3 -> F2 -> F4 -> F2+F4 -> All
    static const std::uint8_t kCycle[] = {0x0F, 0x01, 0x04, 0x05, 0x02, 0x08, 0x0A};
    constexpr std::size_t kCount = sizeof(kCycle) / sizeof(kCycle[0]);

    std::size_t index = 0;
    for (std::size_t i = 0; i < kCount; ++i)
    {
        if (kCycle[i] == betaFaceMask_)
        {
            index = i;
            break;
        }
    }
    betaFaceMask_ = kCycle[(index + 1) % kCount];
    betaStream_->setFaceMask(betaFaceMask_);

    logUi("Faces", "显示面帧 " + faceMaskLabel(betaFaceMask_));
    updateHud();
    renderer_.render();
}

void ViewerApp::switchBetaMode()
{
    if (modeFuture_.valid())
    {
        logUi("Mode", "上一次模式切换尚未完成，忽略点击", false);
        return;
    }

    const int next = (betaWorkMode_ == 2) ? 1 : (betaWorkMode_ == 1 ? 0 : 2);
    const std::string ip = betaStream_ ? betaStream_->config().radarIp : std::string{};
    if (ip.empty())
    {
        logUi("Mode", "尚未连接 CS96 Beta，无法切换", false);
        return;
    }

    requestedBetaMode_ = next;
    betaModeSwitch_ = true;
    controlState_ = "Switching Beta work mode...";
    logUi("Mode", "CS96 Beta: " + std::to_string(betaWorkMode_) + " -> " +
                      std::to_string(next) + "（SSH 改配置并重启设备）");

    modeFuture_ = std::async(std::launch::async, [ip, next] {
        std::string message;
        const bool ok = applyBetaWorkMode(ip, next, message);
        return ok ? std::string{} : message;
    });

    updateHud();
    renderer_.render();
}

void ViewerApp::switchRadarMode()
{
    if (betaStream_)
    {
        switchBetaMode();
        return;
    }

    if (modeFuture_.valid())
    {
        logUi("Mode", "上一次模式切换尚未完成，忽略点击", false);
        return;
    }
    UdpRadarConfig config;
    if (radar_) config = radar_->config();
    else if (const char* ip = std::getenv("CV_RADAR_IP")) config.radarIp = ip;
    const RadarOutputMode previousMode = liveMode_;
    requestedMode_ = liveMode_ == RadarOutputMode::Cloud ? RadarOutputMode::Range : RadarOutputMode::Cloud;
    disconnectRadar();
    std::uint16_t port = 6508;
    try { if (const char* p = std::getenv("CV_CONTROL_PORT")) {
        const int value=std::stoi(p);
        if(value<1 || value>65535) throw std::out_of_range("CV_CONTROL_PORT");
        port=static_cast<std::uint16_t>(value);
    } }
    catch (...)
    {
        controlState_ = "Invalid CV_CONTROL_PORT";
        logUi("Mode", "CV_CONTROL_PORT 非法，模式切换中止", false);
        updateHud();
        return;
    }
    logUi("Mode", std::string(previousMode == RadarOutputMode::Cloud ? "Cloud" : "Range") +
                      " -> " + (requestedMode_ == RadarOutputMode::Cloud ? "Cloud" : "Range") +
                      " (control port " + std::to_string(port) + ")");
    controlState_ = "Switching device mode...";
    const auto mode = requestedMode_;
    modeFuture_ = std::async(std::launch::async, [config,port,mode] {
        return setRadarOutputMode(config.radarIp,port,mode);
    });
    updateHud(); renderer_.render();
}

void ViewerApp::switchRadarRoi()
{
    if (modeFuture_.valid())
    {
        logUi("ROI", "模式切换进行中，忽略点击", false);
        return;
    }
    const bool connected = radarConnected_;
    const int previousRoi = liveRoi_;
    disconnectRadar();
    liveRoi_ = liveRoi_ ? 0 : 1;
    logUi("ROI", "ROI " + std::to_string(previousRoi) + " -> " + std::to_string(liveRoi_) +
                     (connected ? "，重新连接" : "，当前未连接"));
    if (connected) connectRadar();
    updateHud(); renderer_.render();
}

void ViewerApp::loadLiveCalibration()
{
    logUi("Select Calib", "打开标定表选择对话框");
    const auto files = FileDialogService::openFiles();
    if (files.empty())
    {
        logUi("Select Calib", "用户取消或未选择文件", false);
        return;
    }

    // 选择一次即复制到软件目录，后续启动不依赖原始路径。
    std::string copyError;
    const auto stored = importCalibrationFile(files.front(), copyError);
    if (stored.empty())
    {
        logUi("Select Calib", "复制标定表失败: " + copyError, false);
        return;
    }

    algo_lidar::Range6532LegacyCalibrationBin table;
    if (!algo_lidar::loadRange6532LegacyCalibrationBin(stored.string(), table))
    {
        controlState_ = "Calibration table parse failed; previous table retained";
        logUi("Select Calib", "解析失败，保留旧表: " + stored.string(), false);
    }
    else
    {
        liveCalibrationTable_ = std::move(table);
        liveCalibrationPath_ = stored.string();
        calibrationReady_ = true;
        controlState_ = "Calibration table: " + stored.filename().string();

        // 只记住文件，不改变开关状态；开关由 App Cal 按钮独立控制。
        ViewerSettings settings;
        settings.calibrationFile = stored.filename().string();
        settings.calibrationEnabled = liveCalibration_ != 0;
        std::string saveError;
        if (!saveViewerSettings(settings, saveError))
        {
            logUi("Select Calib", "记忆写入失败: " + saveError, false);
        }
        logUi("Select Calib", "已复制并加载 " + stored.string());
    }
    updateHud(); renderer_.render();
}

void ViewerApp::toggleLiveCalibration()
{
    // CS96 Beta 的标定开关独立于 6532：只控制原始点云是否套用标定表。
    if (betaStream_)
    {
        if (!betaCalibration_)
        {
            controlState_ = "CS96 Beta: calibration table missing";
            logUi("App Cal", "CS96 Beta 标定表未加载，当前固定为纯几何解算", false);
            updateHud();
            renderer_.render();
            return;
        }
        betaCalibrationEnabled_ = !betaCalibrationEnabled_;
        betaStream_->setCalibrationEnabled(betaCalibrationEnabled_);
        controlState_ = betaCalibrationEnabled_ ? "CS96 Beta calibration ON"
                                               : "CS96 Beta calibration OFF (raw geometry)";
        logUi("App Cal", betaCalibrationEnabled_
                             ? "CS96 Beta 标定 ON：编码器水平角 + 逐线修正 + 距离零点/系数 + 反射率查表"
                             : "CS96 Beta 标定 OFF：纯几何（光速距离、slot 线性水平角、峰值反射率）");
        updateHud();
        renderer_.render();
        return;
    }

    if (liveMode_ != RadarOutputMode::Range)
    {
        controlState_ = "Cloud = device output; use RANGE for raw/calibrated comparison";
        logUi("App Cal", "当前为 Cloud 模式，设备已输出点云，无需本地校正", false);
        updateHud(); renderer_.render();
        return;
    }
    if (!calibrationReady_)
    {
        controlState_ = "Select calibration table first";
        logUi("App Cal", "尚未选择标定表，请先点 Select Calib", false);
        updateHud(); renderer_.render();
        return;
    }

    liveCalibration_ = liveCalibration_ ? 0 : 1;
    controlState_ = liveCalibration_ ? "App calibration ON" : "App calibration OFF";

    // 记住开关状态，下次启动随标定表一起恢复。
    ViewerSettings settings;
    settings.calibrationFile = std::filesystem::path(liveCalibrationPath_).filename().string();
    settings.calibrationEnabled = liveCalibration_ != 0;
    std::string saveError;
    if (!saveViewerSettings(settings, saveError))
    {
        logUi("App Cal", "记忆写入失败: " + saveError, false);
    }

    if (lastRawLiveFrame_)
    {
        std::lock_guard<std::mutex> lock(radarMutex_);
        // 切换始终从保存的原始帧重算，不在校正后的点云上累加。
        if (!pendingRadarFrame_) pendingRadarFrame_ = lastRawLiveFrame_;
    }

    logUi("App Cal", liveCalibration_ ? "已启用本地标定（TableB）" : "已取消本地标定");
    updateHud(); renderer_.render();
}

FrameHandle ViewerApp::preprocessLiveFrame(const FrameHandle& raw)
{
    if (!liveCalibration_ || !calibrationReady_ || raw->metadata.calibrationState != "raw_range")
    {
        return raw;
    }
    // 默认 TableB；保留 CV_CALIB_MODE=tail3ln 作为算法对比入口。
    const char *mode = std::getenv("CV_CALIB_MODE");
    const bool useTail3 = mode != nullptr && std::string(mode) == "tail3ln";
    return calibrateOnlineRange(raw, liveCalibrationTable_,
                                useTail3 ? OnlineCalibrationMode::Tail3LnFixed
                                         : OnlineCalibrationMode::TableB);
}

void ViewerApp::applyIncomingFrame()
{
    // TCP 控制等待在后台完成，UI 不阻塞；只有收到成功 ACK 后才切换本地解析器。
    if (modeFuture_.valid() && modeFuture_.wait_for(std::chrono::seconds(0)) == std::future_status::ready) {
        const auto error = modeFuture_.get();
        if (betaModeSwitch_) {
            betaModeSwitch_ = false;
            if (error.empty()) {
                betaWorkMode_ = requestedBetaMode_;
                controlState_ = "Beta work mode switched";
                logUi("Mode", "CS96 Beta 已切换到 work_mode=" + std::to_string(betaWorkMode_));
            } else {
                controlState_ = "Beta mode switch failed: " + error;
                logUi("Mode", "CS96 Beta 切换失败: " + error, false);
            }
        } else if (error.empty()) {
            liveMode_ = requestedMode_;
            controlState_ = "Mode ACK OK; waiting for matching complete frame";
            logUi("Mode", "设备已确认模式切换，重新监听 UDP");
            connectRadar();
        } else {
            controlState_ = "Mode switch failed: " + error;
            logUi("Mode", "设备模式切换失败: " + error, false);
        }
        updateHud(); renderer_.render();
    }
    FrameHandle frame;
    {
        std::lock_guard<std::mutex> lock(radarMutex_);
        frame = std::move(pendingRadarFrame_);
        pendingRadarFrame_.reset();
    }
    if (!frame)
    {
        // 等待阶段：2.5s 无包则尝试通过 SSH 启动雷达驱动；12s 仍无包判为失败。
        if (radarState_.load() == RadarState::Waiting)
        {
            const std::uint64_t now = nowMs();
            const RadarStreamStats stats = radar_ ? radar_->stats() : RadarStreamStats{};
            if (stats.packets == 0 && now - radarWaitStartMs_ > 2500 &&
                !autoStartAttempted_.exchange(true))
            {
                if (RadarLauncher::configured())
                {
                    std::string message;
                    RadarLauncher::start(message); // 同步约 3s，仅在无数据时执行一次
                    std::cout << "[cloud_viewer] " << message << "\n";
                    std::lock_guard<std::mutex> lock(radarMessageMutex_);
                    radarMessage_ = message;
                    radarWaitStartMs_ = nowMs();
                }
                else
                {
                    std::lock_guard<std::mutex> lock(radarMessageMutex_);
                    radarMessage_ = "未配置 CV_RADAR_SSH_PASS，请手动运行 tools/connect_radar.sh";
                }
            }

            if (stats.packets == 0 && now - radarWaitStartMs_ > 12000)
            {
                radarState_ = RadarState::Error;
                std::lock_guard<std::mutex> lock(radarMessageMutex_);
                if (radarMessage_.empty())
                {
                    radarMessage_ = betaStream_
                        ? "未收到 CS96 Beta 点云：确认设备 work_mode=2（真实点云）"
                          "且 /etc/ys_conf.ini 的 center_ip 指向本机、center_data_port 为 50007"
                        : "未收到点云：先确认本机有 192.168.137.100 地址"
                          "（sudo ip addr add 192.168.137.100/24 dev enp4s0），"
                          "并确认雷达 soc_example 在运行";
                }
            }
        }
        updateHud();
        return;
    }

    const RadarState previousRadarState = radarState_.exchange(RadarState::Live);
    if (betaStream_)
    {
        const int detected = betaStream_->stats().detectedMode;
        if (detected >= 0)
        {
            betaWorkMode_ = detected;
        }
    }
    if (previousRadarState != RadarState::Live)
    {
        std::ostringstream detail;
        detail << "收到首帧，帧号 " << frame->metadata.frameNumber
               << "，显示点 " << frame->metadata.totalSamples
               << "，有效 " << frame->metadata.validSamples;
        logUi("Radar Live", detail.str());
    }
    lastRawLiveFrame_ = frame;
    frame = preprocessLiveFrame(frame);
    // 必须在清空 currentCloudId_ 之前判断，否则每帧都会误判为首帧并 resetCamera，
    // 导致相机随点云包络反复重算（表现为视角螺旋/抖动）。
    const bool firstLiveFrame = (currentCloudId_ == 0);

    if (currentCloudId_ != 0)
    {
        renderer_.remove(currentCloudId_);
        scene_.remove(currentCloudId_);
        currentCloudId_ = 0;
    }

    currentFrameHandle_ = frame;
    currentRenderable_ = CloudRenderBridge::prepare(frame);

    SceneObject object;
    object.name = "LIVE";
    object.kind = SceneObjectKind::Cloud;
    object.frame = frame;
    object.renderable = currentRenderable_;
    currentCloudId_ = scene_.add(0, std::move(object));

    applyColor();
    if (firstLiveFrame)
    {
        renderer_.resetCamera();
    }
    updateHud();
    renderer_.render();
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
        logUi("Color", "尚未加载点云，无法切换着色", false);
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

    const ColorSource previous = colorSource_;
    for (std::size_t step = 1; step <= order.size(); ++step)
    {
        const ColorSource candidate = order[(start + step) % order.size()];
        if (candidate == ColorSource::SolidColor ||
            currentFrameHandle_->availability.has(candidate))
        {
            colorSource_ = candidate;
            applyColor();
            std::ostringstream detail;
            detail << colorSourceName(previous) << " -> " << colorSourceName(candidate);
            if (lastRange_.valid)
            {
                detail << ", range=[" << formatFloat(lastRange_.minValue) << ", "
                       << formatFloat(lastRange_.maxValue) << "]";
            }
            else
            {
                detail << ", 无有效标量值";
            }
            logUi("Color", detail.str());
            return;
        }
    }

    logUi("Color", "当前点云没有任何可用属性，保持原着色", false);
}

void ViewerApp::stepFrame(int delta)
{
    const char *action = delta < 0 ? "Prev" : "Next";
    if (frozen_)
    {
        logUi(action, "播放已冻结（Space 解冻后再翻帧）", false);
        return;
    }
    if (files_.empty())
    {
        logUi(action, "尚未加载任何文件", false);
        return;
    }
    LoadedFile &file = files_[currentFile_];
    if (file.index.frames.empty())
    {
        logUi(action, "当前文件没有可用帧: " + file.path.filename().string(), false);
        return;
    }

    const int count = static_cast<int>(file.index.frames.size());
    const int previous = static_cast<int>(currentFrame_);
    int next = previous + delta;
    next = (next % count + count) % count;

    loadFrame(currentFile_, static_cast<std::size_t>(next));
    renderer_.render();
    logUi(action, file.path.filename().string() + " 帧 " + std::to_string(previous) +
                     " -> " + std::to_string(next) + " / " + std::to_string(count));
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
        logUi("Pick", "(" + std::to_string(displayX) + "," + std::to_string(displayY) +
                          ") 未命中任何点云", false);
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
    if (files_.empty() && !radarConnected_)
    {
        text << "No point cloud loaded\n";
        text << "Click \"Open Files\" or \"Open Folder\" to import CSV / TXT / PCD\n";
        text << "Keys: C color  R reset  +/- size  Shift+LMB pick\n";
    }
    else
    {
        if (files_.empty())
        {
            text << "LIVE radar stream\n";
        }
        else if (currentFile_ < files_.size())
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

    const RadarState radarState = radarState_.load();
    if (radarState != RadarState::Disconnected)
    {
        const char *stateText = radarState == RadarState::Waiting ? "waiting for data"
                                : radarState == RadarState::Live  ? "LIVE"
                                                                  : "ERROR";
        if (betaStream_)
        {
            const auto stats = betaStream_->stats();
            text << "\nRadar[" << stateText << "]: " << betaStream_->config().radarIp
                 << ":" << betaStream_->config().localPort
                 << " (CS96 Beta)  pkts=" << stats.packets
                 << "  frames=" << stats.frames;
        }
        else
        {
            const RadarStreamStats stats = radar_ ? radar_->stats() : RadarStreamStats{};
            text << "\nRadar[" << stateText << "]: "
                 << (radar_ ? radar_->config().radarIp : std::string("-"))
                 << ":" << (radar_ ? radar_->config().radarPort : 0)
                 << "  pkts=" << stats.packets
                 << "  frames=" << stats.frames
                 << "  fps=" << formatFloat(static_cast<float>(stats.framesPerSecond), 1);
        }
        if (radarState != RadarState::Live)
        {
            std::lock_guard<std::mutex> lock(radarMessageMutex_);
            if (!radarMessage_.empty())
            {
                text << "\n" << radarMessage_;
            }
        }
    }

    text << "\nOnline selected: " << (liveMode_ == RadarOutputMode::Range ? "Range" : "Cloud")
         << " ROI" << unsigned(liveRoi_) << "\n" << controlState_;
    if (currentFrameHandle_ && radarConnected_)
        text << "\nObserved: " << currentFrameHandle_->metadata.algorithmId
             << "  calibration=" << currentFrameHandle_->metadata.calibrationState;
    if (radar_) {
        const auto stats = radar_->stats();
        text << " invalid=" << stats.invalidPackets;
        if (!stats.lastError.empty()) text << "\n" << stats.lastError;
    } else if (betaStream_) {
        const auto stats = betaStream_->stats();
        text << " pkts=" << stats.packets << " frames=" << stats.frames
             << " dropSlots=" << stats.droppedSlots;
    }
    if (betaStream_)
    {
        const char *betaModeName = betaWorkMode_ == 2 ? "Real"
                                   : betaWorkMode_ == 1 ? "Origin"
                                                        : "Hist";
        ui_.setLabel(modeButtonIndex_, std::string("Mode: ") + betaModeName);
    }
    if (faceButtonIndex_ < ui_.size())
    {
        ui_.setLabel(faceButtonIndex_, "Faces: " + faceMaskLabel(betaFaceMask_));
    }
    else
    {
        ui_.setLabel(modeButtonIndex_, liveMode_ == RadarOutputMode::Range ? "Mode: Range" : "Mode: Cloud");
    }
    ui_.setLabel(roiButtonIndex_, std::string("ROI: ")+std::to_string(liveRoi_));
    const char* cal = nullptr;
    if (betaStream_)
    {
        // Beta 的开关与 6532 的 Range/Cloud 无关，统一显示 ON/OFF。
        cal = betaCalibrationEnabled_ ? "App Cal: ON" : "App Cal: OFF";
    }
    else
    {
        cal = liveMode_ == RadarOutputMode::Cloud ? "App Cal: N/A (device Cloud)" :
            !liveCalibration_ ? "App Cal: OFF" : "App Cal: ON";
    }
    ui_.setLabel(calibrationButtonIndex_,cal);
    hudActor_->SetInput(text.str().c_str());

    if (colorButtonIndex_ < ui_.size())
    {
        ui_.setLabel(colorButtonIndex_, std::string("Color: ") + colorSourceName(colorSource_));
    }
    if (connectButtonIndex_ < ui_.size())
    {
        const char *label = "Connect Radar";
        if (radarState == RadarState::Waiting)
        {
            label = "Waiting...";
        }
        else if (radarState == RadarState::Live)
        {
            label = "Disconnect";
        }
        else if (radarState == RadarState::Error)
        {
            label = "Retry";
        }
        ui_.setLabel(connectButtonIndex_, label);
    }
}

int ViewerApp::run(const std::vector<std::string> &files, double fps,
                   const std::string &screenshotPath, bool autoConnectRadar)
{
    if (!renderer_.initialize(1280, 800))
    {
        return 11;
    }

    setupInteraction();

    if (const char* mode=std::getenv("CV_STREAM_MODE"))
        if (std::string(mode)=="range") liveMode_=RadarOutputMode::Range;

    // 恢复上次选择的标定表与开关状态（文件已复制到软件目录 calib/）。
    {
        const ViewerSettings saved = loadViewerSettings();
        if (!saved.calibrationFile.empty())
        {
            const auto path = calibrationDirectory() / saved.calibrationFile;
            if (algo_lidar::loadRange6532LegacyCalibrationBin(path.string(), liveCalibrationTable_))
            {
                calibrationReady_ = true;
                liveCalibrationPath_ = path.string();
                liveCalibration_ = saved.calibrationEnabled ? 1 : 0;
                logUi("Calib", "恢复上次标定表 " + saved.calibrationFile +
                                   (saved.calibrationEnabled ? "（开关 ON）" : "（开关 OFF）"));
            }
            else
            {
                logUi("Calib", "上次标定表读取失败: " + path.string(), false);
            }
        }
    }

    // 命令行/环境变量仍可覆盖：同样复制到软件目录并写入记忆。
    if (const char* path=std::getenv("CV_CALIB_BIN")) {
        std::string copyError;
        const auto stored = importCalibrationFile(path, copyError);
        const auto loadPath = stored.empty() ? std::filesystem::path(path) : stored;
        calibrationReady_=algo_lidar::loadRange6532LegacyCalibrationBin(loadPath.string(),liveCalibrationTable_);
        if (calibrationReady_) {
            liveCalibrationPath_=loadPath.string();
            ViewerSettings settings;
            settings.calibrationFile = loadPath.filename().string();
            settings.calibrationEnabled = liveCalibration_ != 0;
            std::string saveError;
            saveViewerSettings(settings, saveError);
        }
    }
    // 便于自动化验证：设置后启动即执行一次设备搜索。
    if (std::getenv("CV_AUTO_SEARCH") != nullptr)
    {
        searchDevices();
    }

    if (const char *betaIp = std::getenv("CV_BETA_IP"))
    {
        connectBeta(betaIp);
        // 自动化验证：启动时直接把设备工作模式设为指定值。
        if (const char *setMode = std::getenv("CV_BETA_SET_MODE"))
        {
            const int mode = std::atoi(setMode);
            if (mode >= 0 && mode <= 2)
            {
                std::string message;
                const bool ok = applyBetaWorkMode(betaIp, mode, message);
                logUi("Mode", ok ? "已设置 work_mode=" + std::to_string(mode)
                                 : "设置失败: " + message, ok);
                if (ok)
                {
                    betaWorkMode_ = mode;
                }
            }
        }
        if (std::getenv("CV_BETA_MODE_CYCLE") != nullptr)
        {
            switchBetaMode();
        }
    }
    else if (autoConnectRadar)
    {
        connectRadar();
    }

    // 允许空场景启动：用户通过顶部按钮导入文件。
    if (!files.empty())
    {
        std::vector<std::filesystem::path> initialPaths;
        initialPaths.reserve(files.size());
        for (const auto &file : files)
        {
            initialPaths.emplace_back(file);
        }
        // 与 Open Files 共用同一路径，保证启动加载与按钮点击的日志一致。
        importFiles(initialPaths);
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

    // 在线帧消费：接收线程只投递最新帧，主线程每 33 ms 渲染一次。
    auto radarTimer = vtkSmartPointer<vtkCallbackCommand>::New();
    radarTimer->SetCallback([](vtkObject *, unsigned long, void *clientData, void *)
                            {
                                auto *app = static_cast<ViewerApp *>(clientData);
                                if (app)
                                {
                                    app->applyIncomingFrame();
                                }
                            });
    radarTimer->SetClientData(this);
    interactor->AddObserver(vtkCommand::TimerEvent, radarTimer);
    interactor->CreateRepeatingTimer(33);

    updateHud();
    renderer_.render();

    if (!screenshotPath.empty())
    {
        if (autoConnectRadar || std::getenv("CV_BETA_IP") != nullptr)
        {
            // 轮询直到收到首帧或超时：期间会触发“无数据自动 SSH 拉起驱动”。
            int waitMs = 2000;
            if (const char *value = std::getenv("CV_SCREENSHOT_WAIT_MS"))
            {
                const int parsed = std::atoi(value);
                if (parsed > 0) waitMs = parsed;
            }
            const auto deadline = std::chrono::steady_clock::now() +
                                  std::chrono::milliseconds(waitMs);
            while (std::chrono::steady_clock::now() < deadline)
            {
                applyIncomingFrame();
                if (radarState_.load() == RadarState::Live) break;
                std::this_thread::sleep_for(std::chrono::milliseconds(200));
            }
            applyIncomingFrame();
            updateHud();
        }

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
