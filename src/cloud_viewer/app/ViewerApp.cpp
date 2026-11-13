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

#ifndef _WIN32
#include <arpa/inet.h>
#include <sys/socket.h>
#include <unistd.h>
#endif

#include <vtkAxesActor.h>
#include <vtkCaptionActor2D.h>
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

    /// 鼠标移动只做相机变换，不再补推点云帧。
    /// 原实现在每次 mousemove 都调用 pumpLiveFrame()，等于拖动期间把
    /// "相机重绘 + 全量点云重建" 叠在一起，是最直接的掉帧来源。
    /// 点云更新由 33ms 定时器负责，交互期间会被自动暂停，松手后补帧。
    void OnMouseMove() override
    {
        vtkInteractorStyleTrackballCamera::OnMouseMove();
    }

private:
    ViewerApp *app_ = nullptr;
};

vtkStandardNewMacro(SelectionStyle);

// 读取内核到目标雷达的路由源地址；不会发送 UDP 包。
std::string betaRouteSource(const std::string &ip)
{
#ifndef _WIN32
    sockaddr_in remote{};
    remote.sin_family = AF_INET;
    remote.sin_port = htons(50007);
    if (inet_pton(AF_INET, ip.c_str(), &remote.sin_addr) != 1) return {};
    const int fd = socket(AF_INET, SOCK_DGRAM, 0);
    if (fd < 0) return {};
    sockaddr_in local{};
    socklen_t size = sizeof(local);
    const bool ok = connect(fd, reinterpret_cast<sockaddr *>(&remote), sizeof(remote)) == 0 &&
                    getsockname(fd, reinterpret_cast<sockaddr *>(&local), &size) == 0;
    close(fd);
    char address[INET_ADDRSTRLEN]{};
    if (!ok || !inet_ntop(AF_INET, &local.sin_addr, address, sizeof(address))) return {};
    return address;
#else
    (void)ip;
    return {};
#endif
}

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

    // 拖动/缩放期间暂停点云重建：相机操作本身已在连续重绘，
    // 再叠加每秒 20 次的全量点云更新会明显掉帧。松手后补一帧。
    auto interactionCallback = vtkSmartPointer<vtkCallbackCommand>::New();
    interactionCallback->SetCallback(
        [](vtkObject *, unsigned long eventId, void *clientData, void *)
        {
            auto *app = static_cast<ViewerApp *>(clientData);
            if (app)
            {
                app->setInteracting(eventId == vtkCommand::StartInteractionEvent);
            }
        });
    interactionCallback->SetClientData(this);
    interactor->AddObserver(vtkCommand::StartInteractionEvent, interactionCallback);
    interactor->AddObserver(vtkCommand::EndInteractionEvent, interactionCallback);

    // 渲染帧率：每次 Render 完成计数，HUD 每 0.5 秒刷新一次显示。
    if (auto *window = renderer_.window())
    {
        auto renderTickCallback = vtkSmartPointer<vtkCallbackCommand>::New();
        renderTickCallback->SetCallback(
            [](vtkObject *, unsigned long, void *clientData, void *)
            {
                auto *app = static_cast<ViewerApp *>(clientData);
                if (app)
                {
                    app->onRenderTick();
                }
            });
        renderTickCallback->SetClientData(this);
        window->AddObserver(vtkCommand::EndEvent, renderTickCallback);
    }

    hudActor_ = vtkSmartPointer<vtkTextActor>::New();
    hudActor_->GetTextProperty()->SetFontSize(14);
    hudActor_->GetTextProperty()->SetColor(0.92, 0.92, 0.92);
    hudActor_->GetTextProperty()->SetBackgroundColor(0.0, 0.0, 0.0);
    hudActor_->GetTextProperty()->SetBackgroundOpacity(0.35);
    hudActor_->GetPositionCoordinate()->SetCoordinateSystemToNormalizedViewport();
    // 左下角：顶部是按钮区，放这里不会再和按钮重叠；文本从底部向上排版。
    hudActor_->SetPosition(0.01, 0.02);
    renderer_.renderer()->AddActor2D(hudActor_);

    infoActor_ = vtkSmartPointer<vtkTextActor>::New();
    infoActor_->GetTextProperty()->SetFontSize(13);
    infoActor_->GetTextProperty()->SetColor(1.0, 0.85, 0.4);
    infoActor_->GetTextProperty()->SetBackgroundColor(0.0, 0.0, 0.0);
    infoActor_->GetTextProperty()->SetBackgroundOpacity(0.35);
    infoActor_->GetPositionCoordinate()->SetCoordinateSystemToNormalizedViewport();
    // 右下角，与左下角的 HUD 分开，互不遮挡。
    infoActor_->SetPosition(0.70, 0.02);
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
    ui_.addButton("Mode: Cloud", 0.010, 0.863, 0.145, kButtonHeight,
                  [this] { switchRadarMode(); });
    roiButtonIndex_ = ui_.size();
    ui_.addButton("ROI: 0", 0.160, 0.863, 0.100, kButtonHeight,
                  [this] { switchRadarRoi(); });
    ui_.addButton("Select Calib", 0.265, 0.863, 0.145, kButtonHeight,
                  [this] { loadLiveCalibration(); });
    calibrationButtonIndex_ = ui_.size();
    ui_.addButton("App Cal: OFF", 0.415, 0.863, 0.205, kButtonHeight,
                  [this] { toggleLiveCalibration(); });
    // 一键把目录标定包（ys_calibration.ini + ys_encoder_angle.csv + spot_slope_map.csv）
    // 打包回聚合表 cs96beta_calibration.csv。
    // 不能放在 y=0.863 这一排：面选择 F1~F4 占了 0.628~0.863，会盖住。
    // 放设备操作行（y=0.735）右侧的空档 0.450~0.600。
    ui_.addButton("Update Calib", 0.450, 0.735, 0.150, kButtonHeight,
                  [this] { updateCalibTableFromDirectory(); });
    // 水平角解算方案切换：Beta（查表 + 面起始角）/ Alpha（编码盘差分 + 帧中点基准）。
    // 两者是不同代固件对同一物理量的算法，切换后可直接对照点云形态差异。
    angleSchemeButtonIndex_ = ui_.size();
    ui_.addButton("Angle: Beta", 0.610, 0.735, 0.140, kButtonHeight,
                  [this] { toggleHorizontalAngleScheme(); });

    // 面选择：F1..F4 独立开关，任意组合；All 一键全开。
    for (int index = 0; index < 4; ++index)
    {
        faceButtonIndices_[index] = ui_.size();
        const std::string label = "F" + std::to_string(index + 1);
        ui_.addButton(label, 0.628 + 0.060 * index, 0.863, 0.055, kButtonHeight,
                      [this, index] { toggleBetaFace(index); });
    }
    faceAllButtonIndex_ = ui_.size();
    ui_.addButton("All", 0.868, 0.863, 0.070, kButtonHeight,
                  [this]
                  {
                      if (!betaStream_)
                      {
                          logUi("Faces", "当前不是 CS96 Beta，面选择无效", false);
                          return;
                      }
                      betaFaceMask_ = 0x0F;
                      betaStream_->setFaceMask(betaFaceMask_);
                      updateFaceButtons();
                      logUi("Faces", "显示面帧 " + faceMaskLabel(betaFaceMask_));
                      updateHud();
                      renderer_.render();
                  });
    ui_.addButton("Connect Beta IP", 0.010, 0.735, 0.220, kButtonHeight,
                  [this] { connectBetaManual(); });
    startBetaButtonIndex_ = ui_.size();
    ui_.addButton("Start you_see", 0.240, 0.735, 0.200, kButtonHeight,
                  [this] { startBetaProcess(); });

    // 动态设备按钮从该索引开始，刷新时只移除这一段，避免误删固定按钮。
    firstDeviceButtonIndex_ = ui_.size();

    // 未连接时按 Unknown 处理：隐藏型号专属按钮（面选择、ROI）。
    updateModelDependentUi();
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

// ---- 设备 SSH：主机密钥（TOFU）与自动恢复 ---------------------------------

/// 设备主机密钥文件；环境变量优先，默认 ~/.config/cloud_viewer/beta_known_hosts。
static std::string betaKnownHostsPath()
{
    const char *configured = std::getenv("CV_BETA_KNOWN_HOSTS");
    if (configured != nullptr && configured[0] != '\0')
    {
        return configured;
    }
    const char *home = std::getenv("HOME");
    if (home != nullptr && home[0] != '\0')
    {
        return std::string(home) + "/.config/cloud_viewer/beta_known_hosts";
    }
    return {};
}

struct ShellOutput
{
    int rc = -1;
    std::string text;
};

static ShellOutput runShellCommand(const std::string &command)
{
    ShellOutput result;
    FILE *pipe = popen(command.c_str(), "r");
    if (pipe == nullptr)
    {
        result.text = "无法执行 SSH 命令";
        return result;
    }
    char buffer[512];
    while (std::fgets(buffer, sizeof(buffer), pipe) != nullptr)
    {
        result.text += buffer;
    }
    result.rc = pclose(pipe);
    return result;
}

/// 主机密钥与本地记录不一致：通常意味着换了一台雷达，或设备重刷过固件。
static bool hostKeyMismatch(const std::string &output)
{
    return output.find("REMOTE HOST IDENTIFICATION HAS CHANGED") != std::string::npos;
}

/// 只删除该 IP 的旧条目，不影响其它主机；不放开全局校验。
static bool forgetHostKey(const std::string &ip, const std::string &knownHosts)
{
    if (knownHosts.empty())
    {
        return false;
    }
    return std::system(("ssh-keygen -R " + ip + " -f " + shellQuote(knownHosts) +
                        " >/dev/null 2>&1").c_str()) == 0;
}

/// 把 SSH 原始输出翻译成可定位的原因，避免把整段 banner 直接丢给用户。
static std::string betaSshFailureReason(const std::string &output)
{
    if (hostKeyMismatch(output)) return "设备 SSH 主机密钥与本地记录冲突，自动更新信任失败";
    if (output.find("Permission denied") != std::string::npos) return "SSH 密码错误";
    if (output.find("Connection refused") != std::string::npos)
        return "设备 22 端口未开放（sshd 未运行）";
    if (output.find("Connection timed out") != std::string::npos ||
        output.find("No route to host") != std::string::npos)
        return "无法到达设备，检查网线与设备 IP";
    if (output.find("UNSUPPORTED_MODE=") != std::string::npos) return "设备 work_mode 不被支持";
    // 固件在编译期固定了数据模式：改配置后 you_see 会拒绝启动并回滚。
    if (output.find("与当前固件数据模式不匹配") != std::string::npos ||
        output.find("load config fail") != std::string::npos)
        return "该固件不支持此 work_mode（配置与固件数据模式不一致），已回滚原模式";
    if (output.find("CONFIG_INVALID") != std::string::npos ||
        output.find("PORT_CONFIG_INVALID") != std::string::npos)
        return "设备 /etc/ys_conf.ini 缺少 center_ip / center_data_port";
    if (output.find("START_FAILED") != std::string::npos ||
        output.find("RESTART_FAILED") != std::string::npos)
        return "设备 you_see 进程启动失败";
    return output.substr(0, 200);
}

/// 设备的 mode 文件决定加载哪套驱动与库，work_mode 必须与之匹配：
/// histogram -> 0 直方图；point_cloud -> 1/2 原始/真实点云；point_cloud_gray -> 4 灰度。
/// 返回允许的 work_mode 位图（bit N 表示 mode N）。
static std::uint8_t betaAllowedModesForDeviceMode(const std::string &deviceMode)
{
    if (deviceMode == "histogram") return static_cast<std::uint8_t>(1u << 0);
    if (deviceMode == "point_cloud_gray") return static_cast<std::uint8_t>(1u << 4);
    // point_cloud 或未知：只能在同一驱动下切换原始点云/真实点云。
    return static_cast<std::uint8_t>((1u << 1) | (1u << 2));
}

/// 确保 known_hosts 的目录存在；文件本身由 accept-new 自动创建。
/// 首次使用某台设备时文件可以不存在，不应据此直接拒绝。
static bool ensureKnownHostsReady(const std::string &knownHosts, std::string &message)
{
    if (knownHosts.empty())
    {
        message = "无法确定 SSH 主机密钥文件路径";
        return false;
    }
    std::error_code ec;
    const auto directory = std::filesystem::path(knownHosts).parent_path();
    if (!directory.empty())
    {
        std::filesystem::create_directories(directory, ec);
    }
    return true;
}

/// 在设备上执行脚本；主机密钥变更时更新信任并重试一次（新密钥仍会写回文件）。
static bool runBetaSsh(const std::string &ip, const std::string &password,
                       const std::string &knownHosts, const std::string &script,
                       const std::string &remoteArgs, std::string &output)
{
    auto build = [&](const char *policy)
    {
        return "printf '%s' " + shellQuote(script) + " | SSHPASS=" + shellQuote(password) +
               " timeout 25 sshpass -e ssh -o StrictHostKeyChecking=" + policy +
               " -o UserKnownHostsFile=" + shellQuote(knownHosts) +
               " -o ConnectTimeout=6 -o ServerAliveInterval=3 root@" + ip +
               " sh -s" + remoteArgs + " 2>&1";
    };

    // 首次连接用 accept-new：未知主机自动记录进 known_hosts；
    // 已记录主机的密钥变化仍会被拒绝（不是无条件信任）。
    ShellOutput run = runShellCommand(build("accept-new"));
    if (!hostKeyMismatch(run.text))
    {
        output = run.text;
        return run.rc == 0;
    }

    const bool forgotten = forgetHostKey(ip, knownHosts);
    run = runShellCommand(build("accept-new"));
    output = run.text;
    if (forgotten)
    {
        output += "\n[HOST-KEY] 设备主机密钥已变更（换了雷达或设备重刷固件），已更新信任并自动重试";
    }
    return run.rc == 0;
}

/// 极简 HTTP POST（不依赖 curl），用于调用设备自带 CGI。
static bool httpPost(const std::string &ip, std::uint16_t port, const std::string &path,
                     const std::string &body, int timeoutMs, std::string &response)
{
#ifndef _WIN32
    const int fd = ::socket(AF_INET, SOCK_STREAM, 0);
    if (fd < 0)
    {
        return false;
    }
    timeval timeout{};
    timeout.tv_sec = timeoutMs / 1000;
    timeout.tv_usec = (timeoutMs % 1000) * 1000;
    ::setsockopt(fd, SOL_SOCKET, SO_RCVTIMEO, &timeout, sizeof(timeout));
    ::setsockopt(fd, SOL_SOCKET, SO_SNDTIMEO, &timeout, sizeof(timeout));

    sockaddr_in address{};
    address.sin_family = AF_INET;
    address.sin_port = htons(port);
    if (::inet_pton(AF_INET, ip.c_str(), &address.sin_addr) != 1 ||
        ::connect(fd, reinterpret_cast<sockaddr *>(&address), sizeof(address)) != 0)
    {
        ::close(fd);
        return false;
    }

    std::ostringstream request;
    request << "POST " << path << " HTTP/1.1\r\n"
            << "Host: " << ip << "\r\n"
            << "Content-Type: application/x-www-form-urlencoded\r\n"
            << "Content-Length: " << body.size() << "\r\n"
            << "Connection: close\r\n\r\n"
            << body;
    const std::string text = request.str();

    std::size_t sent = 0;
    while (sent < text.size())
    {
        const ssize_t chunk = ::send(fd, text.data() + sent, text.size() - sent, 0);
        if (chunk <= 0)
        {
            ::close(fd);
            return false;
        }
        sent += static_cast<std::size_t>(chunk);
    }

    response.clear();
    char buffer[512];
    while (response.size() < 8192)
    {
        const ssize_t received = ::recv(fd, buffer, sizeof(buffer), 0);
        if (received <= 0)
        {
            break;
        }
        response.append(buffer, static_cast<std::size_t>(received));
    }
    ::close(fd);
    return !response.empty();
#else
    (void)ip;
    (void)port;
    (void)path;
    (void)body;
    (void)timeoutMs;
    (void)response;
    return false;
#endif
}

/// 确保设备 SSH 可用；不通时通过设备自带 set_ssh.cgi 打开。
///
/// 设备的 /www 由 lighttpd 提供，/cgi-bin/set_ssh.cgi 接受 POST "enable=true"，
/// 内部执行 S50sshd start。这是 SSH 关闭后唯一可用的远程恢复通道。
static bool ensureSshReady(const std::string &ip, std::string &message)
{
    if (DeviceDiscovery::tcpPortOpen(ip, 22, 600))
    {
        message = "SSH 已就绪（" + ip + "）";
        return true;
    }

    std::string response;
    if (!httpPost(ip, 80, "/cgi-bin/set_ssh.cgi", "enable=true", 3000, response))
    {
        message = "SSH 未开放，且 " + ip +
                  " 的 web 接口（80）不可达；请稍后再试，或重新上电/现场处理";
        return false;
    }
    if (response.find("\"code\": 0") == std::string::npos &&
        response.find("\"code\":0") == std::string::npos)
    {
        message = "请求开启 SSH 失败，设备返回: " + response.substr(0, 120);
        return false;
    }

    // CGI 只负责拉起 sshd，真正监听 22 还需要一点时间。
    for (int attempt = 0; attempt < 20; ++attempt)
    {
        std::this_thread::sleep_for(std::chrono::milliseconds(500));
        if (DeviceDiscovery::tcpPortOpen(ip, 22, 400))
        {
            message = "SSH 未开放，已通过设备 web 接口开启（" + ip + "）";
            return true;
        }
    }
    message = "已请求开启 SSH，但 22 端口在 10 秒内仍不可用";
    return false;
}

/// 在设备上检查并启动 you_see。只启动进程，不修改任何配置。
static bool startBetaProcessOn(const std::string &ip, const std::string &password,
                               const std::string &knownHosts, std::string &message)
{
    const std::string script = R"SH(set -eu
if pidof you_see >/dev/null; then
  echo ALREADY_RUNNING=$(pidof you_see)
  exit 0
fi
log=/var/log/ys.log
before=0
if [ -f "$log" ]; then before=$(wc -l < "$log"); fi
setsid sh -c 'cd /root && exec ./you_see >>/tmp/you_stdout.log 2>&1' </dev/null >/dev/null 2>&1 &
sleep 4
pid=$(pidof you_see || true)
if [ -z "$pid" ]; then
  echo START_FAILED
  if [ -f "$log" ]; then
    tail -n +$((before + 1)) "$log" 2>/dev/null | grep -E "不匹配|load config fail|FATAL" | head -n 2 || true
  fi
  exit 1
fi
echo STARTED=$pid
)SH";

    std::string output;
    runBetaSsh(ip, password, knownHosts, script, "", output);

    if (output.find("ALREADY_RUNNING=") != std::string::npos)
    {
        message = "设备端 you_see 已在运行（" + ip + "）";
        return true;
    }
    if (output.find("STARTED=") != std::string::npos)
    {
        message = "已启动设备端 you_see（" + ip + "）";
        return true;
    }
    message = "启动 you_see 失败: " + betaSshFailureReason(output);
    return false;
}

// 修改前验证 host key；只在目标 IP 确实变化时重启指定雷达进程。
static bool prepareBetaTarget(const std::string &ip, std::uint16_t dataPort,
                              const std::string &password, std::string &message,
                              std::string &deviceMode, std::string &resolution)
{
    const std::string localIp = betaRouteSource(ip);
    if (localIp.empty()) { message = "无法从路由确定到 " + ip + " 的本机 IPv4"; return false; }
    if (password.empty()) { message = "未配置 Beta SSH 密码"; return false; }
    const std::string knownHosts = betaKnownHostsPath();
    if (!ensureKnownHostsReady(knownHosts, message))
    {
        return false;
    }
    const std::string script = R"SH(set -eu
conf=/etc/ys_conf.ini
expected="$1"
expected_port="$2"
# 设备 mode 文件决定加载哪套驱动/库，work_mode 必须与之匹配（point_cloud/histogram/point_cloud_gray）。
dev_mode=$(cat /mnt/mtdblock2/mode 2>/dev/null || echo unknown)
echo "DEVICE_MODE=$dev_mode"
# 标定表里记录的分辨率：96x800（IMX459）或 120x800（SL100）。
lines_v=$(awk -F= '/^[[:space:]]*lines_v[[:space:]]*=/{v=$2; sub(/;.*/,"",v); gsub(/[^0-9]/,"",v); if (v != "") print v; exit}' /etc/ys_calibration.ini 2>/dev/null || true)
lines_h=$(awk -F= '/^[[:space:]]*lines_h[[:space:]]*=/{v=$2; sub(/;.*/,"",v); gsub(/[^0-9]/,"",v); if (v != "") print v; exit}' /etc/ys_calibration.ini 2>/dev/null || true)
if [ -n "$lines_v" ] && [ -n "$lines_h" ]; then echo "RESOLUTION=${lines_v}x${lines_h}"; fi
mode=$(awk -F '"' '/^[[:space:]]*work_mode[[:space:]]*=/{n++; v=$2} END{if(n==1) print v}' "$conf")
case "$mode" in 0|1|2) ;; *) echo "UNSUPPORTED_MODE=$mode"; exit 1;; esac
old_port=$(awk -F '"' '/^[[:space:]]*center_data_port[[:space:]]*=/{n++; v=$2} END{if(n==1) print v}' "$conf")
[ -n "$old_port" ] || { echo PORT_CONFIG_INVALID; exit 1; }
old=$(awk -F '"' '/^[[:space:]]*center_ip[[:space:]]*=/{n++; v=$2} END{if(n==1) print v}' "$conf")
[ -n "$old" ] || { echo CONFIG_INVALID; exit 1; }
if [ "$old" = "$expected" ] && [ "$old_port" = "$expected_port" ]; then
  if ! pidof you_see >/dev/null; then
    setsid sh -c 'cd /root && exec ./you_see >>/tmp/you_stdout.log 2>&1' </dev/null >/dev/null 2>&1 &
    sleep 3
    pidof you_see >/dev/null || { echo START_FAILED; exit 1; }
    echo PROCESS_STARTED
  fi
  echo CENTER_UNCHANGED="$old"
  exit 0
fi
backup=$(mktemp /tmp/cv_center_backup.XXXXXX)
tmp=$(mktemp /tmp/cv_center_new.XXXXXX)
cp -p "$conf" "$backup"
changed=0
rollback() {
  rc=$?
  if [ "$rc" -ne 0 ] && [ "$changed" = 1 ]; then
    cp "$backup" "$conf"
    pids=$(pidof you_see || true); [ -z "$pids" ] || kill -9 $pids || true
    setsid sh -c 'cd /root && exec ./you_see >>/tmp/you_stdout.log 2>&1' </dev/null >/dev/null 2>&1 &
    echo CENTER_RESTORED
  fi
  rm -f "$backup" "$tmp"
}
trap rollback 0
awk -v ip="$expected" -v port="$expected_port" '
/^[[:space:]]*center_ip[[:space:]]*=/{sub(/"[^"]*"/, "\"" ip "\"")}
/^[[:space:]]*center_data_port[[:space:]]*=/{sub(/"[^"]*"/, "\"" port "\"")}
{print}' "$conf" > "$tmp"
actual=$(awk -F '"' '/^[[:space:]]*center_ip[[:space:]]*=/{n++; v=$2} END{if(n==1) print v}' "$tmp")
actual_port=$(awk -F '"' '/^[[:space:]]*center_data_port[[:space:]]*=/{n++; v=$2} END{if(n==1) print v}' "$tmp")
[ "$actual" = "$expected" ] && [ "$actual_port" = "$expected_port" ] || { echo CONFIG_WRITE_FAILED; exit 1; }
cp "$tmp" "$conf"
changed=1
pids=$(pidof you_see || true)
[ -z "$pids" ] || kill -9 $pids
sleep 1
setsid sh -c 'cd /root && exec ./you_see >>/tmp/you_stdout.log 2>&1' </dev/null >/dev/null 2>&1 &
sleep 3
pidof you_see >/dev/null || { echo RESTART_FAILED; exit 1; }
changed=0
echo CENTER_UPDATED="$old:$old_port"-"$expected:$expected_port"
)SH";
    std::string output;
    const bool sshOk = runBetaSsh(ip, password, knownHosts, script,
                                  " -- " + shellQuote(localIp) + " " + std::to_string(dataPort),
                                  output);
    if (!sshOk || (output.find("CENTER_UNCHANGED=") == std::string::npos &&
                   output.find("CENTER_UPDATED=") == std::string::npos))
    {
        message = "设置 center_ip 失败: " + betaSshFailureReason(output);
        return false;
    }
    const auto modePos = output.find("DEVICE_MODE=");
    if (modePos != std::string::npos)
    {
        const std::size_t begin = modePos + std::string("DEVICE_MODE=").size();
        const std::size_t end = output.find_first_of("\r\n", begin);
        deviceMode = output.substr(begin, end == std::string::npos ? std::string::npos : end - begin);
    }
    const auto resolutionPos = output.find("RESOLUTION=");
    if (resolutionPos != std::string::npos)
    {
        const std::size_t begin = resolutionPos + std::string("RESOLUTION=").size();
        const std::size_t end = output.find_first_of("\r\n", begin);
        resolution = output.substr(begin, end == std::string::npos ? std::string::npos : end - begin);
    }
    message = "目标 " + ip + "，本机 " + localIp + "：" + output;
    return true;
}

void ViewerApp::connectBeta(const std::string &ip)
{
    rememberDevice(ip);
    // 先按转镜雷达初始化（面选择等按钮立即可用）；
    // 拿到设备规格后再细分 Beta（96 线 / IMX459）或 Gamma（120 线 / SL100）。
    setActiveRadarModel(RadarModel::BetaRotating);
    // 用广播中的 work_mode 初始化当前模式，避免 HUD 与实际包格式不一致。
    betaWorkMode_ = 2;
    // 换设备或重连后重新探测固件支持的模式。
    betaUnsupportedModes_ = 0;
    for (const auto &device : discoveredDevices_)
    {
        if (device.ip == ip)
        {
            betaWorkMode_ = device.workMode;
        }
    }

    // 模式 >2 只有灰度点云（work_mode=4，包长 1165B），本解析器没有实现。
    // 设备驱动本身就是 point_cloud，所以不必换固件，只要把配置里的 work_mode
    // 改回 2 并重启 you_see 即可。applyBetaWorkMode 会备份配置、重启后 pidof 校验，
    // 起不来就自动回滚，因此可以安全地自动执行。
    if (betaWorkMode_ > 2)
    {
        const int grayMode = betaWorkMode_;
        logUi("Connect Device", "设备处于灰度和模式 " + std::to_string(grayMode) +
                                    "，正在自动切回 work_mode=2（真实点云）…");
        std::string switchMessage;
        if (!applyBetaWorkMode(ip, 2, switchMessage))
        {
            radarState_ = RadarState::Error;
            {
                std::lock_guard<std::mutex> lock(radarMessageMutex_);
                radarMessage_ = "设备模式 " + std::to_string(grayMode) +
                                " 自动切换失败: " + switchMessage;
            }
            logUi("Connect Device", "自动切换 work_mode=2 失败，未连接: " + switchMessage,
                  false);
            updateHud();
            renderer_.render();
            return;
        }
        logUi("Connect Device", "已自动切到 work_mode=2（真实点云）");
        for (auto &device : discoveredDevices_)
        {
            if (device.ip == ip)
            {
                device.workMode = 2;
            }
        }
        betaWorkMode_ = 2;
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

    // 准备推流目标；失败时仍监听，并在 UI 明确显示原因，便于手动恢复。
    logUi("Connect Device", "正在核对设备 center_ip 和本机路由地址");
    updateHud();
    renderer_.render();
    std::string targetMessage;
    std::string deviceMode;
    std::string deviceResolution;
    const bool targetReady = prepareBetaTarget(ip, config.localPort, betaSshPassword(),
                                               targetMessage, deviceMode, deviceResolution);
    // 记录设备 mode，后续 work_mode 切换只在该驱动支持的集合内循环。
    betaDeviceMode_ = deviceMode;
    betaAllowedModes_ = betaAllowedModesForDeviceMode(deviceMode);
    logUi("Beta Target", targetMessage, targetReady);
    if (!targetReady && targetMessage.find("UNSUPPORTED_MODE=") != std::string::npos)
    {
        betaStream_.reset();
        radarConnected_ = false;
        radarState_ = RadarState::Error;
        {
            std::lock_guard<std::mutex> lock(radarMessageMutex_);
            radarMessage_ = "CS96 Beta 设备当前模式不支持；需要匹配固件";
        }
        updateHud();
        renderer_.render();
        return;
    }

    auto sink = [this](algo_lidar::PointMatrix &&matrix, std::uint64_t frameNumber)
    {
        const int detected = betaStream_ ? betaStream_->stats().detectedMode : 2;
        FrameMetadata metadata;
        // 原始点云与真实点云都是单面 96x800，直方图为 96x512。
        metadata.rows = static_cast<int>(matrix.size());
        metadata.cols = matrix.empty() ? 0 : static_cast<int>(matrix.front().size());
        // 型号由连接时按规格确定（96 线 Beta / 120 线 Gamma），这里如实透传。
        metadata.model = isRotatingModel(activeModel_) ? activeModel_
                                                       : RadarModel::BetaRotating;
        metadata.roiId = 0;
        metadata.frameNumber = frameNumber;
        metadata.timestampUnit = "frame";
        metadata.algorithmId = detected == 1 ? "cs96_beta_origin"
                              : detected == 0 ? "cs96_beta_histogram"
                                              : "cs96_beta_cartesian";
        metadata.calibrationState = detected == 2 ? "device_xyz" : "device_raw";

        // 在线帧只生成当前颜色源，避免每帧为 30 万点填 13 个标量场。
        auto frame = makeFrame(std::move(matrix), metadata, 0, colorSource_);
        std::lock_guard<std::mutex> lock(radarMutex_);
        pendingRadarFrame_ = frame;
    };

    // 标定表在设备规格确定后按型号加载（120 线 Gamma 与 96 线 Beta 不可混用）。

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

    // 规格：优先广播上报（最可靠），其次设备标定表里的 lines_v/lines_h。
    int specLines = 0;
    int specSlots = 0;
    for (const auto &device : discoveredDevices_)
    {
        if (device.ip == ip && device.lineCount > 0 && device.slotsPerFrame > 0)
        {
            specLines = device.lineCount;
            specSlots = device.slotsPerFrame;
            break;
        }
    }
    if (specLines <= 0 && !deviceResolution.empty())
    {
        int parsedLines = 0;
        int parsedSlots = 0;
        if (std::sscanf(deviceResolution.c_str(), "%dx%d", &parsedLines, &parsedSlots) == 2)
        {
            specLines = parsedLines;
            specSlots = parsedSlots;
        }
    }
    if (specLines > 0 && specLines <= 512)
    {
        config.lines = specLines;
    }
    if (specSlots > 0 && specSlots <= 8192)
    {
        config.slots = specSlots;
    }
    // Gamma（120 线 / SL100）板端不做 PA 纠偏，原始点云按帧号判面；
    // Beta（96 线 / IMX459）的 FPGA 帧号实测 99.9% 错位，必须用 PA 判面。
    config.faceFromFrameId = config.lines >= 120;
    logUi("Connect Device", "设备规格 " + std::to_string(config.lines) + "x" +
                                std::to_string(config.slots));
    // 型号按实际规格确立：120 线 = SL100 的 CS120 Gamma，96 线 = IMX459 的 CS96 Beta。
    // 两者共用解析与 UI，差异只在型号标识、包长与默认标定表。
    setActiveRadarModel(config.lines >= 120 ? RadarModel::GammaRotating
                                            : RadarModel::BetaRotating);

    // 原始点云标定表（距离 / 水平角 / 俯仰角 / 反射率）。
    // 默认读取聚合成单文件的 CSV；目录布局作为兼容回退。
    // 120 线（Gamma/SL100）与 96 线（Beta/IMX459）标定表不可混用，按规格选默认路径。
    const bool gammaSpec = config.lines >= 120;
    const std::string modelLabel = gammaSpec ? "CS120 Gamma" : "CS96 Beta";
    if (!betaCalibration_)
    {
        betaCalibration_ = std::make_shared<Cs96BetaCalibration>();
        const std::string csvName =
            gammaSpec ? "cs120gamma_calibration.csv" : "cs96beta_calibration.csv";
        // 资源路径不能只看 cwd：从 IDE、启动器或别的目录启动时 cwd 不是项目根，
        // 会出现「文件明明存在却报无法打开」。按 exe 目录优先，cwd 兜底。
        std::vector<std::filesystem::path> candidates;
        if (const char *override = std::getenv("CV_BETA_CALIB"))
        {
            candidates.emplace_back(override);
        }
        else
        {
            const auto exeDir = executableDirectory();
            candidates.push_back(exeDir / "calib" / csvName); // Select Calib 的落地目录
            candidates.push_back(exeDir / "data" / "param" / csvName);
            candidates.push_back(exeDir.parent_path() / "data" / "param" / csvName);
            candidates.push_back(std::filesystem::path("data") / "param" / csvName);
        }

        std::error_code ec;
        std::filesystem::path source;
        for (const auto &candidate : candidates)
        {
            if (std::filesystem::exists(candidate, ec))
            {
                source = candidate;
                break;
            }
        }
        if (source.empty())
        {
            source = candidates.front();
            const std::filesystem::path legacy =
                source.parent_path() /
                (gammaSpec ? "cs120gamma_calib_data" : "cs96beta_calib_data");
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
            detail << modelLabel << " 标定已加载: " << source.string()
                   << " (" << betaCalibration_->lineCount() << "x" << betaCalibration_->slotCount()
                   << ") start=" << starts[0] << "," << starts[1] << "," << starts[2] << ","
                   << starts[3];
            logUi("Calib", detail.str());
        }
        else
        {
            logUi("Calib", modelLabel + " 标定加载失败: " + calibrationError, false);
            betaCalibration_.reset();
        }
    }

    // 中心线 u 只依赖标定参数；即使整张表因线数不匹配被拒，兜底解算也用它，
    // 避免 120 线被按 (lines-1)/2=59 推导而整体上仰约 2°。
    if (betaCalibration_ && betaCalibration_->loaded())
    {
        config.pitchCenter = betaCalibration_->thetaCenterLine();
    }

    // 标定表必须与设备线数一致，否则原始点云的像素索引与俯仰角会整体错位。
    if (betaCalibration_ && betaCalibration_->loaded() &&
        betaCalibration_->lineCount() != config.lines)
    {
        logUi("Calib", "标定表为 " + std::to_string(betaCalibration_->lineCount()) +
                          " 线，与设备 " + std::to_string(config.lines) +
                          " 线不匹配，本次不套用",
              false);
        betaCalibration_.reset();
    }

    betaStream_->setCalibration(betaCalibration_);
    betaStream_->setCalibrationEnabled(betaCalibrationEnabled_);
    // 水平角方案独立于四项标定开关，连接时单独下发一次。
    {
        auto options = betaStream_->decodeOptions();
        if (options.alphaHorizontalAngle != alphaHorizontalAngle_)
        {
            options.alphaHorizontalAngle = alphaHorizontalAngle_;
            betaStream_->setDecodeOptions(options);
        }
    }
    betaStream_->setFaceMask(betaFaceMask_);

    // 调试用：连接前按环境变量把设备切到指定 work_mode
    //（0=直方图 1=原始点云 2=真实点云），便于无人值守验证原始点云链路。
    if (const char *wmEnv = std::getenv("CV_BETA_WORK_MODE"))
    {
        const int mode = std::atoi(wmEnv);
        if (mode >= 0 && mode <= 2 && mode != betaWorkMode_)
        {
            std::string modeMessage;
            if (applyBetaWorkMode(ip, mode, modeMessage))
            {
                betaWorkMode_ = mode;
                logUi("Mode", "已按 CV_BETA_WORK_MODE 切到 work_mode=" + std::to_string(mode));
            }
            else
            {
                logUi("Mode", "按 CV_BETA_WORK_MODE 切换失败: " + modeMessage, false);
            }
        }
    }

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
        radarMessage_ = targetReady ? std::string{} : targetMessage;
    }
    logUi("Connect Device", "CS96 Beta 监听 UDP " + std::to_string(config.localPort) +
                               "，等待 " + ip + " 推流");
    updateHud();
    renderer_.render();
}

void ViewerApp::connectBetaManual()
{
#ifndef _WIN32
    std::string ip;
    if (const char *value = std::getenv("CV_BETA_MANUAL_IP")) ip = value;
    else
    {
        FILE *dialog = popen("zenity --entry --title='连接 CS96 Beta' --text='请输入雷达 IPv4 地址' 2>/dev/null", "r");
        if (!dialog) { logUi("Connect Beta IP", "无法启动地址输入窗口", false); return; }
        char text[128]{};
        if (std::fgets(text, sizeof(text), dialog)) ip = text;
        const int result = pclose(dialog);
        if (result != 0) return;
    }
    ip.erase(ip.find_last_not_of(" \t\r\n") + 1);
    const auto start = ip.find_first_not_of(" \t\r\n");
    if (start == std::string::npos) return;
    ip.erase(0, start);
    in_addr addr{};
    if (inet_pton(AF_INET, ip.c_str(), &addr) != 1)
    {
        logUi("Connect Beta IP", "请输入有效的 IPv4 地址", false);
        return;
    }
    connectBeta(ip);
#else
    logUi("Connect Beta IP", "请通过 CV_BETA_IP 指定地址", false);
#endif
}

void ViewerApp::searchDevices()
{
    // CV_FORCE_SCAN=1 跳过广播直接扫网段，便于排障与验证主动发现。
    const bool forceScan = std::getenv("CV_FORCE_SCAN") != nullptr;
    if (forceScan)
    {
        logUi("Search Device", "CV_FORCE_SCAN：跳过广播，直接扫描本机网段");
        discoveredDevices_.clear();
    }
    else
    {
        logUi("Search Device", "开始监听 UDP 55000 广播，等待 2200ms (覆盖约两个广播周期)...");
        if (!discovery_)
        {
            discovery_ = std::make_unique<DeviceDiscovery>();
        }
        discovery_->start(55000);

        std::this_thread::sleep_for(std::chrono::milliseconds(2200));
        discoveredDevices_ = discovery_->snapshot();
        discovery_->stop();
    }

    // FL6532 固态雷达不发 55000 广播，只能按已知地址探测控制口 6508。
    std::vector<std::string> fl6532Candidates;
    if (const char *envIp = std::getenv("CV_RADAR_IP"))
    {
        fl6532Candidates.emplace_back(envIp);
    }
    if (!lastFl6532Ip_.empty())
    {
        fl6532Candidates.push_back(lastFl6532Ip_);
    }
    fl6532Candidates.emplace_back("192.168.137.200");

    std::string probeNote;
    const auto fl6532 = DeviceDiscovery::probeFl6532(fl6532Candidates, probeNote);
    logUi("Probe Device", probeNote, !fl6532.empty());
    for (const auto &device : fl6532)
    {
        const bool exists = std::any_of(
            discoveredDevices_.begin(), discoveredDevices_.end(),
            [&device](const DiscoveredDevice &existing) { return existing.ip == device.ip; });
        if (!exists)
        {
            discoveredDevices_.push_back(device);
        }
    }
    // 上次连接的设备若不在结果里，单独确认一次可达性：
    // 它可能只是 you_see 没运行（因此没有广播），设备本身仍在线。
    if (!lastDeviceIp_.empty())
    {
        const bool listed = std::any_of(
            discoveredDevices_.begin(), discoveredDevices_.end(),
            [this](const DiscoveredDevice &device) { return device.ip == lastDeviceIp_; });
        if (!listed)
        {
            const bool sshOpen = DeviceDiscovery::tcpPortOpen(lastDeviceIp_, 22, 400);
            const bool ctrlOpen = DeviceDiscovery::tcpPortOpen(lastDeviceIp_, 6508, 400);
            if (sshOpen || ctrlOpen)
            {
                DiscoveredDevice device;
                device.ip = lastDeviceIp_;
                device.model = ctrlOpen ? "FL6532" : "CS96 Beta";
                device.sshReady = sshOpen;
                device.fromProbe = true;
                device.lastSeenMs = nowMs();
                discoveredDevices_.push_back(device);
                logUi("Probe Device", "上次设备 " + lastDeviceIp_ + " 在线（" + device.model +
                                          "），当前没有广播");
            }
            else
            {
                logUi("Probe Device", "上次设备 " + lastDeviceIp_ + " 不可达（未上电或掉线）", false);
            }
        }
    }

    std::sort(discoveredDevices_.begin(), discoveredDevices_.end(),
              [](const DiscoveredDevice &a, const DiscoveredDevice &b) { return a.ip < b.ip; });

    if (discoveredDevices_.empty())
    {
        // 广播由设备端 you_see 发出，进程没跑就完全搜不到；而 ARP 由内核应答，
        // 只要设备网络栈起来了就能被看见。因此扫描本机网段作为兜底。
        logUi("Search Device", "未收到广播，改扫本机网段（ARP + MAC 前缀）");
        const char *configuredPrefix = std::getenv("CV_RADAR_MAC_PREFIX");
        const std::string macPrefix =
            (configuredPrefix != nullptr) ? std::string(configuredPrefix) : std::string("98:03:8a");
        std::string scanNote;
        const auto scanned = DeviceDiscovery::scanSubnet(macPrefix, scanNote);
        logUi("Scan Device", scanNote, !scanned.empty());

        if (!scanned.empty())
        {
            discoveredDevices_ = scanned;
            std::ostringstream detail;
            for (const auto &device : scanned)
            {
                detail << "[" << device.ip << " mac=" << device.mac
                       << (device.sshReady ? " ssh" : " no-ssh") << "] ";
            }
            logUi("Scan Device", "扫描到 " + std::to_string(scanned.size()) + " 台候选: " +
                                     detail.str());
        }
        else
        {
            logUi("Search Device",
                  "未发现设备：广播无响应，网段内也没有匹配 MAC 前缀的雷达"
                  "（可设 CV_RADAR_MAC_PREFIX= 列出全部邻居）", false);
        }
    }
    else
    {
        logUi("Search Device", "搜索完成，共发现 " + std::to_string(discoveredDevices_.size()) + " 台设备:");
        
        // 逐台设备打印详细信息，利用结构体里的全部字段
        for (const auto &device : discoveredDevices_)
        {
            std::ostringstream detail;
            detail << "发现 [" << device.model << "]"
                   << " | IP: " << device.ip
                   << " | SN: " << device.serialNumber
                   << " | 线数: " << device.lineCount
                   << " | Slots/帧: " << device.slotsPerFrame
                   << " | 工作模式: " << static_cast<int>(device.workMode);
            
            logUi("Search Device", detail.str());
        }
    }

    rebuildDeviceButtons();
    updateHud();
    renderer_.render();
}

void ViewerApp::startBetaProcess()
{
    // 目标优先级：最近操作的设备 > 当前连接的设备 > 搜索结果第一台。
    std::string ip = lastDeviceIp_;
    if (ip.empty() && betaStream_)
    {
        ip = betaStream_->config().radarIp;
    }
    if (ip.empty() && !discoveredDevices_.empty())
    {
        ip = discoveredDevices_.front().ip;
    }
    if (ip.empty())
    {
        logUi("Start you_see", "请先 Search 或连接一台设备", false);
        return;
    }

    const std::string password = betaSshPassword();
    if (password.empty())
    {
        logUi("Start you_see",
              "未配置 Beta SSH 密码（CV_BETA_SSH_PASS 或 ~/.config/cloud_viewer/beta_ssh.conf）",
              false);
        return;
    }
    const std::string knownHosts = betaKnownHostsPath();
    std::string keyFileMessage;
    if (!ensureKnownHostsReady(knownHosts, keyFileMessage))
    {
        logUi("Start you_see", keyFileMessage, false);
        return;
    }

    // 先确认 SSH 可用；关闭时用设备自带 web 接口打开，再执行启动。
    logUi("Start you_see", "检查 " + ip + " 的 SSH 服务");
    updateHud();
    renderer_.render();

    std::string sshMessage;
    if (!ensureSshReady(ip, sshMessage))
    {
        logUi("Start you_see", sshMessage, false);
        updateHud();
        renderer_.render();
        return;
    }
    logUi("Start you_see", sshMessage);

    std::string message;
    const bool ok = startBetaProcessOn(ip, password, knownHosts, message);
    lastDeviceIp_ = ip;
    logUi("Start you_see", message, ok);
    // 只有本次真正拉起进程时才提示等待；已在运行时不必再等。
    if (ok && message.find("已启动") != std::string::npos)
    {
        logUi("Start you_see", "稍等约 3 秒再点 Search，设备开始广播后即可连接");
    }
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
        std::string label;
        if (device.fromScan)
        {
            // 无广播元数据，只能显示 IP 与服务可达性。
            label = device.ip + (device.sshReady ? " [scan+ssh]" : " [scan/no-ssh]");
        }
        else if (device.fromProbe)
        {
            label = device.model + " " + device.ip + " [probe]";
        }
        else
        {
            label = device.model + " " + device.ip;
            if (device.lineCount > 0)
            {
                label += " " + std::to_string(device.lineCount) + "x" +
                         std::to_string(device.slotsPerFrame);
            }
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
    lastDeviceIp_ = device.ip;
    logUi("Connect Device", device.model + " " + device.ip +
                                (device.serialNumber.empty()
                                     ? std::string{}
                                     : " serial=" + device.serialNumber));

    if (device.fromScan)
    {
        // 扫描发现的设备没有广播元数据；22 不通时先尝试用 web 接口打开 SSH。
        if (!device.sshReady)
        {
            std::string sshMessage;
            if (!ensureSshReady(device.ip, sshMessage))
            {
                logUi("Connect Device", sshMessage, false);
                return;
            }
            logUi("Connect Device", sshMessage);
        }
        connectBeta(device.ip);
        return;
    }

    if (device.model == "CS96 Beta" || device.model == "CS96" ||
        device.model == "CS120 Gamma")
    {
        connectBeta(device.ip);
        return;
    }

    radarIpOverride_ = device.ip;
    connectRadar();
}

void ViewerApp::connectRadar()
{
    // 型号在这里确立：6532 没有转镜，面选择等 Beta 专属按钮会被隐藏。
    setActiveRadarModel(RadarModel::Fl6532);
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

        // 与 Beta 一致：在线帧只生成当前颜色源。
        auto frame = makeFrame(std::move(matrix), metadata, 0, colorSource_);
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
        // 记住该地址：下次搜索用它探测，Connect 按钮也优先重连它。
        lastFl6532Ip_ = config.radarIp;
        rememberDevice(config.radarIp);
        radarIpOverride_ = config.radarIp;
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
        // 统一交互：先搜索再连接。没有搜索结果时先补一次搜索。
        if (discoveredDevices_.empty())
        {
            logUi("Connect Radar", "尚无搜索结果，先执行一次搜索");
            searchDevices();
        }
        if (!discoveredDevices_.empty())
        {
            // 优先重连上次的设备，否则用列表第一台。
            std::size_t target = 0;
            if (!lastDeviceIp_.empty())
            {
                for (std::size_t i = 0; i < discoveredDevices_.size(); ++i)
                {
                    if (discoveredDevices_[i].ip == lastDeviceIp_)
                    {
                        target = i;
                        break;
                    }
                }
            }
            connectDiscoveredDevice(target);
            return;
        }
        logUi("Connect Radar", "未发现设备，按默认地址尝试连接", false);
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

    // 脚本以字符串形式经 stdin 交给设备执行，避免多层引号转义问题。
    std::ostringstream scriptBuilder;
    {
        std::ostream &script = scriptBuilder;
        // 配置写入或启动失败时恢复原文件并重启原模式，不能只凭 echo 判断成功。
        script << R"SH(#!/bin/sh
set -eu
conf=/etc/ys_conf.ini
backup=$(mktemp /tmp/cv_mode_backup.XXXXXX)
cp -p "$conf" "$backup"
changed=0
rollback() {
  rc=$?
  if [ "$rc" -ne 0 ] && [ "$changed" = 1 ]; then
    cp "$backup" "$conf"
    pids=$(pidof you_see || true); [ -z "$pids" ] || kill -9 $pids || true
    setsid sh -c 'cd /root && exec ./you_see >>/tmp/you_stdout.log 2>&1' </dev/null >/dev/null 2>&1 &
    echo MODE_RESTORED
  fi
  rm -f "$backup"
}
trap rollback 0
)SH" << "sed -i 's/^work_mode.*/work_mode                      = \"" << mode
               << "\" ;工作模式/' \"$conf\"\n"
               << R"SH(changed=1
log=/var/log/ys.log
log_before=0
if [ -f "$log" ]; then log_before=$(wc -l < "$log"); fi
pids=$(pidof you_see || true)
[ -z "$pids" ] || kill -9 $pids
sleep 1
setsid sh -c 'cd /root && exec ./you_see >>/tmp/you_stdout.log 2>&1' </dev/null >/dev/null 2>&1 &
sleep 3
if ! pidof you_see >/dev/null; then
  echo MODE_START_FAILED
  # you_see 的报错进的是日志文件；把本次新增的关键行带回上位机，便于给出准确原因。
  if [ -f "$log" ]; then
    tail -n +$((log_before + 1)) "$log" 2>/dev/null | grep -E "与当前固件数据模式不匹配|load config fail" | head -n 2 || true
  fi
  exit 1
fi
changed=0
echo SWITCH_OK
)SH";
    }

    const std::string knownHosts = betaKnownHostsPath();
    if (!ensureKnownHostsReady(knownHosts, message))
    {
        return false;
    }

    std::string output;
    // 设备端 setsid 启动后 SSH 可能因 fd 未关闭而超时，以脚本回显为准。
    runBetaSsh(ip, password, knownHosts, scriptBuilder.str(), "", output);
    if (output.find("SWITCH_OK") != std::string::npos)
    {
        message = output;
        return true;
    }
    message = "CS96 Beta work_mode=" + std::to_string(mode) + " 切换失败: " +
              betaSshFailureReason(output);
    return false;
}

void ViewerApp::toggleBetaFace(int index)
{
    if (!betaStream_)
    {
        logUi("Faces", "当前不是 CS96 Beta，面选择无效", false);
        return;
    }
    if (index < 0 || index > 3)
    {
        return;
    }

    const std::uint8_t bit = static_cast<std::uint8_t>(1u << index);
    std::uint8_t mask = static_cast<std::uint8_t>(betaFaceMask_ ^ bit);
    if (mask == 0)
    {
        // 全关会导致没有任何面可显示，回退到全部。
        mask = 0x0F;
        logUi("Faces", "已关闭最后一个面，自动恢复为全部", false);
    }
    betaFaceMask_ = mask;
    betaStream_->setFaceMask(betaFaceMask_);
    updateFaceButtons();

    logUi("Faces", "显示面帧 " + faceMaskLabel(betaFaceMask_));
    updateHud();
    renderer_.render();
}

void ViewerApp::updateFaceButtons()
{
    for (int index = 0; index < 4; ++index)
    {
        const bool enabled = (betaFaceMask_ & (1u << index)) != 0;
        ui_.setLabel(faceButtonIndices_[index],
                     "F" + std::to_string(index + 1) + (enabled ? "*" : ""));
    }
}

void ViewerApp::switchBetaMode()
{
    if (modeFuture_.valid())
    {
        logUi("Mode", "上一次模式切换尚未完成，忽略点击", false);
        return;
    }

    // 常规循环 2 -> 1 -> 0 -> 2，但只在设备 mode 支持的集合内循环，
    // 并跳过该固件运行期已拒绝过的模式（避免反复中断推流）。
    const std::uint8_t allowed =
        betaAllowedModes_ & static_cast<std::uint8_t>(~betaUnsupportedModes_);
    int next = (betaWorkMode_ == 2) ? 1 : (betaWorkMode_ == 1 ? 0 : 2);
    for (int attempt = 0; attempt < 3 && (allowed & (1u << next)) == 0; ++attempt)
    {
        next = (next == 2) ? 1 : (next == 1 ? 0 : 2);
    }
    if ((allowed & (1u << next)) == 0)
    {
        logUi("Mode", "设备 mode=" + (betaDeviceMode_.empty() ? std::string("unknown")
                                                              : betaDeviceMode_) +
                          "，当前没有可切换的 work_mode", false);
        return;
    }
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

void ViewerApp::rememberDevice(const std::string &ip)
{
    if (ip.empty())
    {
        return;
    }
    lastDeviceIp_ = ip;

    ViewerSettings settings = loadViewerSettings();
    if (settings.lastDeviceIp == ip)
    {
        return;
    }
    settings.lastDeviceIp = ip;
    std::string error;
    if (!saveViewerSettings(settings, error))
    {
        std::cout << "[cloud_viewer] 记录设备地址失败: " << error << "\n";
    }
}

void ViewerApp::setActiveRadarModel(RadarModel model)
{
    if (activeModel_ == model)
    {
        return;
    }
    activeModel_ = model;
    updateModelDependentUi();
    logUi("Model", std::string("雷达型号: ") + radarModelName(model));
}

void ViewerApp::updateModelDependentUi()
{
    const bool beta = isRotatingModel(activeModel_);
    const bool fl6532 = activeModel_ == RadarModel::Fl6532;

    // 面选择只对转镜雷达有意义：6532 是固态，没有“面”的概念。
    for (std::size_t index : faceButtonIndices_)
    {
        ui_.setVisible(index, beta);
    }
    ui_.setVisible(faceAllButtonIndex_, beta);

    // Start you_see 只服务 CS96 Beta；未连接时保留可见，便于先拉起设备再连接。
    ui_.setVisible(startBetaButtonIndex_, !fl6532);

    // ROI 分档是 6532 专有概念。
    ui_.setVisible(roiButtonIndex_, fl6532);
}

void ViewerApp::loadBetaCalibration()
{
    const auto files = FileDialogService::openFiles();
    if (files.empty())
    {
        logUi("Select Calib", "用户取消或未选择文件", false);
        return;
    }

    std::string copyError;
    const auto stored = importCalibrationFile(files.front(), copyError);
    if (stored.empty())
    {
        logUi("Select Calib", "复制标定表失败: " + copyError, false);
        return;
    }

    // 用户选的是源文件，实际加载的是复制后的副本（运行时不依赖原始路径）。
    // 两处都打出来：否则"选的是 data/param、日志却显示 build/calib"会让人以为没生效。
    {
        std::error_code sameEc;
        const bool inPlace = std::filesystem::exists(stored, sameEc) &&
                             std::filesystem::equivalent(files.front(), stored, sameEc);
        std::ostringstream where;
        if (inPlace)
        {
            where << "标定表源: " << files.front().string() << "（已在标定目录，直接加载）";
        }
        else
        {
            where << "标定表源: " << files.front().string() << " -> 复制到 " << stored.string()
                  << "（运行时加载副本）";
        }
        logUi("Select Calib", where.str());
    }

    auto calibration = std::make_shared<Cs96BetaCalibration>();
    std::string error;
    if (!calibration->load(stored, error))
    {
        controlState_ = "CS96 Beta calibration parse failed; previous table retained";
        logUi("Select Calib", "CS96 Beta 标定解析失败: " + error, false);
        updateHud();
        renderer_.render();
        return;
    }

    betaCalibration_ = calibration;
    betaCalibrationEnabled_ = true;
    // 正在运行的流持有的是**连接那一刻**的指针。不同步的话，这里换表只改了界面状态，
    // 实际解算仍用旧表（甚至兜底公式），用户完全无从判断有没有生效。
    if (betaStream_)
    {
        betaStream_->setCalibration(betaCalibration_);
        betaStream_->setCalibrationEnabled(betaCalibrationEnabled_);
        logUi("Select Calib", "已把新标定推送给正在运行的解码线程");
    }
    controlState_ = "CS96 Beta calibration: " + stored.filename().string();
    const auto &starts = betaCalibration_->startingAngles();
    std::ostringstream detail;
    detail << "CS96 Beta 标定已加载: " << stored.string()
           << " start=" << starts[0] << "," << starts[1] << "," << starts[2] << "," << starts[3];
    logUi("Select Calib", detail.str());
    updateHud();
    renderer_.render();
}

void ViewerApp::updateCalibTableFromDirectory()
{
    if (!FileDialogService::available())
    {
        logUi("Update Calib", "系统文件对话框不可用", false);
        return;
    }

    const auto chosen = FileDialogService::chooseDirectory();
    if (!chosen.has_value() || chosen->empty())
    {
        logUi("Update Calib", "用户取消目录选择", false);
        return;
    }
    const std::filesystem::path directory = *chosen;

    // 目录布局：ys_calibration.ini + ys_encoder_angle.csv + spot_slope_map.csv
    auto calibration = std::make_shared<Cs96BetaCalibration>();
    std::string error;
    if (!calibration->load(directory, error))
    {
        logUi("Update Calib", "目录标定解析失败: " + error, false);
        return;
    }

    std::error_code ec;
    if (!std::filesystem::exists(directory / "spot_slope_map.csv", ec) &&
        std::filesystem::exists(directory / "spot_slope_map.bin", ec))
    {
        logUi("Update Calib", "目录里只有 spot_slope_map.bin（暂不支持解析），本次不含 FPGA 旋转角修正",
              false);
    }

    // 写回"用户选的那个目录"：Update Calib 的语义就是"用这个目录里的散表重算聚合表"。
    // 以前硬编码成 exe/calib，导致在 data/param 里更新永远看不到变化。
    const auto primaryTarget = directory / "cs96beta_calibration.csv";
    const auto runtimeTarget = executableDirectory() / "calib" / "cs96beta_calibration.csv";
    std::filesystem::create_directories(primaryTarget.parent_path(), ec);

    // 以所选目录里现有的聚合表为基底：目录包里没有的部分（反射率、角度曲线）继续沿用，
    // 否则一次"更新"就会把没提供的 section 整段清空。
    Cs96BetaCalibration previous;
    std::filesystem::path basePath;
    if (std::filesystem::exists(primaryTarget, ec))
    {
        basePath = primaryTarget;
    }
    else if (std::filesystem::exists(runtimeTarget, ec))
    {
        basePath = runtimeTarget;
    }
    if (!basePath.empty())
    {
        std::string ignored;
        previous.load(basePath, ignored);
    }

    // 覆盖前先备份：标定表是现场实测数据，不能只靠记忆回滚。
    if (std::filesystem::exists(primaryTarget, ec))
    {
        const auto stamp =
            std::to_string(std::chrono::system_clock::now().time_since_epoch().count());
        const auto backup = primaryTarget.string() + ".bak." + stamp;
        std::error_code backupEc;
        std::filesystem::copy_file(primaryTarget, backup,
                                   std::filesystem::copy_options::overwrite_existing, backupEc);
        if (backupEc)
        {
            logUi("Update Calib", "备份旧表失败（继续写入）: " + backupEc.message(), false);
        }
        else
        {
            logUi("Update Calib", "旧表已备份: " + backup);
        }
    }

    if (!calibration->exportUnifiedCsv(primaryTarget, error,
                                       previous.loaded() ? &previous : nullptr))
    {
        logUi("Update Calib", "导出聚合表失败: " + error, false);
        return;
    }

    // 立刻回读校验：宁可这次不生效，也不要让写坏的表覆盖掉原来好用的表。
    Cs96BetaCalibration verify;
    if (!verify.load(primaryTarget, error))
    {
        logUi("Update Calib", "回读校验失败（文件已写入但未启用）: " + error, false);
        return;
    }

    // 同步到运行时默认路径（exe/calib），否则下次启动加载的还是旧表。
    bool synced = false;
    if (primaryTarget != runtimeTarget)
    {
        std::filesystem::create_directories(runtimeTarget.parent_path(), ec);
        std::error_code copyEc;
        std::filesystem::copy_file(primaryTarget, runtimeTarget,
                                   std::filesystem::copy_options::overwrite_existing, copyEc);
        synced = !copyEc;
        if (copyEc)
        {
            logUi("Update Calib",
                  "同步到 " + runtimeTarget.string() + " 失败: " + copyEc.message() +
                      "（本次会话已生效）",
                  false);
        }
    }

    betaCalibration_ = calibration;
    betaCalibrationEnabled_ = true;
    if (betaStream_)
    {
        betaStream_->setCalibration(betaCalibration_);
        betaStream_->setCalibrationEnabled(betaCalibrationEnabled_);
    }

    const auto &starts = betaCalibration_->startingAngles();
    std::ostringstream detail;
    detail << "已重算 " << primaryTarget.string() << " (" << verify.lineCount() << "x"
           << verify.slotCount() << ") start=" << starts[0] << "," << starts[1] << ","
           << starts[2] << "," << starts[3];
    if (synced)
    {
        detail << "；已同步到 " << runtimeTarget.string();
    }
    logUi("Update Calib", detail.str());
    controlState_ = "CS96 Beta calibration updated from " + directory.filename().string();
    updateHud();
    renderer_.render();
}

void ViewerApp::toggleHorizontalAngleScheme()
{
    alphaHorizontalAngle_ = !alphaHorizontalAngle_;

    if (betaStream_)
    {
        auto options = betaStream_->decodeOptions();
        options.alphaHorizontalAngle = alphaHorizontalAngle_;
        betaStream_->setDecodeOptions(options);
    }
    else if (betaCalibrationEnabled_)
    {
        // 未连接时先把选择记住，connectBeta 里会同步给流对象。
    }

    ui_.setLabel(angleSchemeButtonIndex_,
                 alphaHorizontalAngle_ ? "Angle: Alpha" : "Angle: Beta");
    logUi("Angle Scheme",
          alphaHorizontalAngle_
              ? "Alpha 方案：绝对编码(1/65536圈) − 帧中点基准(面起点+400×13.6+7)，×2 后过 spot_slope"
              : "Beta 方案：查表角度 + 0.0009×real − 面起始角，×2 + omega0 后过 spot_slope");
    updateHud();
    renderer_.render();
}

void ViewerApp::loadLiveCalibration()
{
    // 按当前型号选择标定格式，避免把 6532 的 bin 当成 Beta 的 CSV 解析。
    if (isRotatingModel(activeModel_))
    {
        logUi("Select Calib", std::string("当前为 ") + radarModelName(activeModel_) +
                                  "，选择聚合 CSV 标定表");
        loadBetaCalibration();
        return;
    }

    if (activeModel_ == RadarModel::Unknown)
    {
        logUi("Select Calib", "尚未连接雷达，按 6532 固态雷达的 bin 格式加载");
    }

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
        // 用读-改-写，避免覆盖掉其它字段（例如 LAST_DEVICE_IP）。
        ViewerSettings settings = loadViewerSettings();
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
                             ? "CS96 Beta 标定 ON：距离系数/零点 + 逐线水平角 + FPGA 旋转角 + 反射率查表"
                             : "CS96 Beta 标定 OFF：四项修正全关，保留编码器表/面起始角/俯仰角");
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

void ViewerApp::setInteracting(bool value)
{
    if (interacting_ == value)
    {
        return;
    }
    interacting_ = value;
    // 松手后立刻补一帧，否则会停在拖动开始时的那一帧数据上。
    if (!value)
    {
        applyIncomingFrame();
    }
    updateHud();
}

void ViewerApp::onRenderTick()
{
    ++renderTicks_;
    const auto now = std::chrono::steady_clock::now();
    const double elapsed = std::chrono::duration<double>(now - fpsStamp_).count();
    if (elapsed < 0.5)
    {
        return;
    }
    displayFps_ = static_cast<double>(renderTicks_) / elapsed;
    renderTicks_ = 0;
    fpsStamp_ = now;
    // 只刷新文本，不在这里主动 Render（本回调就是 Render 结束时触发的）。
    updateHud();
}

void ViewerApp::pumpLiveFrame()
{
    {
        // 只有真的有待消费帧时才进入完整处理，避免鼠标移动时反复刷新 HUD。
        std::lock_guard<std::mutex> lock(radarMutex_);
        if (!pendingRadarFrame_)
        {
            return;
        }
    }
    applyIncomingFrame();
}

void ViewerApp::applyIncomingFrame()
{
    // 拖动/缩放期间不重建点云：把渲染时间全部让给相机操作，
    // 松手时 setInteracting(false) 会补一帧。
    if (interacting_)
    {
        return;
    }

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
                if (error.find("固件不支持") != std::string::npos &&
                    requestedBetaMode_ >= 0 && requestedBetaMode_ <= 2)
                {
                    betaUnsupportedModes_ |= static_cast<std::uint8_t>(1u << requestedBetaMode_);
                    logUi("Mode", "CS96 Beta 切换失败: " + error + "（后续将跳过该模式）", false);
                }
                else
                {
                    logUi("Mode", "CS96 Beta 切换失败: " + error, false);
                }
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
    // 最近一次真正收到帧的时刻。心跳只在真的超过 2 秒没帧时才报：
    // 否则正常推流期间主循环的 idle 调用也会每 2 秒刷一条 "no new frame"，
    // 与 [fps] 交替出现，看起来像在断流。
    static auto lastRealFrameAt = std::chrono::steady_clock::now();
    if (!frame)
    {
        // 心跳：没有新帧也每 2 秒报一次，用来区分"设备没推流/解析慢"与"渲染慢"，
        // 否则只看 [fps] 的空白输出会误以为是渲染卡住了。
        if (std::getenv("CV_LIVE_FPS") != nullptr)
        {
            static auto lastBeat = std::chrono::steady_clock::now();
            const auto now = std::chrono::steady_clock::now();
            const double gap = std::chrono::duration<double>(now - lastBeat).count();
            if (gap >= 2.0 &&
                std::chrono::duration<double>(now - lastRealFrameAt).count() >= 2.0)
            {
                lastBeat = now;
                std::uint64_t beatPackets = 0;
                std::uint64_t beatFrames = 0;
                if (betaStream_)
                {
                    const auto beatStats = betaStream_->stats();
                    beatPackets = beatStats.packets;
                    beatFrames = beatStats.frames;
                }
                std::fprintf(stderr,
                             "[fps]  0.00 fps | no new frame for %.0fs "
                             "(recv pkts=%llu frames=%llu)\n",
                             gap, static_cast<unsigned long long>(beatPackets),
                             static_cast<unsigned long long>(beatFrames));
            }
        }

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
    const auto perfT0 = std::chrono::steady_clock::now();
    frame = preprocessLiveFrame(frame);
    const auto perfT1 = std::chrono::steady_clock::now();
    // 必须在清空 currentCloudId_ 之前判断，否则每帧都会误判为首帧并 resetCamera，
    // 导致相机随点云包络反复重算（表现为视角螺旋/抖动）。
    const bool firstLiveFrame = (currentCloudId_ == 0);

    currentFrameHandle_ = frame;
    // 一帧就是一个面：96×800 = 76800 点、120° FOV，远低于下面的阈值，
    // 正常不会抽稀。阈值保留给 120 线或将来更大的帧兜底；
    // 触发时按 2:1 抽稀，单帧渲染时间减半，HUD 显示 LOD: on。
    RenderRequest renderRequest;
    if (frame->metadata.totalSamples > 250000)
    {
        renderRequest.maxPoints = 200000;
    }
    currentRenderable_ = CloudRenderBridge::prepare(frame, renderRequest);
    const auto perfT2 = std::chrono::steady_clock::now();

    // 复用同一个场景对象 id。
    // 原来每帧 remove + add 会生成新 id，而渲染层是按 id 缓存整条 VTK 管线的，
    // id 一变就只能重建 vtkPoints/Mapper/Actor —— 30 万点时单帧就要几十毫秒。
    if (SceneObject *existing = (currentCloudId_ != 0) ? scene_.find(currentCloudId_) : nullptr)
    {
        existing->frame = frame;
        existing->renderable = currentRenderable_;
    }
    else
    {
        SceneObject object;
        object.name = "LIVE";
        object.kind = SceneObjectKind::Cloud;
        object.frame = frame;
        object.renderable = currentRenderable_;
        currentCloudId_ = scene_.add(0, std::move(object));
    }

    applyColor();
    const auto perfT3 = std::chrono::steady_clock::now();
    if (firstLiveFrame)
    {
        renderer_.resetCamera();
    }
    updateHud();
    renderer_.render();
    const auto perfT4 = std::chrono::steady_clock::now();

    // 帧率观测：每 2 秒吐一行，包含窗口帧率、最大帧间隔与各阶段耗时均值。
    // 只看首帧截图无法判断"持续慢"还是"首帧慢"，必须按时间窗口统计。
    // 用法：CV_LIVE_FPS=1 ./cloud_viewer
    if (std::getenv("CV_LIVE_FPS") != nullptr)
    {
        lastRealFrameAt = perfT4; // 有帧就刷新心跳基准，正常推流不再报"no new frame"
        const auto ms = [](const auto &a, const auto &b)
        {
            return std::chrono::duration<double, std::milli>(b - a).count();
        };
        static auto windowStart = std::chrono::steady_clock::now();
        static auto lastFrame = windowStart;
        static int frames = 0;
        static double sumPre = 0.0, sumPrep = 0.0, sumCol = 0.0, sumRender = 0.0;
        static double maxGap = 0.0;

        maxGap = std::max(maxGap, ms(lastFrame, perfT4));
        lastFrame = perfT4;

        ++frames;
        sumPre += ms(perfT0, perfT1);
        sumPrep += ms(perfT1, perfT2);
        sumCol += ms(perfT2, perfT3);
        sumRender += ms(perfT3, perfT4);

        const double elapsed = ms(windowStart, perfT4) / 1000.0;
        if (elapsed >= 2.0)
        {
            const double n = static_cast<double>(frames);
            std::fprintf(stderr,
                         "[fps] %5.2f fps | frames=%d/%.1fs gap_max=%.0fms | "
                         "pre=%.1f prep=%.1f color=%.1f render=%.1f ms\n",
                         n / elapsed, frames, elapsed, maxGap,
                         sumPre / n, sumPrep / n, sumCol / n, sumRender / n);
            windowStart = perfT4;
            frames = 0;
            maxGap = 0.0;
            sumPre = sumPrep = sumCol = sumRender = 0.0;
        }
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
    const int fileCount = static_cast<int>(files_.size());
    const auto frameCountOf = [this](int index)
    {
        return static_cast<int>(files_[static_cast<std::size_t>(index)].index.frames.size());
    };

    const int previousFile = static_cast<int>(currentFile_);
    const int previousFrame = static_cast<int>(currentFrame_);
    int targetFile = previousFile;
    int targetFrame = previousFrame + delta;

    // 单个 txt 往往只有 1 帧，只在文件内取模会让 Prev/Next 看起来"点了没反应"。
    // 越过边界就切到相邻文件，这样一对按钮就能连续浏览整个文件夹。
    if (targetFrame < 0)
    {
        for (int i = 1; i <= fileCount; ++i)
        {
            const int candidate = ((previousFile - i) % fileCount + fileCount) % fileCount;
            if (frameCountOf(candidate) > 0)
            {
                targetFile = candidate;
                targetFrame = frameCountOf(candidate) - 1;
                break;
            }
        }
    }
    else if (targetFrame >= frameCountOf(previousFile))
    {
        for (int i = 1; i <= fileCount; ++i)
        {
            const int candidate = (previousFile + i) % fileCount;
            if (frameCountOf(candidate) > 0)
            {
                targetFile = candidate;
                targetFrame = 0;
                break;
            }
        }
    }

    if (frameCountOf(targetFile) <= 0)
    {
        logUi(action, "当前没有可用帧", false);
        return;
    }
    if (targetFrame < 0 || targetFrame >= frameCountOf(targetFile))
    {
        targetFrame = 0;
    }

    loadFrame(static_cast<std::size_t>(targetFile), static_cast<std::size_t>(targetFrame));
    // 故意不重置相机：翻页时保持当前视角，方便对比不同文件的同一方位。
    // 点云若跑到视野外，用户按 R（Reset）手动重算即可。
    renderer_.render();

    const std::string fromName =
        files_[static_cast<std::size_t>(previousFile)].path.filename().string();
    const std::string toName =
        files_[static_cast<std::size_t>(targetFile)].path.filename().string();
    const std::string where =
        (targetFile == previousFile) ? fromName : fromName + " -> " + toName;
    logUi(action, where + " 帧 " + std::to_string(previousFrame) + " -> " +
                     std::to_string(targetFrame) + " / " +
                     std::to_string(frameCountOf(targetFile)));
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
    // 原始点云一帧就是 96×800，矩阵列即物理 slot（0~799）。
    // PointData 里记着解析时写入的真实行列，优先用它：
    // 填充列或将来改尺寸时都不会错位。
    int rowIndex = ref.matrixRow;
    int colIndex = ref.matrixCol;
    const bool originLayout =
        object->frame->metadata.algorithmId.find("origin") != std::string::npos;
    if (originLayout && point.pixel_idx < matrix.size() &&
        point.slot_idx < matrix[ref.matrixRow].size())
    {
        rowIndex = static_cast<int>(point.pixel_idx);
        colIndex = static_cast<int>(point.slot_idx);
    }

    text << "frame: " << object->frame->metadata.frameNumber
         << "  row/col: " << rowIndex << "/" << colIndex << "\n";
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
    // 渲染帧率放首行：拖动视角时能直接看到是否掉帧。
    text << "FPS: " << formatFloat(displayFps_, 1);
    if (interacting_)
    {
        text << "  [interacting, cloud paused]";
    }
    text << "\n";

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
        // 标定到底有没有生效，必须显式写出来：之前只靠控制台日志，
        // 手动换表后实际解算仍走旧表，界面上完全看不出来。
        if (isRotatingModel(activeModel_))
        {
            text << "Calib: ";
            if (betaCalibration_ && betaCalibration_->loaded())
            {
                text << (betaCalibrationEnabled_ ? "ON" : "OFF") << "  "
                     << betaCalibration_->lineCount() << "x" << betaCalibration_->slotCount()
                     << "  angle=" << (alphaHorizontalAngle_ ? "Alpha" : "Beta");
            }
            else
            {
                text << "未加载（纯几何兜底）";
            }
            text << "\n";
        }
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
    // 面按钮的标签（F1* 表示启用）；不可见时刷新也无副作用。
    updateFaceButtons();
    if (!betaStream_)
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

    // 调试用：固定初始水平角方案（CV_ALPHA_ANGLE=1 选 Alpha），便于无人值守对比截图。
    if (const char *alphaEnv = std::getenv("CV_ALPHA_ANGLE"))
    {
        alphaHorizontalAngle_ = (std::atoi(alphaEnv) != 0);
        ui_.setLabel(angleSchemeButtonIndex_,
                     alphaHorizontalAngle_ ? "Angle: Alpha" : "Angle: Beta");
    }

    setupInteraction();

    if (const char* mode=std::getenv("CV_STREAM_MODE"))
        if (std::string(mode)=="range") liveMode_=RadarOutputMode::Range;

    // 恢复上次选择的标定表与开关状态（文件已复制到软件目录 calib/）。
    {
        const ViewerSettings saved = loadViewerSettings();
        if (!saved.lastDeviceIp.empty())
        {
            lastDeviceIp_ = saved.lastDeviceIp;
            std::cout << "[cloud_viewer] 上次连接的设备: " << lastDeviceIp_ << "\n";
        }
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
    // 启动即搜索一次（广播 + 6532 探测），让“先搜索再连接”的流程开箱可用。
    // CV_NO_AUTO_SEARCH=1 可关闭，CV_FORCE_SCAN=1 则跳过广播直接扫描。
    if (std::getenv("CV_NO_AUTO_SEARCH") == nullptr)
    {
        searchDevices();
    }

    // 自动化验证：直接对指定 IP 执行一次"启动 you_see"。
    if (const char *startIp = std::getenv("CV_START_BETA_IP"))
    {
        lastDeviceIp_ = startIp;
        startBetaProcess();
    }

    if (const char *betaIp = std::getenv("CV_BETA_IP"))
    {
        connectBeta(betaIp);
        // 自动化验证：启动时直接把设备工作模式设为指定值。
        if (const char *setMode = betaStream_ ? std::getenv("CV_BETA_SET_MODE") : nullptr)
        {
            const int mode = std::atoi(setMode);
            if (mode >= 0 && mode <= 2 &&
                (betaAllowedModes_ & (1u << mode)) == 0)
            {
                logUi("Mode", "设备 mode=" + betaDeviceMode_ + " 不支持 work_mode=" +
                                  std::to_string(mode) + "，已跳过", false);
            }
            else if (mode >= 0 && mode <= 2)
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

    // 坐标轴画在世界原点，轴长固定 1m。
    // 原来用 vtkOrientationMarkerWidget 只能钉在视口角落，而且长度随视口缩放，
    // 看不出真实尺度；改成普通 actor 后长度就是 1m，可用来目测点云比例。
    auto axes = vtkSmartPointer<vtkAxesActor>::New();
    axes->SetPosition(0.0, 0.0, 0.0);
    axes->SetTotalLength(1.0, 1.0, 1.0);
    axes->SetShaftTypeToLine(); // 1m 尺度下圆柱太粗，会挡住点云
    axes->SetConeRadius(0.02);
    // 直接加到主 renderer 之后，X/Y/Z caption 的尺寸按整个视口归一化，
    // 默认会大到遮住画面（widget 里显小是因为它的 viewport 只有屏幕 15%）。
    for (vtkCaptionActor2D *cap : {axes->GetXAxisCaptionActor2D(),
                                   axes->GetYAxisCaptionActor2D(),
                                   axes->GetZAxisCaptionActor2D()})
    {
        if (cap)
        {
            cap->SetWidth(0.04);
            cap->SetHeight(0.015);
            cap->GetCaptionTextProperty()->SetFontSize(12);
        }
    }
    if (auto *vtkRen = renderer_.renderer())
    {
        vtkRen->AddActor(axes);
    }

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

        // 截图前先启一个 0 大小的 orientation widget。
        // 它的 SetEnabled(1) 会把渲染窗口切到多层渲染并完成一次完整绘制，
        // 否则本机这套 VTK 在取窗口像素时拿不到有效 buffer，两个取帧 filter
        // （vtkWindowToImageFilter / vtkRenderLargeImage）都会在 memcpy 越界。
        // viewport 取 0.001 保证它在画面上不可见。
        auto shotAxes = vtkSmartPointer<vtkAxesActor>::New();
        auto shotMarker = vtkSmartPointer<vtkOrientationMarkerWidget>::New();
        shotMarker->SetOrientationMarker(shotAxes);
        shotMarker->SetInteractor(interactor);
        shotMarker->SetViewport(0.0, 0.0, 0.001, 0.001);
        shotMarker->SetEnabled(1);

        renderer_.render();
        std::this_thread::sleep_for(std::chrono::milliseconds(800));
        renderer_.render();

        auto grabber = vtkSmartPointer<vtkWindowToImageFilter>::New();
        grabber->SetInput(renderer_.window());
        grabber->SetInputBufferTypeToRGB();
        grabber->ReadFrontBufferOn();
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
