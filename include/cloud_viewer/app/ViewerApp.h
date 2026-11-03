/**
 * @brief 应用控制器：数据源切换、帧切换、着色、选点与 UI 状态
 */
#pragma once

#include <atomic>
#include <cstdint>
#include <filesystem>
#include <future>
#include <map>
#include <memory>
#include <mutex>
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
#include "cloud_viewer/stream/BetaUdpStream.h"
#include "cloud_viewer/stream/DeviceDiscovery.h"
#include "cloud_viewer/stream/UdpRadarStream.h"

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
    enum class RadarState
    {
        Disconnected = 0,
        Waiting, // 已监听，尚未收到包
        Live,    // 已收到帧
        Error    // 超时或启动失败
    };

    ~ViewerApp();

    /// fps > 0 时启用 mock 定时推帧（模拟在线数据源）；fps = 0 为纯离线交互。
    /// screenshotPath 非空时渲染一帧并保存 PNG，随后退出（用于无头验收）。
    int run(const std::vector<std::string> &files, double fps = 0.0,
            const std::string &screenshotPath = "", bool autoConnectRadar = false);

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

    /// 在线雷达连接开关。
    void toggleRadarConnection();

    /// 主线程消费接收线程投递的最新在线帧。
    void applyIncomingFrame();

private:
    void openFilesDialog();
    void openDirectoryDialog();
    void clearScene();
    void toggleCurrentVisibility();
    void connectRadar();
    void disconnectRadar();
    void switchRadarMode();
    void switchRadarRoi();
    void toggleLiveCalibration();
    void loadLiveCalibration();
    FrameHandle preprocessLiveFrame(const FrameHandle& raw);

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
    std::size_t connectButtonIndex_ = 0;

    std::unique_ptr<UdpRadarStream> radar_;
    std::mutex radarMutex_;
    FrameHandle pendingRadarFrame_;
    bool radarConnected_ = false;

    std::atomic<RadarState> radarState_{RadarState::Disconnected};
    std::atomic<bool> autoStartAttempted_{false};
    std::uint64_t radarWaitStartMs_ = 0;
    mutable std::mutex radarMessageMutex_;
    std::string radarMessage_;
    RadarOutputMode liveMode_ = RadarOutputMode::Cloud;
    std::uint8_t liveRoi_ = 0;
    std::future<std::string> modeFuture_;
    RadarOutputMode requestedMode_ = RadarOutputMode::Cloud;
    std::string controlState_ = "Device calibration: unknown (Cloud passthrough)";
    int liveCalibration_ = 0; // 0=关闭，1=启用（TableB；Range only）。
    bool calibrationReady_ = false;
    algo_lidar::Range6532LegacyCalibrationBin liveCalibrationTable_;
    std::string liveCalibrationPath_;
    FrameHandle lastRawLiveFrame_;
    std::size_t modeButtonIndex_ = 0;
    std::size_t roiButtonIndex_ = 0;
    std::size_t calibrationButtonIndex_ = 0;
    std::size_t searchButtonIndex_ = 0;
    std::size_t firstDeviceButtonIndex_ = 0;
    std::size_t faceButtonIndex_ = 0;
    std::uint8_t betaFaceMask_ = 0x0F;

    // 局域网设备发现（监听 55000 广播）与选中的设备。
    std::unique_ptr<DeviceDiscovery> discovery_;
    std::vector<DiscoveredDevice> discoveredDevices_;
    std::string radarIpOverride_;

    // CS96 Beta 转镜雷达直收（真实点云，1266B/包）。
    std::unique_ptr<BetaUdpStream> betaStream_;
    std::shared_ptr<Cs96BetaCalibration> betaCalibration_;
    /// 原始点云是否应用标定；关闭后距离/水平角/反射率全部退化为纯几何。
    bool betaCalibrationEnabled_ = true;
    int betaWorkMode_ = 2;         // 0=直方图 1=原始点云 2=真实点云
    int requestedBetaMode_ = 2;
    bool betaModeSwitch_ = false;  // modeFuture_ 的结果用于 Beta 模式切换
    /// 该固件明确拒绝过的 work_mode 位图（bit0=mode0）；避免反复做会中断推流的切换。
    std::uint8_t betaUnsupportedModes_ = 0;
    /// 设备 /mnt/mtdblock2/mode 的内容（histogram / point_cloud / point_cloud_gray）。
    std::string betaDeviceMode_;
    /// 该 mode 下允许的 work_mode 位图（bit N 表示 work_mode N）；默认 1/2。
    std::uint8_t betaAllowedModes_ = static_cast<std::uint8_t>((1u << 1) | (1u << 2));

    void searchDevices();
    void rebuildDeviceButtons();
    void connectDiscoveredDevice(std::size_t index);
    void connectBeta(const std::string &ip);
    void connectBetaManual();
    void switchBetaMode();
    void cycleBetaFace();
    static bool applyBetaWorkMode(const std::string &ip, int mode, std::string &message);
    static std::string betaSshPassword();

    vtkSmartPointer<vtkTextActor> hudActor_;
    vtkSmartPointer<vtkTextActor> infoActor_;
    std::string infoText_;
    bool frozen_ = false;
};

} // namespace cloud_viewer
