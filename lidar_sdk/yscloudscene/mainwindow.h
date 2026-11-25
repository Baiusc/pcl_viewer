#ifndef MAINWINDOW_H
#define MAINWINDOW_H

// #include "component/grid_widget.h"

#include "lumomo_ui/base/win_native_window.h"
#include "lumomo_ui/command_deck/command_deck.h"
#include "lumomo_ui/base/floating_panel.h"
#include "lumomo_ui/property_tree/property_tree_widget.h"
#include "ys_lidar_type.h"
#include "ys_lidar_validate_type.h"

#include <memory>
#include <QToolBar>
#include <QAction>
#include <QStackedWidget>  
#include <QComboBox>
#include <QDockWidget>
#include <QTranslator>
#include <QMenuBar>
#include <QLabel>

QT_BEGIN_NAMESPACE
namespace Ui {
class MainWindow;
}
QT_END_NAMESPACE

namespace lumosy {
class FloatingToolbar;
class FirmwareVersionPanel;
}

namespace you_see {

    class NodeFounder;
    class RawhistogramProcessor;
    class OriginPointProcessor; 
    class PointProcessor;
    class RobustMqttClient;
}

#include "calib_dist_types.h" // 多点距离标定表(每像素 LUT 模型)

namespace ys_scene {
    class RawHistogramParser;
    class RawHistogramParseImp;
    class RawDisplayWidget;
    class OriginPointParser;
    class PointcloudParser;
    class Fl6532PointParser;
    class Fl6532RangeParser;
    class MotorWidget;
    class PointcloudDisplay;
    class PlaybackControl;
    class ContinuousDealPoint;
    class ColorMapTypeDistanceThreshold;
    class ROISelect;
    class AppManager;
    class RecordInterval;
    class NodeManager;
    class NodeDetailPanel;
    class StatusPanel;
    class MatrixLidarOptions;
    class MatrixLidarAdapter;
    class Mf150PreviewConfig;
    class SinglePickOverlay;
    class SinglePickList;
    class RoiPickEngine;
    class SinglePickBase;
    class PickInfoPanel;
    }

using namespace ys_scene;
using namespace you_see;

class MainWindow : public lumosy::WinNativeWindow
{
    Q_OBJECT
public:
    MainWindow(QWidget *parent = nullptr);
    ~MainWindow();
    void PoincloudRenderInit();

private:
    // 顶部悬浮面板（NodeManager / FloatingToolbar / PropertyTree 等）起始 Y 坐标
    // 需要大于 command_deck_（菜单栏+工具栏）高度，避免与工具栏交叉
    static constexpr int kPanelTopMargin = 105;

    // raw histogram deal
    ys_scene::RawHistogramParser* raw_histogram_parser_;

    // origin data deal
    ys_scene::OriginPointParser* origin_point_parser_;

    // mf150_fl6532 data deal
    ys_scene::Fl6532PointParser* fl6532_point_parser_{ nullptr };

    // mf150_fl6532 RANGE 模式(原始点云)组帧
    ys_scene::Fl6532RangeParser* fl6532_range_parser_{ nullptr };

    // data point deal
    ys_scene::PointcloudParser* pointcloud_parser_;

    // mqtt client
//    std::shared_ptr<you_see::RobustMqttClient> robust_mqtt_client_;
    ys_scene::MatrixLidarAdapter* matrix_lidar_adapter_;

    // motor 
    ys_scene::MotorWidget *motor_widget_;
    ys_scene::RawDisplayWidget *ui2;
    ys_scene::PointcloudDisplay* pointcloud_display_;
    lumosy::FloatingToolbar* floating_toolbar_{ nullptr };
    ys_scene::PlaybackControl* pointcloud_replay_;

    ys_scene::ContinuousDealPoint* continuous_deal_point_;

    ys_scene::ColorMapTypeDistanceThreshold* colormap_dialog_;
    ys_scene::ROISelect* roi_select_dialog_;
    ys_scene::RecordInterval* record_interval_;
	ys_scene::NodeManager* node_manager_;
    ys_scene::NodeDetailPanel *node_detail_panel_;
	ys_scene::StatusPanel* status_panel_;
    ys_scene::MatrixLidarOptions* matrix_lidar_options_;
    //ys_scene::AppManager* app_manager_;
    lumosy::PropertyTreeWidget* property_tree_widget_;
    lumosy::FirmwareVersionPanel* firmware_version_panel_{ nullptr };
    // 当前固件版本查询的目标设备(即设备 IP)；用于刷新与应答归属校验
    uint32_t firmware_device_ip_{ 0 };

protected:
    void resizeEvent(QResizeEvent* event) override;

private:
    void CreatePropertyTree();
    void LoadLidarModelProperties(const you_see::LidarInfo& node);
    static void PropertySetCallbackTrampoline(const uint32_t handle, const PropertySetAckT* ack, void* context);
    static void PropertyGetCallbackTrampoline(const uint32_t handle, const PropertyGetAckT* ack, void* context);
    void SetupFloatingToolbar();
    void UpdateFloatingToolbarPosition();
    void CreateContinuousDealPoint();

    void UpdateNodeManagerPanelHeights();
    void CreatePlaybackControl();
    void CreateToolActions();
    void CreateViewActions();
    QIcon CreateSvgIcon(const QString& iconPath, int size);
    void SwitchLanguage(const QString &language);
    void RetranslateUi();
    void UpdateMenuText(QMenu* menu);
    void CreateRecordInterval();
	void CreateNodeManager();
    void CreateNodeStatusPanel();
    void CreateMatrixLidarOptions();
    void CreateFirmwareVersionPanel();
    // 向指定设备发起 0x0302 系统固件版本查询(device_ip 为 0 时仅提示未连接)
    void QueryFirmwareVersion(uint32_t device_ip);
    // SDK 网络线程回调：按值拷贝后切回主线程更新面板
    static void FirmwareVersionCallbackTrampoline(const uint32_t handle,
                                                  const you_see::FirmwareVersionInfoT* info,
                                                  void* context);
    //void StartDefaultApps();
    //void CreateMqttClient();
    template<typename T, typename ParentT = std::nullptr_t>
    void CreatUIElements(T*& ptr, const QString& text, const QString& translationKey, ParentT* parent = nullptr)
    {
        if (!ptr) {
            if constexpr (std::is_same_v<T, QMenuBar>) {
                // ptr = new QMenuBar(this);
                // setMenuBar(ptr);
                // CommandDeck already has a MenuBar
            }
            else if constexpr (std::is_same_v<T, QMenu>) {
                if constexpr (std::is_same_v<ParentT, QMenu>) {
                    ptr = parent->addMenu(text);
                }
                else {
                    // Use CommandDeck's MenuBar
                    if (command_deck_ && command_deck_->GetMenuBar()) {
                        ptr = command_deck_->GetMenuBar()->addMenu(text);
                    }
                }
            }
            else if constexpr (std::is_same_v<T, QToolBar>) {
                // ptr = new QToolBar(this);
                // addToolBar(ptr);
                // CommandDeck already has a ToolBar
            }
            else if constexpr (std::is_same_v<T, QAction>) {
                ptr = new QAction(text, this);
                if constexpr (std::is_same_v<ParentT, QMenu>) {
                    parent->addAction(ptr);
                }
                ptr->setText(text);
            }
            else {
                static_assert(always_false<T>::value, "Unsupported type");
            }

            if constexpr (!std::is_same_v<T, QAction> && !std::is_same_v<T, QMenu>) {
                // ptr->setWindowTitle(text);
            }

            if (ptr) {
                ptr->setProperty(translationKey.toUtf8().constData(), text);
            }
        }
    }

    // 辅助模板，用于产生编译错误
    template<typename T>
    struct always_false : std::false_type {};

private slots:
    void SavePointCloud();
    void SavePointCloudFrames(const QString& value);
    void SwitchToEnglish();
    void SwitchToChinese();
    void StausSet(const uint32_t& handle, const uint8_t& mode, bool enabled);
    // toolbar
private:
    lumosy::CommandDeck* command_deck_{ nullptr };
    QMenuBar* menubar_;
    QToolBar* toolbar_;
    QAction* pointcloud_view_;
    QAction* raw_histogram_view_;
    QAction* motor_view_;
    // save_point_cloud_action_ 已移动到 PointcloudDisplay 的 FloatingToolbar 中
    QAction* save_origin_point_action_;
    QAction* roi_select_action_;
    QAction* status_panel_action_;
    QAction* matrix_lidar_options_action_;
    QAction* firmware_version_action_{ nullptr };
    QAction* mf150_preview_action_{ nullptr };
    QAction* cs96_beta_preview_action_{ nullptr };
    bool mf150_preview_enabled_{ true };
    int mf150_preview_echo_index_{ 0 };
    bool mf150_calib_enabled_{ false };
    std::vector<std::vector<float>> mf150_calib_map_;
    bool mf150_multi_calib_enabled_{ false };
    // 多点标定(新版 LUT 模型)：每像素距离查表，跨线程以 shared_ptr 传递
    std::shared_ptr<SslDistCalibMulti> mf150_multi_table_{ nullptr };
    QComboBox* save_pointcloud_frames_;
    QStackedWidget* stacked_widget_;
    QDockWidget* playback_dock_;
    QAction* show_playback_action_;
    QAction* distance_colormap_action_;
    QAction* english_us_action_;
    QAction* chinese_simpled_action_;
    QTranslator translator_;

    // Floating toolbar actions
    QAction* float_save_point_cloud_action_{ nullptr };
    QAction* float_config_tree_action_{ nullptr };

    // 视角切换
    QAction* view_top_action_{ nullptr };
    QAction* view_bottom_action_{ nullptr };
    QAction* view_front_action_{ nullptr };
    QAction* view_back_action_{ nullptr };
    QAction* view_left_action_{ nullptr };
    QAction* view_right_action_{ nullptr };
    QAction* view_iso_front_action_{ nullptr };
    QAction* view_iso_back_action_{ nullptr };

    // 点选相关
    QAction* single_pick_overlay_action_{ nullptr };
    QAction* single_pick_list_action_{ nullptr };
    SinglePickOverlay* overlay_picker_{ nullptr };
    SinglePickList* list_picker_{ nullptr };
    RoiPickEngine* roi_picker_{ nullptr };
    SinglePickBase* previous_pick_engine_{ nullptr };
    PickInfoPanel* pick_info_panel_{ nullptr };

private:
    bool is_node_manager_dock_visible_ = true;
    void UpdateFloatingToolbarIcons(bool enable);
    void CreatePickActions();
    void onOverlayPickToggled(bool checked);
    void onListPickToggled(bool checked);
};

#endif // MAINWINDOW_H
