#ifndef YS_SCENE_NODE_DETAIL_PANEL_H
#define YS_SCENE_NODE_DETAIL_PANEL_H

#include "ys_lidar_type.h"
#include <lumomo_ui/base/floating_panel.h>
#include <QLabel>
#include <QGridLayout>
#include <QString>
#include <QPropertyAnimation>

namespace ys_scene
{
class NodeManager;

/**
 * @brief NodeDetailPanel - 节点详细信息面板
 * 
 * 显示选中的激光雷达节点的详细信息，包括：
 * - 本地PC网络适配器IP
 * - 设备IP
 * - 设备序列号(SN)
 * - 工作模式
 * - 数据模式
 * - 回波数
 * - 线数
 * - 帧数据组成
 */
class NodeDetailPanel : public lumosy::FloatingPanel
{
    Q_OBJECT

public:
    explicit NodeDetailPanel(QWidget* parent = nullptr);
    ~NodeDetailPanel() override;

    /**
     * @brief 绑定到NodeManager，自动跟随并响应点击事件
     * @param manager NodeManager指针
     */
    void AttachToManager(NodeManager* manager);

    /**
     * @brief 设置激光雷达节点详细信息
     * @param info 激光雷达信息结构体
     * @param local_ip 本地PC的IP地址
     */
    void SetLidarInfo(const you_see::LidarInfo& info, const QString& local_ip);

    /**
     * @brief 清空详细信息显示
     */
    void ClearInfo();

    // Slide animations
    void SlideOutToLeft(int duration = 220);
    void SlideInFromLeft(int duration = 220);

public slots:
    /**
     * @brief 更新可用高度时调整面板大小
     * @param available_height 父窗口的可用高度
     */
    void UpdateAvailableHeight(int available_height);

protected:
    void onCollapsed() override;
    void onExpanded() override;
    bool eventFilter(QObject* watched, QEvent* event) override;

private:
    void SetupContentUI();
    void UpdateInfoDisplay();
    void FollowManager();
    QString GetWorkModeText(you_see::NodeWorkMode mode);
    QString GetModelText(you_see::LidarModel model);
    QString FormatSerialNumber(const uint8_t* sn, size_t length);
    QString GetEchoModeText(uint8_t echo_mode);
    QString GetEchoTypeText(uint8_t echo_type);
    QString GetCoordinateModeText(uint8_t coordinate_mode);

private:
    // 信息标签
    QLabel* local_ip_label_;
    QLabel* device_ip_label_;
    QLabel* sn_label_;
    QLabel* hardware_version_label_;
    QLabel* model_label_;
    QLabel* work_mode_label_;
    QLabel* data_mode_label_;
    // QLabel* echo_label_; // Removed
    QLabel* echo_mode_label_;
    QLabel* echo_type_label_;
    QLabel* coordinate_label_;
    QLabel* line_count_label_;
    QLabel* slot_count_label_;
    QLabel* frame_info_label_;

    // 当前显示的设备信息
    you_see::LidarInfo current_info_;
    QString current_local_ip_;
    bool has_info_;
    
    // 绑定的管理器
    NodeManager* manager_;
    
    // active animation pointer
    QPropertyAnimation* slide_animation_{nullptr};
    bool animating_{false};
    bool last_collapsed_state_{false};
    bool expecting_expansion_{false};
};

} // namespace ys_scene

#endif // YS_SCENE_NODE_DETAIL_PANEL_H
