#include "node_detail_panel.h"
#include "tools/ip_utils.h"
#include "ys_lidar_validate_type.h"
#include <algorithm>
#include <QGridLayout>
#include <QFont>
#include <QFrame>
#include <QEvent> // needed for QEvent
#include <QPropertyAnimation>
#include "component/node_manager/node_manager.h" // Needed for NodeManager signals

namespace ys_scene {

NodeDetailPanel::NodeDetailPanel(QWidget* parent)
    : lumosy::FloatingPanel(parent),
      local_ip_label_(nullptr),
      device_ip_label_(nullptr),
      sn_label_(nullptr),
      hardware_version_label_(nullptr),
      model_label_(nullptr),
      work_mode_label_(nullptr),
      data_mode_label_(nullptr),
      // echo_label_(nullptr),
      echo_mode_label_(nullptr),
      echo_type_label_(nullptr),
      coordinate_label_(nullptr),
      line_count_label_(nullptr),
      slot_count_label_(nullptr),
      frame_info_label_(nullptr),
      has_info_(false),
      manager_(nullptr),
      slide_animation_(nullptr),
      animating_(false),
      last_collapsed_state_(false)
{
    // 设置面板标题
    setTitle(tr("Node Details"));
    
    // 应用深色主题样式
    applyStylePreset(StylePreset::Dark);
    
    // 设置折叠方向（与NodeManager保持一致）
    setCollapseDirection(CollapseDirection::Left);
    
    // 不显示标题栏按钮（不需要手动控制隐藏/显示）
    setTitleBarButtonVisible(false);
    
    // 设置不可拖拽
    setDraggable(false);
    
    // 初始化内容区域
    SetupContentUI();
    
    // 初始位置和大小（高度将由NodeManager动态设置）
    setGeometry(20, 500, 260, 300);  // 宽度260，初始高度300，将被动态更新
    setMinimumWidth(260);
    setMinimumHeight(200);  // 设置最小高度
    
    // 初始隐藏
    hide();
}

NodeDetailPanel::~NodeDetailPanel()
{
    if (slide_animation_) {
        slide_animation_->stop();
        delete slide_animation_; slide_animation_ = nullptr;
    }
}

void NodeDetailPanel::SetupContentUI()
{
    // 创建网格布局用于显示信息
    QGridLayout* grid_layout = new QGridLayout();
    grid_layout->setContentsMargins(15, 10, 15, 10);
    grid_layout->setSpacing(10);
    grid_layout->setColumnStretch(0, 0);  // 标签列固定宽度
    grid_layout->setColumnStretch(1, 1);  // 内容列自动拉伸

    // 样式字符串
    QString label_style = "QLabel { color: #999999; font-size: 11px; }";
    QString value_style = "QLabel { color: white; font-size: 11px; padding: 3px; }";

    int row = 0;

    // 本地IP
    QLabel* local_ip_title = new QLabel(tr("Local IP:"), contentWidget());
    local_ip_title->setStyleSheet(label_style);
    local_ip_label_ = new QLabel("--", contentWidget());
    local_ip_label_->setStyleSheet(value_style);
    grid_layout->addWidget(local_ip_title, row, 0, Qt::AlignRight);
    grid_layout->addWidget(local_ip_label_, row, 1);
    row++;

    // 添加分隔线
    QFrame* separator1 = new QFrame(contentWidget());
    separator1->setFrameShape(QFrame::HLine);
    separator1->setStyleSheet("QFrame { background-color: rgba(255, 255, 255, 20); }");
    separator1->setMaximumHeight(1);
    grid_layout->addWidget(separator1, row, 0, 1, 2);
    row++;

    // 设备型号
    QLabel* model_title = new QLabel(tr("Model:"), contentWidget());
    model_title->setStyleSheet(label_style);
    model_label_ = new QLabel("--", contentWidget());
    model_label_->setStyleSheet(value_style);
    grid_layout->addWidget(model_title, row, 0, Qt::AlignRight);
    grid_layout->addWidget(model_label_, row, 1);
    row++;

    // 设备IP
    QLabel* device_ip_title = new QLabel(tr("Device IP:"), contentWidget());
    device_ip_title->setStyleSheet(label_style);
    device_ip_label_ = new QLabel("--", contentWidget());
    device_ip_label_->setStyleSheet(value_style);
    grid_layout->addWidget(device_ip_title, row, 0, Qt::AlignRight);
    grid_layout->addWidget(device_ip_label_, row, 1);
    row++;

    // 序列号
    QLabel* sn_title = new QLabel(tr("Serial Number:"), contentWidget());
    sn_title->setStyleSheet(label_style);
    sn_label_ = new QLabel("--", contentWidget());
    sn_label_->setStyleSheet(value_style);
    grid_layout->addWidget(sn_title, row, 0, Qt::AlignRight);
    grid_layout->addWidget(sn_label_, row, 1);
    row++;

    // 硬件版本
    QLabel* hw_title = new QLabel(tr("Hardware Version:"), contentWidget());
    hw_title->setStyleSheet(label_style);
    hardware_version_label_ = new QLabel("--", contentWidget());
    hardware_version_label_->setStyleSheet(value_style);
    grid_layout->addWidget(hw_title, row, 0, Qt::AlignRight);
    grid_layout->addWidget(hardware_version_label_, row, 1);
    row++;

    // 工作模式
    QLabel* work_mode_title = new QLabel(tr("Work Mode:"), contentWidget());
    work_mode_title->setStyleSheet(label_style);
    work_mode_label_ = new QLabel("--", contentWidget());
    work_mode_label_->setStyleSheet(value_style);
    grid_layout->addWidget(work_mode_title, row, 0, Qt::AlignRight);
    grid_layout->addWidget(work_mode_label_, row, 1);
    row++;

    // 数据模式
    QLabel* data_mode_title = new QLabel(tr("Data Mode:"), contentWidget());
    data_mode_title->setStyleSheet(label_style);
    data_mode_label_ = new QLabel("--", contentWidget());
    data_mode_label_->setStyleSheet(value_style);
    grid_layout->addWidget(data_mode_title, row, 0, Qt::AlignRight);
    grid_layout->addWidget(data_mode_label_, row, 1);
    row++;

    /*
    // 回波数
    QLabel* echo_title = new QLabel(tr("Echo Count:"), contentWidget());
    echo_title->setStyleSheet(label_style);
    echo_label_ = new QLabel("--", contentWidget());
    echo_label_->setStyleSheet(value_style);
    grid_layout->addWidget(echo_title, row, 0, Qt::AlignRight);
    grid_layout->addWidget(echo_label_, row, 1);
    row++;
    */

    // 回波模式
    QLabel* echo_mode_title = new QLabel(tr("Echo Mode:"), contentWidget());
    echo_mode_title->setStyleSheet(label_style);
    echo_mode_label_ = new QLabel("--", contentWidget());
    echo_mode_label_->setStyleSheet(value_style);
    grid_layout->addWidget(echo_mode_title, row, 0, Qt::AlignRight);
    grid_layout->addWidget(echo_mode_label_, row, 1);
    row++;

    // 回波类型
    QLabel* echo_type_title = new QLabel(tr("Echo Type:"), contentWidget());
    echo_type_title->setStyleSheet(label_style);
    echo_type_label_ = new QLabel("--", contentWidget());
    echo_type_label_->setStyleSheet(value_style);
    grid_layout->addWidget(echo_type_title, row, 0, Qt::AlignRight);
    grid_layout->addWidget(echo_type_label_, row, 1);
    row++;

    // 坐标系模式
    QLabel* coordinate_title = new QLabel(tr("Coordinate:"), contentWidget());
    coordinate_title->setStyleSheet(label_style);
    coordinate_label_ = new QLabel("--", contentWidget());
    coordinate_label_->setStyleSheet(value_style);
    grid_layout->addWidget(coordinate_title, row, 0, Qt::AlignRight);
    grid_layout->addWidget(coordinate_label_, row, 1);
    row++;

    // 线数
    QLabel* line_count_title = new QLabel(tr("Line Count:"), contentWidget());
    line_count_title->setStyleSheet(label_style);
    line_count_label_ = new QLabel("--", contentWidget());
    line_count_label_->setStyleSheet(value_style);
    grid_layout->addWidget(line_count_title, row, 0, Qt::AlignRight);
    grid_layout->addWidget(line_count_label_, row, 1);
    row++;

    // slot数量
    QLabel* slot_count_title = new QLabel(tr("Slot Count:"), contentWidget());
    slot_count_title->setStyleSheet(label_style);
    slot_count_label_ = new QLabel("--", contentWidget());
    slot_count_label_->setStyleSheet(value_style);
    grid_layout->addWidget(slot_count_title, row, 0, Qt::AlignRight);
    grid_layout->addWidget(slot_count_label_, row, 1);
    row++;

    // 帧数据组成
    QLabel* frame_info_title = new QLabel(tr("Frame Info:"), contentWidget());
    frame_info_title->setStyleSheet(label_style);
    frame_info_label_ = new QLabel("--", contentWidget());
    frame_info_label_->setStyleSheet(value_style);
    grid_layout->addWidget(frame_info_title, row, 0, Qt::AlignRight);
    grid_layout->addWidget(frame_info_label_, row, 1);
    row++;

    // 添加到内容布局
    contentLayout()->addLayout(grid_layout);
    contentLayout()->addStretch();
}

void NodeDetailPanel::AttachToManager(NodeManager* manager)
{
    if (manager_ == manager) {
        return;
    }

    if (manager_) {
        // Disconnect previous manager if re-attaching (unlikely but safe)
        disconnect(manager_, nullptr, this, nullptr);
        manager_->removeEventFilter(this);
    }

    manager_ = manager;
    if (manager_) {
        // Install event filter to track geometry changes and visibility
        manager_->installEventFilter(this);

        // Store initial collapsed state
        last_collapsed_state_ = manager_->isCollapsed();

        // Connect NodeClicked to show details
        connect(manager_, &NodeManager::NodeClicked, this, &NodeDetailPanel::SetLidarInfo);

        // Connect collapse animation start to trigger slide out only
        connect(manager_, &NodeManager::collapseAnimationStarted, this, [this]() {
            if (!manager_) return;
            // If manager was not collapsed before animation, it is starting to collapse
            if (!last_collapsed_state_) {
                // Only animate slide-out if we actually have info to show
                if (has_info_) {
                    animating_ = true;
                    expecting_expansion_ = false;
                    int anim_duration = manager_->animationDuration();
                    SlideOutToLeft(anim_duration);
                } else {
                    // ensure hidden when there is no info
                    hide();
                }
            }
            // If it's starting to expand, defer SlideIn until collapseAnimationFinished to use final position
        });

        // When manager's collapse animation finishes, react accordingly
        connect(manager_, &NodeManager::collapseAnimationFinished, this, [this]() {
            if (!manager_) return;
            bool now_collapsed = manager_->isCollapsed();
            // If manager finished expanding (now_collapsed == false) -> slide in (only if we have info)
            if (!now_collapsed) {
                if (has_info_) {
                    animating_ = true;
                    // ensure width matches manager before sliding in
                    this->setFixedWidth(manager_->width());
                    int anim_duration = manager_->animationDuration();
                    SlideInFromLeft(anim_duration);
                }
            } else {
                // finished collapsing, ensure hidden
                hide();
            }
            last_collapsed_state_ = now_collapsed;
        });

        // Also keep collapsedChanged to update state if needed
        connect(manager_, &NodeManager::collapsedChanged, this, [this](bool collapsed) {
            last_collapsed_state_ = collapsed;
        });

        // Connect ParentResized to update our height
        connect(manager_, &NodeManager::ParentResized, this, &NodeDetailPanel::UpdateAvailableHeight);

        // Initial follow
        FollowManager();
    }
}

void NodeDetailPanel::SlideOutToLeft(int duration)
{
    if (!manager_) return;
    if (slide_animation_) { slide_animation_->stop(); delete slide_animation_; slide_animation_ = nullptr; }

    // Ensure visible before animating
    if (!isVisible()) show();

    QRect start = geometry();
    // target x: move left enough to be fully off-screen
    int target_x = -width() - 20; // extra margin
    QRect end(target_x, start.y(), start.width(), start.height());

    slide_animation_ = new QPropertyAnimation(this, "geometry");
    slide_animation_->setDuration(duration > 0 ? duration : 220);
    slide_animation_->setStartValue(start);
    slide_animation_->setEndValue(end);
    slide_animation_->setEasingCurve(QEasingCurve::InCubic);
    connect(slide_animation_, &QPropertyAnimation::finished, this, [this]() {
        // After sliding out, hide to avoid focus issues
        this->hide();
        animating_ = false;
        if (slide_animation_) { delete slide_animation_; slide_animation_ = nullptr; }
    });
    slide_animation_->start();
}

void NodeDetailPanel::SlideInFromLeft(int duration)
{
    if (!manager_) return;
    if (slide_animation_) { slide_animation_->stop(); delete slide_animation_; slide_animation_ = nullptr; }

    // Compute target position based on manager
    int target_x = manager_->x();
    int target_y = manager_->y() + manager_->height() + 10;

    // start off-screen at left
    int start_x = -width() - 20;
    QRect start(start_x, target_y, width(), height());
    QRect end(target_x, target_y, width(), height());

    // Place at start before showing
    setGeometry(start);
    show();

    slide_animation_ = new QPropertyAnimation(this, "geometry");
    slide_animation_->setDuration(duration > 0 ? duration : 220);
    slide_animation_->setStartValue(start);
    slide_animation_->setEndValue(end);
    slide_animation_->setEasingCurve(QEasingCurve::OutCubic);
    connect(slide_animation_, &QPropertyAnimation::finished, this, [this]() {
        animating_ = false;
        if (slide_animation_) { delete slide_animation_; slide_animation_ = nullptr; }
    });
    slide_animation_->start();
}

void NodeDetailPanel::SetLidarInfo(const you_see::LidarInfo& info, const QString& localIp)
{
    current_info_ = info;
    current_local_ip_ = localIp;
    has_info_ = true;

    UpdateInfoDisplay();

    // 如果之前是隐藏的，现在显示出来
    if (!isVisible()) {
        show();
        // Ensure position is correct when showing
        FollowManager();
    }
}

void NodeDetailPanel::ClearInfo()
{
    has_info_ = false;
    
    local_ip_label_->setText("--");
    device_ip_label_->setText("--");
    sn_label_->setText("--");
    hardware_version_label_->setText("--");
    model_label_->setText("--");
    work_mode_label_->setText("--");
    data_mode_label_->setText("--");
    // echo_label_->setText("--");
    echo_mode_label_->setText("--");
    echo_type_label_->setText("--");
    coordinate_label_->setText("--");
    line_count_label_->setText("--");
    slot_count_label_->setText("--");
    frame_info_label_->setText("--");

    hide();
}

bool NodeDetailPanel::eventFilter(QObject* watched, QEvent* event)
{
    if (watched == manager_) {
        if ((event->type() == QEvent::Move || event->type() == QEvent::Resize) && !animating_) {
            FollowManager();
        } else if (event->type() == QEvent::Show) {
             // ignore show; animation will handle showing
        } else if (event->type() == QEvent::Hide) {
             // when manager is hidden, ensure we hide as well
             hide();
        }
    }
    return lumosy::FloatingPanel::eventFilter(watched, event);
}

void NodeDetailPanel::FollowManager()
{
    if (!manager_ || animating_) {
        return;
    }

    // 计算新位置：NodeManager的X坐标，Y坐标为NodeManager底部 + 10像素间距
    int new_x = manager_->x();
    int new_y = manager_->y() + manager_->height() + 10;
    
    move(new_x, new_y);
}

void NodeDetailPanel::UpdateInfoDisplay()
{
    if (!has_info_) {
        return;
    }

    using lumosy::IPUtils;

    // 本地IP
    local_ip_label_->setText(current_local_ip_);

    // 设备型号
    model_label_->setText(GetModelText(current_info_.model));

    // 设备IP
    QString device_ip = IPUtils::Uint32ToIPString(current_info_.ip);
    device_ip_label_->setText(device_ip);

    // 序列号
    QString sn = QString::fromLatin1(reinterpret_cast<const char*>(current_info_.serial_number),
                                     static_cast<int>(sizeof(current_info_.serial_number)))
                     .section(QLatin1Char('\0'), 0, 0)
                     .trimmed();
    if (sn.isEmpty()) {
        sn = tr("No valid SN read");
    }
    sn_label_->setText(sn);

    QString hw_version = QString::fromLatin1(reinterpret_cast<const char*>(current_info_.hardware_version),
                                             static_cast<int>(sizeof(current_info_.hardware_version)))
                             .section(QLatin1Char('\0'), 0, 0)
                             .trimmed();
    if (hw_version.isEmpty()) {
        hw_version = "--";
    } else {
        if (hw_version.startsWith('A', Qt::CaseInsensitive)) {
            hw_version += "(Alpha)";
        } else if (hw_version.startsWith('B', Qt::CaseInsensitive)) {
            hw_version += "(Beta)";
        }
    }
    hardware_version_label_->setText(hw_version);

    // 工作模式
    QString work_mode = GetWorkModeText(current_info_.node_work_mode);
    work_mode_label_->setText(work_mode);

    // 数据模式（基于工作模式）
    QString data_mode;
    LidarDataMode data_mode_enum = static_cast<LidarDataMode>(current_info_.node_work_mode);
    switch (data_mode_enum) {
    case LidarDataMode::kOriginHistogram:
        data_mode = tr("Origin Histogram");
        break;
    case LidarDataMode::kOriginPoint:
        data_mode = tr("Origin Point");
        break;
    case LidarDataMode::kCartesianPoint:
        data_mode = tr("Cartesian Point");
        break;
    case LidarDataMode::kGray:
        data_mode = tr("Gray");
        break;
    default:
        data_mode = tr("Unknown");
        break;
    }
    data_mode_label_->setText(data_mode);

    /*
    // 回波数
    if (current_info_.echo_count > 0) {
        echo_label_->setText(QString::number(current_info_.echo_count));
    } else {
        echo_label_->setText("N/A");
    }
    */

    // 回波模式/类型
    echo_mode_label_->setText(GetEchoModeText(current_info_.echo_mode));
    echo_type_label_->setText(GetEchoTypeText(current_info_.echo_type));

    // 坐标系
    coordinate_label_->setText(GetCoordinateModeText(current_info_.coordinate_mode));

    // 线数
    if (current_info_.line_count > 0) {
        line_count_label_->setText(QString::number(current_info_.line_count));
    } else {
        line_count_label_->setText("N/A");
    }

    if (current_info_.slots_per_frame > 0) {
        slot_count_label_->setText(QString::number(current_info_.slots_per_frame));
    } else {
        slot_count_label_->setText("N/A");
    }

    // 帧数据组成（格式：线数 x 每线点数）
    if (current_info_.line_count > 0 && current_info_.slots_per_frame > 0) {
        QString frame_info = QString("%1 x %2")
            .arg(current_info_.line_count)
            .arg(current_info_.slots_per_frame);
        frame_info_label_->setText(frame_info);
    } else {
        frame_info_label_->setText("N/A");
    }
}

QString NodeDetailPanel::GetWorkModeText(you_see::NodeWorkMode mode)
{
    switch (mode) {
    case you_see::NodeWorkMode::KRawHistogram:
        return tr("Raw Histogram");
    case you_see::NodeWorkMode::KOriginCloud:
        return tr("Origin Cloud");
    case you_see::NodeWorkMode::KPointcloud:
        return tr("Point Cloud");
    case you_see::NodeWorkMode::KGray:
        return tr("Gray");
    default:
        return tr("Unknown");
    }
}

QString NodeDetailPanel::GetModelText(you_see::LidarModel model)
{
    switch (model) {
    case you_see::LidarModel::KCS:
        return tr("CS(Component Scanning LiDAR)");
    case you_see::LidarModel::KMF:
        return tr("MF(Flash LiDAR)");
    default:
        return tr("Unknown");
    }
}

QString NodeDetailPanel::FormatSerialNumber(const uint8_t* sn, size_t length)
{
    if (!sn || length == 0) {
        return "--";
    }

    QString result;
    for (size_t i = 0; i < length; ++i) {
        if (sn[i] == 0) {
            break;
        }
        result += QString("%1").arg(sn[i], 2, 16, QChar('0')).toUpper();
        if (i < length - 1 && (i + 1) % 4 == 0) {
            result += "-";
        }
    }
    
    if (result.isEmpty()) {
        return "--";
    }
    
    return result;
}

QString NodeDetailPanel::GetEchoModeText(uint8_t echo_mode)
{
    switch (echo_mode) {
    case 0:
        return tr("Single");
    case 1:
        return tr("Dual");
    default:
        return tr("Unknown");
    }
}

QString NodeDetailPanel::GetEchoTypeText(uint8_t echo_type)
{
    switch (echo_type) {
    case 0:
        return tr("Strongest");
    case 1:
        return tr("Last");
    case 2:
        return tr("First");
    default:
        return tr("Unknown");
    }
}

QString NodeDetailPanel::GetCoordinateModeText(uint8_t coordinate_mode)
{
    switch (coordinate_mode) {
    case 0:
        return tr("Cartesian");
    case 1:
        return tr("Spherical");
    default:
        return tr("Unknown");
    }
}

void NodeDetailPanel::onCollapsed()
{
    // 完全隐藏（不使用动画）
    hide();
}

void NodeDetailPanel::onExpanded()
{
    // 如果有信息，则显示
    if (has_info_) {
        show();
    }
}

void NodeDetailPanel::UpdateAvailableHeight(int available_height)
{
    if (!manager_) {
        return;
    }
        
    // 计算每个面板应该占用的高度
    // available_height 是从顶部偏移(y=85)到主界面底部的可用高度
    // 两个面板之间有10px间距，所以每个面板高度 = (available_height - 10) / 2
    int panel_height = (available_height - 10) / 2;
    
    // 设置最小高度限制
    panel_height = std::max(panel_height, 200);
    
    // 更新自己的高度
    setFixedHeight(panel_height);
    
    // 更新位置以跟随manager
    FollowManager();
}

} // namespace ys_scene
