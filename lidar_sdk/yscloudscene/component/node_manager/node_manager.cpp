#include "component/switch_button/switch_button.h"
#include "business/matrix_lidar/matrix_lidar_adapter.h"
#include "tools/ip_utils.h"
#include "log/logging.h"
#include "node_manager.h"
#include "ys_lidar_validate_type.h"
#include "component/node_detail_panel/node_detail_panel.h"
#include <algorithm>
#include <iostream>
#include <QVBoxLayout>
#include <QHBoxLayout>
#include <QLabel>
#include <QTimer>
#include <QPixmap>
#include <QPainter>
#include <QFont>
#include <QTreeWidget>
#include <QNetworkInterface>

namespace ys_scene {

NodeManager::NodeManager(QWidget* parent) : 
    lumosy::FloatingPanel(parent),
    tree_widget_(nullptr),
    pending_connect_ip_(""),
    lidar_infos_(),
    lidar_mutex_(),
    model_categories_(),
    matrix_lidar_infos_(),
    matrix_lidar_mutex_(),
    matrix_adapter_(std::make_shared<MatrixLidarAdapter>())
{
    // 设置面板标题
    setTitle(tr("Node Manager"));
    
    // 应用深色主题样式
    applyStylePreset(StylePreset::Dark);
    
    // 设置折叠方向
    setCollapseDirection(CollapseDirection::Left);
    
    // 加载折叠/展开图标
    QString collapsed_icon_path = ":/lumomo_ui/resources/icons/angle-double-right.svg";
    QString expanded_icon_path = ":/lumomo_ui/resources/icons/angle-double-left.svg";
    
    collapsed_icon_ = QIcon(collapsed_icon_path);
    expanded_icon_ = QIcon(expanded_icon_path);
    
    // 如果图标加载失败，创建文本图标作为后备
    if (collapsed_icon_.isNull() || expanded_icon_.isNull()) {
        // 创建带文本的图标
        QPixmap collapsed_pixmap(24, 24);
        collapsed_pixmap.fill(Qt::transparent);
        QPainter collapsed_painter(&collapsed_pixmap);
        collapsed_painter.setPen(QColor(187, 175, 171));  // #bbafab
        collapsed_painter.setFont(QFont("Arial", 16, QFont::Bold));
        collapsed_painter.drawText(collapsed_pixmap.rect(), Qt::AlignCenter, "»");
        collapsed_icon_ = QIcon(collapsed_pixmap);
        
        QPixmap expanded_pixmap(24, 24);
        expanded_pixmap.fill(Qt::transparent);
        QPainter expanded_painter(&expanded_pixmap);
        expanded_painter.setPen(QColor(187, 175, 171));
        expanded_painter.setFont(QFont("Arial", 16, QFont::Bold));
        expanded_painter.drawText(expanded_pixmap.rect(), Qt::AlignCenter, "«");
        expanded_icon_ = QIcon(expanded_pixmap);
    }
    
    // 设置初始图标（展开状态）
    setTitleBarIcon(expanded_icon_);
    
    // 连接标题栏按钮信号
    connect(this, &FloatingPanel::titleBarButtonClicked, this, [this]() {
        toggleCollapsed(true);
    });
    
    // 调整初始位置：增加Y坐标，向 down 移动
    // Y 与 MainWindow 中 kPanelTopMargin(105) 保持一致，避免与工具栏交叉
    // 高度将由MainWindow动态设置，这里不设置固定高度
    setGeometry(20, 105, 225, 400);  // 宽度225，初始高度400，将被MainWindow覆盖
    setMinimumWidth(225);
    setMinimumHeight(200);  // 设置最小高度
    setDraggable(false);  // 固定位置，不可拖动
    
    // 初始化内容区域
    SetupContentUI();
}

NodeManager::~NodeManager()
{
}

void NodeManager::SetupContentUI()
{
    // 创建树形控件
    tree_widget_ = new QTreeWidget();
    tree_widget_->setHeaderHidden(true);
    tree_widget_->setIndentation(15);

    // 设置树形控件样式
    QString style =
        "QTreeWidget { "
        "   background-color: transparent;"
        "   border: none;"
        "}"
        "QTreeWidget::item { "
        "   background-color: transparent;"
        "   margin: 0px;"
        "   padding: 0px;"
        "}"
        "QTreeWidget::branch:has-children {"
        "   background: transparent;"
        "}"
        "QTreeWidget::branch:closed:has-children {"
        "   image: url(:/lumomo_ui/resources/icons/angle-right.svg);"
        "}"
        "QTreeWidget::branch:open:has-children {"
        "   image: url(:/lumomo_ui/resources/icons/angle-down.svg);"
        "}";

    tree_widget_->setStyleSheet(style);

    // 启用垂直滚动条，当内容超出时显示
    tree_widget_->setVerticalScrollBarPolicy(Qt::ScrollBarAsNeeded);
    tree_widget_->setHorizontalScrollBarPolicy(Qt::ScrollBarAlwaysOff);
    
    // 设置 size policy，让 tree_widget_ 可以扩展填充可用空间
    tree_widget_->setSizePolicy(QSizePolicy::Expanding, QSizePolicy::Expanding);

    // 连接树项点击信号
    connect(tree_widget_, &QTreeWidget::itemClicked, this, &NodeManager::OnTreeItemClicked);

    // 添加到内容布局，设置拉伸因子为1，让tree_widget填满空间
    contentLayout()->addWidget(tree_widget_, 1);
    
    // 初始化所有模型分类
    InitializeModelCategories();
}

void NodeManager::onCollapsed()
{
    // 折叠时切换图标
    setTitleBarIcon(collapsed_icon_);
}

void NodeManager::onExpanded()
{
    // 展开时切换图标
    setTitleBarIcon(expanded_icon_);
}

void NodeManager::onTitleBarButtonClicked()
{
    // 标题栏按钮点击时的自定义行为
    // 默认在构造函数中已连接到 toggleCollapsed
}

void NodeManager::resizeEvent(QResizeEvent* event)
{
    FloatingPanel::resizeEvent(event);
}

void NodeManager::moveEvent(QMoveEvent* event)
{
    FloatingPanel::moveEvent(event);
}

void NodeManager::OnTreeItemClicked(QTreeWidgetItem* item, int column)
{
    if (!item) {
        return;
    }

    // 检查是否是分类节点
    if (item->data(0, Qt::UserRole + 3).toBool()) {
        // 这是分类节点，不处理
        return;
    }

    // 获取IP地址
    QString ip_str = item->data(0, Qt::UserRole).toString();
    if (ip_str.isEmpty()) {
        return;
    }

    // 查找对应的设备信息
    you_see::LidarInfo info;
    bool found = false;
    
    {
        std::lock_guard<std::mutex> lock(lidar_mutex_);
        auto it = lidar_infos_.find(ip_str);
        if (it != lidar_infos_.end()) {
            info = it->second;
            found = true;
        }
    }

    if (found) {
        // 获取本地IP
        QString local_ip = GetLocalIpForDevice(info.ip);
        
        // 发出信号并更新详细面板
        emit NodeClicked(info, local_ip);
    }
}

QString NodeManager::GetLocalIpForDevice(uint32_t device_ip)
{
    using lumosy::IPUtils;
    
    // 将设备IP转换为字符串格式
    QString device_ip_str = IPUtils::Uint32ToIPString(device_ip);
    
    // 获取所有网络接口
    QList<QNetworkInterface> interfaces = QNetworkInterface::allInterfaces();
    
    for (const QNetworkInterface& interface : interfaces) {
        // 跳过未启用的接口
        if (!(interface.flags() & QNetworkInterface::IsUp) ||
            !(interface.flags() & QNetworkInterface::IsRunning)) {
            continue;
        }
        
        // 跳过回环接口
        if (interface.flags() & QNetworkInterface::IsLoopBack) {
            continue;
        }
        
        // 获取该接口的所有地址
        QList<QNetworkAddressEntry> entries = interface.addressEntries();
        for (const QNetworkAddressEntry& entry : entries) {
            QHostAddress addr = entry.ip();
            
            // 只处理IPv4地址
            if (addr.protocol() != QAbstractSocket::IPv4Protocol) {
                continue;
            }
            
            // 检查是否在同一子网
            QHostAddress subnet_mask = entry.netmask();
            QHostAddress device_addr(device_ip_str);
            
            // 简单的同子网检查
            quint32 local_network = addr.toIPv4Address() & subnet_mask.toIPv4Address();
            quint32 device_network = device_addr.toIPv4Address() & subnet_mask.toIPv4Address();
            
            if (local_network == device_network) {
                return addr.toString();
            }
        }
    }
    
    // 如果没有找到匹配的，返回第一个有效的IPv4地址
    for (const QNetworkInterface& interface : interfaces) {
        if (!(interface.flags() & QNetworkInterface::IsUp) ||
            !(interface.flags() & QNetworkInterface::IsRunning) ||
            (interface.flags() & QNetworkInterface::IsLoopBack)) {
            continue;
        }
        
        QList<QNetworkAddressEntry> entries = interface.addressEntries();
        for (const QNetworkAddressEntry& entry : entries) {
            if (entry.ip().protocol() == QAbstractSocket::IPv4Protocol) {
                return entry.ip().toString();
            }
        }
    }
    
    return tr("N/A");
}

void NodeManager::InitializeModelCategories()
{
    // 预先创建所有模型分类节点
    GetOrCreateModelCategory(you_see::LidarModel::KCS);
    GetOrCreateModelCategory(you_see::LidarModel::KMF);
    GetOrCreateModelCategory(you_see::LidarModel::KUnknown);
    
    // 初始化后更新高度
    UpdateNodeManagerHeight();
}

QTreeWidgetItem* NodeManager::GetOrCreateModelCategory(you_see::LidarModel model)
{
    auto it = model_categories_.find(model);
    if (it != model_categories_.end()) {
        return it->second;
    }

    // 创建新的分类节点
    QTreeWidgetItem* category = new QTreeWidgetItem(tree_widget_);
    QString model_name = GetModelName(model);
    category->setText(0, model_name);
    category->setExpanded(true);
    
    // 设置分类节点的字体和样式
    QFont font = category->font(0);
    font.setBold(true);
    font.setPointSize(10);
    category->setFont(0, font);
    
    // 设置分类节点文字颜色为 #b7b5b3
    QColor color("#b7b5b3");
    category->setForeground(0, QBrush(color));

    // 设置节点高度以增加间距
    category->setSizeHint(0, QSize(0, 35));
    
    // 存储分类节点类型标记
    category->setData(0, Qt::UserRole, QVariant::fromValue(static_cast<int>(model)));
    category->setData(0, Qt::UserRole + 3, true); // 标记为分类节点
    
    model_categories_[model] = category;
    tree_widget_->addTopLevelItem(category);
    
    return category;
}

QString NodeManager::GetModelName(you_see::LidarModel model)
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

QTreeWidgetItem* NodeManager::FindNodeItem(const QString& ip_str)
{
    for (int i = 0; i < tree_widget_->topLevelItemCount(); ++i) {
        QTreeWidgetItem* category = tree_widget_->topLevelItem(i);
        if (!category) continue;
        
        for (int j = 0; j < category->childCount(); ++j) {
            QTreeWidgetItem* child = category->child(j);
            if (child && child->data(0, Qt::UserRole).toString() == ip_str) {
                return child;
            }
        }
    }
    return nullptr;
}

void NodeManager::AddNodeList(you_see::LidarInfo& node, void* context)
{
    using lumosy::IPUtils;
    QString ip_str = IPUtils::Uint32ToIPString(node.ip);
    const uint32_t ip_value = node.ip;
    uint64_t revision;
    {
        std::lock_guard<std::mutex> lock(lidar_mutex_);
        revision = ++node_update_revisions_[ip_str];
    }

    QMetaObject::invokeMethod(this, [this, ip_str, ip_value, node, revision]() {
        bool was_connected = false;
        {
            std::lock_guard<std::mutex> lock(lidar_mutex_);
            // 接收器切换会连续发布清理和就绪状态，只处理最新快照。
            if (node_update_revisions_[ip_str] != revision) {
                return;
            }
            auto it = lidar_infos_.find(ip_str);
            if (it != lidar_infos_.end()) {
                was_connected = it->second.is_connected && !it->second.instruction_processor_stoped &&
                                !it->second.data_processor_stoped;
            }
            if (node.offline) {
                lidar_infos_.erase(ip_str);
            }
            else {
                lidar_infos_[ip_str] = node;
            }
        }
        if (node.offline) {
            if (was_connected) {
                emit StatusPanelQueryStartSig(node.unique_id, false);
                emit LidarConnectionChanged(ip_value, false);
            }
            RemoveNodeItem(ip_str);
            return;
        }

        // 连接成功沿：上轮未连接 -> 本轮已连接，通知上层按型号加载属性配置
        if (!was_connected && node.is_connected &&
            !node.data_processor_stoped && !node.instruction_processor_stoped) {
            emit LidarConnected(node);
        }

        // 查找现有项
        QTreeWidgetItem* existing_item = FindNodeItem(ip_str);

        // 如果项目已存在
        if (existing_item) {
            UpdateExistingNode(ip_str, node);
            return;
        }
        else {
            CreateNewNodeItem(ip_str, node);
            UpdateExistingNode(ip_str, node);
        }
    }, Qt::QueuedConnection);
}

void NodeManager::RefreshNodeList()
{

}

void NodeManager::DisconnectNodeSlot(const uint32_t& ip, bool connected)
{
    if (!connected) {
		DisconnectNode(ip);
    }
}

void NodeManager::DisconnectAllConnectedNode()
{
    DisconnectConnectedNode();
}

void NodeManager::NodeManagerTrampoline(you_see::LidarInfo* node, void* context)
{
	auto* manager = reinterpret_cast<NodeManager*>(context);
	if (manager && node) {
		manager->AddNodeList(*node, context);
	}
}

void NodeManager::AddMatrixLidarNode(const QString &ip, int port)
{
    using lumosy::IPUtils;
    MatrixLidarInfo info;
    info.ip = ip.toStdString();
    info.port = port;
    info.is_connected = false;
    info.offline = false;
    info.unique_id = IPUtils::IPStringToUint32(ip); // 使用IP地址转换为唯一ID

    {
        std::lock_guard<std::mutex> lock(matrix_lidar_mutex_);
        matrix_lidar_infos_[ip] = info;
    }

    // 创建Matrix Lidar节点项
    CreateMatrixLidarNodeItem(ip, port);
}

void NodeManager::UpdateMatrixLidarStatus(const QString &ip, bool is_connected)
{
    {
        std::lock_guard<std::mutex> lock(matrix_lidar_mutex_);
        auto it = matrix_lidar_infos_.find(ip);
        if (it != matrix_lidar_infos_.end()) {
            it->second.is_connected = is_connected;
        }
    }

    // 更新UI
    UpdateMatrixLidarItem(ip, is_connected);
}

void NodeManager::OnMatrixLidarConnected(const QString &ip, bool success)
{
    UpdateMatrixLidarStatus(ip, success);
}

QString NodeManager::Ip2StringT(uint32_t &ip_val)
{
    using lumosy::IPUtils;
    return IPUtils::Uint32ToIPString(ip_val);
}

void NodeManager::UpdateNodeManagerHeight()
{
    // 高度由 MainWindow::UpdateNodeManagerPanelHeights() 控制
    // 此函数现在只用于通知布局需要更新
    if (tree_widget_) {
        tree_widget_->update();
    }
}

void NodeManager::DisconnectConnectedNode()
{
    // 遍历所有模型分类
    for (int i = 0; i < tree_widget_->topLevelItemCount(); ++i) {
        QTreeWidgetItem* category = tree_widget_->topLevelItem(i);
        if (!category) continue;

        // 遍历分类下的所有节点
        for (int j = 0; j < category->childCount(); ++j) {
            QTreeWidgetItem* item = category->child(j);
            if (!item) continue;

            QWidget* container = tree_widget_->itemWidget(item, 0);
            if (!container) continue;

            SwitchButton* switchBtn = container->findChild<SwitchButton*>();
            if (switchBtn && switchBtn->getState() == SwitchButton::State::On) {
                // 检查是否是YouSee节点
                if (!item->data(0, Qt::UserRole + 1).isValid() ||
                    item->data(0, Qt::UserRole + 1).toInt() == static_cast<int>(NodeType::kYouSee)) {

                    QString connected_ip = item->data(0, Qt::UserRole).toString();
                    emit LidarConnectionChanged(lumosy::IPUtils::IPStringToUint32(connected_ip), false);
                }
                // 检查是否是Matrix Lidar节点
                else if (item->data(0, Qt::UserRole + 1).toInt() == static_cast<int>(NodeType::kMatrixLidar)) {
                    QString connected_ip = item->data(0, Qt::UserRole).toString();
                    int port = item->data(0, Qt::UserRole + 2).toInt();
                    emit MatrixLidarConnectionChanged(connected_ip, port, false);
                }
            }
        }
    }
}

void NodeManager::DisconnectNode(const uint32_t& ip)
{
    QString ip_str = lumosy::IPUtils::Uint32ToIPString(const_cast<uint32_t&>(ip));
    
	//检查节点是否存在且已连接
    bool node_connected = false;
    {
		std::lock_guard<std::mutex> lock(lidar_mutex_);
		auto it = lidar_infos_.find(ip_str);
		if (it != lidar_infos_.end() && it->second.is_connected) {
			node_connected = true;
		}
    }

    if (!node_connected) {
        return;
    }

	// 发送断开连接信号
	emit LidarConnectionChanged(ip, false);

	// 更新UI中的按钮状态
    QTreeWidgetItem* item = FindNodeItem(ip_str);
    if (item) {
        QWidget* container = tree_widget_->itemWidget(item, 0);
        if (container) {
            SwitchButton* switch_btn = container->findChild<SwitchButton*>();
            if (switch_btn && switch_btn->getState() == SwitchButton::State::On) {
                switch_btn->setState(SwitchButton::State::Off);
            }
        }
    }
}

void NodeManager::UpdateExistingNode(const QString& ip_str, const you_see::LidarInfo& node)
{
    QTreeWidgetItem* item = FindNodeItem(ip_str);
    if (!item) {
        return;
    }

    QWidget* container = tree_widget_->itemWidget(item, 0);
    if (!container) {
        return;
    }

    SwitchButton* switchBtn = container->findChild<SwitchButton*>();
    if (!switchBtn) {
        return;
    }

    const bool connected = node.is_connected && !node.instruction_processor_stoped && !node.data_processor_stoped;
    const bool stopped = node.instruction_processor_stoped && node.data_processor_stoped;
    const auto next_state = connected ? SwitchButton::State::On
                                     : (stopped ? SwitchButton::State::Off : SwitchButton::State::Pending);
    const auto previous_state = switchBtn->getState();
    if (switchBtn->getState() != next_state) {
        LOG_INFO("Node state update: peer={}, connected={}, command_stopped={}, data_stopped={}, switch={}->{}",
            ip_str.toStdString(), node.is_connected, node.instruction_processor_stoped, node.data_processor_stoped,
            static_cast<int>(switchBtn->getState()), static_cast<int>(next_state));
    }

    switchBtn->setState(next_state);
    emit StatusPanelQueryStartSig(node.unique_id, connected);
    if (next_state == SwitchButton::State::Off && previous_state != SwitchButton::State::Off) {
        emit LidarConnectionChanged(node.ip, false);
        // 处理待连接的IP（如果有）
        if (!pending_connect_ip_.isEmpty()) {
            const auto pending_ip = pending_connect_ip_;
            pending_connect_ip_.clear();
            emit LidarConnectionChanged(lumosy::IPUtils::IPStringToUint32(pending_ip), true);
        }
    }
    if (connected && node.node_work_mode == NodeWorkMode::KOriginCloud) {
        emit QueryPsCalibrationParamsSig(node.unique_id);
    }
}

void NodeManager::RemoveNodeItem(const QString& ip_str)
{
    QTreeWidgetItem* item = FindNodeItem(ip_str);
    if (item) {
        QTreeWidgetItem* parent = item->parent();
        if (parent) {
            parent->removeChild(item);
            delete item;
            
            // 不再删除空的分类节点，保持所有分类始终可见
            
            UpdateNodeManagerHeight();
        }
    }
}

void NodeManager::CreateNewNodeItem(const QString& ip_str, const you_see::LidarInfo& node)
{
    this->setMinimumWidth(260);

    bool had_any_node = false;
    for (int i = 0; i < tree_widget_->topLevelItemCount(); ++i) {
        QTreeWidgetItem* category = tree_widget_->topLevelItem(i);
        if (category && category->childCount() > 0) {
            had_any_node = true;
            break;
        }
    }

    // 获取或创建对应型号的分类节点
    QTreeWidgetItem* category = GetOrCreateModelCategory(node.model);

    // 创建新的节点项
    QTreeWidgetItem* new_item = new QTreeWidgetItem(category);
    new_item->setData(0, Qt::UserRole, ip_str);
    QWidget* container = CreateNodeItemWidget(ip_str, node);
    new_item->setSizeHint(0, container->sizeHint());

    tree_widget_->setItemWidget(new_item, 0, container);
    category->addChild(new_item);
    category->setExpanded(true);
    
    UpdateNodeManagerHeight();

    if (!had_any_node) {
        tree_widget_->setCurrentItem(new_item);
        OnTreeItemClicked(new_item, 0);
    }
}

QWidget* NodeManager::CreateNodeItemWidget(const QString& ip_str, const you_see::LidarInfo& node)
{
    QWidget* container = new QWidget(tree_widget_);
    QHBoxLayout* layout = new QHBoxLayout(container);
    layout->setContentsMargins(10, 5, 10, 5);
    layout->setSpacing(10);
    layout->setAlignment(Qt::AlignVCenter);

    QLabel* ip_label = new QLabel(tr("Device IP: %1").arg(ip_str), container);
    ip_label->setStyleSheet("QLabel { background-color: transparent; color: white; }");

    SwitchButton* switchBtn = new SwitchButton(container);
    switchBtn->setFixedSize(50, 25);
    switchBtn->setOnText("On");
    switchBtn->setOffText("Off");
    switchBtn->setState(SwitchButton::State::Off);

    // 连接信号
    connect(switchBtn, &SwitchButton::stateChangeRequested, this,
        [this, ip_str](bool checked) {
            QTreeWidgetItem* item = FindNodeItem(ip_str);
            if (item) {
                tree_widget_->setCurrentItem(item);
                OnTreeItemClicked(item, 0);
            }
            if (checked) {
                // 检查是否有节点已连接
                bool has_connected = false;
                for (int i = 0; i < tree_widget_->topLevelItemCount(); ++i) {
                    QTreeWidgetItem* category = tree_widget_->topLevelItem(i);
                    if (!category) continue;

                    for (int j = 0; j < category->childCount(); ++j) {
                        QTreeWidgetItem* item = category->child(j);
                        if (!item) continue;

                        QWidget* container = tree_widget_->itemWidget(item, 0);
                        if (!container) continue;

                        SwitchButton* btn = container->findChild<SwitchButton*>();
                        if (btn && btn->getState() == SwitchButton::State::On) {
                            has_connected = true;
                            break;
                        }
                    }
                    if (has_connected) break;
                }

                if (has_connected) {
                    // 如果已有节点连接，存储待连接的IP
                    pending_connect_ip_ = ip_str;
                    DisconnectConnectedNode();
                }
                else {
                    // 否则，直接连接
                    emit LidarConnectionChanged(lumosy::IPUtils::IPStringToUint32(ip_str), true);
                }
            }
            else {
                // 断开连接的逻辑
                pending_connect_ip_.clear();
                emit LidarConnectionChanged(lumosy::IPUtils::IPStringToUint32(ip_str), false);
            }
        });

    layout->addWidget(ip_label);
    layout->addWidget(switchBtn);
    container->setLayout(layout);
    container->setSizePolicy(QSizePolicy::Expanding, QSizePolicy::Fixed);
    container->setMinimumHeight(45);
    container->setAttribute(Qt::WA_Hover, true);

    // 设置样式表
    container->setStyleSheet(
        "QWidget {"
        "   background-color: rgba(255, 69, 0, 50);"
        "   color: white;"
        "   border-radius: 8px;"
        "   margin: 4px 0px;"  // Add vertical margin for spacing
        "}"
        "QWidget:hover {"
        "   background-color: rgba(255, 69, 0, 90);"
        "}"
    );

    return container;
}

void NodeManager::CreateMatrixLidarNodeItem(const QString &ip, int port)
{
    this->setMinimumWidth(260);

    bool had_any_node = false;
    for (int i = 0; i < tree_widget_->topLevelItemCount(); ++i) {
        QTreeWidgetItem* category = tree_widget_->topLevelItem(i);
        if (category && category->childCount() > 0) {
            had_any_node = true;
            break;
        }
    }

    // 检查是否已存在
    QTreeWidgetItem* existing = FindNodeItem(ip);
    if (existing) {
        return; // 已存在，不重复添加
    }

    // Matrix Lidar 归类到 Matrix 型号下
    QTreeWidgetItem* category = GetOrCreateModelCategory(you_see::LidarModel::KMF);

    QTreeWidgetItem* new_item = new QTreeWidgetItem(category);
    new_item->setData(0, Qt::UserRole, ip);
    new_item->setData(0, Qt::UserRole + 1, QVariant::fromValue(NodeType::kMatrixLidar));
    new_item->setData(0, Qt::UserRole + 2, port);

    QWidget* container = CreateMatrixLidarItemWidget(ip, port);
    new_item->setSizeHint(0, container->sizeHint());

    tree_widget_->setItemWidget(new_item, 0, container);
    category->addChild(new_item);
    category->setExpanded(true);
    
    UpdateNodeManagerHeight();

    if (!had_any_node) {
        tree_widget_->setCurrentItem(new_item);
        OnTreeItemClicked(new_item, 0);
    }
}

QWidget *NodeManager::CreateMatrixLidarItemWidget(const QString &ip, int port)
{
    QWidget *container = new QWidget(tree_widget_);
    QHBoxLayout *layout = new QHBoxLayout(container);
    layout->setContentsMargins(10, 5, 10, 5);
    layout->setSpacing(10);
    layout->setAlignment(Qt::AlignVCenter);

    QLabel *ip_label = new QLabel(tr("Device IP: %1").arg(ip), container);
    ip_label->setStyleSheet("QLabel { background-color: transparent; color: white; }");

    SwitchButton *switchBtn = new SwitchButton(container);
    switchBtn->setFixedSize(50, 25);
    switchBtn->setOnText("On");
    switchBtn->setOffText("Off");
    switchBtn->setState(SwitchButton::State::Off);

    // 连接信号
    connect(switchBtn, &SwitchButton::stateChangeRequested, this, [this, ip, port](bool checked) {
        QTreeWidgetItem* item = FindNodeItem(ip);
        if (item) {
            tree_widget_->setCurrentItem(item);
            OnTreeItemClicked(item, 0);
        }
        if (checked) {
            // 检查是否有节点已连接（包括Matrix Lidar和YouSee节点）
            bool has_connected = false;
            for (int i = 0; i < tree_widget_->topLevelItemCount(); ++i) {
                QTreeWidgetItem* category = tree_widget_->topLevelItem(i);
                if (!category) continue;

                for (int j = 0; j < category->childCount(); ++j) {
                    QTreeWidgetItem* item = category->child(j);
                    if (!item) continue;

                    QWidget* container = tree_widget_->itemWidget(item, 0);
                    if (!container) continue;

                    SwitchButton* btn = container->findChild<SwitchButton*>();
                    if (btn && btn->getState() == SwitchButton::State::On) {
                        has_connected = true;
                        break;
                    }
                }
                if (has_connected) break;
            }

            if (has_connected) {
                // 如果已有节点连接，先断开连接

                DisconnectConnectedNode();

                // 延迟一小段时间后再连接Matrix Lidar
                QTimer::singleShot(200, this,
                                   [this, ip, port]() { emit MatrixLidarConnectionChanged(ip, port, true); });
            }
            else {
                // 否则，直接连接
                emit MatrixLidarConnectionChanged(ip, port, true);
            }
        }
        else {
            // 断开连接
            emit MatrixLidarConnectionChanged(ip, port, false);
        }
    });

    layout->addWidget(ip_label);
    layout->addWidget(switchBtn);
    container->setLayout(layout);
    container->setSizePolicy(QSizePolicy::Expanding, QSizePolicy::Fixed);
    container->setMinimumHeight(45);
    container->setAttribute(Qt::WA_Hover, true);

    // 设置样式表 - 使用不同的颜色区分Matrix Lidar节点
    container->setStyleSheet("QWidget {"
                             "   background-color: rgba(75, 0, 130, 50);" // 深紫色背景，区别于YouSee节点
                             "   color: white;"
                             "   border-radius: 8px;"
                             "   margin: 4px 0px;"
                             "}"
                             "QWidget:hover {"
                             "   background-color: rgba(75, 0, 130, 90);"
                             "}");

    return container;
}

void NodeManager::UpdateMatrixLidarItem(const QString &ip, bool is_connected)
{
    QTreeWidgetItem* item = FindNodeItem(ip);
    if (!item) {
        return;
    }

    if (item->data(0, Qt::UserRole + 1).toInt() == static_cast<int>(NodeType::kMatrixLidar)) {
        QWidget* container = tree_widget_->itemWidget(item, 0);
        if (container) {
            SwitchButton* switchBtn = container->findChild<SwitchButton*>();
            if (switchBtn) {
                switchBtn->setState(is_connected ? SwitchButton::State::On : SwitchButton::State::Off);
            }
        }
    }
}

void NodeManager::DisconnectMatrixLidar(const QString &ip)
{
    {
        std::lock_guard<std::mutex> lock(matrix_lidar_mutex_);
        auto it = matrix_lidar_infos_.find(ip);
        if (it != matrix_lidar_infos_.end()) {
            it->second.is_connected = false;
        }
    }

    // 找到对应的项并更新状态
    QTreeWidgetItem* item = FindNodeItem(ip);
    if (!item) {
        return;
    }

    if (item->data(0, Qt::UserRole + 1).toInt() == static_cast<int>(NodeType::kMatrixLidar)) {
        QWidget* container = tree_widget_->itemWidget(item, 0);
        if (container) {
            SwitchButton* switchBtn = container->findChild<SwitchButton*>();
            if (switchBtn) {
                switchBtn->setState(SwitchButton::State::Off);
            }
        }
    }

    // 通知MatrixLidarAdapter断开连接
    emit MatrixLidarConnectionChanged(ip, 0, false);
}

void NodeManager::UpdateAvailableHeight(int available_height)
{
    // 计算每个面板应该占用的高度
    // available_height 是从顶部偏移(y=105)到主界面底部的可用高度
    // 两个面板之间有10px间距，所以每个面板高度 = (available_height - 10) / 2
    int panel_height = (available_height - 10) / 2;
    
    // 设置最小高度限制
    panel_height = std::max(panel_height, 200);
    
    // 更新NodeManager的高度
    
    // 如果是折叠状态，不要改变高度，因为FloatingPanel已经控制了高度
    // 如果是展开状态，才更新高度
    if (!isCollapsed()) {
        setFixedHeight(panel_height);
    } else {
        // 如果已折叠，只更新记录的展开大小，不直接设置高度，避免破坏折叠状态
        QSize current_expanded = expanded_size_;
        current_expanded.setHeight(panel_height);
        expanded_size_ = current_expanded;
        
        // 确保展开时的位置也随父窗口更新（例如左侧面板保持在左侧，但Y坐标可能需要微调？）
        // MainWindow中 geometry(20, 105, ...) 位置是固定的，所以 expanded_position_ 应该不需要频繁变动
        // 除非我们想让它跟随窗口resize移动。
        // 目前 NodeManager 固定在 (20, 105)。
    }
    
    // 考虑到尽量少改动基类，我们可以这样做：
    // 如果折叠中，我们不调用 setFixedHeight，
    // 而是等到 onExpanded 时，或者 expand 之前，更新高度。
    
    // 另一种方法：先 expand(false)，设置高度，再 collapse(false)。但这会有闪烁且不仅是不雅。

    // 观察 FloatingPanel::performCollapseAnimation(false) (即 expand)，
    // 它使用 end_rect = QRect(expanded_position_, expanded_size_);
    // 所以如果 collapsed时 resize了 window，我们需要更新 expanded_size_。
    
    // 让我们修改 FloatingPanel 增加 setExpandedSize 接口，似乎是最好的办法。
    // 但是这里我不能修改 FloatingPanel 头文件（用户没说可以改这个，或者改动太大）。
    // 不，我可以改 FloatingPanel。
    
    // 让我们先看看能否通过 NodeManager 解决。
    // NodeManager 继承 FloatingPanel。
    // FloatingPanel 保护成员 expanded_size_ 可见吗？
    // 查看 floating_panel.h

    
    // 发出信号通知NodeDetailPanel更新高度
    emit ParentResized(available_height);
}

} // namespace ys_scene
