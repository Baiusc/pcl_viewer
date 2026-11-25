#ifndef YS_SCENE_NODE_MANAGER_H
#define YS_SCENE_NODE_MANAGER_H

#include "ys_lidar_type.h"
#include <lumomo_ui/base/floating_panel.h>
#include <QTreeWidget>
#include <QIcon>
#include <unordered_map>
#include <cstdint>
#include <mutex>
#include <QResizeEvent>
#include <QMoveEvent>

namespace ys_scene
{
class MatrixLidarAdapter;

enum class NodeType { 
	kYouSee, 
	kMatrixLidar 
};

struct MatrixLidarInfo
{
    std::string ip;
    int port;
    bool is_connected;
    bool offline;
    uint32_t unique_id;
};

class NodeManager : public lumosy::FloatingPanel
{
	Q_OBJECT
public:
	explicit NodeManager(QWidget* parent = nullptr);
	~NodeManager();
	
	void AddNodeList(you_see::LidarInfo& node, void* context);
	void RefreshNodeList();
	void DisconnectAllConnectedNode();
	static void NodeManagerTrampoline(you_see::LidarInfo* node, void* context);

	// 用于添加Matrix Lidar节点
    void AddMatrixLidarNode(const QString& ip, int port);
    void UpdateMatrixLidarStatus(const QString& ip, bool is_connected);

signals:
	void LidarConnectionChanged(const uint32_t& ip, bool enabled);
	void StatusPanelQueryStartSig(const uint32_t handle, bool start);
	void QueryPsCalibrationParamsSig(const uint32_t handle);
	void MatrixLidarConnectionChanged(const QString& ip, int port, bool enabled);

	// 节点连接成功时发出（沿触发：仅在上轮未连接 -> 本轮已连接时发出一次），携带最新广播信息
	void LidarConnected(const you_see::LidarInfo& node);
	
	// 节点被点击时发出信号
	void NodeClicked(const you_see::LidarInfo& info, const QString& local_ip);
	
	// 父窗口大小变化时发出信号
	void ParentResized(int available_height);

public slots:
void DisconnectNodeSlot(const uint32_t& ip, bool connected);
void OnMatrixLidarConnected(const QString &ip, bool success);
void UpdateAvailableHeight(int available_height);

public: //For Test
	QString Ip2StringT(uint32_t& ip_val);

protected:
	// 重写基类虚函数
	void onCollapsed() override;
	void onExpanded() override;
	void onTitleBarButtonClicked() override;

	// 窗口事件重写
	void resizeEvent(QResizeEvent* event) override;
	void moveEvent(QMoveEvent* event) override;

private:
	void SetupContentUI();
	void UpdateNodeManagerHeight();
	void DisconnectConnectedNode();
	void DisconnectNode(const uint32_t& ip);
	
	// 树形结构辅助方法
	void InitializeModelCategories();
	QTreeWidgetItem* GetOrCreateModelCategory(you_see::LidarModel model);
	QString GetModelName(you_see::LidarModel model);
	QTreeWidgetItem* FindNodeItem(const QString& ip_str);
	
	// 节点点击处理
	void OnTreeItemClicked(QTreeWidgetItem* item, int column);
	QString GetLocalIpForDevice(uint32_t device_ip);
	
	// AddNodeList的辅助方法
	void UpdateExistingNode(const QString& ip_str, const you_see::LidarInfo& node);
	void RemoveNodeItem(const QString& ip_str);
	void CreateNewNodeItem(const QString& ip_str, const you_see::LidarInfo& node);
	QWidget* CreateNodeItemWidget(const QString& ip_str, const you_see::LidarInfo& node);

	// 用于处理Matrix Lidar节点
	void CreateMatrixLidarNodeItem(const QString &ip, int port);
    QWidget *CreateMatrixLidarItemWidget(const QString &ip, int port);
    void UpdateMatrixLidarItem(const QString &ip, bool is_connected);
    void DisconnectMatrixLidar(const QString &ip);

private:
	QTreeWidget* tree_widget_;
	QString pending_connect_ip_;
	std::unordered_map<QString, you_see::LidarInfo> lidar_infos_;
	std::unordered_map<QString, uint64_t> node_update_revisions_;
	std::mutex lidar_mutex_;
	
	// 模型分类节点缓存
	std::unordered_map<you_see::LidarModel, QTreeWidgetItem*> model_categories_;

	// matrix_lidar相关成员
    std::unordered_map<QString, MatrixLidarInfo> matrix_lidar_infos_;
    std::mutex matrix_lidar_mutex_;
    std::shared_ptr<MatrixLidarAdapter> matrix_adapter_;
    
    // 折叠/展开图标
    QIcon collapsed_icon_;
    QIcon expanded_icon_;
};

}// namespace ys_scene

#endif // !YS_SCENE_NODE_MANAGER_H
