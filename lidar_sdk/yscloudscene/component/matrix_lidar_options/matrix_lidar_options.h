#ifndef MATRIX_LIDAR_OPTIONS_H
#define MATRIX_LIDAR_OPTIONS_H

#include <QDialog>

QT_BEGIN_NAMESPACE

namespace Ui {
class MatrixLidarOptions;
}

QT_END_NAMESPACE

namespace ys_scene {

class MatrixLidarAdapter;

struct MatrixLidarDevice
{
    std::string name;
    std::string device_ip;
    int device_port;
    std::string server_ip;
    int server_port;
};

class MatrixLidarOptions : public QDialog
{
    Q_OBJECT
  public:
    MatrixLidarOptions(QWidget *parent = nullptr);
    ~MatrixLidarOptions();

  signals:
    // 用于通知NodeManager添加Matrix Lidar节点
    void AddMatrixLidarNodeRequested(const QString &ip, int port);

  private slots:
    // 复选框状态更改处理函数
    void OnCbIpOptioinsStateChanged(int state);

    // 按钮点击事件处理函数
    void OnPbAddNodeClicked();
    void OnPbRemoveNodeClicked();
    void OnPbQueryIpClicked();
    void OnPbSetIpClicked();
    void OnDeviceSelectionChanged(int index);
  private:
    // 初始化 Matrix Lidar 配置
    void InitializeMatrixLidarConfig();
    // 获取设备数量
    int GetMatrixLidarDeviceCount();
    // 获取所有配置的设备
    std::vector<MatrixLidarDevice> GetMatrixLidarDevices();
    // 添加或更新设备
    void AddOrUpdateMatrixLidarDevice(const MatrixLidarDevice &device);
    // 删除设备
    void RemoveMatrixLidarDevice(const std::string &name);
    // 保存所有设备配置到文件
    void SaveDeviceConfigurations(const std::vector<MatrixLidarDevice> &devices);
    // 查找可连通的设备IP
    bool FindConnectableDeviceIP(std::string &connectable_ip, int &connectable_port);
    // 尝试连接到指定IP进行测试
    bool TestConnection(const std::string &ip, int port);
    void LoadDeviceConfigurations();
    void InitializeConfigFilePath();

    // 切换UI模式的函数
    void UpdateUIForAddNodeMode();
    void UpdateUIForIPSettingsMode();
    void SetWidgetOpacity(QWidget *widget, qreal opacity);

    // 切换动画效果
    void AnimateTransition(bool toIPSettingsMode);
  private:
    Ui::MatrixLidarOptions *ui;
    std::string config_file_path_;

    // 临时的MatrixLidarAdapter，用于IP查询和设置操作
    std::unique_ptr<MatrixLidarAdapter> temp_adapter_;
};

} // namespace ys_scene

#endif // MATRIX_LIDAR_OPTIONS_H