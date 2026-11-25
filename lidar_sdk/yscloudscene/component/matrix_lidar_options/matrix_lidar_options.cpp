#include "matrix_lidar_options.h"
#include "./ui_matrix_lidar_options.h"
#include "matrix_lidar/matrix_lidar_adapter.h"
#include "config/config_service.h"
#include "log/logging.h"
#include "tiny_hut/common_toolset.h"
#include <QGraphicsOpacityEffect>
#include <QInputDialog>
#include <QMessageBox>
#include <QParallelAnimationGroup>
#include <QPropertyAnimation>
#include <QRegularExpression>

namespace ys_scene {

MatrixLidarOptions::MatrixLidarOptions(QWidget *parent)
    : QDialog(parent),
      ui(new Ui::MatrixLidarOptions)
{
    ui->setupUi(this);
    ui->w_device_ip->SetIP(QString("192.168.1.1"));
    ui->w_server_ip->SetIP(QString("192.168.1.1"));

    // 连接复选框状态改变信号
    connect(ui->cb_ip_optioins, &QCheckBox::stateChanged, this, &MatrixLidarOptions::OnCbIpOptioinsStateChanged);
    connect(ui->pb_add_node, &QPushButton::clicked, this, &MatrixLidarOptions::OnPbAddNodeClicked);
    connect(ui->pb_remove_node, &QPushButton::clicked, this, &MatrixLidarOptions::OnPbRemoveNodeClicked);
    connect(ui->pb_query_ip, &QPushButton::clicked, this, &MatrixLidarOptions::OnPbQueryIpClicked);
    connect(ui->pb_set_ip, &QPushButton::clicked, this, &MatrixLidarOptions::OnPbSetIpClicked);

    // 连接设备选择的信号
    connect(ui->cb_devices, QOverload<int>::of(&QComboBox::currentIndexChanged), this,
            &MatrixLidarOptions::OnDeviceSelectionChanged);
    InitializeConfigFilePath();

    // 初始化为添加节点模式
    UpdateUIForAddNodeMode();

    // 加载设备配置
    LoadDeviceConfigurations();
}

MatrixLidarOptions::~MatrixLidarOptions()
{
    delete ui;
}

void MatrixLidarOptions::InitializeConfigFilePath()
{
    std::string config_dir = you_see::toolset::JoinPaths(you_see::toolset::GetExecuteDirectory(), "configs");
    config_file_path_ = you_see::toolset::JoinPaths(config_dir, "matrix_lidar_config.json");
}

void MatrixLidarOptions::SaveDeviceConfigurations(const std::vector<MatrixLidarDevice> &devices)
{
    auto &config_service = you_see::ConfigService::Instance();

    config_service.SaveArrayConfig<MatrixLidarDevice>(
        config_file_path_, "devices", devices,
        [](const MatrixLidarDevice &device, you_see::ConfigService::JsonBuilder &builder) {
            builder.AddString("name", device.name);
            builder.AddString("device_ip", device.device_ip);
            builder.AddInt("device_port", device.device_port);
            builder.AddString("server_ip", device.server_ip);
            builder.AddInt("server_port", device.server_port);
        });

    LOG_INFO("Saved Matrix Lidar configuration to file: {}", config_file_path_);
}

bool MatrixLidarOptions::FindConnectableDeviceIP(std::string &connectable_ip, int &connectable_port)
{
    // 1. 首先尝试当前选中设备的配置IP
    int current_index = ui->cb_devices->currentIndex();
    std::vector<MatrixLidarDevice> devices = GetMatrixLidarDevices();

    if (current_index > 0 && current_index <= (int) devices.size()) {
        const auto &current_device = devices[current_index - 1];

        LOG_INFO("Trying current device configuration: {}:{}", current_device.device_ip, current_device.device_port);

        if (TestConnection(current_device.device_ip, current_device.device_port)) {
            connectable_ip = current_device.device_ip;
            connectable_port = current_device.device_port;
            LOG_INFO("Successfully connected using current device configuration");
            return true;
        }
    }

    // 2. 如果当前配置无法连接，尝试界面显示的IP（可能用户已经修改过）
    QString display_ip = ui->w_device_ip->GetIP();
    int display_port = ui->le_device_port->text().toInt();

    if (!display_ip.isEmpty() && display_port > 0) {
        LOG_INFO("Trying displayed IP configuration: {}:{}", display_ip.toStdString(), display_port);

        if (TestConnection(display_ip.toStdString(), display_port)) {
            connectable_ip = display_ip.toStdString();
            connectable_port = display_port;
            LOG_INFO("Successfully connected using displayed IP configuration");
            return true;
        }
    }

    // 3. 如果都无法连接，提示用户输入当前可连通的IP
    LOG_WARN("Cannot connect using saved configurations, requesting user input");

    QMessageBox msgBox(this);
    msgBox.setWindowTitle(tr("Connection Required"));
    msgBox.setText(tr("Cannot connect to the device using saved configurations.\n"
                      "Please provide the current connectable IP address to proceed with IP setting."));
    msgBox.setIcon(QMessageBox::Information);
    msgBox.setStandardButtons(QMessageBox::Ok | QMessageBox::Cancel);

    if (msgBox.exec() != QMessageBox::Ok) {
        return false;
    }

    // 弹出输入对话框
    bool ok;
    QString input_ip =
        QInputDialog::getText(this, tr("Current Device IP"), tr("Enter the current connectable IP address:"),
                              QLineEdit::Normal, "192.168.137.200", &ok);

    if (!ok || input_ip.isEmpty()) {
        return false;
    }

    // 验证用户输入的IP
    int input_port = 6001; // 默认端口，也可以让用户输入
    if (TestConnection(input_ip.toStdString(), input_port)) {
        connectable_ip = input_ip.toStdString();
        connectable_port = input_port;
        LOG_INFO("Successfully connected using user-provided IP: {}:{}", connectable_ip, connectable_port);
        return true;
    }

    QMessageBox::warning(this, tr("Connection Failed"),
                         tr("Cannot connect to the provided IP address: %1").arg(input_ip));
    return false;
}

void MatrixLidarOptions::OnCbIpOptioinsStateChanged(int state)
{
    if (state == Qt::Checked) {
        // 切换到IP设置模式
        AnimateTransition(true);
    }
    else {
        // 切换到添加节点模式
        AnimateTransition(false);
    }
}

void MatrixLidarOptions::OnPbAddNodeClicked()
{
    // 获取用户输入的IP和端口
    QString device_ip = ui->w_device_ip->GetIP();
    QString port_text = ui->le_device_port->text();
    int device_port = port_text.toInt();

    if (device_ip.isEmpty() || port_text.isEmpty()) {
        QMessageBox::warning(this, tr("Input Error"), tr("Please enter both device IP and port"));
        return;
    }

    // 验证端口
    if (device_port <= 0 || device_port > 65535) {
        QMessageBox::warning(this, tr("Input Error"), tr("Port must be between 1 and 65535"));
        return;
    }

    // 如果是新设备并且用户选择了保存，则保存设备
    int currentIndex = ui->cb_devices->currentIndex();
    if (currentIndex == 0) {
        bool save = QMessageBox::question(this, tr("Save Device"), tr("Do you want to save this device configuration?"),
                                          QMessageBox::Yes | QMessageBox::No) == QMessageBox::Yes;
        if (save) {
            bool ok;
            QString name = QInputDialog::getText(this, tr("Device Name"), tr("Enter a name for this device:"),
                                                 QLineEdit::Normal, tr("New Device"), &ok);
            if (ok && !name.isEmpty()) {
                // 保存设备配置
                MatrixLidarDevice device;
                device.name = name.toStdString();
                device.device_ip = device_ip.toStdString();
                device.device_port = device_port;
                device.server_ip = ui->w_server_ip->GetIP().toStdString();
                device.server_port = ui->le_server_port->text().toInt();

                AddOrUpdateMatrixLidarDevice(device);

                // 重新加载设备列表并选择新添加的设备
                LoadDeviceConfigurations();

                // 查找并选择刚刚添加的设备
                for (int i = 0; i < ui->cb_devices->count(); i++) {
                    if (ui->cb_devices->itemText(i) == name) {
                        ui->cb_devices->setCurrentIndex(i);
                        break;
                    }
                }
            }
        }
    }
    else {
        // 更新现有设备配置
        std::vector<MatrixLidarDevice> devices = GetMatrixLidarDevices();
        if (currentIndex > 0 && currentIndex <= (int) devices.size()) {
            // 提示用户是否要更新设备配置
            bool update =
                QMessageBox::question(this, tr("Update Device"), tr("Do you want to update this device configuration?"),
                                      QMessageBox::Yes | QMessageBox::No) == QMessageBox::Yes;
            if (update) {
                MatrixLidarDevice device = devices[currentIndex - 1];
                device.device_ip = device_ip.toStdString();
                device.device_port = device_port;
                device.server_ip = ui->w_server_ip->GetIP().toStdString();
                device.server_port = ui->le_server_port->text().toInt();

                AddOrUpdateMatrixLidarDevice(device);

                // 重新加载设备列表并保持当前选择
                LoadDeviceConfigurations();
            }
        }
    }

    // 发射信号，请求添加节点
    emit AddMatrixLidarNodeRequested(device_ip, device_port);

    // 成功添加节点后关闭对话框
    QMessageBox::information(this, tr("Success"), tr("Matrix Lidar node added successfully"));
    accept();
}

void MatrixLidarOptions::OnPbRemoveNodeClicked()
{
    int currentIndex = ui->cb_devices->currentIndex();
    std::vector<MatrixLidarDevice> devices = GetMatrixLidarDevices();

    if (currentIndex <= 0 || currentIndex > (int) devices.size()) {
        QMessageBox::warning(this, tr("Error"), tr("Please select a device to remove"));
        return;
    }

    // 确认删除
    QString deviceName = ui->cb_devices->currentText();
    bool confirm = QMessageBox::question(this, tr("Confirm Remove"),
                                         tr("Are you sure you want to remove device \"%1\"?").arg(deviceName),
                                         QMessageBox::Yes | QMessageBox::No) == QMessageBox::Yes;

    if (confirm) {
        // 删除设备配置
        RemoveMatrixLidarDevice(devices[currentIndex - 1].name);

        // 重新加载设备列表
        LoadDeviceConfigurations();

        QMessageBox::information(this, tr("Success"), tr("Device removed successfully"));
    }
}

void MatrixLidarOptions::OnPbQueryIpClicked()
{
    // 获取设备IP和端口
    QString device_ip = ui->w_device_ip->GetIP();
    QString device_port_text = ui->le_device_port->text();
    int device_port = device_port_text.toInt();

    if (device_ip.isEmpty() || device_port_text.isEmpty()) {
        QMessageBox::warning(this, tr("Input Error"), tr("Please enter device IP and port"));
        return;
    }

    // 验证端口
    if (device_port <= 0 || device_port > 65535) {
        QMessageBox::warning(this, tr("Input Error"), tr("Port must be between 1 and 65535"));
        return;
    }

    // 创建临时的MatrixLidarAdapter用于查询操作
    temp_adapter_ = std::make_unique<MatrixLidarAdapter>(MatrixLidarAdapter::TCP_ONLY_INIT);

    QApplication::setOverrideCursor(Qt::WaitCursor);

    // 使用TCP连接方法
    if (!temp_adapter_->ConnectTcp(device_ip.toStdString(), device_port)) {
        QApplication::restoreOverrideCursor();
        QMessageBox::warning(this, tr("Connection Error"),
                             tr("Failed to connect to Matrix LiDAR at %1:%2").arg(device_ip).arg(device_port));
        temp_adapter_.reset(); // 确保资源释放
        return;
    }

    // 查询IP设置
    std::string queried_device_ip, queried_host_ip;
    int queried_device_port, queried_host_port;

    bool success =
        temp_adapter_->QueryIpPort(queried_device_ip, queried_device_port, queried_host_ip, queried_host_port);

    // 断开TCP连接并释放资源
    temp_adapter_->DisconnectTcp();
    temp_adapter_.reset();

    QApplication::restoreOverrideCursor();

    if (success) {
        // 更新UI显示查询结果
        ui->w_device_ip->SetIP(QString::fromStdString(queried_device_ip));
        ui->le_device_port->setText(QString::number(queried_device_port));
        ui->w_server_ip->SetIP(QString::fromStdString(queried_host_ip));
        ui->le_server_port->setText(QString::number(queried_host_port));

        QMessageBox::information(this, tr("Success"),
                                 tr("Successfully queried Matrix LiDAR IP settings:\nDevice: %1:%2\nHost: %3:%4")
                                     .arg(QString::fromStdString(queried_device_ip))
                                     .arg(queried_device_port)
                                     .arg(QString::fromStdString(queried_host_ip))
                                     .arg(queried_host_port));
    }
    else {
        QMessageBox::warning(this, tr("Query Error"), tr("Failed to query IP settings from Matrix LiDAR"));
    }
}

void MatrixLidarOptions::OnPbSetIpClicked()
{
    // 获取目标IP和端口设置
    QString target_device_ip = ui->w_device_ip->GetIP();
    QString target_device_port_text = ui->le_device_port->text();
    int target_device_port = target_device_port_text.toInt();

    QString target_server_ip = ui->w_server_ip->GetIP();
    QString target_server_port_text = ui->le_server_port->text();
    int target_server_port = target_server_port_text.toInt();

    // 验证输入
    if (target_device_ip.isEmpty() || target_device_port_text.isEmpty() || target_server_ip.isEmpty() ||
        target_server_port_text.isEmpty()) {
        QMessageBox::warning(this, tr("Input Error"), tr("Please enter all IP and port information"));
        return;
    }

    if (target_device_port <= 0 || target_device_port > 65535 || target_server_port <= 0 ||
        target_server_port > 65535) {
        QMessageBox::warning(this, tr("Input Error"), tr("Port must be between 1 and 65535"));
        return;
    }

    // 确认设置操作
    int result = QMessageBox::question(this, tr("Confirm Settings"),
                                       tr("Are you sure you want to set the following IP configuration?\n\n"
                                          "Device: %1:%2\nHost: %3:%4\n\n"
                                          "Note: The system will automatically find the current connectable IP.")
                                           .arg(target_device_ip)
                                           .arg(target_device_port)
                                           .arg(target_server_ip)
                                           .arg(target_server_port),
                                       QMessageBox::Yes | QMessageBox::No);

    if (result != QMessageBox::Yes) {
        return;
    }

    QApplication::setOverrideCursor(Qt::WaitCursor);

    // 智能查找当前可连通的IP
    std::string connectable_ip;
    int connectable_port;

    if (!FindConnectableDeviceIP(connectable_ip, connectable_port)) {
        QApplication::restoreOverrideCursor();
        QMessageBox::warning(this, tr("Connection Error"), tr("Cannot find a connectable IP address for the device."));
        return;
    }

    LOG_INFO("Using connectable IP {}:{} to set target configuration: {}:{}", connectable_ip, connectable_port,
             target_device_ip.toStdString(), target_device_port);

    // 使用找到的可连通IP创建连接
    temp_adapter_ = std::make_unique<MatrixLidarAdapter>(MatrixLidarAdapter::TCP_ONLY_INIT);

    if (!temp_adapter_->ConnectTcp(connectable_ip, connectable_port)) {
        QApplication::restoreOverrideCursor();
        QMessageBox::warning(this, tr("Connection Error"), tr("Failed to establish connection to the device."));
        temp_adapter_.reset();
        return;
    }

    // 设置目标IP配置
    bool success = temp_adapter_->SetIpPort(target_device_ip.toStdString(), target_device_port,
                                            target_server_ip.toStdString(), target_server_port);

    // 断开连接并清理资源
    temp_adapter_->DisconnectTcp();
    temp_adapter_.reset();

    QApplication::restoreOverrideCursor();

    if (success) {
        QMessageBox::information(this, tr("Success"),
                                 tr("Successfully set Matrix LiDAR IP configuration.\n\n"
                                    "The device IP has been changed from %1:%2 to %3:%4\n"
                                    "You can now connect using the new IP address.")
                                     .arg(QString::fromStdString(connectable_ip))
                                     .arg(connectable_port)
                                     .arg(target_device_ip)
                                     .arg(target_device_port));

        // 自动更新界面显示为新的IP配置
        ui->w_device_ip->SetIP(target_device_ip);
        ui->le_device_port->setText(target_device_port_text);

        // 询问是否保存新配置
        bool save =
            QMessageBox::question(this, tr("Save Configuration"), tr("Do you want to save this new configuration?"),
                                  QMessageBox::Yes | QMessageBox::No) == QMessageBox::Yes;

        if (save) {
            int currentIndex = ui->cb_devices->currentIndex();
            MatrixLidarDevice device;

            if (currentIndex > 0) {
                std::vector<MatrixLidarDevice> devices = GetMatrixLidarDevices();
                if (currentIndex <= (int) devices.size()) {
                    device = devices[currentIndex - 1];
                }
            }
            else {
                bool ok;
                QString name =
                    QInputDialog::getText(this, tr("Device Name"), tr("Enter a name for this device configuration:"),
                                          QLineEdit::Normal, tr("Updated Device"), &ok);
                if (!ok || name.isEmpty()) {
                    return;
                }
                device.name = name.toStdString();
            }

            // 更新设备配置为新的IP信息
            device.device_ip = target_device_ip.toStdString();
            device.device_port = target_device_port;
            device.server_ip = target_server_ip.toStdString();
            device.server_port = target_server_port;

            AddOrUpdateMatrixLidarDevice(device);
            LoadDeviceConfigurations();
        }
    }
    else {
        QMessageBox::warning(this, tr("Setting Error"), tr("Failed to set IP configuration on Matrix LiDAR"));
    }
}

void MatrixLidarOptions::OnDeviceSelectionChanged(int index)
{
    // 获取所有设备
    std::vector<MatrixLidarDevice> devices = GetMatrixLidarDevices();

    if (index == 0) {
        // 新设备，清空输入框
        ui->w_device_ip->SetIP("192.168.137.200");
        ui->le_device_port->setText("6001");
        ui->w_server_ip->SetIP("192.168.137.100");
        ui->le_server_port->setText("6002");
    }
    else if (index > 0 && index <= (int) devices.size()) {
        // 选择现有设备
        const auto &device = devices[index - 1];
        ui->w_device_ip->SetIP(QString::fromStdString(device.device_ip));
        ui->le_device_port->setText(QString::number(device.device_port));
        ui->w_server_ip->SetIP(QString::fromStdString(device.server_ip));
        ui->le_server_port->setText(QString::number(device.server_port));
    }
}

void MatrixLidarOptions::InitializeMatrixLidarConfig()
{
    static bool initialized = false;
    if (initialized)
        return;

    // 检查配置文件是否存在
    if (!you_see::toolset::FileExists(config_file_path_)) {
        // 确保目录存在
        std::string execute_directory = you_see::toolset::GetExecuteDirectory();
        you_see::toolset::DirectoryInit("configs", execute_directory);

        // 创建默认设备
        std::vector<MatrixLidarDevice> defaultDevices = {
            {"default device", "192.168.137.200", 6001, "192.168.137.100", 6002}};

        // 使用封装的方法保存配置
        SaveDeviceConfigurations(defaultDevices);
    }
    else {
        // 加载配置文件
        you_see::ConfigService::Instance().LoadConfig(config_file_path_);
        LOG_INFO("Matrix Lidar configuration loaded");
    }

    initialized = true;
}

int MatrixLidarOptions::GetMatrixLidarDeviceCount()
{
    InitializeMatrixLidarConfig();

    auto &config_service = you_see::ConfigService::Instance();

    try {
        // 获取设备数组大小
        // 首先检查devices键是否存在
        if (!config_service.HasKey("devices")) {
            return 0;
        }

        // 获取devices数组的大小
        int count = config_service.GetArraySize("devices");
        return count;
    }
    catch (const std::exception &e) {
        LOG_ERROR("Error getting Matrix Lidar device count: {}", e.what());
        return 0;
    }
}

std::vector<MatrixLidarDevice> MatrixLidarOptions::GetMatrixLidarDevices()
{
    InitializeMatrixLidarConfig();

    std::vector<MatrixLidarDevice> devices;
    auto &config_service = you_see::ConfigService::Instance();

    try {
        // 首先检查 devices 键是否存在
        if (!config_service.HasKey("devices")) {
            LOG_WARN("No devices found in configuration");
            return devices;
        }

        // 获取 devices 数组
        std::vector<you_see::ConfigService::JsonObject> devicesArray = config_service.GetArray("devices");

        // 遍历数组，解析每个设备
        for (const auto &deviceObj : devicesArray) {
            MatrixLidarDevice device;

            device.name = deviceObj.GetString("name", "unnamed device");
            device.device_ip = deviceObj.GetString("device_ip", "192.168.137.200");
            device.device_port = deviceObj.GetInt("device_port", 6001);
            device.server_ip = deviceObj.GetString("server_ip", "192.168.137.100");
            device.server_port = deviceObj.GetInt("server_port", 6002);

            devices.push_back(device);
        }
    }
    catch (const std::exception &e) {
        LOG_ERROR("Error loading Matrix Lidar devices: {}", e.what());
    }

    return devices;
}

void MatrixLidarOptions::AddOrUpdateMatrixLidarDevice(const MatrixLidarDevice &device)
{
    // 确保配置已初始化
    InitializeMatrixLidarConfig();

    auto devices = GetMatrixLidarDevices();

    // 查找是否已存在同名设备
    bool found = false;
    for (size_t i = 0; i < devices.size(); i++) {
        if (devices[i].name == device.name) {
            // 更新设备
            devices[i] = device;
            found = true;
            break;
        }
    }

    // 如果不存在则添加
    if (!found) {
        devices.push_back(device);
    }

    // 保存配置
    SaveDeviceConfigurations(devices);
}

void MatrixLidarOptions::RemoveMatrixLidarDevice(const std::string &name)
{
    // 确保配置已初始化
    InitializeMatrixLidarConfig();

    auto devices = GetMatrixLidarDevices();

    // 移除同名设备
    std::vector<MatrixLidarDevice> new_devices;
    for (const auto &device : devices) {
        if (device.name != name) {
            new_devices.push_back(device);
        }
    }

    // 保存新的设备列表
    SaveDeviceConfigurations(new_devices);
}

bool MatrixLidarOptions::TestConnection(const std::string &ip, int port)
{
    // 创建临时适配器进行连接测试
    auto test_adapter = std::make_unique<MatrixLidarAdapter>(MatrixLidarAdapter::TCP_ONLY_INIT);

    // 设置较短的超时时间，避免用户等待过久
    bool connected = test_adapter->ConnectTcp(ip, port);

    if (connected) {
        test_adapter->DisconnectTcp();
    }

    return connected;
}

void MatrixLidarOptions::LoadDeviceConfigurations()
{
    // 初始化配置
    InitializeMatrixLidarConfig();

    // 获取所有设备
    std::vector<MatrixLidarDevice> devices = GetMatrixLidarDevices();

    // 保存当前选中的设备名称
    QString currentDeviceName;
    int currentIndex = ui->cb_devices->currentIndex();
    if (currentIndex > 0 && currentIndex <= (int) devices.size()) {
        currentDeviceName = QString::fromStdString(devices[currentIndex - 1].name);
    }

    // 清空下拉框
    ui->cb_devices->clear();

    // 添加一个新设备选项
    ui->cb_devices->addItem(tr("-- New Device --"));

    // 添加现有设备
    int selectedIndex = 0;
    for (size_t i = 0; i < devices.size(); i++) {
        const auto &device = devices[i];
        ui->cb_devices->addItem(QString::fromStdString(device.name));

        // 如果与之前选中的设备名称相同，记录索引
        if (!currentDeviceName.isEmpty() && currentDeviceName == QString::fromStdString(device.name)) {
            selectedIndex = i + 1;
        }
    }

    // 如果有设备，选择之前选择的设备或第一个设备
    if (!devices.empty()) {
        ui->cb_devices->setCurrentIndex(selectedIndex > 0 ? selectedIndex : 1);
    }
}

void MatrixLidarOptions::UpdateUIForAddNodeMode()
{
    // 只显示必要的组件，隐藏不需要的组件
    ui->lb_server_ip->setVisible(false);
    ui->w_server_ip->setVisible(false);
    ui->lb_server_port->setVisible(false);
    ui->le_server_port->setVisible(false);

    // 启用添加节点相关按钮，禁用IP设置相关按钮
    ui->pb_add_node->setVisible(true);
    ui->pb_remove_node->setVisible(true);
    ui->pb_query_ip->setVisible(false);
    ui->pb_set_ip->setVisible(false);
    // 优化按钮布局
    ui->horizontalLayout->removeWidget(ui->pb_query_ip);
    ui->horizontalLayout->removeWidget(ui->pb_set_ip);

    // 重新安排按钮顺序
    if (ui->horizontalLayout->indexOf(ui->pb_add_node) == -1) {
        ui->horizontalLayout->addWidget(ui->pb_add_node);
    }
    ui->horizontalLayout->addWidget(ui->pb_remove_node);
    // 调整窗口大小以适应内容
    adjustSize();
}

void MatrixLidarOptions::UpdateUIForIPSettingsMode()
{
    // 显示所有组件
    ui->lb_server_ip->setVisible(true);
    ui->w_server_ip->setVisible(true);
    ui->lb_server_port->setVisible(true);
    ui->le_server_port->setVisible(true);

    // 启用IP设置相关按钮，禁用添加节点相关按钮
    ui->pb_add_node->setVisible(false);
    ui->pb_query_ip->setVisible(true);
    ui->pb_set_ip->setVisible(true);
    ui->pb_remove_node->setVisible(false);
    // 优化按钮布局
    ui->horizontalLayout->removeWidget(ui->pb_add_node);

    // 重新安排按钮顺序
    if (ui->horizontalLayout->indexOf(ui->pb_query_ip) == -1) {
        ui->horizontalLayout->addWidget(ui->pb_query_ip);
    }
    if (ui->horizontalLayout->indexOf(ui->pb_set_ip) == -1) {
        ui->horizontalLayout->addWidget(ui->pb_set_ip);
    }
    // 调整窗口大小以适应内容
    adjustSize();
}

void MatrixLidarOptions::SetWidgetOpacity(QWidget *widget, qreal opacity)
{
    QGraphicsOpacityEffect *effect = new QGraphicsOpacityEffect(widget);
    effect->setOpacity(opacity);
    widget->setGraphicsEffect(effect);

    // 存储效果，以便在后续的动画中使用
    widget->setProperty("opacityEffect", QVariant::fromValue(static_cast<void *>(effect)));
}

void MatrixLidarOptions::AnimateTransition(bool toIPSettingsMode)
{
    // 创建一个动画组，同时处理多个动画
    QParallelAnimationGroup *animGroup = new QParallelAnimationGroup(this);

    if (toIPSettingsMode) {
        // 先显示需要动画的组件，但设置为0高度
        ui->lb_server_ip->setVisible(true);
        ui->w_server_ip->setVisible(true);
        ui->lb_server_port->setVisible(true);
        ui->le_server_port->setVisible(true);

        ui->lb_server_ip->setMaximumHeight(0);
        ui->w_server_ip->setMaximumHeight(0);
        ui->lb_server_port->setMaximumHeight(0);
        ui->le_server_port->setMaximumHeight(0);

        // 创建高度动画
        QPropertyAnimation *heightAnim1 = new QPropertyAnimation(ui->lb_server_ip, "maximumHeight");
        heightAnim1->setDuration(300);
        heightAnim1->setStartValue(0);
        heightAnim1->setEndValue(ui->lb_device_ip->height());
        animGroup->addAnimation(heightAnim1);

        QPropertyAnimation *heightAnim2 = new QPropertyAnimation(ui->w_server_ip, "maximumHeight");
        heightAnim2->setDuration(300);
        heightAnim2->setStartValue(0);
        heightAnim2->setEndValue(ui->w_device_ip->height());
        animGroup->addAnimation(heightAnim2);

        QPropertyAnimation *heightAnim3 = new QPropertyAnimation(ui->lb_server_port, "maximumHeight");
        heightAnim3->setDuration(300);
        heightAnim3->setStartValue(0);
        heightAnim3->setEndValue(ui->lb_device_port->height());
        animGroup->addAnimation(heightAnim3);

        QPropertyAnimation *heightAnim4 = new QPropertyAnimation(ui->le_server_port, "maximumHeight");
        heightAnim4->setDuration(300);
        heightAnim4->setStartValue(0);
        heightAnim4->setEndValue(ui->le_device_port->height());
        animGroup->addAnimation(heightAnim4);

        // 按钮切换的不透明度动画
        ui->pb_query_ip->setVisible(true);
        ui->pb_set_ip->setVisible(true);

        // 设置初始不透明度
        SetWidgetOpacity(ui->pb_query_ip, 0.0);
        SetWidgetOpacity(ui->pb_set_ip, 0.0);
        SetWidgetOpacity(ui->pb_add_node, 1.0);

        // 添加节点按钮的淡出动画
        QGraphicsOpacityEffect *addNodeEffect =
            static_cast<QGraphicsOpacityEffect *>(ui->pb_add_node->property("opacityEffect").value<void *>());

        QPropertyAnimation *addNodeAnim = new QPropertyAnimation(addNodeEffect, "opacity");
        addNodeAnim->setDuration(300);
        addNodeAnim->setStartValue(1.0);
        addNodeAnim->setEndValue(0.0);
        animGroup->addAnimation(addNodeAnim);

        // 查询IP按钮的淡入动画
        QGraphicsOpacityEffect *queryIpEffect =
            static_cast<QGraphicsOpacityEffect *>(ui->pb_query_ip->property("opacityEffect").value<void *>());

        QPropertyAnimation *queryIpAnim = new QPropertyAnimation(queryIpEffect, "opacity");
        queryIpAnim->setDuration(300);
        queryIpAnim->setStartValue(0.0);
        queryIpAnim->setEndValue(1.0);
        animGroup->addAnimation(queryIpAnim);

        // 设置IP按钮的淡入动画
        QGraphicsOpacityEffect *setIpEffect =
            static_cast<QGraphicsOpacityEffect *>(ui->pb_set_ip->property("opacityEffect").value<void *>());

        QPropertyAnimation *setIpAnim = new QPropertyAnimation(setIpEffect, "opacity");
        setIpAnim->setDuration(300);
        setIpAnim->setStartValue(0.0);
        setIpAnim->setEndValue(1.0);
        animGroup->addAnimation(setIpAnim);

        // 动画完成后更新UI
        connect(animGroup, &QParallelAnimationGroup::finished, [this]() { UpdateUIForIPSettingsMode(); });
    }
    else {
        // 创建高度动画
        QPropertyAnimation *heightAnim1 = new QPropertyAnimation(ui->lb_server_ip, "maximumHeight");
        heightAnim1->setDuration(300);
        heightAnim1->setStartValue(ui->lb_server_ip->height());
        heightAnim1->setEndValue(0);
        animGroup->addAnimation(heightAnim1);

        QPropertyAnimation *heightAnim2 = new QPropertyAnimation(ui->w_server_ip, "maximumHeight");
        heightAnim2->setDuration(300);
        heightAnim2->setStartValue(ui->w_server_ip->height());
        heightAnim2->setEndValue(0);
        animGroup->addAnimation(heightAnim2);

        QPropertyAnimation *heightAnim3 = new QPropertyAnimation(ui->lb_server_port, "maximumHeight");
        heightAnim3->setDuration(300);
        heightAnim3->setStartValue(ui->lb_server_port->height());
        heightAnim3->setEndValue(0);
        animGroup->addAnimation(heightAnim3);

        QPropertyAnimation *heightAnim4 = new QPropertyAnimation(ui->le_server_port, "maximumHeight");
        heightAnim4->setDuration(300);
        heightAnim4->setStartValue(ui->le_server_port->height());
        heightAnim4->setEndValue(0);
        animGroup->addAnimation(heightAnim4);

        // 按钮切换的不透明度动画
        ui->pb_add_node->setVisible(true);

        // 设置初始不透明度
        SetWidgetOpacity(ui->pb_add_node, 0.0);
        SetWidgetOpacity(ui->pb_query_ip, 1.0);
        SetWidgetOpacity(ui->pb_set_ip, 1.0);

        // 添加节点按钮的淡入动画
        QGraphicsOpacityEffect *addNodeEffect =
            static_cast<QGraphicsOpacityEffect *>(ui->pb_add_node->property("opacityEffect").value<void *>());

        QPropertyAnimation *addNodeAnim = new QPropertyAnimation(addNodeEffect, "opacity");
        addNodeAnim->setDuration(300);
        addNodeAnim->setStartValue(0.0);
        addNodeAnim->setEndValue(1.0);
        animGroup->addAnimation(addNodeAnim);

        // 查询IP按钮的淡出动画
        QGraphicsOpacityEffect *queryIpEffect =
            static_cast<QGraphicsOpacityEffect *>(ui->pb_query_ip->property("opacityEffect").value<void *>());

        QPropertyAnimation *queryIpAnim = new QPropertyAnimation(queryIpEffect, "opacity");
        queryIpAnim->setDuration(300);
        queryIpAnim->setStartValue(1.0);
        queryIpAnim->setEndValue(0.0);
        animGroup->addAnimation(queryIpAnim);

        // 设置IP按钮的淡出动画
        QGraphicsOpacityEffect *setIpEffect =
            static_cast<QGraphicsOpacityEffect *>(ui->pb_set_ip->property("opacityEffect").value<void *>());

        QPropertyAnimation *setIpAnim = new QPropertyAnimation(setIpEffect, "opacity");
        setIpAnim->setDuration(300);
        setIpAnim->setStartValue(1.0);
        setIpAnim->setEndValue(0.0);
        animGroup->addAnimation(setIpAnim);

        // 动画完成后更新UI
        connect(animGroup, &QParallelAnimationGroup::finished, [this]() { UpdateUIForAddNodeMode(); });
    }

    // 启动动画组
    animGroup->start(QAbstractAnimation::DeleteWhenStopped);
}

} // namespace ys_scene