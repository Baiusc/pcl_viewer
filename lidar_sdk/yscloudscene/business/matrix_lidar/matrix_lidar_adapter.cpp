#include "matrix_lidar_adapter.h"
#include "log/logging.h"

#if (_WINDLL || _WINDOWS || _WIN32)
#include <winsock2.h>
#endif

namespace ys_scene {
MatrixLidarAdapter::MatrixLidarAdapter(InitMode mode)
    : connected_(false),
      tcp_connected_(false),
      local_port_(6002),
      device_port_(6001),
      device_ip_("")
{
    if (mode == FULL_INIT) {
        init_processer(this);
        LOG_INFO("MatrixLidarAdapter initialized with full mode including point cloud processor");
    }
    else {
        LOG_INFO("MatrixLidarAdapter initialized with TCP-only mode");
    }
}

MatrixLidarAdapter::~MatrixLidarAdapter()
{
    if (connected_) {
        DisConnect();
    }

    if (tcp_connected_) {
        DisconnectTcp();
    }
}

bool MatrixLidarAdapter::Connect(const std::string &ip, int port)
{// 检查当前状态，避免重复连接
    ConnectionState current_state = get_connection_state();
    if (current_state == STATE_CONNECTED) {
        if (device_ip_ == ip && device_port_ == port && connected_) {
            LOG_INFO("Already connected to matrix LiDAR at {}:{}", ip, port);
            return true; // 已连接到相同设备
        }

        // 连接到不同设备，先断开当前连接
        LOG_INFO("Disconnecting from current device before connecting to {}:{}", ip, port);
        DisConnect();
    }
    else if (current_state == STATE_CONNECTING) {
        LOG_WARN("Connection in progress, please wait");
        return false;
    }

    // 清理之前的资源
    if (connected_) {
        DisConnect();
    }

    // 更新设备信息
    device_ip_ = ip;
    device_port_ = port;

    LOG_INFO("Connecting to matrix LiDAR at {}:{}", ip, port);

    // 调用matrix_process.h中的连接函数
    int result = link_device(local_port_, device_ip_, device_port_);

    if (result == EC_OK) {
        connected_ = true;
        LOG_INFO("Successfully connected to matrix LiDAR");
        return true;
    }
    else if (result == EC_ALREADY_CONNECTED) {
        // 特殊处理已连接状态
        LOG_WARN("Device reports already connected, checking state");

        if (get_connection_state() == STATE_CONNECTED) {
            connected_ = true;
            LOG_INFO("Successfully connected to matrix LiDAR");
            return true;
        }
        else {
            // 状态不一致，尝试断开后重连
            LOG_WARN("Connection state inconsistent, trying to reconnect");
            break_device(); // 强制断开
            std::this_thread::sleep_for(std::chrono::milliseconds(500));

            result = link_device(local_port_, device_ip_, device_port_);
            if (result == EC_OK) {
                connected_ = true;
                LOG_INFO("Successfully reconnected to matrix LiDAR");
                return true;
            }
        }
    }

    // 处理连接失败
    LOG_ERROR("Failed to connect to matrix LiDAR. Error code: {} ({})", result, get_error_string(result));

    // 对特定错误进行处理
    switch (result) {
    case EC_BIND_FAILD:
        LOG_ERROR("Port binding failed. Port {} might be in use", local_port_);
        break;
    case EC_SOCKET_CREATE_FAILED:
        LOG_ERROR("Socket creation failed. Network might be unavailable");
        break;
    }
    return false;
}

bool MatrixLidarAdapter::DisConnect()
{
    if (!connected_) {
        return true; // 已经断开连接
    }

    LOG_INFO("Disconnecting from matrix LiDAR");

    // 先将本地状态更新为断开
    connected_ = false;

    // 调用matrix_process.h中的断开连接函数
    int result = break_device();

    if (result == EC_OK) {
        LOG_INFO("Successfully disconnected from matrix LiDAR");
        return true;
    }
    else if (result == EC_NOT_CONNECTED) {
        LOG_WARN("Device reports not connected, but local state was connected");
        return true; // 虽然有警告，但仍然认为断开成功
    }
    else if (result == EC_RESOURCE_BUSY) {
        LOG_WARN("Device is busy, disconnect operation may be incomplete");
        // 尝试等待一段时间后检查状态
        std::this_thread::sleep_for(std::chrono::milliseconds(1000));
        if (get_connection_state() == STATE_DISCONNECTED) {
            LOG_INFO("Device eventually disconnected successfully");
            return true;
        }
        LOG_WARN("Device may still be partially connected");
        return false;
    }
    else {
        LOG_ERROR("Failed to disconnect from matrix LiDAR. Error code:{} ({})", result, get_error_string(result));
        return false;
    }
}

bool MatrixLidarAdapter::IsConnected() const
{
    return connected_;
}

bool MatrixLidarAdapter::ConnectTcp(const std::string &ip, int port)
{
    // 如果已经连接，先断开
    if (tcp_connected_) {
        DisconnectTcp();
    }

    LOG_INFO("Connecting to matrix LiDAR via TCP at {}:{}", ip, port);

    // 建立TCP连接
    int result = link_to_device_tcp(ip, 2000); // 使用2秒超时

    if (result == FS_EC_OK) {
        tcp_connected_ = true;
        LOG_INFO("Successfully connected to matrix LiDAR via TCP");
        return true;
    }
    else {
        LOG_ERROR("Failed to connect to matrix LiDAR via TCP. Error code: {}", result);

        // 对特定错误提供更详细的信息
        switch (result) {
        case FS_EC_INIT_SOCK_FAILD:
            LOG_ERROR("Failed to initialize socket");
            break;
        case FS_EC_LINK_CTRLSRV_FAILD:
            LOG_ERROR("Failed to link to control server");
            break;
        case FS_EC_LINK_ERROR:
            LOG_ERROR("Network link error");
            break;
        default:
            LOG_ERROR("Unknown error");
        }

        return false;
    }
}

bool MatrixLidarAdapter::DisconnectTcp()
{
    if (!tcp_connected_) {
        return true; // 已经断开连接
    }

    LOG_INFO("Disconnecting from matrix LiDAR via TCP");

    // 断开TCP连接
    break_link_tcp();

    // 无论结果如何，都将状态置为未连接
    tcp_connected_ = false;

    LOG_INFO("Disconnected from matrix LiDAR via TCP");
    return true;
}

bool MatrixLidarAdapter::IsTcpConnected() const
{
    return tcp_connected_;
}

bool MatrixLidarAdapter::QueryIpPort(std::string &device_ip, int &device_port, std::string &host_ip, int &host_port)
{
    // 检查TCP连接状态
    if (!tcp_connected_) {
        LOG_ERROR("Cannot query IP/Port: Not connected to Matrix LiDAR via TCP");
        return false;
    }

    // 获取TCP控制实例
    FsTcpTransControl *tcp = FsTcpTransControl::getInstance();
    if (!tcp) {
        LOG_ERROR("Failed to get TCP control instance");
        return false;
    }

    // 查询IP端口设置
    fs_ipport_t ipport_t = {0};
    bool get_ret = tcp->get_ipport(&ipport_t);

    if (!get_ret) {
        LOG_ERROR("Failed to query IP/Port from Matrix LiDAR");
        return false;
    }

    // 转换IP地址
    struct in_addr ip_addr;
    ip_addr.s_addr = ipport_t.lidar_ip;
    device_ip = inet_ntoa(ip_addr);

    struct in_addr host_addr;
    host_addr.s_addr = ipport_t.host_ip;
    host_ip = inet_ntoa(host_addr);

    // 设置端口
    device_port = ipport_t.lidar_port;
    host_port = ipport_t.host_port;

    LOG_INFO("Successfully queried IP/Port: Device {}:{}, Host {}:{}", device_ip, device_port, host_ip, host_port);

    return true;
}

bool MatrixLidarAdapter::SetIpPort(const std::string &device_ip, int device_port, const std::string &host_ip,
                                   int host_port)
{
    // 检查TCP连接状态
    if (!tcp_connected_) {
        LOG_ERROR("Cannot set IP/Port: Not connected to Matrix LiDAR via TCP");
        return false;
    }

    // 获取TCP控制实例
    FsTcpTransControl *tcp = FsTcpTransControl::getInstance();
    if (!tcp) {
        LOG_ERROR("Failed to get TCP control instance");
        return false;
    }

    // 准备IP端口设置
    fs_ipport_t ipport_t = {0};
    ipport_t.lidar_ip = inet_addr(device_ip.c_str());
    ipport_t.host_ip = inet_addr(host_ip.c_str());
    ipport_t.lidar_port = static_cast<uint16_t>(device_port);
    ipport_t.host_port = static_cast<uint16_t>(host_port);

    // 设置IP端口
    bool set_ret = tcp->set_ipport(ipport_t);

    if (!set_ret) {
        LOG_ERROR("Failed to set IP/Port on Matrix LiDAR");
        return false;
    }

    LOG_INFO("Successfully set IP/Port: Device {}:{}, Host {}:{}", device_ip, device_port, host_ip, host_port);

    return true;
}

void MatrixLidarAdapter::SetPointcloudCallback(std::function<void(CloudFramePtr)> callback)
{
    pointcloud_callback_ = callback;
}

void MatrixLidarAdapter::notice_point_cloud(oneFramePoi &poi_frame)
{
    // 收到点云数据时调用
    if (pointcloud_callback_) {
        // 转换点云数据格式并调用回调
        CloudFramePtr frame = ConvertPointCloud(poi_frame);
        if (frame && !frame->data.empty()) {
            pointcloud_callback_(frame);
        }
    }
}

void MatrixLidarAdapter::notice_imu(fs_lidar_imu_t &imu_t)
{
}

CloudFramePtr MatrixLidarAdapter::ConvertPointCloud(const oneFramePoi &poi_frame)
{
    static uint32_t frame_count = 0;

    // 假设HEIGHT和WIDTH是从global.h定义的常量
    auto frame = std::make_shared<CloudFrame<CartesianPoint>>(frame_count++, // 帧编号递增
                                                              HEIGHT * WIDTH // 预分配足够的空间
    );

    // 将点从点阵格式转换为您的格式
    for (int col = 0; col < WIDTH; col++ ) {
        for (int row = 0; row < HEIGHT; row++) {
            const onePoi &point = poi_frame[row][col];

            // 跳过无效点或距离为0的点
            /*if (point.pt3d.x == 0 && point.pt3d.y == 0 && point.pt3d.z == 0) {
                continue;
            }*/

            // 创建CartesianPoint并填充数据
            CartesianPoint cp;

            // 坐标转换 - cm转换成m
            cp.x.f = static_cast<float>(point.pt3d.x) * 0.01f;
            cp.y.f = static_cast<float>(point.pt3d.y) * 0.01f;
            cp.z.f = static_cast<float>(point.pt3d.z) * 0.01f;

            // 强度值（反射率）
            cp.reflectivity = static_cast<float>(point.reflectivity);

            // 添加到帧数据中
            frame->receive_times++;
            frame->data.push_back(cp);
        }
        frame->slot_counts.push_back(col);
    }
    // 设置帧元数据
    frame->ms_timestamp = poi_frame[0][0].timestamp;
    return frame;
}

std::pair<std::string, int>
MatrixLidarAdapter::FindReachableIP(const std::vector<std::pair<std::string, int>> &candidate_ips)
{
    return std::pair<std::string, int>();
}

} // namespace ys_scene
