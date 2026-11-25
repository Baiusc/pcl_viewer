#ifndef YS_SCENE_MATRIX_LIDAR_ADAPTER_H_
#define YS_SCENE_MATRIX_LIDAR_ADAPTER_H_

#include "matrix_process.h"
#include "FsTcpTransControl.h"
#include "ys_lidar_type.h"
#include "ys_scene_def.h"
#include <string.h>
#include <memory>
#include <functional>

namespace ys_scene {

class MatrixLidarAdapter : public FrameDataListener {
public:
    enum InitMode {
        FULL_INIT,    // 完全初始化，包括点云处理器
        TCP_ONLY_INIT // 仅初始化TCP功能，不包括点云处理器
    };

	MatrixLidarAdapter(InitMode mode = FULL_INIT);
	~MatrixLidarAdapter();

	bool Connect(const std::string& ip, int port = 6001);
	bool DisConnect();
	bool IsConnected() const;

    bool ConnectTcp(const std::string &ip, int port = 6001);
    bool DisconnectTcp();
    bool IsTcpConnected() const;

	// IP查询和设置方法
    bool QueryIpPort(std::string &device_ip, int &device_port, std::string &host_ip, int &host_port);
    bool SetIpPort(const std::string &device_ip, int device_port, const std::string &host_ip, int host_port);
	void SetPointcloudCallback(std::function<void(CloudFramePtr)> callback);

	// FrameDataListener重写
    void notice_point_cloud(oneFramePoi &poi_frame) override;
	void notice_imu(fs_lidar_imu_t& imu_t) override;

private:
    CloudFramePtr ConvertPointCloud(const oneFramePoi &poi_frame);
	std::pair<std::string, int> FindReachableIP(const std::vector<std::pair<std::string, int>> &candidate_ips);
private:
	bool connected_;
	bool tcp_connected_;
	std::string device_ip_;
	int local_port_;
	int device_port_;
	std::function<void(CloudFramePtr)> pointcloud_callback_;
};

} // namespace ys_scene

#endif
