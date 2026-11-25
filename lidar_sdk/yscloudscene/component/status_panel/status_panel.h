#ifndef STATUS_PANEL_H
#define STATUS_PANEL_H

#include <QDialog>
#include "ys_lidar_type.h"

class QLabel;
class QTimer;
class QPushButton;

namespace ys_scene
{

class CollapsibleGroupBox;
class IPAddressWidget;

class StatusPanel : public QDialog
{
    Q_OBJECT
public:
    explicit StatusPanel(QWidget *parent = nullptr);
    void SetCurrentDeviceIP(uint32_t ip);

signals:
    void LidarWorkModeChanged(const uint32_t& handle, const uint8_t& mode, bool enabled);
	void LidarConnectionStatusChanged(const uint32_t& ip, bool connected);

private:
    CollapsibleGroupBox* data_port_group_;
    CollapsibleGroupBox* fpga_group_;
    CollapsibleGroupBox* motor_mirror_group_;
    CollapsibleGroupBox* component_group_;
    CollapsibleGroupBox* arm_group_;

    QLabel* fpga_done_;
    QLabel* fpga_work_mode_;
    QLabel* fpga_work_temp_;

    QLabel* motor_status_;
    QLabel* mirror_angle_;
    QLabel* motor_cycles_;

    QLabel* spi_status_;
    QLabel* ld_temp_;
    QLabel* spad_temp_;
    QLabel* vop_voltage_;

    QLabel* arm_status_;
    uint32_t current_device_ip_;

    void SetupFPGAGroup();
    void SetupMotorMirrorGroup();
    void SetupComponentGroup();
    void SetupArmGroup();

private slots:
    void QueryNodeStatusT();
    static void StaticQueryNodeStatusCallback(const uint32_t handle, const DeviceStatusT* status, void* context);
    void HandleQueryNodeStatus(const uint32_t handle, const DeviceStatusT* status, void* context);

  public slots:
	void NodeStatusQueryStart(const uint32_t handle, bool start);

};

}// namespace ys_scene

#endif // STATUS_PANEL_H
