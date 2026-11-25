#include "status_panel.h"
#include "toast/toast_tips.h"
#include "collapsible_groupbox/collapsible_groupbox.h"
#include "ip_address/ipaddress_widget.h"
#include "tools/ip_utils.h"
#include "log/logging.h"
#include "ys_lidar_user.h"
#include <QLabel>
#include <QVBoxLayout>
#include <QTimer>
#include <QPushButton>
#include <QHBoxLayout>
#include <QMessageBox>

namespace ys_scene
{
    StatusPanel::StatusPanel(QWidget* parent) :
        QDialog(parent),
        current_device_ip_(0)
    {
        // 给整个面板添加渐变背景
        setStyleSheet(R"(
        StatusPanel {
            background: #242424;
            border-radius: 10px;
        }
        QLabel {
            color: #B0B0B0;
            font-size: 12px;
        }
        QLabel[class="status-normal"] {
            color: #4CAF50;
            font-weight: bold;
        }
        QLabel[class="status-warning"] {
            color: #FFC107;
            font-weight: bold;
        }
        QLabel[class="status-error"] {
            color: #F44336;
            font-weight: bold;
        }
        QLabel[class="value"] {
            color: #E0E0E0;
            font-weight: bold;
        }
    )");

        QVBoxLayout* main_layout = new QVBoxLayout(this);
        main_layout->setSpacing(5);
        main_layout->setContentsMargins(5, 5, 5, 5);

        // 创建各个组
        // SetupNetworkGroup();
        SetupFPGAGroup();
        SetupMotorMirrorGroup();
        SetupComponentGroup();
        SetupArmGroup();

        // 添加到主布局
        // main_layout->addWidget(network_group_);
        main_layout->addWidget(fpga_group_);
        main_layout->addWidget(motor_mirror_group_);
        main_layout->addWidget(component_group_);
        main_layout->addWidget(arm_group_);
        main_layout->addStretch();

        // 设置固定宽度
        setFixedWidth(300);
    }

    void StatusPanel::SetCurrentDeviceIP(uint32_t ip)
    {
        current_device_ip_ = ip;
    }

    /*void StatusPanel::SetupNetworkGroup()
    {
        // Removed as per request to move to PropertyTreeWidget
    }*/

    void StatusPanel::SetupFPGAGroup()
    {
        fpga_group_ = new CollapsibleGroupBox(tr("FPGA Test"), this);
        QGridLayout* layout = new QGridLayout(fpga_group_->GetContent());

        layout->addWidget(new QLabel(tr("Test Result:")), 0, 0);
        fpga_done_ = new QLabel(tr("Normal"));
        fpga_done_->setStyleSheet("color: green;");
        layout->addWidget(fpga_done_, 0, 1);

        layout->addWidget(new QLabel(tr("Work Mode:")), 1, 0);
        fpga_work_mode_ = new QLabel(tr("Normal"));
        layout->addWidget(fpga_work_mode_, 1, 1);

        layout->addWidget(new QLabel(tr("Temp:")), 2, 0);
        fpga_work_temp_ = new QLabel(tr("0℃"));
        layout->addWidget(fpga_work_temp_, 2, 1);
    }

    void StatusPanel::SetupMotorMirrorGroup()
    {
        motor_mirror_group_ = new CollapsibleGroupBox(tr("Motor&Mirror"), this);
        QGridLayout* layout = new QGridLayout(motor_mirror_group_->GetContent());

        // 这里添加示例电机状态
        layout->addWidget(new QLabel(tr("Status:")), 0, 0);
        motor_status_ = new QLabel(tr("Normal"));
        layout->addWidget(motor_status_, 0, 1);

        layout->addWidget(new QLabel(tr("Mirror Status:")), 1, 0);
        mirror_angle_ = new QLabel(tr("123°"));
        layout->addWidget(mirror_angle_, 1, 1);

        layout->addWidget(new QLabel(tr("Motor Speed:")), 2, 0);
        motor_cycles_ = new QLabel(tr("11"));
        layout->addWidget(motor_cycles_, 2, 1);

    }

    void StatusPanel::SetupComponentGroup()
    {
        component_group_ = new CollapsibleGroupBox(tr("Component"), this);
        QGridLayout* layout = new QGridLayout(component_group_->GetContent());

        // 这里添加示例温度状态
        layout->addWidget(new QLabel(tr("SpadStatus")), 0, 0);
        spi_status_ = new QLabel(tr("Normal"));
        layout->addWidget(spi_status_, 0, 1);

        layout->addWidget(new QLabel(tr("LD Temp")), 1, 0);
        ld_temp_ = new QLabel(tr("21℃"));
        layout->addWidget(ld_temp_, 1, 1);

        layout->addWidget(new QLabel(tr("Spad Temp")), 2, 0);
        spad_temp_ = new QLabel(tr("21℃"));
        layout->addWidget(spad_temp_, 2, 1);

        layout->addWidget(new QLabel(tr("Vop Votage")), 3, 0);
        vop_voltage_ = new QLabel(tr("21V"));
        layout->addWidget(vop_voltage_, 3, 1);
    }

    void StatusPanel::SetupArmGroup()
    {
        arm_group_ = new CollapsibleGroupBox(tr("Arm"), this);
        QGridLayout* layout = new QGridLayout(arm_group_->GetContent());
        layout->addWidget(new QLabel(tr("Status")), 0, 0);
        arm_status_ = new QLabel(tr("Free"));
        layout->addWidget(arm_status_, 0, 1);
    }

    void StatusPanel::StaticQueryNodeStatusCallback(const uint32_t handle, const DeviceStatusT* status, void* context)
    {
        auto* self = static_cast<StatusPanel*>(context);
        if (self) {
            self->HandleQueryNodeStatus(handle, status, context);
        }
    }

    void StatusPanel::HandleQueryNodeStatus(const uint32_t handle, const DeviceStatusT* status, void* context)
    {
        LOG_INFO("StatusPanel::HandleQueryNodeStatus");
        // 先复制需要的数据,因为 status 指针可能在其他线程被释放
        DeviceStatusT status_copy = *status;

        // 使用 Qt 的跨线程调用机制,确保在主线程中更新界面
        QMetaObject::invokeMethod(this, [this, status_copy, handle]() {
            if (status_copy.ret_code.ack_code == static_cast<uint8_t>(NodeStatus::KFailed)) {
                QString title(tr("warning! Node status"));
                QString text(tr("Query node status failed!"));
                // ToastTips::Instance().ShowTip(ToastPreset::WARNING_DARK, title, text);
                // TODO: 处理查询失败的情况
                // 首先将当前IP设置为0
                SetCurrentDeviceIP(0);
				//emit LidarConnectionStatusChanged(handle, false);
				return;
            }
            // 更新当前ip
            SetCurrentDeviceIP(handle);
            QString fpga_done;
            switch (static_cast<FpgaTest>(status_copy.fpga_done)) {
            case KTestDone:
                fpga_done = tr("Done");
                break;
            case KTestUndone:
                fpga_done = tr("Undone");
                break;
            default:
                break;
            }
            fpga_done_->setText(fpga_done);

            QString fpga_status;
            switch (static_cast<LidarState>(status_copy.fpga_workmode)) {
            case LidarState::kLidarFree:
                fpga_status = tr("Free");
                break;
            case LidarState::kLidarScan:
                fpga_status = tr("Scan");
                break;
            case LidarState::kLidarFault:
                fpga_status = tr("Fault");
                break;
            case LidarState::kLidarDebug:
                fpga_status = tr("Debug");
                break;
            case LidarState::kLidarUnknown:
                fpga_status = tr("Unknown");
                break;
            default:
                break;
            }
            fpga_work_mode_->setText(fpga_status);      
            fpga_work_temp_->setText(QString("%1 ℃").arg(status_copy.fpga_temp));
            QString motor_status;
            switch (static_cast<MotorStatus>(status_copy.motor_status)) {
            case KMotorNormal:
                motor_status = tr("Normal");
                break;
            case KMotorAbnormal:
                motor_status = tr("Abnormal");
                break;
            default:
                break;
            }
            motor_status_->setText(motor_status);

            QString mirror_angle;
            switch (static_cast<MirrorAngle>(status_copy.mirror_angle)) {
            case KMirrorAngelNormal:
                mirror_angle = tr("Normal");
                break;
            case KMirroAngleAbnormal:
                mirror_angle = tr("Abnormal");
                break;
            default:
                break;
            }
            mirror_angle_->setText(mirror_angle);

            motor_cycles_->setText(QString("%1").arg(status_copy.motor_cycles));

            QString spad_spi_status;
            switch (static_cast<SpadSpiStatus>(status_copy.spi_status)) {
            case KSpadSpiNormal:
                spad_spi_status = tr("Normal");
                break;
            case KSpadSpiAbnormal:
                spad_spi_status = tr("Abnormal");
                break;
            default:
                break;
            }
            spi_status_->setText(spad_spi_status);

            ld_temp_->setText(QString("%1 ℃").arg(status_copy.ld_temp));
            spad_temp_->setText(QString("%1 ℃").arg(status_copy.spad_temp));
            vop_voltage_->setText(QString("%1 ℃").arg(status_copy.vop_voltage));

			emit LidarWorkModeChanged(handle, status_copy.arm_status.arm_status, true);
            QString arm_status;
            switch (static_cast<LidarState>(status_copy.arm_status.arm_status)) {
            case LidarState::kLidarFree:
                arm_status = tr("Free");
                break;
            case LidarState::kLidarScan:
                arm_status = tr("Scan");
                break;
            case LidarState::kLidarFault:
                arm_status = tr("Fault");
                break;
            case LidarState::kLidarDebug:
                arm_status = tr("Debug");
                break;
            case LidarState::kLidarUnknown:
                arm_status = tr("Unknown");
                break;
            default:
                break;
            }
            arm_status_->setText(arm_status);
            LOG_INFO("UI updated with device status");
            }

            //fpga_work_mode_->setText(QString::number(status_copy.fpga_work_mode));
            //fpga_work_temp_->setText(QString("%1℃").arg(status_copy.fpga_temperature));

            //// 电机和反射镜相关
            //mirror_angle_->setText(QString("%1°").arg(status_copy.mirror_angle));
            //motor_cycles_->setText(QString::number(status_copy.motor_cycles));

            //// 组件状态
            //spi_status_->setText(status_copy.spi_status ? tr("Normal") : tr("Error"));
            //ld_temp_->setText(QString("%1℃").arg(status_copy.ld_temperature));
            //spad_temp_->setText(QString("%1℃").arg(status_copy.spad_temperature));
            //vop_voltage_->setText(QString("%1V").arg(status_copy.vop_voltage));


        
, Qt::QueuedConnection);
    }

// Removed network logic
//
//
//
//
//
//
//
//
//
//
//
//
//
//
//
//
//
//
//
//
//
//
//
//
//
//
//
//
//
//
//
//
//
//
//
//
//
//
//
//
//
//
//
//
//
//
//
//
//
//
//
//
//
//
//
//
//
//
//
//
//
//
//
//
//
//
//
//
//
//
//
//
//
//
//
//
//
//
//
//
//
//
//
//
//
//
//
//
//
//
//
//
//
//
//
//
//
//
//
//
//
//
//
//
//
//
//
//
//
//
//
//
//
//
//
//
//
//
//
//
//
//
//
//
//
//
//
//
//
//
//
//
//
//
//
//
//
//
//
//
//
//
//
//
//
//
//
//
//
//
//
//
//
//
//
//
//
//
//
//
//
//
//
//
//
//
//
//
//
//
//
//
//
//
//
//
//
//
//
//
//
//
//

void StatusPanel::NodeStatusQueryStart(const uint32_t handle, bool start)
{
	if (start) {
		QueryNodeStatusT();
	}else{
        SetCurrentDeviceIP(0);
    }
}

void StatusPanel::QueryNodeStatusT()
{
    // 使用包装后的回调函数
    //QueryNodeStatus(0, callback, this);
    QueryNodeStatus(0, &StatusPanel::StaticQueryNodeStatusCallback, this);
    /*QueryNodeStatus(0,
        [](const uint32_t handle, const DeviceStatusT* status, void* context) {
            auto* self = static_cast<StatusPanel*>(context);
            if (self) {
                self->HandleQueryNodeStatus(handle, status, context);
            }
        },
        this
    );*/
}

}// namespace ys_scene
