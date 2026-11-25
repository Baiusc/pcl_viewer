#pragma once

#include "business/device_management/sn_client.h"

#include <QDialog>

class QComboBox;
class QLabel;
class QLineEdit;
class QPushButton;
class QSpinBox;

namespace ys_scene {

class SnDialog : public QDialog
{
    Q_OBJECT
  public:
    explicit SnDialog(QWidget *parent);
    void reject() override;
  protected:
    void closeEvent(QCloseEvent *event) override;
  private:
    lidar_network::Endpoint Endpoint() const;
    lidar_sn::WorkOrder Order() const;
    void RefreshDevices();
    void ClearIdentity();
    void Update();
    void ConfirmOrder();

    lidar_sn::SnClient client_;
    lidar_sn::Identity current_;
    QString discovered_mac_;
    bool online_read_ = false;
    QComboBox *nic_;
    QComboBox *devices_;
    QLineEdit *ip_;
    QSpinBox *port_;
    QPushButton *read_;
    QPushButton *diagnostic_;
    QPushButton *write_;
    QPushButton *close_;
    QLabel *identity_;
    QLabel *duplicate_;
    QLabel *order_error_;
    QLabel *status_;
    QLineEdit *work_order_;
    QLineEdit *operator_;
    QLineEdit *mac_;
    QLineEdit *sn_;
};

} // namespace ys_scene
