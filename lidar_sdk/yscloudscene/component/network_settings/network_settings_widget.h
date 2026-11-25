#ifndef NETWORK_SETTINGS_WIDGET_H
#define NETWORK_SETTINGS_WIDGET_H

#include <QWidget>
#include "ys_lidar_type.h"

class QPushButton;
class QTimer;

namespace ys_scene
{

class IPAddressWidget;

class NetworkSettingsWidget : public QWidget
{
    Q_OBJECT
public:
    explicit NetworkSettingsWidget(QWidget *parent = nullptr);
    ~NetworkSettingsWidget();

    void SetDeviceHandle(uint32_t handle);

private:
    IPAddressWidget* ip_widget_;
    IPAddressWidget* gateway_widget_;
    IPAddressWidget* mask_widget_;
    QPushButton *query_ip_button_;
    QPushButton *set_ip_button_;
    
    QTimer *query_ip_timeout_timer_;
    QTimer *set_ip_timeout_timer_;
    
    uint32_t current_device_handle_;

    void SetupUI();

private slots:
    void OnQueryIPClicked();
    void OnSetIPClicked();
    void OnQueryIPTimeout();
    void OnSetIPTimeout();
    
    // Static callbacks for C API
    static void StaticGetNodeIPCallback(const uint32_t handle, ys_status status, const NodeIPT *ip_info, void *context);
    static void StaticSetStaticIPCallback(const uint32_t handle, ys_status status, void *context);
    
    void HandleGetNodeIPResult(const uint32_t handle, ys_status status, const NodeIPT *ip_info);
    void HandleSetStaticIPResult(const uint32_t handle, ys_status status);
};

} // namespace ys_scene

#endif // NETWORK_SETTINGS_WIDGET_H
