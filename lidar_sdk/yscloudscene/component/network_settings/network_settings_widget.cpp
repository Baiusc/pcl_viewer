#include "network_settings_widget.h"
#include "component/ip_address/ipaddress_widget.h"
#include "tools/ip_utils.h"
#include "ys_lidar_user.h"
#include <QLabel>
#include <QVBoxLayout>
#include <QGridLayout>
#include <QPushButton>
#include <QMessageBox>
#include <QTimer>
#include <QDebug>

namespace ys_scene
{

NetworkSettingsWidget::NetworkSettingsWidget(QWidget *parent)
    : QWidget(parent)
    , current_device_handle_(0)
{
    SetupUI();
    
    query_ip_timeout_timer_ = new QTimer(this);
    set_ip_timeout_timer_ = new QTimer(this);
    
    query_ip_timeout_timer_->setInterval(1000); // 1s
    set_ip_timeout_timer_->setInterval(1000); // 1s
    
    query_ip_timeout_timer_->setSingleShot(true);
    set_ip_timeout_timer_->setSingleShot(true);
    
    connect(query_ip_timeout_timer_, &QTimer::timeout, this, &NetworkSettingsWidget::OnQueryIPTimeout);
    connect(set_ip_timeout_timer_, &QTimer::timeout, this, &NetworkSettingsWidget::OnSetIPTimeout);
}

NetworkSettingsWidget::~NetworkSettingsWidget()
{
}

void NetworkSettingsWidget::SetDeviceHandle(uint32_t handle)
{
    current_device_handle_ = handle;
}

void NetworkSettingsWidget::SetupUI()
{
    // Use Vertical Layout for the main container
    QVBoxLayout* mainLayout = new QVBoxLayout(this);
    mainLayout->setContentsMargins(0, 0, 0, 0);
    mainLayout->setSpacing(10);
    
    // Group everything in a grid layout similar to StatusPanel but transparent
    QWidget* contentWidget = new QWidget(this);
    QGridLayout* layout = new QGridLayout(contentWidget);
    layout->setContentsMargins(0, 0, 0, 0);
    layout->setSpacing(10);
    
    // IP Address
    QLabel* ipLabel = new QLabel(tr("IP:"), this);
    ipLabel->setStyleSheet("color: #e0e0e0; font-weight: bold;");
    layout->addWidget(ipLabel, 0, 0);
    
    ip_widget_ = new IPAddressWidget(this);
    ip_widget_->SetIP("192.168.1.10"); // Default
    layout->addWidget(ip_widget_, 0, 1);
    
    // Gateway
    QLabel* gwLabel = new QLabel(tr("Gateway:"), this);
    gwLabel->setStyleSheet("color: #e0e0e0; font-weight: bold;");
    layout->addWidget(gwLabel, 1, 0);
    
    gateway_widget_ = new IPAddressWidget(this);
    gateway_widget_->SetIP("192.168.1.1");
    layout->addWidget(gateway_widget_, 1, 1);
    
    // Mask
    QLabel* maskLabel = new QLabel(tr("Mask:"), this);
    maskLabel->setStyleSheet("color: #e0e0e0; font-weight: bold;");
    layout->addWidget(maskLabel, 2, 0);
    
    mask_widget_ = new IPAddressWidget(this, true, IPAddressWidget::IPMode::SubnetMask);
    mask_widget_->SetIP("255.255.255.0");
    layout->addWidget(mask_widget_, 2, 1);
    
    // Buttons
    QHBoxLayout *button_layout = new QHBoxLayout();
    query_ip_button_ = new QPushButton(tr("Query IP"), this);
    set_ip_button_ = new QPushButton(tr("Set IP"), this);
    
    // Button styling for dark theme
    QString btnStyle = "QPushButton { background-color: #555; color: white; border: 1px solid #777; border-radius: 4px; padding: 5px; }"
                       "QPushButton:hover { background-color: #666; }"
                       "QPushButton:pressed { background-color: #444; }"
                       "QPushButton:disabled { background-color: #333; color: #888; }";
    query_ip_button_->setStyleSheet(btnStyle);
    set_ip_button_->setStyleSheet(btnStyle);

    button_layout->addStretch();
    button_layout->addWidget(query_ip_button_);
    button_layout->addWidget(set_ip_button_);
    
    layout->addLayout(button_layout, 3, 0, 1, 2);
    
    mainLayout->addWidget(contentWidget);
    mainLayout->addStretch();
    
    connect(query_ip_button_, &QPushButton::clicked, this, &NetworkSettingsWidget::OnQueryIPClicked);
    connect(set_ip_button_, &QPushButton::clicked, this, &NetworkSettingsWidget::OnSetIPClicked);
}

void NetworkSettingsWidget::OnQueryIPClicked()
{
    if (current_device_handle_ == 0) {
        // Can be embedded in a Property Tree so maybe use a overlay warning or log
        // For now use message box but ensure parent is this widget
        QMessageBox::warning(this, tr("Warning"), tr("No device connected."));
        return;
    }
    
    query_ip_button_->setEnabled(false);
    query_ip_button_->setText(tr("Querying..."));
    
    query_ip_timeout_timer_->start();
    
    GetNodeIP(current_device_handle_, &NetworkSettingsWidget::StaticGetNodeIPCallback, this);
}

void NetworkSettingsWidget::OnSetIPClicked()
{
    if (current_device_handle_ == 0) {
         QMessageBox::warning(this, tr("Warning"), tr("No device connected."));
         return;
    }
    
    StaticNodeIPT ip_info;
    using lumosy::IPUtils;
    
    ip_info.ip_address = IPUtils::IPStringToUint32(ip_widget_->GetIP());
    ip_info.gateway_address = IPUtils::IPStringToUint32(gateway_widget_->GetIP());
    ip_info.net_mask = IPUtils::IPStringToUint32(mask_widget_->GetIP());
    
    set_ip_button_->setEnabled(false);
    set_ip_button_->setText(tr("Setting..."));
    
    set_ip_timeout_timer_->start();
    
    SetStaticIP(current_device_handle_, &ip_info, &NetworkSettingsWidget::StaticSetStaticIPCallback, this);
}

void NetworkSettingsWidget::OnQueryIPTimeout()
{
    query_ip_button_->setEnabled(true);
    query_ip_button_->setText(tr("Query IP"));
    QMessageBox::warning(this, tr("Timeout"), tr("Query IP Timeout"));
}

void NetworkSettingsWidget::OnSetIPTimeout()
{
    set_ip_button_->setEnabled(true);
    set_ip_button_->setText(tr("Set IP"));
    QMessageBox::warning(this, tr("Timeout"), tr("Set IP Timeout"));
}

void NetworkSettingsWidget::StaticGetNodeIPCallback(const uint32_t handle, ys_status status, const NodeIPT *ip_info, void *context)
{
    NetworkSettingsWidget* self = static_cast<NetworkSettingsWidget*>(context);
    if (self) {
        // Ensure UI updates run on main thread if callback is from another thread
        // QMetaObject::invokeMethod...
        // Assuming callback might be on another thread, usually for safety in Qt
        QMetaObject::invokeMethod(self, [self, handle, status, ip_info]() {
            self->HandleGetNodeIPResult(handle, status, ip_info);
        }, Qt::QueuedConnection);
    }
}

void NetworkSettingsWidget::StaticSetStaticIPCallback(const uint32_t handle, ys_status status, void *context)
{
    NetworkSettingsWidget* self = static_cast<NetworkSettingsWidget*>(context);
    if (self) {
         QMetaObject::invokeMethod(self, [self, handle, status]() {
            self->HandleSetStaticIPResult(handle, status);
        }, Qt::QueuedConnection);
    }
}

void NetworkSettingsWidget::HandleGetNodeIPResult(const uint32_t handle, ys_status status, const NodeIPT *ip_info)
{
    Q_UNUSED(handle);
    query_ip_timeout_timer_->stop();
    query_ip_button_->setEnabled(true);
    query_ip_button_->setText(tr("Query IP"));
    
    if (status == 0 && ip_info) { // Success check
        using lumosy::IPUtils;
        
        ip_widget_->SetIP(IPUtils::Uint32ToIPString(ip_info->ip_address));
        gateway_widget_->SetIP(IPUtils::Uint32ToIPString(ip_info->gateway_address));
        mask_widget_->SetIP(IPUtils::Uint32ToIPString(ip_info->net_mask));
        
        // Optional: show toast
    } else {
        QMessageBox::warning(this, tr("Error"), tr("Failed to query IP."));
    }
}

void NetworkSettingsWidget::HandleSetStaticIPResult(const uint32_t handle, ys_status status)
{
     Q_UNUSED(handle);
     set_ip_timeout_timer_->stop();
     set_ip_button_->setEnabled(true);
     set_ip_button_->setText(tr("Set IP"));
     
     if (status == 0) { // Success check
         QMessageBox::information(this, tr("Success"), tr("IP Set Successfully. Device may reboot."));
     } else {
         QMessageBox::warning(this, tr("Error"), tr("Failed to set IP."));
     }
}

} // namespace ys_scene
