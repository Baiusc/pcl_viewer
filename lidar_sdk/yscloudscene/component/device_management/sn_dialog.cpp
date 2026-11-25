#include "sn_dialog.h"

#include <QCloseEvent>
#include <QComboBox>
#include <QFormLayout>
#include <QGroupBox>
#include <QHBoxLayout>
#include <QLabel>
#include <QLineEdit>
#include <QMessageBox>
#include <QPushButton>
#include <QSignalBlocker>
#include <QSpinBox>
#include <QVBoxLayout>

namespace ys_scene {
namespace {

QString DeviceKey(const lidar_network::Announcement &device)
{
    return QStringLiteral("%1/%2/%3").arg(device.mac, device.endpoint.address).arg(device.endpoint.port);
}

} // namespace

SnDialog::SnDialog(QWidget *parent)
    : QDialog(parent)
{
    setWindowTitle(tr("设备 SN 管理"));
    resize(640, 490);
    auto *layout = new QVBoxLayout(this);
    auto *device_group = new QGroupBox(tr("设备身份"), this);
    auto *device_form = new QFormLayout(device_group);
    nic_ = new QComboBox(device_group);
    for (const auto &nic : QNetworkInterface::allInterfaces()) {
        if (!nic.flags().testFlag(QNetworkInterface::IsUp)) {
            continue;
        }
        QStringList addresses;
        for (const auto &entry : nic.addressEntries()) {
            if (entry.ip().protocol() == QAbstractSocket::IPv4Protocol) {
                addresses.append(entry.ip().toString());
            }
        }
        if (!addresses.isEmpty()) {
            nic_->addItem(QStringLiteral("%1 · %2").arg(nic.humanReadableName(), addresses.join(", ")), nic.name());
        }
    }
    devices_ = new QComboBox(device_group);
    ip_ = new QLineEdit(device_group);
    ip_->setPlaceholderText(QStringLiteral("192.168.1.100"));
    port_ = new QSpinBox(device_group);
    port_->setRange(1, 65535);
    port_->setValue(80);
    read_ = new QPushButton(tr("读取 SN"), device_group);
    diagnostic_ = new QPushButton(tr("只读诊断"), device_group);
    identity_ = new QLabel(device_group);
    identity_->setTextFormat(Qt::PlainText);
    identity_->setTextInteractionFlags(Qt::TextSelectableByMouse);
    identity_->setWordWrap(true);
    duplicate_ = new QLabel(tr("检测到不同 MAC 使用同一 SN。请人工核对。"), device_group);
    duplicate_->setWordWrap(true);
    device_form->addRow(tr("电脑网卡"), nic_);
    device_form->addRow(tr("发现的设备"), devices_);
    device_form->addRow(tr("设备 IPv4"), ip_);
    device_form->addRow(tr("HTTP 端口"), port_);
    device_form->addRow(read_, diagnostic_);
    device_form->addRow(identity_);
    device_form->addRow(duplicate_);
    layout->addWidget(device_group);

    auto *order_group = new QGroupBox(tr("产线工单"), this);
    auto *order_form = new QFormLayout(order_group);
    work_order_ = new QLineEdit(order_group);
    operator_ = new QLineEdit(order_group);
    mac_ = new QLineEdit(order_group);
    mac_->setPlaceholderText(tr("扫描或输入设备标签 MAC"));
    sn_ = new QLineEdit(order_group);
    sn_->setPlaceholderText(QStringLiteral("CS96XXXB01026020001"));
    order_error_ = new QLabel(order_group);
    order_error_->setWordWrap(true);
    order_form->addRow(tr("工单号"), work_order_);
    order_form->addRow(tr("操作员"), operator_);
    order_form->addRow(tr("设备标签 MAC"), mac_);
    order_form->addRow(tr("MES 分配的 SN"), sn_);
    order_form->addRow(order_error_);
    layout->addWidget(order_group);

    status_ = new QLabel(this);
    status_->setTextFormat(Qt::PlainText);
    status_->setTextInteractionFlags(Qt::TextSelectableByMouse);
    status_->setWordWrap(true);
    if (!client_.IsListening()) {
        status_->setText(tr("无法监听 UDP 5354。可手动填写 IPv4。"));
    }
    layout->addWidget(status_);
    auto *actions = new QHBoxLayout;
    actions->addStretch();
    write_ = new QPushButton(tr("写入 SN"), this);
    close_ = new QPushButton(tr("关闭"), this);
    actions->addWidget(write_);
    actions->addWidget(close_);
    layout->addLayout(actions);

    connect(nic_, qOverload<int>(&QComboBox::currentIndexChanged), this, [this] {
        ClearIdentity();
        RefreshDevices();
    });
    connect(devices_, qOverload<int>(&QComboBox::activated), this, [this](int index) {
        ClearIdentity();
        const auto key = devices_->itemData(index).toString();
        for (const auto &device : client_.Devices()) {
            if (device.endpoint.interface_id == nic_->currentData().toString() && DeviceKey(device) == key) {
                const QSignalBlocker blocker(port_);
                ip_->setText(device.endpoint.address);
                port_->setValue(device.endpoint.port);
                discovered_mac_ = device.mac;
                break;
            }
        }
        Update();
    });
    const auto manual_input = [this] {
        devices_->setCurrentIndex(0);
        ClearIdentity();
    };
    connect(ip_, &QLineEdit::textEdited, this, manual_input);
    connect(port_, qOverload<int>(&QSpinBox::valueChanged), this, manual_input);
    for (auto *field : {work_order_, operator_, mac_, sn_}) {
        connect(field, &QLineEdit::textChanged, this, &SnDialog::Update);
    }
    connect(read_, &QPushButton::clicked, this, [this] {
        online_read_ = true;
        client_.Read(Endpoint(), discovered_mac_, false);
    });
    connect(diagnostic_, &QPushButton::clicked, this, [this] {
        online_read_ = false;
        client_.Read(Endpoint(), discovered_mac_, true);
    });
    connect(write_, &QPushButton::clicked, this, &SnDialog::ConfirmOrder);
    connect(close_, &QPushButton::clicked, this, &SnDialog::reject);
    connect(&client_, &lidar_sn::SnClient::StatusChanged, status_, &QLabel::setText);
    connect(&client_, &lidar_sn::SnClient::BusyChanged, this, &SnDialog::Update);
    connect(&client_, &lidar_sn::SnClient::DevicesChanged, this, &SnDialog::RefreshDevices);
    connect(&client_, &lidar_sn::SnClient::IdentityChanged, this,
            [this](const lidar_sn::Identity &identity, const lidar_network::Endpoint &endpoint) {
                current_ = identity;
                if (!identity.mac.isEmpty() &&
                    (endpoint.address != Endpoint().address || endpoint.port != port_->value())) {
                    const QSignalBlocker blocker(port_);
                    discovered_mac_ = identity.mac;
                    ip_->setText(endpoint.address);
                    port_->setValue(endpoint.port);
                }
                Update();
            });
    RefreshDevices();
}

lidar_network::Endpoint SnDialog::Endpoint() const
{
    quint32 address;
    return {nic_->currentData().toString(),
            lidar_network::ParseIpv4(ip_->text(), address) && address >= 0x01000000u && address < 0xe0000000u
                ? QHostAddress(address).toString()
                : QString(),
            port_->value()};
}

lidar_sn::WorkOrder SnDialog::Order() const
{
    return {work_order_->text().trimmed(), operator_->text().trimmed(),
            lidar_network::NormalizeMac(mac_->text().trimmed()), lidar_sn::NormalizeSn(sn_->text())};
}

void SnDialog::RefreshDevices()
{
    const auto previous = devices_->currentData().toString();
    const QSignalBlocker blocker(devices_);
    devices_->clear();
    devices_->addItem(tr("手动输入 IPv4"));
    for (const auto &device : client_.Devices()) {
        if (device.endpoint.interface_id != nic_->currentData().toString()) {
            continue;
        }
        const QString label = QStringLiteral("%1 · %2:%3 · %4")
                                  .arg(device.mac, device.endpoint.address)
                                  .arg(device.endpoint.port)
                                  .arg(device.sn.isEmpty() ? device.identity : device.sn);
        devices_->addItem(client_.IsDuplicate(device.sn, device.mac) ? tr("%1（重复 SN）").arg(label) : label,
                          DeviceKey(device));
    }
    devices_->setCurrentIndex(qMax(0, devices_->findData(previous)));
    Update();
}

void SnDialog::ClearIdentity()
{
    current_ = {};
    discovered_mac_.clear();
    online_read_ = false;
    status_->clear();
    Update();
}

void SnDialog::Update()
{
    const bool editable = !client_.IsBusy();
    for (auto *input : {static_cast<QWidget *>(nic_), static_cast<QWidget *>(devices_), static_cast<QWidget *>(ip_),
                        static_cast<QWidget *>(port_), static_cast<QWidget *>(work_order_),
                        static_cast<QWidget *>(operator_), static_cast<QWidget *>(mac_), static_cast<QWidget *>(sn_)}) {
        input->setEnabled(editable);
    }
    const auto endpoint = Endpoint();
    read_->setEnabled(editable && !endpoint.address.isEmpty() && !endpoint.interface_id.isEmpty());
    diagnostic_->setEnabled(read_->isEnabled());
    close_->setEnabled(editable);

    if (current_.mac.isEmpty()) {
        identity_->clear();
    }
    else {
        identity_->setText(tr("%1 · MAC %2\n当前 SN：%3\n身份：%4（%5） · mDNS：%6%7")
                               .arg(current_.model, current_.mac,
                                    current_.sn.isEmpty() ? tr("未读到有效 SN") : current_.sn, current_.identity,
                                    current_.source, current_.mdns_running ? tr("运行中") : tr("未运行"),
                                    current_.simulation ? tr(" · 模拟环境") : QString()));
    }
    const auto order = Order();
    const auto decision = lidar_sn::CheckTarget(current_, order);
    const bool duplicate = client_.IsDuplicate(current_.sn, current_.mac) || client_.IsDuplicate(order.sn, order.mac);
    duplicate_->setVisible(duplicate);
    write_->setText(decision == lidar_sn::WriteDecision::KAlreadyWritten ? tr("核验已写入的 SN") : tr("写入 SN"));
    write_->setEnabled(read_->isEnabled() && online_read_ && !duplicate &&
                       decision != lidar_sn::WriteDecision::KBlocked);
    if (!current_.sn.isEmpty() && !order.sn.isEmpty() && current_.sn != order.sn) {
        write_->setText(tr("已有其他 SN，需人工核对"));
    }
    QString error;
    if (!sn_->text().trimmed().isEmpty() && order.sn.isEmpty()) {
        error = tr("SN 格式无效，请核对大写字母、生产周和当周序号。");
    }
    else if (!mac_->text().trimmed().isEmpty() && order.mac.isEmpty()) {
        error = tr("设备标签 MAC 格式无效。");
    }
    else if (!current_.mac.isEmpty() && !order.mac.isEmpty() && current_.mac != order.mac) {
        error = tr("标签 MAC 与在线设备不一致。");
    }
    else if ((!current_.model.isEmpty() &&
              current_.model.compare(QStringLiteral("CS96-beta"), Qt::CaseInsensitive) != 0) ||
             (!order.sn.isEmpty() && !order.sn.startsWith("CS96"))) {
        error = tr("此写码流程适用于 CS96 Beta。");
    }
    order_error_->setText(error);
    order_error_->setVisible(!error.isEmpty());
}

void SnDialog::ConfirmOrder()
{
    const auto order = Order();
    const auto decision = lidar_sn::CheckTarget(current_, order);
    if (!write_->isEnabled() || decision == lidar_sn::WriteDecision::KBlocked) {
        return;
    }
    QMessageBox confirmation(QMessageBox::Question, windowTitle(),
                             tr("请确认此 SN 已由 MES 分配、唯一，并与扫描的设备标签及工单一致。"),
                             QMessageBox::Yes | QMessageBox::No, this);
    confirmation.setTextFormat(Qt::PlainText);
    confirmation.setInformativeText(
        tr("工单 %1 · 操作员 %2\nMAC %3 · IPv4 %4\nSN %5\n型号码 %6 · 硬件版本 %7 · 组装地 %8 · 生产年周 %9")
            .arg(order.number, order.operator_id, order.mac, Endpoint().address, order.sn, order.sn.left(7),
                 order.sn.mid(7, 3), order.sn.at(10) == '0' ? tr("成都") : tr("德州"), order.sn.mid(11, 4)));
    confirmation.setDefaultButton(QMessageBox::No);
    if (confirmation.exec() != QMessageBox::Yes) {
        return;
    }
    online_read_ = false;
    client_.Start(Endpoint(), order, decision);
}

void SnDialog::reject()
{
    if (!client_.IsBusy()) {
        QDialog::reject();
    }
}

void SnDialog::closeEvent(QCloseEvent *event)
{
    if (client_.IsBusy()) {
        event->ignore();
        return;
    }
    QDialog::closeEvent(event);
}

} // namespace ys_scene
