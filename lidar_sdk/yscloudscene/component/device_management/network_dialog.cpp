#include "network_dialog.h"

#include "business/device_management/network_access.h"

#include <QComboBox>
#include <QElapsedTimer>
#include <QFormLayout>
#include <QFutureWatcher>
#include <QJsonDocument>
#include <QLabel>
#include <QLineEdit>
#include <QNetworkDatagram>
#include <QNetworkInterface>
#include <QPlainTextEdit>
#include <QPushButton>
#include <QSpinBox>
#include <QTimer>
#include <QUdpSocket>
#include <QtConcurrentRun>
#include <curl/curl.h>
#include <memory>

namespace ys_scene {
namespace {
using namespace lidar_network;

class Panel : public QWidget
{
public:
    explicit Panel(QWidget* parent);

private:
    enum class Request { KRead, KDiagnostic, KPreflight, KSet, KConfirm, KPoll };
    void Receive();
    void Send(Request kind, const Endpoint& endpoint, const QJsonObject& body = {});
    void Handle(Request kind, const Endpoint& endpoint, const Reply& reply);
    void Poll();
    void Start();
    void Update();
    Endpoint Selected() const;
    QString Source(const QString& address) const;

    QComboBox* nic_;
    QComboBox* devices_;
    QComboBox* mode_;
    QLineEdit *ip_, *first_, *last_, *mask_, *gateway_;
    QSpinBox* port_;
    QLabel* message_;
    QPlainTextEdit* snapshot_;
    QPushButton *read_, *diagnostic_, *apply_, *stop_;
    QUdpSocket udp_;
    QTimer timer_;
    QFutureWatcher<Reply> http_;
    QHash<QString, Endpoint> candidates_;
    Endpoint verified_;
    QJsonObject current_;
    Task task_;
    bool busy_ = false;
    bool waiting_ = false;
    int poll_index_ = 0;
    QElapsedTimer waiting_since_;
};

Panel::Panel(QWidget* parent)
    : QWidget(parent)
{
    auto* form = new QFormLayout(this);
    nic_ = new QComboBox(this);
    for (const auto& nic : QNetworkInterface::allInterfaces()) {
        if (!nic.flags().testFlag(QNetworkInterface::IsUp)) {
            continue;
        }
        QStringList addresses;
        for (const auto& entry : nic.addressEntries()) {
            if (entry.ip().protocol() == QAbstractSocket::IPv4Protocol) {
                addresses << QString("%1/%2").arg(entry.ip().toString()).arg(entry.prefixLength());
            }
        }
        if (!addresses.isEmpty()) {
            nic_->addItem(QString("%1 [%2] %3").arg(nic.humanReadableName()).arg(nic.index())
                              .arg(addresses.join(", ")), nic.name());
        }
    }
    devices_ = new QComboBox(this);
    ip_ = new QLineEdit(this);
    port_ = new QSpinBox(this);
    port_->setRange(1, 65535);
    port_->setValue(80);
    mode_ = new QComboBox(this);
    mode_->addItem(tr("自动获取"), "auto");
    mode_->addItem(tr("指定范围自动选址"), "pool");
    mode_->addItem(tr("固定 IP"), "static");
    first_ = new QLineEdit(this);
    last_ = new QLineEdit(this);
    mask_ = new QLineEdit(this);
    gateway_ = new QLineEdit(this);
    read_ = new QPushButton(tr("查询网络状态"), this);
    diagnostic_ = new QPushButton(tr("查询只读诊断"), this);
    apply_ = new QPushButton(tr("提交网络配置"), this);
    stop_ = new QPushButton(tr("停止等待"), this);
    message_ = new QLabel(this);
    message_->setWordWrap(true);
    snapshot_ = new QPlainTextEdit(this);
    snapshot_->setReadOnly(true);
    form->addRow(tr("电脑网卡"), nic_);
    form->addRow(tr("发现的设备"), devices_);
    form->addRow(tr("设备 IPv4"), ip_);
    form->addRow(tr("管理端口"), port_);
    form->addRow(read_, diagnostic_);
    form->addRow(tr("配置模式"), mode_);
    form->addRow(tr("固定地址或范围起点"), first_);
    form->addRow(tr("范围终点"), last_);
    form->addRow(tr("子网掩码"), mask_);
    form->addRow(tr("网关（可选）"), gateway_);
    form->addRow(apply_, stop_);
    form->addRow(message_);
    form->addRow(tr("设备快照"), snapshot_);

    connect(&udp_, &QUdpSocket::readyRead, this, &Panel::Receive);
    if (!udp_.bind(QHostAddress::AnyIPv4, kDiscoveryPort, QUdpSocket::ShareAddress | QUdpSocket::ReuseAddressHint)) {
        message_->setText(tr("无法监听 UDP 5354。仍可手动输入设备地址。"));
    }
    timer_.setInterval(900);
    connect(&timer_, &QTimer::timeout, this, &Panel::Poll);
    connect(nic_, qOverload<int>(&QComboBox::currentIndexChanged), this, [this] {
        devices_->clear();
        candidates_.clear();
        verified_ = {};
        current_ = {};
        snapshot_->clear();
        Update();
    });
    connect(devices_, qOverload<int>(&QComboBox::activated), this, [this](int index) {
        const auto entry = candidates_.value(devices_->itemData(index).toString());
        ip_->setText(entry.address);
        port_->setValue(entry.port);
    });
    connect(ip_, &QLineEdit::textChanged, this, &Panel::Update);
    connect(port_, qOverload<int>(&QSpinBox::valueChanged), this, &Panel::Update);
    connect(mode_, qOverload<int>(&QComboBox::currentIndexChanged), this, &Panel::Update);
    connect(read_, &QPushButton::clicked, this, [this] { Send(Request::KRead, Selected()); });
    connect(diagnostic_, &QPushButton::clicked, this, [this] { Send(Request::KDiagnostic, Selected()); });
    connect(apply_, &QPushButton::clicked, this, &Panel::Start);
    connect(stop_, &QPushButton::clicked, this, [this] {
        waiting_ = false;
        timer_.stop();
        verified_ = {};
        message_->setText(tr("已停止客户端等待，设备上的任务仍可能继续。"));
        Update();
    });
    Update();
}

Endpoint Panel::Selected() const
{
    quint32 ip;
    return {nic_->currentData().toString(),
            ParseIpv4(ip_->text(), ip) ? QHostAddress(ip).toString() : QString(), port_->value()};
}

void Panel::Receive()
{
    const auto nic = QNetworkInterface::interfaceFromName(nic_->currentData().toString());
    while (udp_.hasPendingDatagrams()) {
        const auto device = ParseAnnouncement(udp_.receiveDatagram(4097), nic);
        if (device.mac.isEmpty()) {
            continue;
        }
        const auto& endpoint = device.endpoint;
        candidates_.insert(device.mac, endpoint);
        const auto label = QString("%1 · %2 · %3:%4")
                               .arg(device.identity, device.mac, endpoint.address).arg(endpoint.port);
        const int index = devices_->findData(device.mac);
        if (index < 0) {
            devices_->addItem(label, device.mac);
        } else {
            devices_->setItemText(index, label);
        }
    }
}

void Panel::Send(Request kind, const Endpoint& endpoint, const QJsonObject& body)
{
    const QString source = SourceAddress(endpoint);
    if (busy_ || source.isEmpty()) {
        if (!busy_ && kind != Request::KPoll) {
            message_->setText(tr("所选网卡没有到 %1 的同网段 IPv4 地址。").arg(endpoint.address));
        }
        return;
    }
    busy_ = true;
    Update();
    connect(&http_, &QFutureWatcher<Reply>::finished, this, [this, kind, endpoint] {
        http_.disconnect(this);
        busy_ = false;
        Handle(kind, endpoint, http_.result());
        Update();
    });
    http_.setFuture(QtConcurrent::run(Query, endpoint, source, body, kind == Request::KDiagnostic));
}

void Panel::Handle(Request kind, const Endpoint& endpoint, const Reply& reply)
{
    const bool valid = reply.error.isEmpty() && IsSnapshot(reply.snapshot);
    if (!reply.snapshot.isEmpty()) {
        snapshot_->setPlainText(QString::fromUtf8(QJsonDocument(reply.snapshot).toJson()));
    }
    if (kind == Request::KRead || kind == Request::KDiagnostic) {
        verified_ = {};
        current_ = {};
        const auto expected_mac = devices_->currentData().toString();
        const auto discovered = candidates_.value(expected_mac);
        const bool identity_ok = discovered.address != endpoint.address || discovered.port != endpoint.port ||
                                 NormalizeMac(reply.snapshot.value("mac").toString()) == expected_mac;
        if (kind == Request::KRead && valid && reply.status == 200 && identity_ok &&
            reply.snapshot.value("address").toString() == endpoint.address) {
            verified_ = endpoint;
            current_ = reply.snapshot;
        }
        if (!identity_ok) {
            message_->setText(tr("HTTP 设备身份与发现结果不一致，已禁用写入。"));
        } else {
            message_->setText(valid && reply.status == 200
                                    ? (reply.snapshot.value("simulation").toBool() ? tr("模拟环境") : tr("已读取设备状态"))
                                    : tr("查询失败（HTTP %1） %2 %3").arg(reply.status).arg(reply.error,
                                          reply.snapshot.value("message").toString()));
        }
        return;
    }
    if (kind == Request::KSet) {
        if (valid && task_.Accept(int(reply.status), reply.snapshot)) {
            message_->setText(tr("设置已受理，正在寻找新地址。"));
        } else if (!reply.error.isEmpty() || reply.status == 202 || reply.status >= 500) {
            message_->setText(tr("设置结果未知。仅继续查询，未取得任务号时不会确认。"));
        } else {
            waiting_ = false;
            timer_.stop();
            message_->setText(tr("设备拒绝设置（HTTP %1） %2").arg(reply.status)
                                  .arg(reply.snapshot.value("message").toString()));
        }
        return;
    }
    if (kind == Request::KPreflight) {
        if (valid && reply.status == 200 && reply.snapshot.value("configuration_valid").toBool() &&
            !reply.snapshot.value("pending").toBool() && reply.snapshot.value("phase") != QJsonValue("FAULT") &&
            NormalizeMac(reply.snapshot.value("mac").toString()) == task_.mac &&
            reply.snapshot.value("identity").toString() == task_.identity &&
            reply.snapshot.value("epoch").toString() == task_.epoch &&
            reply.snapshot.value("operation_id").toString() == task_.previous_operation &&
            reply.snapshot.value("simulation").toBool() == task_.simulation &&
            reply.snapshot.value("address").toString() == endpoint.address) {
            Send(Request::KSet, endpoint, SetRequest(task_.requested));
        } else {
            waiting_ = false;
            timer_.stop();
            message_->setText(tr("提交前未能重新核对设备身份和任务状态，请重新查询。"));
        }
        return;
    }
    if (!waiting_ || !valid || reply.status != 200) {
        if (kind == Request::KConfirm && waiting_) {
            message_->setText(tr("确认响应不可用，继续查询同一任务。"));
        }
        return;
    }
    switch (task_.Evaluate(endpoint, reply.snapshot)) {
    case Decision::KConfirm:
        task_.confirmation_sent = true;
        message_->setText(tr("新地址身份和任务已核对，正在确认。设备剩余 %1 毫秒。")
                              .arg(reply.snapshot.value("confirm_remaining_ms").toInt()));
        Send(Request::KConfirm, endpoint, ConfirmRequest(task_.operation));
        return;
    case Decision::KCommitted:
        waiting_ = false;
        ip_->setText(endpoint.address);
        message_->setText(tr("配置已保存，设备地址 %1。").arg(endpoint.address));
        break;
    case Decision::KFailed:
        waiting_ = false;
        message_->setText(tr("设备报告配置失败或回滚（%1） %2")
                              .arg(reply.snapshot.value("operation_code").toInt())
                              .arg(reply.snapshot.value("error").toString()));
        break;
    case Decision::KStop:
        waiting_ = false;
        message_->setText(tr("设备、任务或当前地址无法核对，已停止自动确认。"));
        break;
    case Decision::KObserve:
        return;
    }
    timer_.stop();
}

void Panel::Poll()
{
    if (busy_ || !waiting_) {
        return;
    }
    if (waiting_since_.elapsed() >= 180000) {
        waiting_ = false;
        timer_.stop();
        message_->setText(tr("客户端等待结束，设备结果仍未知。可重新查询当前状态。"));
        Update();
        return;
    }
    QVector<Endpoint> options{task_.original};
    const auto discovered = candidates_.value(task_.mac);
    if (!discovered.address.isEmpty() && discovered.address != task_.original.address) {
        options.append(discovered);
    }
    const auto fixed = task_.requested.value("address").toString();
    if (!fixed.isEmpty() && fixed != task_.original.address && fixed != discovered.address) {
        options.append({task_.original.interface_id, fixed, task_.original.port});
    }
    Send(Request::KPoll, options.at(poll_index_++ % options.size()));
}

void Panel::Start()
{
    if (!apply_->isEnabled()) {
        return;
    }
    QJsonObject input{{"mode", mode_->currentData().toString()}};
    if (input.value("mode") != QJsonValue("auto")) {
        input.insert("netmask", mask_->text());
        input.insert("gateway", gateway_->text());
        if (input.value("mode") == QJsonValue("pool")) {
            input.insert("start", first_->text());
            input.insert("end", last_->text());
        } else {
            input.insert("address", first_->text());
        }
    }
    const auto config = NormalizeConfig(input);
    if (!config.error.isEmpty()) {
        message_->setText(config.error);
        return;
    }
    if (SourceAddress(verified_).isEmpty()) {
        message_->setText(tr("所选网卡无法访问当前设备，请重新查询。"));
        return;
    }
    if (input.value("mode") != QJsonValue("auto")) {
        const auto first = config.value.value(input.value("mode") == QJsonValue("pool") ? "start" : "address").toString();
        const auto last = config.value.value("end").toString(first);
        const auto source = SourceAddress({verified_.interface_id, first, verified_.port});
        if (source.isEmpty() || source != SourceAddress({verified_.interface_id, last, verified_.port})) {
            message_->setText(tr("所选网卡没有覆盖目标地址范围的本地 IPv4，请先配置电脑网卡。"));
            return;
        }
    }
    task_ = {verified_, NormalizeMac(current_.value("mac").toString()), current_.value("identity").toString(),
             current_.value("epoch").toString(), current_.value("operation_id").toString(), config.value,
             current_.value("simulation").toBool(), {}, false};
    waiting_ = true;
    waiting_since_.start();
    verified_ = {};
    poll_index_ = 0;
    Send(Request::KPreflight, task_.original);
    timer_.start();
}

void Panel::Update()
{
    const bool editable = !busy_ && !waiting_;
    nic_->setEnabled(editable);
    devices_->setEnabled(editable);
    ip_->setEnabled(editable);
    port_->setEnabled(editable);
    mode_->setEnabled(editable);
    const auto mode = mode_->currentData().toString();
    first_->setEnabled(editable && mode != "auto");
    last_->setEnabled(editable && mode == "pool");
    mask_->setEnabled(editable && mode != "auto");
    gateway_->setEnabled(editable && mode != "auto");
    const auto endpoint = Selected();
    read_->setEnabled(editable && !endpoint.address.isEmpty());
    diagnostic_->setEnabled(read_->isEnabled());
    stop_->setEnabled(waiting_ && !busy_);
    const auto transaction = current_.value("transaction_phase").toString();
    apply_->setEnabled(editable && !verified_.address.isEmpty() && endpoint.address == verified_.address &&
                       endpoint.port == verified_.port && endpoint.interface_id == verified_.interface_id &&
                       current_.value("configuration_valid").toBool() && !current_.value("pending").toBool() &&
                       (transaction == "IDLE" || transaction == "COMMITTED" || transaction == "FAILED" ||
                        transaction == "ROLLED_BACK") && current_.value("phase") != QJsonValue("FAULT"));
}
} // namespace

NetworkDialog::NetworkDialog(QWidget* parent)
    : QDialog(parent)
{
    setWindowTitle(tr("设备 IP 发现与配置"));
    resize(740, 650);
    auto* layout = new QFormLayout(this);
    layout->addRow(new Panel(this));
}

} // namespace ys_scene
