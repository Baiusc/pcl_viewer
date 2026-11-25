#include "mode_management_widget.h"
#include <QComboBox>
#include <QFormLayout>
#include <QHideEvent>
#include <QLabel>
#include <QLineEdit>
#include <QPushButton>
#include <QShowEvent>
#include <QSignalBlocker>

namespace ys_scene {
ModeManagementWidget::ModeManagementWidget(QWidget *parent)
    : QWidget(parent)
{
    auto *layout = new QFormLayout(this);
    layout->addRow(new QLabel(tr("设备模式管理"), this));
    address_ = new QLineEdit(this);
    address_->setPlaceholderText(tr("设备 IP 或 http://地址:端口"));
    auto *query = new QPushButton(tr("查询"), this);
    layout->addRow(address_, query);
    modes_ = new QComboBox(this);
    apply_ = new QPushButton(tr("执行切换"), this);
    layout->addRow(modes_, apply_);
    state_ = new QLabel(tr("请选择设备"), this);
    result_ = new QLabel(this);
    for (auto *label : {state_, result_}) {
        label->setWordWrap(true);
        label->setTextFormat(Qt::PlainText);
        label->setTextInteractionFlags(Qt::TextSelectableByMouse);
        layout->addRow(label);
    }
    connect(query, &QPushButton::clicked, this, [this] {
        client_.SetAddress(address_->text());
        client_.SetActive(true);
        client_.Refresh();
    });
    connect(address_, &QLineEdit::textEdited, this, [this] { client_.SetActive(false); });
    connect(address_, &QLineEdit::returnPressed, query, &QPushButton::click);
    connect(apply_, &QPushButton::clicked, this, [this] { client_.Switch(modes_->currentData().toInt()); });
    connect(&client_, &ManagementClient::Changed, this, &ModeManagementWidget::Update);
    connect(&client_, &ManagementClient::Message, result_, &QLabel::setText);
    Update();
}

void ModeManagementWidget::SetAddress(const QString &address)
{
    address_->setText(address);
    client_.SetAddress(address);
}

void ModeManagementWidget::showEvent(QShowEvent *event)
{
    QWidget::showEvent(event);
    client_.SetActive(true);
}

void ModeManagementWidget::hideEvent(QHideEvent *event)
{
    client_.SetActive(false);
    QWidget::hideEvent(event);
}

void ModeManagementWidget::Update()
{
    const auto &s = client_.Snapshot();
    const int selected = modes_->currentIndex() < 0 ? s.actual : modes_->currentData().toInt();
    QSignalBlocker blocker(modes_);
    modes_->clear();
    for (int mode : s.modes)
        modes_->addItem(ManagementModeName(mode), mode);
    const int index = modes_->findData(selected);
    if (index >= 0)
        modes_->setCurrentIndex(index);
    modes_->setEnabled(client_.CanSwitch());
    apply_->setEnabled(client_.CanSwitch() && modes_->count() > 0);
    if (s.operation_id.isEmpty()) {
        state_->setText(tr("尚无管理状态"));
        return;
    }
    state_->setText(QStringLiteral("%1%2 / %3\n当前：%4\n阶段：%5\n设备信息：%6\nSN 尚未接入管理接口")
                        .arg((client_.IsFresh() ? QString() : QStringLiteral("【非实时】")) +
                                 (s.simulation ? QStringLiteral("【模拟环境】") : QString()),
                             s.model, s.version, ManagementModeName(s.actual), s.phase,
                             s.error.isEmpty() ? QStringLiteral("无错误") : s.error));
    state_->setToolTip(QStringLiteral("operation_id: %1\nconfirmed: %2\ndesired: %3")
                           .arg(s.operation_id)
                           .arg(s.confirmed)
                           .arg(s.desired));
}
} // namespace ys_scene
