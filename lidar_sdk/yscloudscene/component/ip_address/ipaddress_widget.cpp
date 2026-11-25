#include "ipaddress_widget.h"
#include <QAction>
#include <QApplication>
#include <QBitArray>
#include <QClipboard>
#include <QDebug>
#include <QEvent>
#include <QHBoxLayout>
#include <QKeyEvent>
#include <QMenu>
#include <QPropertyAnimation>
#include <QSequentialAnimationGroup>
#include <QTimer>

namespace ys_scene {

IPAddressWidget::IPAddressWidget(QWidget *parent, bool editable, IPMode mode)
    : QWidget(parent),
      editable_(editable),
      tooltip_(nullptr),
      mode_(mode),
      inputCompleted_(false)
{
    // 创建延迟验证计时器
    validationTimer_ = new QTimer(this);
    validationTimer_->setSingleShot(true);
    validationTimer_->setInterval(200); // 0.8秒后验证
    connect(validationTimer_, &QTimer::timeout, this, &IPAddressWidget::ValidateWithDelay);

    SetupUI();
}

IPAddressWidget::~IPAddressWidget()
{
    if (tooltip_) {
        tooltip_->deleteLater();
    }
}

QString IPAddressWidget::GetIP() const
{
    return QString("%1.%2.%3.%4")
        .arg(octets_[0]->value())
        .arg(octets_[1]->value())
        .arg(octets_[2]->value())
        .arg(octets_[3]->value());
}

void IPAddressWidget::SetIP(const QString &ip)
{
    QStringList parts = ip.split('.');
    if (parts.size() != 4)
        return;

    // 标记为完成输入
    inputCompleted_ = true;

    // 添加一个小动画效果
    QSequentialAnimationGroup *anim_group = new QSequentialAnimationGroup(this);

    for (int i = 0; i < 4; ++i) {
        QPropertyAnimation *anim = new QPropertyAnimation(octets_[i], "value");
        anim->setDuration(200);
        anim->setStartValue(octets_[i]->value());
        anim->setEndValue(parts[i].toInt());
        anim->setEasingCurve(QEasingCurve::OutQuad);
        anim_group->addAnimation(anim);
    }

    connect(anim_group, &QSequentialAnimationGroup::finished, this, &IPAddressWidget::ValidateAndEmit);
    anim_group->start(QAbstractAnimation::DeleteWhenStopped);
}

bool IPAddressWidget::IsValid() const
{
    return GetInvalidMessage().isEmpty();
}

QString IPAddressWidget::GetInvalidMessage() const
{
    int first = octets_[0]->value();
    int second = octets_[1]->value();
    int third = octets_[2]->value();
    int fourth = octets_[3]->value();

    if (mode_ == IPMode::SubnetMask) {
        // 子网掩码验证
        return ValidateSubnetMask(first, second, third, fourth);
    }
    else {
        // 普通IP地址验证
        return ValidateNormalIP(first, second, third, fourth);
    }
}

QString IPAddressWidget::ValidateNormalIP(int first, int second, int third, int fourth) const
{
    // 输入未完成时的宽松验证
    if (!inputCompleted_) {
        // 对于未完成的输入，只做基本的范围检查
        if (first > 255 || second > 255 || third > 255 || fourth > 255) {
            return "每部分值应在0-255之间";
        }
        return "";
    }

    // 完整输入的严格验证
    // 检查每个位置的范围
    if (first == 0)
        return "无效IP: 0.x.x.x 是保留地址";

    if (first == 127)
        return "无效IP: 127.x.x.x 是回环地址";

    if (first == 255 && second == 255 && third == 255 && fourth == 255)
        return "无效IP: 255.255.255.255 是广播地址";

    if (first > 223)
        return "无效IP: 超出有效范围 (1-223)";

    // 检查特殊地址段
    if (first == 169 && second == 254)
        return "无效IP: 169.254.x.x 是链路本地地址";

    if (first == 192 && second == 0 && third == 0)
        return "无效IP: 192.0.0.x 是IETF协议分配";

    if (first == 192 && second == 88 && third == 99)
        return "无效IP: 192.88.99.x 是IPv6到IPv4中继";

    // 如果需要禁止使用私有IP，可以取消下面的注释
    // if (first == 10 ||
    //     (first == 172 && second >= 16 && second <= 31) ||
    //     (first == 192 && second == 168))
    //     return "无效IP: 私有网络地址";

    return ""; // IP有效，返回空字符串
}

QString IPAddressWidget::ValidateSubnetMask(int first, int second, int third, int fourth) const
{
    // 输入未完成时的宽松验证
    if (!inputCompleted_) {
        // 对于未完成的输入，只做基本的范围检查
        if (first > 255 || second > 255 || third > 255 || fourth > 255) {
            return "每部分值应在0-255之间";
        }
        return "";
    }

    // 子网掩码必须是连续的1后跟连续的0
    unsigned int mask = (first << 24) | (second << 16) | (third << 8) | fourth;

    // 创建一个位数组，用于检查连续性
    QBitArray bits(32);
    for (int i = 0; i < 32; i++) {
        bits.setBit(i, (mask & (1 << (31 - i))));
    }

    // 找到第一个0的位置
    int firstZero = 0;
    while (firstZero < 32 && bits.testBit(firstZero)) {
        firstZero++;
    }

    // 检查从firstZero开始的所有位是否都是0
    for (int i = firstZero; i < 32; i++) {
        if (bits.testBit(i)) {
            return "无效子网掩码: 必须是连续的1后跟连续的0";
        }
    }

    // 特殊情况检查
    if (mask == 0) {
        return "无效子网掩码: 不能全为0";
    }

    // 常见子网掩码检查（可选）
    if (firstZero > 0 && firstZero < 8) {
        return "警告: 不常见的子网掩码，推荐使用255.0.0.0";
    }
    else if (firstZero > 8 && firstZero < 16) {
        return "警告: 不常见的子网掩码，推荐使用255.255.0.0";
    }
    else if (firstZero > 16 && firstZero < 24) {
        return "警告: 不常见的子网掩码，推荐使用255.255.255.0";
    }

    return "";
}

void IPAddressWidget::SetMode(IPMode mode)
{
    mode_ = mode;

    // 更新UI提示和验证
    if (mode_ == IPMode::SubnetMask) {
        setPlaceholderText("255.255.255.0"); // 设置示例子网掩码

        // 为子网掩码模式添加上下文菜单
        for (int i = 0; i < 4; ++i) {
            octets_[i]->setContextMenuPolicy(Qt::CustomContextMenu);
            connect(octets_[i], &QWidget::customContextMenuRequested, this, &IPAddressWidget::ShowSubnetMenu);

            // 添加工具提示
            octets_[i]->setToolTip("右键点击查看常用子网掩码");
        }
    }
    else {
        setPlaceholderText("192.168.1.1"); // 设置示例IP
    }

    // 重新验证当前值
    inputCompleted_ = true;
    ValidateAndEmit();
}

IPAddressWidget::IPMode IPAddressWidget::GetMode() const
{
    return mode_;
}

void IPAddressWidget::setPlaceholderText(const QString &text)
{
    QStringList parts = text.split('.');
    if (parts.size() != 4)
        return;

    for (int i = 0; i < 4 && i < parts.size(); ++i) {
        if (editable_) {
            octets_[i]->setSpecialValueText(parts[i]);
        }
    }
}

void IPAddressWidget::ShowSubnetMenu(const QPoint &pos)
{
    if (mode_ != IPMode::SubnetMask || !editable_)
        return;

    QMenu menu(this);
    menu.setStyleSheet(R"(
        QMenu {
            background-color: #333333;
            color: #e0e0e0;
            border: 1px solid #444444;
            border-radius: 4px;
        }
        QMenu::item {
            padding: 6px 20px;
        }
        QMenu::item:selected {
            background-color: #4a9eff;
        }
    )");

    QAction *mask8 = menu.addAction("255.0.0.0 (/8)");
    QAction *mask16 = menu.addAction("255.255.0.0 (/16)");
    QAction *mask24 = menu.addAction("255.255.255.0 (/24)");
    QAction *mask30 = menu.addAction("255.255.255.252 (/30)");
    QAction *mask32 = menu.addAction("255.255.255.255 (/32)");

    QObject::connect(mask8, &QAction::triggered, this, [this]() { SetIP("255.0.0.0"); });

    QObject::connect(mask16, &QAction::triggered, this, [this]() { SetIP("255.255.0.0"); });

    QObject::connect(mask24, &QAction::triggered, this, [this]() { SetIP("255.255.255.0"); });

    QObject::connect(mask30, &QAction::triggered, this, [this]() { SetIP("255.255.255.252"); });

    QObject::connect(mask32, &QAction::triggered, this, [this]() { SetIP("255.255.255.255"); });

    menu.exec(static_cast<QWidget *>(sender())->mapToGlobal(pos));
}

bool IPAddressWidget::eventFilter(QObject *obj, QEvent *event)
{
    // 当焦点离开任何八位组输入框时，视为完成输入
    if (event->type() == QEvent::FocusOut) {
        for (int i = 0; i < octets_.size(); ++i) {
            if (obj == octets_[i]) {
                // 检查焦点是否转移到了另一个八位组
                bool focusStaysInWidget = false;
                for (int j = 0; j < octets_.size(); ++j) {
                    if (octets_[j]->hasFocus()) {
                        focusStaysInWidget = true;
                        break;
                    }
                }

                if (!focusStaysInWidget) {
                    // 焦点离开了整个控件，标记为完成输入
                    inputCompleted_ = true;
                    validationTimer_->stop();
                    ValidateAndEmit();
                }
                break;
            }
        }
    }

    return QWidget::eventFilter(obj, event);
}

void IPAddressWidget::ValidateWithDelay()
{
    // 当延时验证触发时，标记为完成输入
    inputCompleted_ = true;
    ValidateAndEmit();
}

void IPAddressWidget::SetupUI()
{
    QHBoxLayout *layout = new QHBoxLayout(this);
    layout->setSpacing(2); // 让各部分靠得更紧密
    layout->setContentsMargins(1, 1, 1, 1);

    // 创建4个数字框
    for (int i = 0; i < 4; ++i) {
        QSpinBox *octet = new QSpinBox(this);
        octet->setRange(0, 255);
        octet->setButtonSymbols(QAbstractSpinBox::NoButtons); // 不要上下箭头
        octet->setAlignment(Qt::AlignCenter);
        octet->setFixedWidth(50); // 固定宽度，看起来更整齐

        // 来个超酷的新样式！✨
        QString style = editable_ ?
                                  R"(
            QSpinBox {
                border: none;
                border-radius: 6px;  /* 圆润的边角 */
                background: #2D2D2D;  /* 暗色背景 */
                color: #E0E0E0;      /* 亮色文字 */
                padding: 4px;
                font-size: 13px;
            }

            QSpinBox:hover {
                background: #353535;  /* 悬停时稍微变亮 */
                border: 1px solid #3D3D3D;
            }

            QSpinBox:focus {
                background: #383838;  /* 获得焦点时的背景 */
                border: 1px solid #4A4A4A;
            }
            )"
                                  :
                                  R"(
            QSpinBox {
                border: none;
                border-radius: 6px;
                background: #2D2D2D;
                color: #E0E0E0;
                padding: 4px;
                font-size: 13px;
            }
            )";

        octet->setStyleSheet(style);

        if (!editable_) {
            octet->setReadOnly(true);
            octet->setButtonSymbols(QAbstractSpinBox::NoButtons);
        }

        // 添加事件过滤器
        octet->installEventFilter(this);

        // 连接编辑完成信号
        connect(octet, &QSpinBox::editingFinished, this, [this]() {
            // 当用户完成一个八位组的编辑时，延迟验证
            validationTimer_->stop();
            validationTimer_->start();
        });

        // 当数字超过100时自动跳到下一个框
        if (i < 3) {
            connect(octet, &QSpinBox::valueChanged, this, [this, i, octet](int value) {
                if (value > 99 && octet->hasFocus()) {
                    octets_[i + 1]->setFocus();
                }

                // 重置验证计时器
                validationTimer_->stop();
                validationTimer_->start();

                // 如果正在输入，暂时不显示错误提示
                inputCompleted_ = false;
            });
        }
        else {
            // 最后一个输入框改变值时，准备进行验证
            connect(octet, &QSpinBox::valueChanged, this, [this](int) {
                validationTimer_->stop();
                validationTimer_->start();
                inputCompleted_ = false;
            });
        }

        // 事件过滤器类定义
        class OctetEventFilter : public QObject
        {
          public:
            OctetEventFilter(IPAddressWidget *widget, int index)
                : QObject(widget),
                  widget_(widget),
                  index_(index)
            {
            }
          protected:
            bool eventFilter(QObject *obj, QEvent *event) override
            {
                if (event->type() == QEvent::KeyPress) {
                    QKeyEvent *keyEvent = static_cast<QKeyEvent *>(event);
                    // 处理点号和回车键
                    if ((keyEvent->key() == Qt::Key_Period || keyEvent->key() == Qt::Key_Return) && index_ < 3) {
                        widget_->octets_[index_ + 1]->setFocus();
                        return true;
                    }
                    // 处理左右方向键
                    if (keyEvent->key() == Qt::Key_Left && index_ > 0) {
                        widget_->octets_[index_ - 1]->setFocus();
                        return true;
                    }
                    if (keyEvent->key() == Qt::Key_Right && index_ < 3) {
                        widget_->octets_[index_ + 1]->setFocus();
                        return true;
                    }
                }
                return QObject::eventFilter(obj, event);
            }
          private:
            IPAddressWidget *widget_;
            int index_;
        };

        // 安装键盘事件过滤器
        octet->installEventFilter(new OctetEventFilter(this, i));

        // 添加到控件列表
        octets_.append(octet);
        layout->addWidget(octet);

        // 添加点号（最后一个不加）
        if (i < 3) {
            QLabel *dot = new QLabel(".");
            // 点号也要时尚一下！
            dot->setStyleSheet("color: #4A9EFF; font-weight: bold; font-size: 15px;");
            dots_.append(dot);
            layout->addWidget(dot);
        }
    }

    // 设置TAB顺序
    for (int i = 0; i < 3; ++i) {
        setTabOrder(octets_[i], octets_[i + 1]);
    }

    // 给整个控件添加一个好看的容器样式
    setStyleSheet(R"(
        IPAddressWidget {
            background: #242424;
            border-radius: 8px;
            padding: 4px;
        }
    )");

    // 添加右键菜单支持
    if (mode_ == IPMode::SubnetMask) {
        for (int i = 0; i < 4; ++i) {
            octets_[i]->setContextMenuPolicy(Qt::CustomContextMenu);
            connect(octets_[i], &QWidget::customContextMenuRequested, this, &IPAddressWidget::ShowSubnetMenu);

            // 添加工具提示
            if (i == 0)
                octets_[i]->setToolTip("第1部分: 通常为255");
            else if (i == 1)
                octets_[i]->setToolTip("第2部分: 常用值 255, 0");
            else if (i == 2)
                octets_[i]->setToolTip("第3部分: 常用值 255, 0");
            else
                octets_[i]->setToolTip("第4部分: 常用值 0, 255, 252, 248等");
        }
    }

    // 根据模式设置默认值
    if (mode_ == IPMode::SubnetMask) {
        SetIP("255.255.255.0");
    }
    else {
        SetIP("192.168.1.1");
    }
}

void IPAddressWidget::ValidateAndEmit()
{
    QString ip = GetIP();

    // 只有当用户完成输入时才显示错误提示
    if (inputCompleted_) {
        QString errorMessage = GetInvalidMessage();
        if (!errorMessage.isEmpty()) {
            ShowTooltip(errorMessage);
        }
        else {
            // 如果提示正在显示，则隐藏它
            if (tooltip_ && tooltip_->isVisible()) {
                HideTooltip();
            }
        }
    }

    // 无论IP是否有效，都发出信号
    emit IpChanged(ip);
}

void IPAddressWidget::CreateTooltip()
{
    // 创建提示框
    tooltip_ = new QWidget(nullptr); // 使用nullptr而不是this，使其成为顶级窗口
    tooltip_->setWindowFlags(Qt::ToolTip | Qt::FramelessWindowHint);
    tooltip_->setAttribute(Qt::WA_TranslucentBackground);
    tooltip_->setStyleSheet(R"(
        background-color: #FF4A4A;
        border-radius: 5px;
        color: white;
        font-weight: bold;
        border: 1px solid #FF6767;
    )");

    QHBoxLayout *layout = new QHBoxLayout(tooltip_);
    layout->setContentsMargins(10, 6, 10, 6);

    QLabel *icon = new QLabel();
    icon->setText("⚠️");
    icon->setStyleSheet("font-size: 14px; border: none;");

    QLabel *text = new QLabel();
    text->setObjectName("tooltipText");
    text->setStyleSheet("font-size: 12px; border: none;");
    text->setWordWrap(true); // 支持长文本换行

    layout->addWidget(icon);
    layout->addWidget(text);

    tooltip_->hide();
}

void IPAddressWidget::ShowTooltip(const QString &message)
{
    if (!tooltip_) {
        CreateTooltip();
    }

    QLabel *textLabel = tooltip_->findChild<QLabel *>("tooltipText");
    textLabel->setText(message);
    tooltip_->adjustSize();

    // 限制提示框最大宽度
    if (tooltip_->width() > 300) {
        tooltip_->setFixedWidth(300);
        tooltip_->adjustSize();
    }

    // 将提示框定位在控件下方
    QPoint pos = mapToGlobal(QPoint(width() / 2 - tooltip_->width() / 2, height()));
    tooltip_->move(pos);

    // 带动画显示
    QPropertyAnimation *anim = new QPropertyAnimation(tooltip_, "windowOpacity");
    anim->setDuration(200);
    anim->setStartValue(0.0);
    anim->setEndValue(1.0);
    anim->setEasingCurve(QEasingCurve::OutQuad);

    tooltip_->show();
    anim->start(QAbstractAnimation::DeleteWhenStopped);

    // 3秒后自动隐藏
    QTimer::singleShot(3000, this, &IPAddressWidget::HideTooltip);
}

void IPAddressWidget::HideTooltip()
{
    if (tooltip_ && tooltip_->isVisible()) {
        QPropertyAnimation *anim = new QPropertyAnimation(tooltip_, "windowOpacity");
        anim->setDuration(200);
        anim->setStartValue(1.0);
        anim->setEndValue(0.0);
        anim->setEasingCurve(QEasingCurve::InQuad);

        connect(anim, &QPropertyAnimation::finished, tooltip_, &QWidget::hide);
        anim->start(QAbstractAnimation::DeleteWhenStopped);
    }
}

} // namespace ys_scene