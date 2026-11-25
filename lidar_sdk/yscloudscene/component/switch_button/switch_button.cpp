#include "switch_button.h"
#include "log/logging.h"

namespace ys_scene {

    SwitchButton::SwitchButton(QWidget* parent) :
        QWidget(parent),
        currentState(State::Off),
        onText("On"),
        offText("Off") {
        setMinimumSize(40, 20);
    }

    SwitchButton::State SwitchButton::getState() const {
        return currentState;
    }

    void SwitchButton::setState(State state) {
        if (currentState != state) {
            currentState = state;
            LOG_WARN("the current state is:{}", static_cast<int>(currentState));

            update();
        }
    }

    void SwitchButton::setOnText(const QString& text) {
        onText = text;
        update();
    }

    void SwitchButton::setOffText(const QString& text) {
        offText = text;
        update();
    }

    QSize SwitchButton::sizeHint() const {
        return QSize(40, 20);
    }

    void SwitchButton::paintEvent(QPaintEvent*) {
        QPainter painter(this);
        painter.setRenderHint(QPainter::Antialiasing);

        // 根据状态决定颜色
        QColor backgroundColor;
        switch (currentState) {
        case State::On:
            backgroundColor = QColor("#5ead53");  // 绿
            break;
        case State::Pending:
            backgroundColor = QColor("#C8A27A");  // 橙，表示等待状态
            break;
        case State::Off:
            backgroundColor = QColor("#dd6f4e");  // 红
            break;
        }

        // 绘制背景
        QRect backgroundRect = rect();
        painter.setBrush(backgroundColor);
        painter.setPen(Qt::NoPen);
        painter.drawRoundedRect(backgroundRect, height() / 2, height() / 2);

        // 绘制滑块
        int diameter = height() - 6;
        int x = (currentState == State::On) ? width() - diameter - 3 : 3;
        QRect knobRect(x, 3, diameter, diameter);
        painter.setBrush(QColor("#d7dfd3"));
        painter.drawEllipse(knobRect);

        // 绘制文字
        painter.setPen(Qt::white);
        painter.setFont(QFont("Arial", height() / 3));
        QString text = (currentState == State::On) ? onText : offText;

        int padding = 5;
        QRect textRect = rect().adjusted(padding, 0, -padding, 0);

        int flag = (currentState == State::On) ?
            (Qt::AlignVCenter | Qt::AlignLeft) :
            (Qt::AlignVCenter | Qt::AlignRight);

        // Pending 状态下不显示文字
        if (currentState != State::Pending) {
            painter.drawText(textRect, flag, text);
        }
    }

    void SwitchButton::mousePressEvent(QMouseEvent* event) {
        if (event->button() == Qt::LeftButton && currentState != State::Pending) {
            State oldState = currentState;
            setState(State::Pending);

            bool requestedState = (oldState == State::Off);
            //LOG_WARN("the current state is:{}", static_cast<int>(requestedState));

            emit stateChangeRequested(requestedState);
        }
    }

    void SwitchButton::resizeEvent(QResizeEvent* event) {
        QWidget::resizeEvent(event);
        update();
    }

} // namespace ys_scene
