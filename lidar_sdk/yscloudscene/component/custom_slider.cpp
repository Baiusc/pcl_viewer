#include "custom_slider.h"
#include <QPainter>
#include <QStyle>
#include <QStyleOptionSlider>
#include <QTime>

namespace ys_scene {
CustomSlider::CustomSlider(QWidget* parent):
    QSlider(Qt::Horizontal, parent),
    realmin_(0),
    realmax_(90000),
    extrarange_(1500)
{
    setStyleSheet(
        "QSlider::groove:horizontal {"
        "    background: darkGray;"
        "    height: 1px;"
        "}"
        "QSlider::handle:horizontal {"
        "    background: transparent;"
        "    width: 1px;"
        "}"
    );
}

void CustomSlider::paintEvent(QPaintEvent* event)
{
    QSlider::paintEvent(event);

    QPainter painter(this);
    painter.setRenderHint(QPainter::Antialiasing);

    // Calculate the real slider rect
    extrarange_ = realmax_ * 0.02;
    QRect sliderRect = rect().adjusted(extrarange_ * width() / (maximum() - minimum()), 0,
        -extrarange_ * width() / (maximum() - minimum()), 0);

    // Draw background line
    painter.setPen(QPen(QColor("#1ECABC"), 1));
    painter.drawLine(sliderRect.left(), height() / 2, sliderRect.right(), height() / 2);

    // Draw filled part
    qreal position = (RealValue() - realmin_) * 1.0 / (realmax_ - realmin_);
    int filledWidth = static_cast<int>(position * sliderRect.width());
    painter.setPen(QPen(QColor("#e42303"), 1));
    painter.drawLine(sliderRect.left(), height() / 2, sliderRect.left() + filledWidth, height() / 2);

    // Draw ticks and labels
    painter.setPen(QPen(QColor("#1ECABC"), 1));
    int tickInterval, labelInterval;

    if (realmax_ <= 2000) {
        tickInterval = 200;
        labelInterval = 1;
    }
    else if (realmax_ <= 10000) {
        tickInterval = 1000;
        labelInterval = 2;
    }
    else if (realmax_ <= 300000) {
        tickInterval = 5000;
        labelInterval = 3;
    }
    else if (realmax_ <= 600000) {
        tickInterval = 30000;
        labelInterval = 3;
    }
    else {
        tickInterval = 60000;
        labelInterval = 3;
    }

    for (int i = 0; i <= realmax_/tickInterval; ++i) {
        int pos = QStyle::sliderPositionFromValue(realmin_, realmax_, i * tickInterval, sliderRect.width()) + sliderRect.left();

        if (pos <= sliderRect.left() + filledWidth) {
            // Draw completed ticks and labels in #e42303
            painter.setPen(QPen(QColor("#e42303"), 1));
            painter.drawLine(pos, height() - (i % labelInterval ? 25 : 20), pos, height());

            if (i % labelInterval == 0 || i == realmax_ / tickInterval) {
                QTime time(0, 0, 0, 0);
                time = time.addMSecs(i * tickInterval);
                painter.drawText(pos - 15, height() - 13, time.toString("m:ss"));
            }
        }
        else {
            // Draw remaining ticks and labels in gray
            painter.setPen(QPen(QColor("#1ECABC"), 1));
            painter.drawLine(pos, height() - (i % labelInterval ? 25 : 20), pos, height());

            if (i % labelInterval == 0) {
                painter.setPen(QColor("#1ECABC"));
                QTime time(0, 0, 0, 0);
                time = time.addMSecs(i * tickInterval);
                painter.drawText(pos - 15, height() - 13, time.toString("m:ss"));
            }
        }
    }

    // Draw end time label
    QTime endTime(0, 0, 0, 0);
    endTime = endTime.addMSecs(realmax_);
    painter.setPen(QColor("#1ECABC"));
    painter.drawText(sliderRect.right() - 30, height() - 13, endTime.toString("m:ss"));

    // Draw cursor
    painter.setPen(QPen(Qt::red, 2));
    int handlePos = QStyle::sliderPositionFromValue(realmin_, realmax_, RealValue(), sliderRect.width()) + sliderRect.left();
    painter.drawLine(handlePos, 0, handlePos, height());

    QPolygon triangle;
    triangle << QPoint(handlePos, 8) << QPoint(handlePos - 6, 0) << QPoint(handlePos + 6, 0);
    painter.setBrush(Qt::red);
    painter.drawPolygon(triangle);
}

void CustomSlider::mousePressEvent(QMouseEvent* event)
{
    if (event->button() == Qt::LeftButton) {
        int value = QStyle::sliderValueFromPosition(minimum(), maximum(), event->pos().x(), width());
        setValue(value);
        emit sliderPressed();
    }
    QSlider::mousePressEvent(event);
}

void CustomSlider::mouseMoveEvent(QMouseEvent* event)
{
    int value = QStyle::sliderValueFromPosition(minimum(), maximum(), event->pos().x(), width());
    setValue(value);
    emit sliderMoved(value);
    QSlider::mouseMoveEvent(event);
}

void CustomSlider::SetRealRange(int min, int max)
{
    realmin_ = min;
    realmax_ = max;
    setRange(realmin_ - extrarange_, realmax_ + extrarange_);
    update();
}

int CustomSlider::RealValue() const
{
    return qBound(realmin_, value(), realmax_);
}

void CustomSlider::SetProgressPosition(int value)
{
    if (value < 0 || value > realmax_ / 100) {
        return;
    }

    int progress = value * 100; // 将帧编号转换为毫秒数
    setValue(progress);
}

void CustomSlider::Reset()
{
    SetProgressPosition(0);
}

}// namespace ys_scene 