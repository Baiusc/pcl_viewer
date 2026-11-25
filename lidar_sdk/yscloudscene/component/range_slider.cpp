#include "range_slider.h"

#include <QPainter>

#include <algorithm>
#include <cmath>

namespace ys_scene {

RangeSlider::RangeSlider(QWidget* parent)
    : QWidget(parent),
      min_limit_(0),
      max_limit_(100),
      min_value_(0),
      max_value_(100),
      active_handle_(-1)
{
    setMinimumWidth(120);
}

void RangeSlider::SetLimits(int min_limit, int max_limit)
{
    if (max_limit <= min_limit) {
        return;
    }
    min_limit_ = min_limit;
    max_limit_ = max_limit;
    SetValues(min_value_, max_value_);
}

void RangeSlider::SetValues(int min_value, int max_value)
{
    // 钳制到合法总范围内，并保证区间非空（min < max）
    min_value = std::clamp(min_value, min_limit_, max_limit_);
    max_value = std::clamp(max_value, min_limit_, max_limit_);
    if (min_value >= max_value) {
        if (min_value >= max_limit_) {
            min_value = max_limit_ - 1;
        }
        max_value = min_value + 1;
    }
    if (min_value != min_value_ || max_value != max_value_) {
        min_value_ = min_value;
        max_value_ = max_value;
        update();
    }
}

int RangeSlider::MinimumValue() const
{
    return min_value_;
}

int RangeSlider::MaximumValue() const
{
    return max_value_;
}

QSize RangeSlider::sizeHint() const
{
    return QSize(240, 30);
}

QSize RangeSlider::minimumSizeHint() const
{
    return QSize(120, 24);
}

void RangeSlider::paintEvent(QPaintEvent* event)
{
    Q_UNUSED(event);
    QPainter painter(this);
    painter.setRenderHint(QPainter::Antialiasing);

    const int track_height = 4;
    const int center_y = height() / 2;
    const int left = kHandleRadius;
    const int right = width() - kHandleRadius;
    if (right <= left) {
        return;
    }

    // 轨道
    painter.setPen(Qt::NoPen);
    painter.setBrush(QColor(60, 64, 74));
    painter.drawRoundedRect(
        QRectF(left, center_y - track_height / 2.0, right - left, track_height), 2, 2);

    // 选区高亮
    const int x1 = ValueToPos(min_value_);
    const int x2 = ValueToPos(max_value_);
    painter.setBrush(QColor(120, 190, 230));
    painter.drawRoundedRect(
        QRectF(x1, center_y - track_height / 2.0, x2 - x1, track_height), 2, 2);

    // 手柄
    painter.setPen(QPen(QColor(160, 168, 180), 1));
    painter.setBrush(QColor(240, 242, 246));
    painter.drawEllipse(QPointF(x1, center_y), kHandleRadius, kHandleRadius);
    painter.drawEllipse(QPointF(x2, center_y), kHandleRadius, kHandleRadius);
}

void RangeSlider::mousePressEvent(QMouseEvent* event)
{
    if (event->button() != Qt::LeftButton) {
        return;
    }
    const int pos = event->pos().x();
    const int dist_min = std::abs(pos - ValueToPos(min_value_));
    const int dist_max = std::abs(pos - ValueToPos(max_value_));
    active_handle_ = (dist_min <= dist_max) ? 0 : 1;
    DragTo(pos);
}

void RangeSlider::mouseMoveEvent(QMouseEvent* event)
{
    if (active_handle_ >= 0) {
        DragTo(event->pos().x());
    }
}

void RangeSlider::mouseReleaseEvent(QMouseEvent* event)
{
    if (event->button() == Qt::LeftButton) {
        active_handle_ = -1;
    }
}

int RangeSlider::ValueToPos(int value) const
{
    const int span = max_limit_ - min_limit_;
    const int usable = width() - kHandleDiameter;
    if (span <= 0 || usable <= 0) {
        return kHandleRadius;
    }
    const double ratio = static_cast<double>(value - min_limit_) / span;
    return kHandleRadius + static_cast<int>(std::lround(ratio * usable));
}

int RangeSlider::PosToValue(int pos) const
{
    const int span = max_limit_ - min_limit_;
    const int usable = width() - kHandleDiameter;
    if (span <= 0 || usable <= 0) {
        return min_limit_;
    }
    const double ratio = std::clamp(static_cast<double>(pos - kHandleRadius) / usable, 0.0, 1.0);
    return min_limit_ + static_cast<int>(std::lround(ratio * span));
}

void RangeSlider::DragTo(int pos)
{
    int new_min = min_value_;
    int new_max = max_value_;
    if (active_handle_ == 0) {
        new_min = std::clamp(PosToValue(pos), min_limit_, max_value_ - 1);
    } else {
        new_max = std::clamp(PosToValue(pos), min_value_ + 1, max_limit_);
    }
    if (new_min != min_value_ || new_max != max_value_) {
        min_value_ = new_min;
        max_value_ = new_max;
        update();
        emit RangeChanged(min_value_, max_value_);
    }
}

}// namespace ys_scene
