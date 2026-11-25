#include "colormapper.h"
#include "config/config_service.h"
#include <QDebug>

namespace ys_scene {
ColorMapper::ColorMapper(QWidget* parent)
    : QWidget{ parent },
    a{ 50 },
    d{ 250 },
    startx_{ 50 },
    endy_{ 250 },
    sense_value_{ 10 },
    dis_color_interval_{ 150 },
    distance_{ 240 },
    color_max_{ 255 },
    color_interval_{ 50 }
{
    chroma_img_ = new QImage(":/image/chromatography.png");
    if (chroma_img_ == nullptr) {
        qDebug() << "Failed to load image";
    }
    auto& config = you_see::ConfigService::Instance();
    // 映射距离值
    int a = config.Get<int>("distance.end", 240);
    int b = config.Get<int>("distance.far", 100);
    int c = config.Get<int>("distance.close", 20);
    int d = config.Get<int>("distance.start", 0);

    // 计算距离值在ColorMapper中的位置
    int distance_range = endy_ - startx_;
    int a_pos = endy_ - static_cast<int>((a - d) * 1.0 * distance_range / (distance_ - d) + 0.5);
    int b_pos = endy_ - static_cast<int>((b - d) * 1.0 * distance_range / (distance_ - d) + 0.5);
    int c_pos = endy_ - static_cast<int>((c - d) * 1.0 * distance_range / (distance_ - d) + 0.5);
    int d_pos = endy_;
    // 映射颜色值
    int e = config.Get<int>("distance_color.far_end", 100);   
    int f = config.Get<int>("distance_color.far_begin", 50);    
    int g = config.Get<int>("distance_color.middle_end", 50);
    int h = config.Get<int>("distance_color.middle_begin", 0);
    int i = config.Get<int>("distance_color.close_end", 255);
    int j = config.Get<int>("distance_color.close_begin", 50);

    // 计算颜色值在ColorMapper中的位置
    int color_range = endy_ - startx_;
    int e_pos = startx_ + (color_max_ - e) * color_range / color_max_;
    int f_pos = startx_ + (color_max_ - f) * color_range / color_max_;
    int g_pos = startx_ + (color_max_ - g) * color_range / color_max_;
    int h_pos = startx_ + (color_max_ - h) * color_range / color_max_;
    int i_pos = startx_ + (color_max_ - i) * color_range / color_max_;
    int j_pos = startx_ + (color_max_ - j) * color_range / color_max_;

    distance_sliders_.emplace_back(a_pos, "end", static_cast<uint16_t>(Qt::red), PointDistance::kEndPos);
    distance_sliders_.emplace_back(b_pos, "far", static_cast<uint16_t>(Qt::green), PointDistance::kFarPos);
    distance_sliders_.emplace_back(c_pos, "close", static_cast<uint16_t>(Qt::blue), PointDistance::kClosePos);
    distance_sliders_.emplace_back(d_pos, "start", static_cast<uint16_t>(Qt::magenta), PointDistance::kStartPos);

    color_sliders_.emplace_back(e_pos, "", static_cast<uint16_t>(Qt::red), DistanceColor::kFarEndColor);
    color_sliders_.emplace_back(f_pos, "", static_cast<uint16_t>(Qt::green), DistanceColor::kFarStartColor);
    color_sliders_.emplace_back(g_pos, "", static_cast<uint16_t>(Qt::green), DistanceColor::kMiddleEndColor);
    color_sliders_.emplace_back(h_pos, "", static_cast<uint16_t>(Qt::blue), DistanceColor::kMiddleStartColor);
    color_sliders_.emplace_back(i_pos, "", static_cast<uint16_t>(Qt::blue), DistanceColor::kCloseEndColor);
    color_sliders_.emplace_back(j_pos, "", static_cast<uint16_t>(Qt::magenta), DistanceColor::kCloseStartColor);

    setMouseTracking(true);
    setGeometry(0, 0, 365, 275);
    setMinimumSize(365, 275);
}

ColorMapper::ColorMapper(DisMapList distance, DisColorMapList color, QWidget* parent)
    : QWidget{ parent },
    a{ 50 },
    d{ 250 },
    startx_{ 50 },
    endy_{ 250 },
    sense_value_{ 10 },
    dis_color_interval_{ 150 },
    distance_{ 240 },
    color_max_{ 255 },
    color_interval_{ 50 },
    distance_sliders_{ distance },
    color_sliders_{ color }
{
    chroma_img_ = new QImage("E:\\workspace\\ColorMapeer\\image\\chromatography.png");
    if (chroma_img_ == nullptr) {
        qDebug() << "Failed to load image";
    }
    setMouseTracking(true);
    setGeometry(0, 0, 365, 275);
    setMinimumSize(365, 275); }

void ColorMapper::paintEvent(QPaintEvent* event)
{
    QPainter painter(this);
    painter.setRenderHint(QPainter::Antialiasing);
    int starty = startx_;
    // 绘制距离组件
    painter.setPen(QPen(Qt::black, 2));
    painter.drawLine(startx_, 50, startx_, endy_);
    painter.drawText(30, 30, "Distance (m)");

    //painter.setPen(Qt::yellow);
    painter.setPen(QColor("#eb680b"));
    for (const auto& distance : distance_sliders_) {
        int y_val = distance.distance;
        QString y_str = QString::fromStdString(distance.name);
        QColor y_color(static_cast<Qt::GlobalColor>(distance.color));

        // draw abcd slider
        drawSlider(&painter, startx_, y_val, y_str, y_color);
        // draw distance slider value label
        painter.drawText(startx_, y_val - 5, QString::number((endy_ - y_val) * distance_ / 200));
    }

    // 绘制连接线
    for (auto i = 0; i < distance_sliders_.size() - 1; i++) {
        // set painter color
        QColor color;
        if (i > 0) {
            color = QColor(static_cast<Qt::GlobalColor>(distance_sliders_[i + 1].color));
        }
        else {
            color = QColor(static_cast<Qt::GlobalColor>(distance_sliders_[i].color));
        }
        // paint distance line

        painter.setPen(QPen(color, 3));
        painter.drawLine(startx_,
            distance_sliders_[i].distance,
            startx_,
            distance_sliders_[i + 1].distance);

        auto color_x = startx_ + dis_color_interval_ + color_interval_ * i;

        painter.setPen(QPen(Qt::black, 2));// draw color component vertical line
        painter.drawLine(color_x, 50, color_x, endy_);
        painter.drawText(color_x - 10, 30, QString("Color%1").arg(i + 1));
        drawColorRange(&painter, color_x, 50, endy_);
        for (int j = 0; j < 2; j++) {
            painter.setPen(QPen(color, 2));
            // draw color slider
            drawSlider(&painter, color_x, color_sliders_[i * 2 + j].color_value, "", color_sliders_[i * 2 + j].color);

            painter.drawLine(startx_, distance_sliders_[i + j].distance, color_x, color_sliders_[i * 2 + j].color_value);

            // draw white indicator
            painter.setPen(QPen(Qt::white, 2));
            painter.drawLine(color_x, color_sliders_[i * 2 + j].color_value, color_x + 20, color_sliders_[i * 2 + j].color_value);

            // draw black mapped color value
            painter.setPen(QPen(Qt::black, 2));
            painter.drawText(color_x - 20, starty, "255");
            painter.drawText(color_x - 20, endy_, "0");
            int map_color = color_max_ - (color_sliders_[i * 2 + j].color_value - starty) * color_max_ / (endy_ - starty);
            painter.drawText(color_x + 10, color_sliders_[i * 2 + j].color_value + 15, QString::number(map_color));
        }
        painter.setPen(QPen(color, 3));
        painter.drawLine(color_x, color_sliders_[i * 2].color_value, color_x, color_sliders_[i * 2 + 1].color_value);
    }
}

void ColorMapper::mouseMoveEvent(QMouseEvent* event)
{
    int y = event->pos().y();
    uint16_t& sliderPos = *draggingSlider;
    if (!draggingSlider) {
        return;
    }

    for (auto it = distance_sliders_.begin(); it != distance_sliders_.end(); ++it) {
        const auto& distance = *it;

        if (it == distance_sliders_.begin() || it == std::prev(distance_sliders_.end())) {
            continue;
        }

        // 获取前一个元素
        const auto& prev_distance = *std::prev(it);

        // 获取后一个元素
        const auto& next_distance = *std::next(it);

        if (draggingSlider == &(distance.distance)) {
            sliderPos = qBound((int)prev_distance.distance, y, (int)next_distance.distance);
            int real_distance = (endy_ - sliderPos) * distance_ / (endy_ - startx_);
            emit DistanceSliderMoved(real_distance, distance.dis_type);
            //qDebug() << "DistanceSliderMoved" << real_distance;
        }
    }

    for (auto it = color_sliders_.begin(); it != color_sliders_.end(); ++it) {
        if (draggingSlider == &it->color_value) {
            sliderPos = qBound(50, y, 250);
            int color_value = color_max_ - (sliderPos - startx_) * color_max_ / (endy_ - startx_);
            emit ColorSliderMoved(color_value, it->color_value_type);
            qDebug() << "ColorSliderMoved" << color_value;
        }
    }
    update();
}

void ColorMapper::mousePressEvent(QMouseEvent* event)
{
    int x = event->pos().x();
    int y = event->pos().y();
    for (const auto& distance : distance_sliders_) {
        if (distance == distance_sliders_.front() ||
            distance == distance_sliders_.back()) {
            continue;
        }
        if (abs(x - startx_) < sense_value_) {
            if (abs(y - distance.distance) < sense_value_) {
                draggingSlider = const_cast<uint16_t*>(&distance.distance);
            }
        }
    }

    int times = 0;
    for (const auto& color : color_sliders_) {
        auto x_pos = startx_ + dis_color_interval_ + color_interval_ * (times / 2);
        times++;
        if (abs(x - x_pos) >= 10) {
            continue;
        }
        if (abs(y - color.color_value) < 10) {
            draggingSlider = const_cast<uint16_t*>(&color.color_value);
        }
    }
}

void ColorMapper::mouseReleaseEvent(QMouseEvent* event)
{
    if (draggingSlider) {
        for (const auto& distance : distance_sliders_) {
            if (draggingSlider == &distance.distance) {
                int real_distance = (endy_ - distance.distance) * distance_ / (endy_ - startx_);
                emit DistanceSliderReleased(real_distance, distance.dis_type);
                auto set_type = "";
                switch (distance.dis_type){                        
                case PointDistance::kStartPos:
                    set_type = "distance.start";
                    break;
                case PointDistance::kClosePos:
                    set_type = "distance.close";
                    break;
                case PointDistance::kFarPos:
                    set_type = "distance.far";
                    break;
                case PointDistance::kEndPos:
                    set_type = "distance.end";
                    break;
                default:
                    break;
                }
                you_see::ConfigService::Instance().Set<int>(set_type, real_distance);
                you_see::ConfigService::Instance().SaveAllConfigs();
                break;
            }
        }

        for (const auto& color : color_sliders_) {
            if (draggingSlider == &color.color_value) {
                int color_value = color_max_ - (color.color_value - startx_) * color_max_ / (endy_ - startx_);
                emit ColorSliderReleased(color_value, color.color_value_type);
                auto set_type = "";
                switch (color.color_value_type) {
                case DistanceColor::kCloseStartColor:
                    set_type = "distance_color.close_begin";
                    break;
                case DistanceColor::kCloseEndColor:
                    set_type = "distance_color.close_end";
                    break;
                case DistanceColor::kMiddleStartColor:
                    set_type = "distance_color.middle_begin";
                    break;
                case DistanceColor::kMiddleEndColor:
                    set_type = "distance_color.middle_end";
                    break;
                case DistanceColor::kFarStartColor:
                    set_type = "distance_color.far_begin";
                    break;
                case DistanceColor::kFarEndColor:
                    set_type = "distance_color.far_end";
                    break;
                default:
                    break;
                }
                you_see::ConfigService::Instance().Set<int>(set_type, color_value);
                you_see::ConfigService::Instance().SaveAllConfigs();
                break;
            }
        }
    }
    draggingSlider = nullptr;
}

void ColorMapper::resizeEvent(QResizeEvent* event)
{
    QWidget::resizeEvent(event);
    int width = event->size().width();
    dis_color_interval_ = width - startx_ - 3 * color_interval_;
    update();
}
};