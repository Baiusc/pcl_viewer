#ifndef COLORMAPPER_H_
#define COLORMAPPER_H_

#include "ys_scene_def.h"
#include <QWidget>
#include <QPainter>
#include <QMouseEvent>
#include <QImage>

namespace ys_scene {
class ColorMapper : public QWidget
{
    Q_OBJECT
public:
    explicit ColorMapper(QWidget* parent = nullptr);
    explicit ColorMapper(DisMapList distance, DisColorMapList color, QWidget* parent = nullptr);

protected:
    void paintEvent(QPaintEvent* event) override;
    void mouseMoveEvent(QMouseEvent* event) override;
    void mousePressEvent(QMouseEvent* event) override;
    void mouseReleaseEvent(QMouseEvent* event) override;
    void resizeEvent(QResizeEvent* event) override;

signals:
    void DistanceSliderMoved(int value, PointDistance distance_type);
    void DistanceSliderReleased(int value, PointDistance distance_type);
    void ColorSliderMoved(int value, DistanceColor color_type);
    void ColorSliderReleased(int value, DistanceColor color_type);

private:
    void drawSlider(QPainter* painter, int x, int y, const QString& label, const QColor& color) {
        painter->setBrush(color);
        painter->drawEllipse(x - 5, y - 5, 10, 10);
        painter->drawText(x + 10, y + 5, label);
    }

    void drawColorRange(QPainter* painter, int x, int start, int end) {
        QLinearGradient gradient(x, start, x, end);
        float step = 1.0 * (end - start) / chroma_img_->height();
        for (int i = start; i <= end; i++) {
            int pixelY = (i - start) / step;
            pixelY = qBound(0, pixelY, chroma_img_->height() - 1);
            QRgb rgb = chroma_img_->pixel(0, pixelY);
            gradient.setColorAt((i - start) * 1.0 / (end - start), QColor(rgb));
        }
        painter->fillRect(x, start, 20, end - start, gradient);
    }

    int a = 70, b = 100, c = 180, d = 230;
    int e = 50, f = 100, g = 130, h = 160, i = 190, j = 250;
    DisMapList distance_sliders_;
    DisColorMapList color_sliders_;
    int startx_;
    int endy_;
    int sense_value_;
    int dis_color_interval_;
    int distance_;
    int color_max_;
    const int color_interval_;
    QImage* chroma_img_;
    uint16_t* draggingSlider = nullptr;
};

} // namespace ys_scene
#endif // COLORMAPPER_H_
