#ifndef YS_SCENE_RANGE_SLIDER_H_
#define YS_SCENE_RANGE_SLIDER_H_

#include <QWidget>
#include <QSize>
#include <QPaintEvent>
#include <QMouseEvent>

namespace ys_scene {

// 双端滑块：通过拖动两个手柄选择半开区间 [min_value_, max_value_)
// 用于交互式设定计算峰值的 bins 范围
class RangeSlider : public QWidget {
    Q_OBJECT
public:
    explicit RangeSlider(QWidget* parent = nullptr);

    // 数值总范围（半开 [min_limit, max_limit)），重设后自动钳制当前选区
    void SetLimits(int min_limit, int max_limit);
    // 程序化设置选区（不发信号），值会被钳制到总范围内并保证非空
    void SetValues(int min_value, int max_value);
    int MinimumValue() const;
    int MaximumValue() const;

    QSize sizeHint() const override;
    QSize minimumSizeHint() const override;

signals:
    // 用户拖动手柄时发射（选区有实际变化）
    void RangeChanged(int min_value, int max_value);

protected:
    void paintEvent(QPaintEvent* event) override;
    void mousePressEvent(QMouseEvent* event) override;
    void mouseMoveEvent(QMouseEvent* event) override;
    void mouseReleaseEvent(QMouseEvent* event) override;

private:
    int ValueToPos(int value) const;
    int PosToValue(int pos) const;
    void DragTo(int pos);

private:
    int min_limit_;
    int max_limit_;
    int min_value_;
    int max_value_;
    int active_handle_;   // -1: 无，0: 左手柄，1: 右手柄

    static constexpr int kHandleRadius = 8;
    static constexpr int kHandleDiameter = kHandleRadius * 2;
};

}// namespace ys_scene
#endif // YS_SCENE_RANGE_SLIDER_H_
