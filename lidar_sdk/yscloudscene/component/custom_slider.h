#ifndef YS_SCENE_CUSTOM_SLIDER_H_
#define YS_SCENE_CUSTOM_SLIDER_H_

#include <QSlider>
#include <QMouseEvent>

namespace ys_scene {
class CustomSlider : public QSlider
{
public:
    CustomSlider(QWidget* parent = nullptr);

protected:
    void paintEvent(QPaintEvent* event) override;
    void mousePressEvent(QMouseEvent* event) override;
    void mouseMoveEvent(QMouseEvent* event) override;

public:
    void SetRealRange(int min, int max);
    int RealValue() const;
    void SetProgressPosition(int value);
    void Reset();

private:
    int realmin_;
    int realmax_;
    int extrarange_;

signals:
    void RealValueChanged(int value);
};

}// namespace ys_scene 

#endif // YS_SCENE_CUSTOM_SLIDER_H_