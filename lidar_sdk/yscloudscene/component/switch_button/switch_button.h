#ifndef YS_SCENE_SWITCH_BUTTON_H
#define YS_SCENE_SWITCH_BUTTON_H

#include <QWidget>
#include <QPainter>
#include <QMouseEvent>
#include <QApplication>
#include <QResizeEvent>

namespace ys_scene {
    class SwitchButton : public QWidget
    {
        Q_OBJECT

    public:
        // 定义按钮状态枚举
        enum class State {
            Off,        // 关闭状态
            Pending,    // 等待响应状态
            On         // 开启状态
        };

        explicit SwitchButton(QWidget* parent = nullptr);

        // 获取当前状态
        State getState() const;

        // 设置状态
        void setState(State state);

        // 设置开关的文字
        void setOnText(const QString& text);
        void setOffText(const QString& text);

        // 提供推荐大小
        QSize sizeHint() const override;

    signals:
        void stateChangeRequested(bool checked);

    protected:
        void paintEvent(QPaintEvent*) override;
        void mousePressEvent(QMouseEvent* event) override;
        void resizeEvent(QResizeEvent* event) override;

    private:
        State currentState;    // 当前状态
        QString onText;        // "开" 状态显示的文字
        QString offText;       // "关" 状态显示的文字
    };
} //namespace ys_scene

#endif // YS_SCENE_SWITCH_BUTTON_H
