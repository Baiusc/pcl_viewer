#ifndef COLLAPSIBLE_GROUPBOX_H
#define COLLAPSIBLE_GROUPBOX_H

#include <QWidget>
#include <QGroupBox>

class QVBoxLayout;

namespace ys_scene
{

class CollapsibleGroupBox : public QGroupBox
{
    Q_OBJECT
public:
    explicit CollapsibleGroupBox(const QString &title, QWidget *parent = nullptr);
    void SetCollapsed(bool collapse);
    QWidget* GetContent();
protected:
    void mousePressEvent(QMouseEvent *event) override;
    void paintEvent(QPaintEvent *event) override;

private:
    void ToggleCollapsed();
    void UpdateTriangleGeometry();

private:
    bool collapsed_ = false;
    QWidget *content_;
    QVBoxLayout *main_layout_;
    QPolygon triangle_;
};

}// namespace ys_scene

#endif // COLLAPSIBLE_GROUPBOX_H
