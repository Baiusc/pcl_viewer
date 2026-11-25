#include "collapsible_groupbox.h"
#include <QVBoxLayout>
#include <QGridLayout>
#include <QLabel>
#include <QMouseEvent>
#include <QStyle>
#include <QPainter>
#include <QPainterPath>

namespace ys_scene
{

CollapsibleGroupBox::CollapsibleGroupBox(const QString &title, QWidget *parent)
: QGroupBox(title, parent)
{
    setCheckable(false);
    // 创建主布局
    main_layout_ = new QVBoxLayout(this);
    main_layout_->setContentsMargins(5, 15, 5, 5);  // 为标题留出空间

    // 创建内容容器
    content_ = new QWidget(this);
    main_layout_->addWidget(content_);

    // 新的时尚样式！
    setStyleSheet(R"(
        CollapsibleGroupBox {
            border: 1px solid #5A5A5A;
            border-radius: 8px;
            margin-top: 0.5em;
            background: qlineargradient(x1:0, y1:0, x2:0, y2:1,
                                      stop:0 #3D3D3D,
                                      stop:1 #2D2D2D);
            color: #E0E0E0;
        }
        CollapsibleGroupBox::title {
            subcontrol-origin: margin;
            left: 25px;
            padding: 0 8px;
            background-color: transparent;
            font-weight: bold;
            font-size: 13px;
        }
        CollapsibleGroupBox:hover {
            border: 1px solid #6A6A6A;
            background: qlineargradient(x1:0, y1:0, x2:0, y2:1,
                                      stop:0 #424242,
                                      stop:1 #323232);
        }
    )");
}

void CollapsibleGroupBox::SetCollapsed(bool collapse)
{
    if (collapsed_ == collapse){
        return;
    }
    collapsed_ = collapse;
    if (collapsed_) {
        content_->hide();
        setFixedHeight(30);
    } else {
        content_->show();
        setFixedHeight(content_->sizeHint().height() + 40);
    }
    update();
}

QWidget *CollapsibleGroupBox::GetContent()
{
    return content_;
}

void CollapsibleGroupBox::paintEvent(QPaintEvent *event)
{
    QGroupBox::paintEvent(event);
    UpdateTriangleGeometry();

    QPainter painter(this);
    painter.setRenderHint(QPainter::Antialiasing);

    // 添加阴影效果
    QColor shadowColor(0, 0, 0, 30);
    painter.setPen(Qt::NoPen);
    painter.setBrush(shadowColor);
    painter.translate(2, 2);
    painter.drawPolygon(triangle_);

    // 主三角形渐变
    QLinearGradient gradient(triangle_.boundingRect().topLeft(),
                             triangle_.boundingRect().bottomRight());
    gradient.setColorAt(0, QColor("#6A6A6A"));
    gradient.setColorAt(1, QColor("#4A4A4A"));

    painter.translate(-2, -2);
    painter.setBrush(gradient);
    painter.drawPolygon(triangle_);
}

void CollapsibleGroupBox::mousePressEvent(QMouseEvent *event)
{
    if (event->button() == Qt::LeftButton) {
        QPointF pos = event->pos();

        // 使用QPainterPath来进行点击检测
        QPainterPath path;
        path.addPolygon(triangle_);

        // 获取三角形的边界矩形
        QRectF triangleBounds = triangle_.boundingRect();

        // 扩大检测区域（可以根据需要调整扩大的大小）
        qreal expand = 10.0;
        triangleBounds.adjust(-expand, -expand, expand, expand);

        // 调试输出
        qDebug() << "Mouse Position:" << pos;
        qDebug() << "Triangle Points:" << triangle_;
        qDebug() << "Triangle Bounds:" << triangleBounds;

        // 先检查点是否在扩展的边界矩形内（这样可以快速排除明显不在区域内的点击）
        if (triangleBounds.contains(pos)) {
            // 如果在扩展边界内，再检查是否在扩展的三角形区域内
            QPainterPath expandedPath;
            expandedPath.addPolygon(triangle_);
            // 扩展路径
            QPainterPathStroker stroker;
            stroker.setWidth(expand * 2);
            QPainterPath expanded = stroker.createStroke(expandedPath);

            if (expanded.contains(pos)) {
                ToggleCollapsed();
                update();
                return;
            }
        }
    }
    QGroupBox::mousePressEvent(event);
}

void CollapsibleGroupBox::ToggleCollapsed()
{
    SetCollapsed(!collapsed_);
}

void CollapsibleGroupBox::UpdateTriangleGeometry()
{
    QRect titleRect = this->style()->subControlRect(QStyle::CC_GroupBox, nullptr,
                                                    QStyle::SC_GroupBoxLabel, this);

    int triangleSize = 8;
    int margin = 5;
    // 计算三角形中心点的位置
    QPoint centerPos(width() - margin - triangleSize, titleRect.center().y() + 15);

    triangle_.clear();
    if (collapsed_) {
        // 绘制一个向左的等腰三角形
        triangle_ << QPoint(centerPos.x() + triangleSize/2, centerPos.y() - triangleSize/2)  // 右上角
                  << QPoint(centerPos.x() + triangleSize/2, centerPos.y() + triangleSize/2)   // 右下角
                  << QPoint(centerPos.x() - triangleSize/2, centerPos.y());                   // 左中点
    } else {
        // 绘制一个向下的等腰三角形
        triangle_ << QPoint(centerPos.x() - triangleSize/2, centerPos.y() - triangleSize/2)  // 左上角
                  << QPoint(centerPos.x() + triangleSize/2, centerPos.y() - triangleSize/2)   // 右上角
                  << QPoint(centerPos.x(), centerPos.y() + triangleSize/2);                   // 下中点
    }
}
}// namespace ys_scene
