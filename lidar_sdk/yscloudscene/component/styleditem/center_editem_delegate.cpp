#include "center_editem_delegate.h"
#include <QPainter>

CenteredItemDelegate::CenteredItemDelegate(QObject *parent)
	: QStyledItemDelegate(parent)
{
}

CenteredItemDelegate::~CenteredItemDelegate()
{
}

void CenteredItemDelegate::initStyleOption(QStyleOptionViewItem* option, const QModelIndex& index) const
{
	QStyledItemDelegate::initStyleOption(option, index);
	option->displayAlignment = Qt::AlignCenter;
}

void CenteredItemDelegate::paint(QPainter* painter, const QStyleOptionViewItem& option, const QModelIndex& index) const
{
    QStyleOptionViewItem opt = option;
    initStyleOption(&opt, index);

    // 设置背景颜色为父控件的背景颜色
    QColor backgroundColor = opt.palette.color(QPalette::Window);
    painter->fillRect(opt.rect, backgroundColor);

    // 设置字体和大小
    QFont font = opt.font;
    font.setFamily("Arial");   // 设置字体
    font.setPointSize(14);     // 设置字号
    opt.font = font;

    // 设置文本颜色
    opt.palette.setColor(QPalette::Text, QColor(Qt::white));

    // 绘制边框
    painter->save();
    painter->setPen(QPen(QColor("#059669"), 2)); // 黑色边框，宽度为2
    painter->drawRect(opt.rect);
    painter->restore();


    // 设置文本居中对齐
    opt.displayAlignment = Qt::AlignCenter;

    // 调用基类的 paint 方法绘制
    QStyledItemDelegate::paint(painter, opt, index);
}
