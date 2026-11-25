#pragma once

#include <QStyledItemDelegate>

class CenteredItemDelegate : public QStyledItemDelegate
{

public:
	CenteredItemDelegate(QObject* parent = nullptr);
	~CenteredItemDelegate();
	void initStyleOption(QStyleOptionViewItem* option, const QModelIndex& index) const override;
	void paint(QPainter* painter, const QStyleOptionViewItem& option, const QModelIndex& index) const override;
};
