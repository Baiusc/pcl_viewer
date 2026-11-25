#include "toast_tips.h"
#include <QFont>

namespace ys_scene {
ToastTips::ToastTips(QWidget* parent)
	: QWidget(parent)
{
	Toast::setPosition(ToastPosition::BOTTOM_MIDDLE);
	Toast::setAlwaysOnMainScreen(true);
}

ToastTips::~ToastTips()
{
}

void ToastTips::ShowTip(ToastPreset type, QString& title, QString& text)
{
	Toast* toast = new Toast(this);
	QFont title_font;
	title_font.setPixelSize(14);
	title_font.setBold(true);
	toast->setTitleFont(title_font);
	QFont text_font;
	text_font.setPixelSize(12);
	toast->setTextFont(text_font);
	toast->setTitle(title);
	toast->setText(text);
	toast->setDuration(2000);
	toast->setBorderRadius(5);
	toast->applyPreset(type);
	toast->show();
}
} // namespace ys_scene 

