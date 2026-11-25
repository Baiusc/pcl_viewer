#include "continuousdealpoint.h"
#include "you_see_sdk/include/ys_lidar_validate_type.h"
#include <QPushButton>

using namespace you_see;

namespace ys_scene {
ContinuousDealPoint::ContinuousDealPoint(QWidget* parent)
	: QWidget(parent, Qt::Window | Qt::WindowStaysOnTopHint)
{
	ui.setupUi(this);
	setWindowTitle("ContinuousDealPoint");
	ui.cb_side1->setChecked(true);
	ui.cb_side2->setChecked(true);
	ui.cb_side3->setChecked(true);
	ui.cb_side4->setChecked(true);
	connect(ui.pb_display_confirm, &QPushButton::clicked, this, &ContinuousDealPoint::OnDisplayConfirm);
	connect(ui.pb_frame_confirm, &QPushButton::clicked, this, &ContinuousDealPoint::OnSaveConfirm);
	connect(ui.cb_all_sides_display, &QCheckBox::stateChanged, this, &ContinuousDealPoint::OnAllSidesDisplay);
}

ContinuousDealPoint::~ContinuousDealPoint()
{

}

void ContinuousDealPoint::OnDisplayConfirm()
{
	QList<bool> display_sides;
	display_sides.reserve(4);
	display_sides << true << true << true << true;
	display_sides[0] = ui.cb_side1->checkState();
	display_sides[1] = ui.cb_side2->checkState();
	display_sides[2] = ui.cb_side3->checkState();
	display_sides[3] = ui.cb_side4->checkState();
	emit DisplaySide(display_sides, ToInt(PrismFaces::four_sided_mirror));
	close();
}

void ContinuousDealPoint::OnSaveConfirm()
{
	QString count = ui.cb_save_frame->currentText();
	emit PointcloudSave(count);
	close();
}

void ContinuousDealPoint::OnAllSidesDisplay(int state)
{
	if (state == Qt::Checked) {
		ui.cb_side1->setChecked(true);
		ui.cb_side2->setChecked(true);
		ui.cb_side3->setChecked(true);
		ui.cb_side4->setChecked(true);
		QList<bool> display_sides;
		display_sides.reserve(4);
		display_sides << true << true << true << true;
		emit DisplaySide(display_sides, ToInt(PrismFaces::four_sided_mirror));
		emit AllFacesDisplay(true);
	}
	else if (state == Qt::Unchecked) {
		emit AllFacesDisplay(false);
	}
}

void ContinuousDealPoint::ShowModal()
{
	setWindowModality(Qt::ApplicationModal);
	show();
}

}
