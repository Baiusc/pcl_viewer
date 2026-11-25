#ifndef YS_SCENE_CONTINUOUS_DEAL_POINT_H_
#define YS_SCENE_CONTINUOUS_DEAL_POINT_H_

#include <QWidget>
#include <QList>
#include "ui_continuousdealpoint.h"

namespace ys_scene {
class ContinuousDealPoint : public QWidget
{
	Q_OBJECT

public:
	ContinuousDealPoint(QWidget* parent = nullptr);
	~ContinuousDealPoint();
	using BoolPackege = QList<bool>;
signals:
	void DisplaySide(BoolPackege side_status, int counts);
	void PointcloudSave(QString count);
	void AllFacesDisplay(bool enable);

public slots:
	void ShowModal();
	void OnDisplayConfirm();
	void OnSaveConfirm();
	void OnAllSidesDisplay(int state);

private:
	Ui::ContinuousDealPoint ui;
};
} // namespace ys_scene
#endif // YS_SCENE_CONTINUOUS_DEAL_POINT_H_