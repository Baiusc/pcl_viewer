#ifndef	YS_SCENE_TOAST_TIPS_
#define YS_SCENE_TOAST_TIPS_

#include <QWidget>
#include "tiny_hut/singleton2.h"
#include "Toast.h"

using namespace you_see;

namespace ys_scene {
class ToastTips : public QWidget , public Singleton2<ToastTips>
{
	Q_OBJECT
	friend class Singleton2<ToastTips>;

public:
	ToastTips(QWidget* parent = nullptr);
	~ToastTips();
	void ShowTip(ToastPreset type, QString& title, QString& text);
};
} //namespace ys_scene

#endif // YS_SCENE_TOAST_TIPS_