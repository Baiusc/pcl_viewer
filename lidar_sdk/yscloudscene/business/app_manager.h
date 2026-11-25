#ifndef YS_SCENE_APP_MANAGER_H_
#define YS_SCENE_APP_MANAGER_H_

#include <QObject>
#include <QList>
#include <QProcess>
#include <QDir>
#include <QDebug>

namespace ys_scene {

class AppManager : public QObject {
	Q_OBJECT

public:
	AppManager(QObject* parent = nullptr);
	~AppManager();
	bool StartApp(const QString& app_path, const QString& app_name);

private:
	bool IsProcessRunning(const QString& process_name);

private:
	QList<QProcess*> child_processes_;
};

} // namespace ys_scene

#endif // YS_SCENE_APP_MANAGER_H_