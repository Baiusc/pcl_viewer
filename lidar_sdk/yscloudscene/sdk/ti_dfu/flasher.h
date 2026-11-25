#pragma once

#include <QObject>
#include <QProcess>
#include <QTimer>
#include <QVector>

namespace ti_dfu {

enum class Workflow { KDirectUpdate, KFlashWriter };

struct Options
{
    QString tool_path;
    QString boot_directory;
    QString image_path;
    Workflow workflow = Workflow::KDirectUpdate;
};

class Flasher : public QObject
{
    Q_OBJECT

public:
    explicit Flasher(QObject* parent = nullptr);
    bool Start(const Options& options);
    bool IsRunning() const;

signals:
    void StageChanged(const QString& stage, int index, int total);
    void ProgressChanged(int percent);
    void LogReceived(const QString& output);
    void Completed(bool success, const QString& detail);

private:
    struct Step
    {
        QString name;
        QString interface_name;
        QString file;
        int alt;
        bool require_spl_pair = false;
    };

    void Probe();
    void Run(const QStringList& arguments);
    void ReadOutput();
    void ProcessFinished(int exit_code, QProcess::ExitStatus status);
    void HandleProbe();
    void Finish(bool success, const QString& detail);

    QProcess process_;
    QTimer timeout_;
    QVector<Step> steps_;
    QString tool_;
    QString output_;
    QString device_path_;
    int step_ = -1;
    int probe_attempts_ = 0;
    int progress_ = -1;
    bool probing_ = false;
    bool timed_out_ = false;
    bool download_done_ = false;
};

} // namespace ti_dfu
