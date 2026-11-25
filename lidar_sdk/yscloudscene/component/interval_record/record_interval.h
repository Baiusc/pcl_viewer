#ifndef RECORD_INTERVAL_H
#define RECORD_INTERVAL_H

#include <QDialog>

namespace Ui {
class record_interval;
}

namespace ys_scene {
class RecordInterval : public QDialog
{
    Q_OBJECT

public:
    explicit RecordInterval(QWidget* parent = nullptr);
    ~RecordInterval();

signals:
    void LinesType(int type);
    void SetIntervalEnable(bool enable);
    void SetIntervalValue(int value);

private:
    Ui::record_interval* ui;
};

}

#endif // RECORD_INTERVAL_H
