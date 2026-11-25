#ifndef CONTINUOUSSAVEORIGIN_H
#define CONTINUOUSSAVEORIGIN_H

#include <QDialog>

namespace Ui {
class ContinuousSaveOrigin;
}

namespace ys_scene {
class ContinuousSaveOrigin : public QDialog
{
    Q_OBJECT

public:
    explicit ContinuousSaveOrigin(QWidget *parent = nullptr);
    ~ContinuousSaveOrigin();
    std::tuple<int, int, std::string> GetSaveParams();

signals:
	void AlgorithmRepairGroundEnable(int state);

private slots:
    void OnAlgorithmRepairGroundStatusChanged(int state);

private:
    Ui::ContinuousSaveOrigin *ui;
};
}// namespace ys_scene

#endif // CONTINUOUSSAVEORIGIN_H
