#ifndef CONTINUOUSSAVERAW_H
#define CONTINUOUSSAVERAW_H

#include <QDialog>

namespace Ui {
class ContinuousSaveRaw;
}

namespace ys_scene {
class ContinuousSaveRaw : public QDialog
{
    Q_OBJECT

public:
    explicit ContinuousSaveRaw(QWidget *parent = nullptr);
    ~ContinuousSaveRaw();
    std::tuple<int, std::string> GetSaveParams();

private:
    Ui::ContinuousSaveRaw *ui;
};
}// namespace ys_scene
#endif // CONTINUOUSSAVERAW_H
