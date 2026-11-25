#include "continuoussaveraw.h"
#include "ui_continuoussaveraw.h"

namespace ys_scene {
ContinuousSaveRaw::ContinuousSaveRaw(QWidget *parent)
    : QDialog(parent)
    , ui(new Ui::ContinuousSaveRaw)
{
    ui->setupUi(this);
    ui->dsb_distance->setValue(20.00000);
    ui->dsb_distance->setRange(0.00000, 10000.00000);
    ui->dsb_distance->setDecimals(5);
}

ContinuousSaveRaw::~ContinuousSaveRaw()
{
    delete ui;
}
std::tuple<int, std::string>  ContinuousSaveRaw::GetSaveParams()
{
    int count = ui->cb_save_quantity->currentText().toInt();
    std::string prefix = "v" + ui->cb_spot_number->currentText().toStdString() + "_"
        + ui->cb_reflectivity->currentText().toStdString() + "_" + std::to_string(ui->dsb_distance->value()) + "_";
    return std::tuple<int, std::string>(count, prefix);
}
}// namespace ys_scene 
