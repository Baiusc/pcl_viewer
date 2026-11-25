#include "continuoussaveorigin.h"
#include "ui_continuoussaveorigin.h"

namespace ys_scene {
ContinuousSaveOrigin::ContinuousSaveOrigin(QWidget *parent) :
    QDialog(parent),
    ui(new Ui::ContinuousSaveOrigin)
{
    ui->setupUi(this);
    ui->dsb_distance->setValue(20.00000);
    ui->dsb_distance->setRange(0.00000, 10000.00000);
    ui->dsb_distance->setDecimals(5);
    connect(ui->cb_al_repair_ground, &QCheckBox::stateChanged, this, 
        &ContinuousSaveOrigin::OnAlgorithmRepairGroundStatusChanged);
}

ContinuousSaveOrigin::~ContinuousSaveOrigin()
{
    delete ui;
}

void ContinuousSaveOrigin::OnAlgorithmRepairGroundStatusChanged(int state)
{
	emit AlgorithmRepairGroundEnable(state);
}

std::tuple<int, int, std::string> ContinuousSaveOrigin::GetSaveParams()
{
    int count = ui->cb_save_quantity->currentText().toInt();
    int side = ui->cb_side->currentText().toInt();
    std::string prefix = "v" + ui->cb_side->currentText().toStdString() + "_"
        + ui->cb_reflectivity->currentText().toStdString() + "_" + std::to_string(ui->dsb_distance->value()) + "_";
    return std::tuple<int, int, std::string>(count, side, prefix);
}
}// namespace ys_scene
