#include "record_interval.h"
#include "ui_record_interval.h"
#include "config/config_service.h"
#include "ys_lidar_validate_type.h"
#include <QStyleFactory>

namespace ys_scene {
using namespace you_see;
RecordInterval::RecordInterval(QWidget *parent) :
    QDialog(parent),
    ui(new Ui::record_interval)
{
    ui->setupUi(this);
    setAttribute(Qt::WA_DeleteOnClose, false);
    ui->dial_interval->setRange(0, 50);
    ui->dial_interval->setSingleStep(1);
    ui->dial_interval->setNotchesVisible(true);
    ui->dial_interval->setStyle(QStyleFactory::create("Fusion"));

    auto& config = ConfigService::Instance();
    ui->dial_interval->setValue(config.Get<int>("interval_record.interval_time", 2000) / 1000);
    int line_type = config.Get<int>("interval_record.line_mode", 3);
    switch (static_cast<PointcloudRecordType>(line_type)) {
    case PointcloudRecordType::odd_number_lines:
        ui->rb_odd->setChecked(true);
        break;
    case PointcloudRecordType::even_lines:
        ui->rb_even->setChecked(true);
        break;
    case PointcloudRecordType::total_lines:
        ui->rb_total->setChecked(true);
        break;
    default:
        break;
    }

    connect(ui->dial_interval, &QDial::valueChanged, this, [this, &config]() {
        int interval = ui->dial_interval->value() * 100;
        emit SetIntervalValue(interval);
        config.Set<int>("interval_record.interval_time", interval);
        });

    connect(ui->rb_even, &QRadioButton::clicked, this, [this, &config]() {
        emit LinesType(2);
        config.Set<int>("interval_record.line_mode", 2);
        });

    connect(ui->rb_odd, &QRadioButton::clicked, this, [this, &config]() {
        emit LinesType(1);
        config.Set<int>("interval_record.line_mode", 1);
        });

    connect(ui->rb_total, &QRadioButton::clicked, this, [this, &config]() {
        emit LinesType(3);
        config.Set<int>("interval_record.line_mode", 3);
        });
    
    connect(ui->tb_interval_cancel, &QToolButton::clicked, this, [this, &config]() {
        emit SetIntervalEnable(false);
        config.Set<bool>("interval_record.enable_interval_recording", false);
        });

    connect(ui->tb_interval_confirm, &QToolButton::clicked, this, [this, &config]() {
        emit SetIntervalEnable(true);
        config.Set<bool>("interval_record.enable_interval_recording", true);
        });
}

RecordInterval::~RecordInterval()
{
    delete ui;
}

}

