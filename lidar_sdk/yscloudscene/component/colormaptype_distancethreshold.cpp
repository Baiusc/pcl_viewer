#include "colormaptype_distancethreshold.h"
#include "ui_colormaptype_distancethreshold.h"
#include "config/config_service.h"

namespace ys_scene {

ColorMapTypeDistanceThreshold::ColorMapTypeDistanceThreshold(QWidget* parent) :
    QDialog(parent),
    ui(new Ui::ColorMapTypeDistanceThreshold)
{
    ui->setupUi(this);
    ui->dsb_min_threhold->setValue(1.0);
    ui->dsb_min_threhold->setRange(0.0, 500.0);

    ui->dsb_max_threhold->setValue(110.0);
    ui->dsb_max_threhold->setRange(0.0, 500.0);
    this->setWindowTitle(tr("distance_colormap"));
    int method_type = you_see::ConfigService::Instance().Get<int>("map_method", 0);
    auto mm = static_cast<MappingMethod>(method_type);
    switch (mm) {
    case MappingMethod::kXAxisMap:
        ui->rb_x->setChecked(true);
        break;
    case MappingMethod::kYAxisMap:
        ui->rb_y->setChecked(true);
        break;
    case MappingMethod::kZAxisMap:
        ui->rb_z->setChecked(true);
        break;
    case MappingMethod::kReflectivetyMap:
        ui->rb_reflectivity->setChecked(true);
        break;
    };

    connect(ui->tb_distance_confirm, &QToolButton::clicked, 
        this, &ColorMapTypeDistanceThreshold::GetDistanceThrehold);
    connect(ui->tb_algo_confirm, &QToolButton::clicked,
        this, &ColorMapTypeDistanceThreshold::GetAlgorithmItems);
    connect(ui->rb_x, &QRadioButton::toggled, 
        this, &ColorMapTypeDistanceThreshold::HandleMapTypeToggle);
    connect(ui->rb_y, &QRadioButton::toggled,
        this, &ColorMapTypeDistanceThreshold::HandleMapTypeToggle);
    connect(ui->rb_z, &QRadioButton::toggled,
        this, &ColorMapTypeDistanceThreshold::HandleMapTypeToggle);
    connect(ui->rb_reflectivity, &QRadioButton::toggled,
        this, &ColorMapTypeDistanceThreshold::HandleMapTypeToggle);
    connect(ui->rb_odd, &QRadioButton::toggled, this, [this]() {
        emit LinesShowMode(1);
        });
    connect(ui->rb_even, &QRadioButton::toggled, this, [this]() {
        emit LinesShowMode(2);
        });
    connect(ui->rb_total, &QRadioButton::toggled, this, [this]() {
        emit LinesShowMode(3);
        });
}

ColorMapTypeDistanceThreshold::~ColorMapTypeDistanceThreshold()
{
    delete ui;
}

void ColorMapTypeDistanceThreshold::GetDistanceThrehold()
{
    emit DistanceShowThrehold(std::tuple<double, double>(ui->dsb_min_threhold->value(),
        ui->dsb_max_threhold->value()));
}

void ColorMapTypeDistanceThreshold::GetAlgorithmItems()
{
    AlgorithmItemEnable algoitems;
    algoitems.de_tail_filter = ui->cb_detail_filter->isChecked();
    emit AlgorithmItemEnableSig(algoitems);
}

ColorMapper* ColorMapTypeDistanceThreshold::GetColorMapper()
{
    return ui->cm_distance_color;
}

void ColorMapTypeDistanceThreshold::HandleMapTypeToggle(bool checked)
{
    auto *sender_obj = sender();
    if (sender_obj && checked) {
        QString name = sender_obj->objectName();
        MappingMethod mm;
        if (name == "rb_x") {
            mm = MappingMethod::kXAxisMap;
        }
        else if (name == "rb_y") {
            mm = MappingMethod::kYAxisMap;
        }
        else if (name == "rb_z") {
            mm = MappingMethod::kZAxisMap;
        }
        else if (name == "rb_reflectivity") {
            mm = MappingMethod::kReflectivetyMap;
        }
        emit ColorMapType(mm);
        you_see::ConfigService::Instance().Set<int>("map_method", static_cast<int>(mm));
        you_see::ConfigService::Instance().SaveAllConfigs();
    }
}

} // namespace ys_scenes
