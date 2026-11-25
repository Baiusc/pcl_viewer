#include "roi_select.h"
#include "ui_roi_select.h"
#include "config/config_service.h"

namespace ys_scene {
ROISelect::ROISelect(QWidget* parent) :
    QDialog(parent),
    ui(new Ui::ROISelect),
    roi_select_type_(ROISelectType::kROICancel),
    optional_slot_number_(0)
{
    ui->setupUi(this);
    ui->sb_slotmin->setRange(0, 798);
    ui->sb_slotmax->setRange(0, 799);
    ui->sb_pixelmin->setRange(0, 798);
    ui->sb_pixelmax->setRange(0, 799);

    roi_select_.min_slot = you_see::ConfigService::Instance().Get<int>("cloud_roi.min_slot", 0);
    ui->sb_slotmin->setValue(roi_select_.min_slot);

    roi_select_.max_slot = you_see::ConfigService::Instance().Get<int>("cloud_roi.max_slot", 799);
    ui->sb_slotmax->setValue(roi_select_.max_slot);

    roi_select_.min_pixel = you_see::ConfigService::Instance().Get<int>("cloud_roi.min_pixel", 0);
    ui->sb_pixelmin->setValue(roi_select_.min_pixel);

    // 默认取单 SLOT 最大像素索引：alpha/beta(96 像素)不变，CS96-gamma(120 像素)才不会被截掉 96~119
    roi_select_.max_pixel = you_see::ConfigService::Instance().Get<int>("cloud_roi.max_pixel", kPointRoiDefaultMaxPixel);
    ui->sb_pixelmax->setValue(roi_select_.max_pixel);

    optional_slot_number_ = you_see::ConfigService::Instance().Get<int>("optional_slot_number", 800);
    auto optional_slot_number_switch = static_cast<OptionalSlotNumber>(optional_slot_number_);
    int cb_index = 0;
    switch (optional_slot_number_switch) {
    case OptionalSlotNumber::KSlot200:
        cb_index = 0;
        break;
    case OptionalSlotNumber::KSlot400:
        cb_index = 1;
        break;
    case OptionalSlotNumber::KSlot600:
        cb_index = 2;
        break;
    case OptionalSlotNumber::KSlot800:
        cb_index = 3;
        break;
    default:
        cb_index = 3;
        break;
    }
    ui->cb_optional_slot_number->setCurrentIndex(cb_index);

    connect(ui->pb_reset, &QPushButton::clicked, this, [this]() {
        roi_select_type_ = ROISelectType::kROIReset;
        you_see::ConfigService::Instance().Set<int>("cloud_roi.min_slot", 0);
        you_see::ConfigService::Instance().Set<int>("cloud_roi.max_slot", 799);
        you_see::ConfigService::Instance().Set<int>("cloud_roi.min_pixel", 0);
        you_see::ConfigService::Instance().Set<int>("cloud_roi.max_pixel", kPointRoiDefaultMaxPixel);
        emit ROISelectTypeSig(roi_select_type_, PointCloudROIA());
        hide();
        });
    connect(ui->pb_cancel, &QPushButton::clicked, this, [this]() {
        roi_select_type_ = ROISelectType::kROICancel;
        hide();
        });
    connect(ui->pb_confirm, &QPushButton::clicked, this, [this]() {
        roi_select_type_ = ROISelectType::kROISet;
        roi_select_.min_slot = ui->sb_slotmin->value();
        roi_select_.max_slot = ui->sb_slotmax->value();
        roi_select_.min_pixel = ui->sb_pixelmin->value();
        roi_select_.max_pixel = ui->sb_pixelmax->value();
        you_see::ConfigService::Instance().Set<int>("cloud_roi.min_slot", roi_select_.min_slot);
        you_see::ConfigService::Instance().Set<int>("cloud_roi.max_slot", roi_select_.max_slot);
        you_see::ConfigService::Instance().Set<int>("cloud_roi.min_pixel", roi_select_.min_pixel);
        you_see::ConfigService::Instance().Set<int>("cloud_roi.max_pixel", roi_select_.max_pixel);
        emit ROISelectTypeSig(roi_select_type_, roi_select_);
        hide();
    });

    /*connect(ui->cb_optional_slot_number, &QComboBox::currentTextChanged, this, [this](int index) {
        int value = ui->cb_optional_slot_number->itemText(index).toInt();
        emit OptionalSlotNumberSig(value);
    });*/
    connect(ui->cb_optional_slot_number, QOverload<int>::of(&QComboBox::currentIndexChanged), [this](int index) {
        if (index >= 0) {
            // 获取当前项目的数据
            QString current_data = ui->cb_optional_slot_number->itemText(index);
            int value = current_data.toInt();
            emit OptionalSlotNumberSig(value);
            you_see::ConfigService::Instance().Set<int>("optional_slot_number", value);
        }
    });

    // raw histogram max calc offset set slider 
    ui->hs_max_calc_offset->setRange(0, 512);
    connect(ui->hs_max_calc_offset, &QSlider::valueChanged, this, &ROISelect::OnMaxCalOffsetSliderValueChanged);

    // Minimum data proportion of single frame original histogram
    ui->hs_raw_proportion->setRange(0, 100000);
    connect(ui->hs_raw_proportion, &QSlider::valueChanged, this, &ROISelect::OnHistogramDataProportionValueChanged);
}

void ROISelect::OnMaxCalOffsetSliderValueChanged(int value)
{
    ui->label_ma_calc_offset->setText(QString::number(value));
    emit RawHistogramOffsetSig(value);
}

ROISelect::~ROISelect()
{
    delete ui;
}
ROISelectType ROISelect::GetRoiSelectType()
{
    return roi_select_type_;
}

void ROISelect::OnHistogramDataProportionValueChanged(int proportion)
{
    double real_proportion = proportion / 100000.0;
    ui->label_raw_proportion->setText(QString::number(real_proportion, 'f', 5));
    emit RawHistogramDataProportion(real_proportion);
}
} // namespace ys_scene
