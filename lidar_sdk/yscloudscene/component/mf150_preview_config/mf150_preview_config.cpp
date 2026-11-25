#include "mf150_preview_config.h"

#include <QCheckBox>
#include <QComboBox>
#include <QDialogButtonBox>
#include <QDoubleSpinBox>
#include <QFrame>
#include <QGroupBox>
#include <QHBoxLayout>
#include <QHeaderView>
#include <QLabel>
#include <QPushButton>
#include <QRadioButton>
#include <QTableWidget>
#include <QVBoxLayout>

#include "lumosy/buddies/lumomo_ui/flow_pipeline/flow_pipeline.h"

namespace ys_scene {

Mf150PreviewConfig::Mf150PreviewConfig(QWidget* parent)
    : QDialog(parent)
{
    setWindowTitle(tr("MF150 Preview Config"));
    auto* layout = new QVBoxLayout(this);

    cb_enable_preview_ = new QCheckBox(tr("Enable point cloud preview"), this);
    cb_enable_preview_->setChecked(true);
    layout->addWidget(cb_enable_preview_);

    auto* echo_group = new QGroupBox(tr("Echo Selection"), this);
    auto* echo_layout = new QHBoxLayout(echo_group);

    rb_echo0_ = new QRadioButton(tr("First echo (echo 0)"), echo_group);
    rb_echo1_ = new QRadioButton(tr("Second echo (echo 1)"), echo_group);
    rb_echo2_ = new QRadioButton(tr("Third echo (echo 2)"), echo_group);
    rb_echo0_->setChecked(true);

    echo_layout->addWidget(rb_echo0_);
    echo_layout->addWidget(rb_echo1_);
    echo_layout->addWidget(rb_echo2_);
    layout->addWidget(echo_group);

    connect(cb_enable_preview_, &QCheckBox::toggled, this, [this](bool checked) {
        emit PreviewEnabledChanged(checked);
    });
    connect(rb_echo0_, &QRadioButton::toggled, this, &Mf150PreviewConfig::OnEchoSelectionChanged);
    connect(rb_echo1_, &QRadioButton::toggled, this, &Mf150PreviewConfig::OnEchoSelectionChanged);
    connect(rb_echo2_, &QRadioButton::toggled, this, &Mf150PreviewConfig::OnEchoSelectionChanged);

    SetupSinglePointUi(layout);
    SetupMultiPointUi(layout);

    auto* button_box = new QDialogButtonBox(QDialogButtonBox::Ok | QDialogButtonBox::Cancel, this);
    connect(button_box, &QDialogButtonBox::accepted, this, &QDialog::accept);
    connect(button_box, &QDialogButtonBox::rejected, this, &QDialog::reject);
    layout->addWidget(button_box);
}

void Mf150PreviewConfig::SetupSinglePointUi(QVBoxLayout* layout)
{
    auto* calib_group = new QGroupBox(tr("Single-Point Calibration"), this);
    auto* calib_layout = new QVBoxLayout(calib_group);

    auto* dist_layout = new QHBoxLayout();
    dist_layout->addWidget(new QLabel(tr("Real distance (m):"), calib_group));
    sb_dist_real_ = new QDoubleSpinBox(calib_group);
    sb_dist_real_->setRange(0.1, 100.0);
    sb_dist_real_->setValue(2.5);
    sb_dist_real_->setDecimals(2);
    sb_dist_real_->setSingleStep(0.1);
    sb_dist_real_->setSuffix(" m");
    dist_layout->addWidget(sb_dist_real_);

    dist_layout->addWidget(new QLabel(tr("Reflectivity:"), calib_group));
    cb_refl_single_ = new QComboBox(calib_group);
    cb_refl_single_->addItems({"10%", "30%", "50%", "70%", "80%", "90%"});
    cb_refl_single_->setEditable(true);
    dist_layout->addWidget(cb_refl_single_);
    calib_layout->addLayout(dist_layout);

    auto* checkbox_layout = new QHBoxLayout();
    cb_enable_calib_ = new QCheckBox(tr("Apply calibration table to preview"), calib_group);
    cb_enable_calib_->setEnabled(false);
    connect(cb_enable_calib_, &QCheckBox::toggled, this, &Mf150PreviewConfig::SingleCalibEnabledChanged);
    checkbox_layout->addWidget(cb_enable_calib_);

    cb_sync_save_single_ = new QCheckBox(tr("Auto save raw data"), calib_group);
    checkbox_layout->addWidget(cb_sync_save_single_);
    calib_layout->addLayout(checkbox_layout);

    btn_start_calib_ = new QPushButton(tr("Start Calibration"), calib_group);
    connect(btn_start_calib_, &QPushButton::clicked, this, &Mf150PreviewConfig::OnStartCalibration);

    btn_save_single_raw_ = new QPushButton(tr("Save Raw Data"), calib_group);
    connect(btn_save_single_raw_, &QPushButton::clicked, this, &Mf150PreviewConfig::OnSaveSingleRawData);

    auto* calib_btn_layout = new QHBoxLayout();
    calib_btn_layout->addWidget(btn_start_calib_);
    calib_btn_layout->addWidget(btn_save_single_raw_);
    calib_layout->addLayout(calib_btn_layout);

    flow_pipeline_ = new lumosy::FlowPipeline(calib_group);
    flow_pipeline_->SetNodes({
        tr("Init"),
        tr("Build Table"),
        tr("Compute"),
        tr("Complete")
    });
    calib_layout->addWidget(flow_pipeline_);

    lbl_calib_status_ = new QLabel(calib_group);
    lbl_calib_status_->setWordWrap(true);
    calib_layout->addWidget(lbl_calib_status_);

    layout->addWidget(calib_group);
}

void Mf150PreviewConfig::SetupMultiPointUi(QVBoxLayout* layout)
{
    auto* multi_group = new QGroupBox(tr("Multi-Point Calibration"), this);
    auto* multi_layout = new QVBoxLayout(multi_group);

    auto* collect_layout = new QHBoxLayout();
    collect_layout->addWidget(new QLabel(tr("Current distance (m):"), multi_group));
    sb_multi_dist_real_ = new QDoubleSpinBox(multi_group);
    sb_multi_dist_real_->setRange(0.1, 100.0);
    sb_multi_dist_real_->setValue(2.5);
    sb_multi_dist_real_->setDecimals(2);
    sb_multi_dist_real_->setSingleStep(0.5);
    sb_multi_dist_real_->setSuffix(" m");
    collect_layout->addWidget(sb_multi_dist_real_);

    collect_layout->addWidget(new QLabel(tr("Reflectivity:"), multi_group));
    cb_refl_multi_ = new QComboBox(multi_group);
    cb_refl_multi_->addItems({"10%", "30%", "50%", "70%", "80%", "90%"});
    cb_refl_multi_->setEditable(true);
    collect_layout->addWidget(cb_refl_multi_);

    btn_collect_frame_ = new QPushButton(tr("Collect Frame"), multi_group);
    connect(btn_collect_frame_, &QPushButton::clicked, this, &Mf150PreviewConfig::OnCollectFrame);
    collect_layout->addWidget(btn_collect_frame_);
    multi_layout->addLayout(collect_layout);

    auto* table_label = new QLabel(tr("Collected frames:"), multi_group);
    multi_layout->addWidget(table_label);

    table_collected_ = new QTableWidget(0, 4, multi_group);
    table_collected_->setHorizontalHeaderLabels({
        tr("No."),
        tr("Distance (m)"),
        tr("Reflectivity"),
        tr("Status")
    });
    table_collected_->horizontalHeader()->setStretchLastSection(true);
    table_collected_->horizontalHeader()->setSectionResizeMode(0, QHeaderView::ResizeToContents);
    table_collected_->horizontalHeader()->setSectionResizeMode(1, QHeaderView::Stretch);
    table_collected_->setSelectionBehavior(QAbstractItemView::SelectRows);
    table_collected_->setSelectionMode(QAbstractItemView::SingleSelection);
    table_collected_->setEditTriggers(QAbstractItemView::NoEditTriggers);
    table_collected_->setMinimumHeight(200);
    table_collected_->setMaximumHeight(400);
    multi_layout->addWidget(table_collected_);

    btn_delete_selected_ = new QPushButton(tr("Delete Selected"), multi_group);
    connect(btn_delete_selected_, &QPushButton::clicked, this, &Mf150PreviewConfig::OnDeleteSelectedFrame);
    multi_layout->addWidget(btn_delete_selected_);

    auto* roi_control_layout = new QHBoxLayout();
    btn_toggle_roi_ = new QPushButton(tr("Select ROI"), multi_group);
    btn_toggle_roi_->setCheckable(true);
    connect(btn_toggle_roi_, &QPushButton::clicked, this, &Mf150PreviewConfig::OnToggleRoiPickMode);
    roi_control_layout->addWidget(btn_toggle_roi_);

    cb_show_roi_ = new QCheckBox(tr("Show ROI"), multi_group);
    cb_show_roi_->setChecked(true);
    connect(cb_show_roi_, &QCheckBox::toggled, this, &Mf150PreviewConfig::RoiVisibilityChanged);
    roi_control_layout->addWidget(cb_show_roi_);

    lbl_roi_info_ = new QLabel(multi_group);
    roi_control_layout->addWidget(lbl_roi_info_);
    roi_control_layout->addStretch();
    multi_layout->addLayout(roi_control_layout);

    auto* checkbox_layout = new QHBoxLayout();
    cb_enable_multi_calib_ = new QCheckBox(tr("Apply multi-point calibration to preview"), multi_group);
    cb_enable_multi_calib_->setEnabled(false);
    connect(cb_enable_multi_calib_, &QCheckBox::toggled, this, &Mf150PreviewConfig::MultiCalibEnabledChanged);
    checkbox_layout->addWidget(cb_enable_multi_calib_);

    cb_sync_save_multi_ = new QCheckBox(tr("Auto save raw data"), multi_group);
    checkbox_layout->addWidget(cb_sync_save_multi_);
    multi_layout->addLayout(checkbox_layout);

    btn_start_multi_calib_ = new QPushButton(tr("Start Multi-Point Calibration"), multi_group);
    btn_start_multi_calib_->setEnabled(false);
    connect(btn_start_multi_calib_, &QPushButton::clicked, this, &Mf150PreviewConfig::OnStartMultiCalibration);

    btn_save_multi_raw_ = new QPushButton(tr("Save Raw Data"), multi_group);
    connect(btn_save_multi_raw_, &QPushButton::clicked, this, &Mf150PreviewConfig::OnSaveMultiRawData);

    auto* multi_btn_layout = new QHBoxLayout();
    multi_btn_layout->addWidget(btn_start_multi_calib_);
    multi_btn_layout->addWidget(btn_save_multi_raw_);
    multi_layout->addLayout(multi_btn_layout);

    multi_flow_pipeline_ = new lumosy::FlowPipeline(multi_group);
    multi_flow_pipeline_->SetNodes({
        tr("Init"),
        tr("Cluster"),
        tr("Clean"),
        tr("Compute"),
        tr("Fit")
    });
    multi_layout->addWidget(multi_flow_pipeline_);

    lbl_multi_calib_status_ = new QLabel(multi_group);
    lbl_multi_calib_status_->setWordWrap(true);
    multi_layout->addWidget(lbl_multi_calib_status_);

    // 新版多点标定 = 每像素距离 LUT 查表模型，此处显示表概况
    lbl_multi_summary_ = new QLabel(tr("LUT: --"), multi_group);
    lbl_multi_summary_->setWordWrap(true);
    multi_layout->addWidget(lbl_multi_summary_);

    layout->addWidget(multi_group);
}

void Mf150PreviewConfig::SetPreviewEnabled(bool enabled)
{
    cb_enable_preview_->setChecked(enabled);
}

void Mf150PreviewConfig::SetEchoIndex(int index)
{
    switch (index) {
    case 0: rb_echo0_->setChecked(true); break;
    case 1: rb_echo1_->setChecked(true); break;
    case 2: rb_echo2_->setChecked(true); break;
    }
}

bool Mf150PreviewConfig::IsPreviewEnabled() const
{
    return cb_enable_preview_->isChecked();
}

int Mf150PreviewConfig::GetEchoIndex() const
{
    if (rb_echo0_->isChecked()) return 0;
    if (rb_echo1_->isChecked()) return 1;
    if (rb_echo2_->isChecked()) return 2;
    return 0;
}

void Mf150PreviewConfig::SetCalibrationCallback(CalibrationCallback cb)
{
    calib_callback_ = std::move(cb);
}

void Mf150PreviewConfig::SetCalibrationEnabled(bool enabled)
{
    cb_enable_calib_->setChecked(enabled);
}

bool Mf150PreviewConfig::IsCalibrationEnabled() const
{
    return cb_enable_calib_->isChecked();
}

void Mf150PreviewConfig::SetCalibMapLoaded(bool loaded)
{
    cb_enable_calib_->setEnabled(loaded);
    if (loaded) {
        lbl_calib_status_->setText(tr("Calibration table loaded."));
        lbl_calib_status_->setStyleSheet("color: green;");
    } else {
        lbl_calib_status_->setText(tr("No calibration table loaded."));
        lbl_calib_status_->setStyleSheet("color: gray;");
    }
}

lumosy::FlowPipeline* Mf150PreviewConfig::GetFlowPipeline() const
{
    return flow_pipeline_;
}

void Mf150PreviewConfig::SetCollectFrameCallback(CollectFrameCallback cb)
{
    collect_frame_callback_ = std::move(cb);
}

void Mf150PreviewConfig::SetDeleteFrameCallback(DeleteFrameCallback cb)
{
    delete_frame_callback_ = std::move(cb);
}

void Mf150PreviewConfig::SetMultiCalibCallback(MultiCalibCallback cb)
{
    multi_calib_callback_ = std::move(cb);
}

void Mf150PreviewConfig::SetSaveSingleRawDataCallback(SaveRawDataCallback cb)
{
    save_single_raw_data_callback_ = std::move(cb);
}

void Mf150PreviewConfig::SetSaveMultiRawDataCallback(SaveRawDataCallback cb)
{
    save_multi_raw_data_callback_ = std::move(cb);
}

void Mf150PreviewConfig::SetRoiPickModeCallback(RoiPickModeCallback cb)
{
    roi_pick_mode_callback_ = std::move(cb);
}

void Mf150PreviewConfig::SetRoiInfo(int row, int col, int width, int height)
{
    if (lbl_roi_info_) {
        lbl_roi_info_->setText(tr("R:%1 C:%2 W:%3 H:%4")
            .arg(row).arg(col).arg(width).arg(height));
    }
}

void Mf150PreviewConfig::SetRoiPickModeActive(bool active)
{
    if (btn_toggle_roi_) {
        btn_toggle_roi_->setChecked(active);
        btn_toggle_roi_->setText(active ? tr("Cancel ROI") : tr("Select ROI"));
    }
}

void Mf150PreviewConfig::OnToggleRoiPickMode()
{
    bool active = btn_toggle_roi_->isChecked();
    btn_toggle_roi_->setText(active ? tr("Cancel ROI") : tr("Select ROI"));
    if (roi_pick_mode_callback_) {
        roi_pick_mode_callback_(active);
    }
}

void Mf150PreviewConfig::SetMultiCalibEnabled(bool enabled)
{
    cb_enable_multi_calib_->setChecked(enabled);
}

bool Mf150PreviewConfig::IsMultiCalibEnabled() const
{
    return cb_enable_multi_calib_->isChecked();
}

void Mf150PreviewConfig::SetMultiCalibMapLoaded(bool loaded)
{
    cb_enable_multi_calib_->setEnabled(loaded);
    if (loaded) {
        lbl_multi_calib_status_->setText(tr("Multi-point calibration loaded."));
        lbl_multi_calib_status_->setStyleSheet("color: green;");
    } else {
        lbl_multi_calib_status_->setText(tr("No multi-point calibration loaded."));
        lbl_multi_calib_status_->setStyleSheet("color: gray;");
    }
}

lumosy::FlowPipeline* Mf150PreviewConfig::GetMultiFlowPipeline() const
{
    return multi_flow_pipeline_;
}

void Mf150PreviewConfig::OnStartCalibration()
{
    if (!calib_callback_) {
        return;
    }

    float dist_real = static_cast<float>(sb_dist_real_->value());
    std::string calib_dir = "calibration/mf150";
    std::string bin_path = calib_dir + "/map_solid.bin";

    flow_pipeline_->ResetAll();
    flow_pipeline_->SetStatus(0, lumosy::FlowStatus::KInProgress);
    lbl_calib_status_->setText(tr("Starting calibration..."));

    btn_start_calib_->setEnabled(false);

    calib_callback_(dist_real, bin_path);

    btn_start_calib_->setEnabled(true);
}

void Mf150PreviewConfig::OnCollectFrame()
{
    if (!collect_frame_callback_) {
        return;
    }

    float dist_real = static_cast<float>(sb_multi_dist_real_->value());

    if (collect_frame_callback_(dist_real)) {
        int row = table_collected_->rowCount();
        table_collected_->insertRow(row);
        table_collected_->setItem(row, 0, new QTableWidgetItem(QString::number(row + 1)));
        table_collected_->setItem(row, 1, new QTableWidgetItem(
            QString::number(dist_real, 'f', 2) + " m"));
        table_collected_->setItem(row, 2, new QTableWidgetItem(cb_refl_multi_->currentText()));
        table_collected_->setItem(row, 3, new QTableWidgetItem(tr("Collected")));

        btn_start_multi_calib_->setEnabled(table_collected_->rowCount() >= 2);
    }
}

void Mf150PreviewConfig::OnDeleteSelectedFrame()
{
    auto selected = table_collected_->selectionModel()->selectedRows();
    if (selected.isEmpty()) {
        return;
    }

    int row = selected.first().row();
    table_collected_->removeRow(row);

    for (int i = 0; i < table_collected_->rowCount(); ++i) {
        table_collected_->item(i, 0)->setText(QString::number(i + 1));
    }

    if (delete_frame_callback_) {
        delete_frame_callback_(row);
    }

    btn_start_multi_calib_->setEnabled(table_collected_->rowCount() >= 2);
}

void Mf150PreviewConfig::OnStartMultiCalibration()
{
    if (!multi_calib_callback_) {
        return;
    }

    std::string calib_dir = "calibration/mf150";
    std::string bin_path = calib_dir + "/map_solid.bin";

    multi_flow_pipeline_->ResetAll();
    multi_flow_pipeline_->SetStatus(0, lumosy::FlowStatus::KInProgress);
    lbl_multi_calib_status_->setText(tr("Starting multi-point calibration..."));

    btn_collect_frame_->setEnabled(false);
    btn_delete_selected_->setEnabled(false);
    btn_start_multi_calib_->setEnabled(false);

    multi_calib_callback_(bin_path);

    btn_collect_frame_->setEnabled(true);
    btn_delete_selected_->setEnabled(true);
    btn_start_multi_calib_->setEnabled(table_collected_->rowCount() >= 2);
}

void Mf150PreviewConfig::OnEchoSelectionChanged()
{
    auto* btn = qobject_cast<QRadioButton*>(sender());
    if (btn && btn->isChecked()) {
        emit EchoIndexChanged(GetEchoIndex());
    }
}

void Mf150PreviewConfig::OnSaveSingleRawData()
{
    if (save_single_raw_data_callback_) {
        save_single_raw_data_callback_();
    }
}

void Mf150PreviewConfig::OnSaveMultiRawData()
{
    if (save_multi_raw_data_callback_) {
        save_multi_raw_data_callback_();
    }
}

bool Mf150PreviewConfig::IsSingleSyncSaveEnabled() const
{
    return cb_sync_save_single_ && cb_sync_save_single_->isChecked();
}

bool Mf150PreviewConfig::IsMultiSyncSaveEnabled() const
{
    return cb_sync_save_multi_ && cb_sync_save_multi_->isChecked();
}

std::string Mf150PreviewConfig::GetSingleSyncSaveReflectivity() const
{
    if (!cb_refl_single_) return "10%";
    return cb_refl_single_->currentText().toStdString();
}

std::string Mf150PreviewConfig::GetMultiSyncSaveReflectivity() const
{
    if (!cb_refl_multi_) return "10%";
    return cb_refl_multi_->currentText().toStdString();
}

void Mf150PreviewConfig::SetMultiCalibSummary(int valid_pixels, int max_dist_pts)
{
    if (lbl_multi_summary_) {
        lbl_multi_summary_->setText(tr("LUT: %1 valid pixels, max %2 distance point(s)")
            .arg(valid_pixels).arg(max_dist_pts));
    }
}

} // namespace ys_scene
