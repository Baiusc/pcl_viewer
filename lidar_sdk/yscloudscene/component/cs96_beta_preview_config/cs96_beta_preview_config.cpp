#include "cs96_beta_preview_config.h"

#include <QCheckBox>
#include <QFileDialog>
#include <QGroupBox>
#include <QHBoxLayout>
#include <QLabel>
#include <QLineEdit>
#include <QPushButton>
#include <QVBoxLayout>

namespace ys_scene {

Cs96BetaPreviewConfig::Cs96BetaPreviewConfig(QWidget* parent)
    : QDialog(parent)
{
    setWindowTitle(tr("CS96 Beta Preview Configuration"));
    setMinimumSize(500, 300);

    auto* layout = new QVBoxLayout(this);

    auto* preview_group = new QGroupBox(tr("Preview Control"), this);
    auto* preview_layout = new QVBoxLayout(preview_group);

    cb_enable_preview_ = new QCheckBox(tr("Enable CS96 Beta Preview"), preview_group);
    preview_layout->addWidget(cb_enable_preview_);

    layout->addWidget(preview_group);

    auto* config_group = new QGroupBox(tr("Calibration Files"), this);
    auto* config_layout = new QVBoxLayout(config_group);

    auto* ini_layout = new QHBoxLayout();
    ini_layout->addWidget(new QLabel(tr("ys_calibration.ini:"), config_group));
    le_ini_path_ = new QLineEdit(config_group);
    le_ini_path_->setPlaceholderText(tr("calibration/cs96_beta/ys_calibration.ini"));
    ini_layout->addWidget(le_ini_path_);
    btn_browse_ini_ = new QPushButton(tr("Browse..."), config_group);
    ini_layout->addWidget(btn_browse_ini_);
    config_layout->addLayout(ini_layout);

    auto* csv_layout = new QHBoxLayout();
    csv_layout->addWidget(new QLabel(tr("ys_encoder_angle.csv:"), config_group));
    le_csv_path_ = new QLineEdit(config_group);
    le_csv_path_->setPlaceholderText(tr("calibration/cs96_beta/ys_encoder_angle.csv"));
    csv_layout->addWidget(le_csv_path_);
    btn_browse_csv_ = new QPushButton(tr("Browse..."), config_group);
    csv_layout->addWidget(btn_browse_csv_);
    config_layout->addLayout(csv_layout);

    auto* btn_layout = new QHBoxLayout();
    btn_load_config_ = new QPushButton(tr("Load Configuration"), config_group);
    btn_layout->addWidget(btn_load_config_);
    btn_layout->addStretch();
    config_layout->addLayout(btn_layout);

    lbl_config_status_ = new QLabel(tr("Not loaded"), config_group);
    config_layout->addWidget(lbl_config_status_);

    layout->addWidget(config_group);

    layout->addStretch();

    connect(btn_browse_ini_, &QPushButton::clicked, this, &Cs96BetaPreviewConfig::OnBrowseIni);
    connect(btn_browse_csv_, &QPushButton::clicked, this, &Cs96BetaPreviewConfig::OnBrowseCsv);
    connect(btn_load_config_, &QPushButton::clicked, this, &Cs96BetaPreviewConfig::OnLoadConfig);
    connect(cb_enable_preview_, &QCheckBox::toggled, this, &Cs96BetaPreviewConfig::PreviewEnabledChanged);
}

void Cs96BetaPreviewConfig::SetPreviewEnabled(bool enabled)
{
    cb_enable_preview_->setChecked(enabled);
}

bool Cs96BetaPreviewConfig::IsPreviewEnabled() const
{
    return cb_enable_preview_->isChecked();
}

void Cs96BetaPreviewConfig::SetLoadConfigCallback(LoadConfigCallback cb)
{
    load_config_callback_ = std::move(cb);
}

void Cs96BetaPreviewConfig::SetConfigStatus(bool loaded, const std::string& status_msg)
{
    if (loaded) {
        lbl_config_status_->setStyleSheet("color: green;");
    } else {
        lbl_config_status_->setStyleSheet("color: red;");
    }
    lbl_config_status_->setText(QString::fromStdString(status_msg));
}

void Cs96BetaPreviewConfig::OnBrowseIni()
{
    QString path = QFileDialog::getOpenFileName(this,
        tr("Select ys_calibration.ini"),
        "calibration/cs96_beta",
        tr("INI files (*.ini);;All files (*)"));
    if (!path.isEmpty()) {
        le_ini_path_->setText(path);
    }
}

void Cs96BetaPreviewConfig::OnBrowseCsv()
{
    QString path = QFileDialog::getOpenFileName(this,
        tr("Select ys_encoder_angle.csv"),
        "calibration/cs96_beta",
        tr("CSV files (*.csv);;All files (*)"));
    if (!path.isEmpty()) {
        le_csv_path_->setText(path);
    }
}

void Cs96BetaPreviewConfig::OnLoadConfig()
{
    if (!load_config_callback_) {
        SetConfigStatus(false, "No callback set");
        return;
    }

    std::string ini_path = le_ini_path_->text().toStdString();
    std::string csv_path = le_csv_path_->text().toStdString();

    if (ini_path.empty() || csv_path.empty()) {
        SetConfigStatus(false, "Please select both calibration files");
        return;
    }

    lbl_config_status_->setText(tr("Loading..."));
    lbl_config_status_->setStyleSheet("color: orange;");

    bool success = load_config_callback_(ini_path, csv_path);

    if (success) {
        SetConfigStatus(true, "Configuration loaded successfully");
    } else {
        SetConfigStatus(false, "Failed to load configuration");
    }
}

} 
