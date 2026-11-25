#include "cs96_beta_param_loader.h"
#include "cs96_beta_calib_params.h"

#include <QFile>
#include <QSettings>
#include <QString>
#include <QStringList>
#include <QTextStream>

#include <cmath>
#include <filesystem>

namespace ys_scene {

bool Cs96BetaParamLoader::LoadFromIni(Cs96BetaCalibParams& params, const std::string& ini_path)
{
    if (!std::filesystem::exists(ini_path)) {
        return false;
    }

    QSettings ini(QString::fromStdString(ini_path), QSettings::IniFormat);

    params.lines_v_ = ini.value("parameter/lines_v", 96).toInt();
    params.lines_h_ = ini.value("parameter/lines_h", 800).toInt();
    params.omega0_ = ini.value("parameter/omega0", -60.0f).toFloat();
    params.u_ = ini.value("parameter/u", 47).toInt();

    double theta_min = ini.value("parameter/theta_min", -10.0).toDouble();
    double theta_max = ini.value("parameter/theta_max", 10.0).toDouble();
    double theta_tilt = ini.value("parameter/theta_tilt", -2.5).toDouble();

    params.sat_peak_max_ = ini.value("parameter/sat_peak_max", 1612).toInt();
    params.sat_peak_min_ = ini.value("parameter/sat_peak_min", 0).toInt();

    std::vector<uint16_t> angle_face(params.kFaces, 0);
    std::vector<uint16_t> side_angular_offset(params.kFaces, 0);

    for (int i = 0; i < params.kFaces; i++) {
        QString key = QString("standard_encoder/angle_face_%1").arg(i + 1);
        angle_face[i] = static_cast<uint16_t>(ini.value(key, 0).toUInt());
    }

    for (int i = 0; i < params.kFaces; i++) {
        QString key = QString("side_angular_offset/side%1_angular_offset").arg(i);
        side_angular_offset[i] = static_cast<uint16_t>(ini.value(key, 0).toUInt());
    }

    for (int i = 0; i < params.kFaces; i++) {
        params.starting_angle_[i] = static_cast<float>(
            (angle_face[i] * 256 + side_angular_offset[i])) / 256.0f * 360.0f / 240.0f;
    }

    params.omega_delta_.resize(params.lines_v_, 0.0f);
    for (int i = 0; i < params.lines_v_; i++) {
        QString key = QString("horizontal_angular_offset/omega_delta%1").arg(i);
        params.omega_delta_[i] = ini.value(key, -1.3f).toFloat();
    }

    params.d_delta_.resize(params.lines_v_, 0.0f);
    for (int i = 0; i < params.lines_v_; i++) {
        QString key = QString("zero_distance_offset/d%1_delta").arg(i);
        params.d_delta_[i] = ini.value(key, 0.0f).toFloat();
    }

    params.a_.resize(params.lines_v_, 1.0f);
    for (int i = 0; i < params.lines_v_; i++) {
        QString key = QString("distance_factor/a%1").arg(i);
        params.a_[i] = ini.value(key, 1.0f).toFloat();
    }

    params.CalculateTheta();

    return true;
}

bool Cs96BetaParamLoader::LoadEncoderAngle(Cs96BetaCalibParams& params, const std::string& csv_path)
{
    QFile file(QString::fromStdString(csv_path));
    if (!file.open(QIODevice::ReadOnly | QIODevice::Text)) {
        return false;
    }

    params.encoder_angle_table_.clear();

    QTextStream stream(&file);
    bool is_header = true;

    while (!stream.atEnd()) {
        QString line = stream.readLine().trimmed();
        if (line.isEmpty()) {
            continue;
        }

        if (is_header) {
            is_header = false;
            continue;
        }

        QStringList cols = line.split(',');
        if (cols.size() < 2) {
            continue;
        }

        bool ok = false;
        cols[0].toInt(&ok);
        if (!ok) {
            continue;
        }

        float angle = cols[1].toFloat();
        params.encoder_angle_table_.push_back(angle);
    }

    file.close();
    return !params.encoder_angle_table_.empty();
}

} 
