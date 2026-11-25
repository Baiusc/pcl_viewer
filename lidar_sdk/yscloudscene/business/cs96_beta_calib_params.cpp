#include "cs96_beta_calib_params.h"

#include <cmath>
#include <limits>

namespace ys_scene {

Cs96BetaCalibParams::Cs96BetaCalibParams()
{
    starting_angle_.resize(kFaces, 0.0f);
    theta_.resize(kLinesVertical, 0.0f);
    omega_delta_.resize(kLinesVertical, 0.0f);
    d_delta_.resize(kLinesVertical, 0.0f);
    a_.resize(kLinesVertical, 1.0f);
    ld_intensity_.resize(kLinesVertical, 702.0f);
}

void Cs96BetaCalibParams::Init(const std::string& ini_path, const std::string& encoder_csv_path)
{
    inited_ = false;
}

void Cs96BetaCalibParams::CalculateTheta()
{
    float theta_min = -10.0f;
    float theta_max = 10.0f;
    float theta_tilt = -2.5f;

    float step = (theta_max - theta_min) / static_cast<float>(lines_v_);

    for (int i = lines_v_ - 1, j = 0; i >= 0; i--, j++) {
        float tmp_theta = static_cast<float>(i - u_) * step + theta_tilt;
        theta_[j] = tmp_theta * static_cast<float>(kRadian);
    }
    inited_ = true;
}

float Cs96BetaCalibParams::CalculateHorizontalAngle(int frame_id, uint32_t mirror_angle, uint16_t slot_number)
{
    float starting_angle = (frame_id >= 0 && frame_id < static_cast<int>(starting_angle_.size()))
        ? starting_angle_[frame_id] : 0.0f;

    float horizontal_corner = 0.0f;

    if (slot_number != 0) {
        int scale = static_cast<int>((mirror_angle >> 12) & 0xFF);
        int real_angle = static_cast<int>(mirror_angle & 0xFFF);

        if (scale >= static_cast<int>(encoder_angle_table_.size())) {
            // 与下位机一致：不再使用 mirror_angle 兜底计算，scale 越界视为无效数据
            return std::numeric_limits<float>::quiet_NaN();
        }

        float angle = encoder_angle_table_[scale] + 0.0009f * static_cast<float>(real_angle);

        horizontal_corner = angle - starting_angle;

        if (horizontal_corner > 180.0f) {
            horizontal_corner -= 360.0f;
        } else if (horizontal_corner < -180.0f) {
            horizontal_corner += 360.0f;
        }
    } else {
        horizontal_corner = omega0_;
    }

    horizontal_corner = horizontal_corner * 2.0f + omega0_;

    return horizontal_corner;
}

int Cs96BetaCalibParams::GetFaceIdByScale(uint32_t mirror_angle)
{
    int scale = static_cast<int>((mirror_angle >> 12) & 0xFF);

    // 4 个面起始发光刻度序号是 8，68，128，188
    if (9 <= scale && scale < 69) {
        return 0;
    }
    if (69 <= scale && scale < 129) {
        return 1;
    }
    if (129 <= scale && scale < 189) {
        return 2;
    }
    return 3; // 189 <= scale || scale < 9
}

float Cs96BetaCalibParams::CalculateDistance(int pixel_index, uint32_t ft) const
{
    if (ft <= kErrorFt) {
        return 0.0f;
    }

    float a = (pixel_index < static_cast<int>(a_.size())) ? a_[pixel_index] : 1.0f;
    float d_delta = (pixel_index < static_cast<int>(d_delta_.size())) ? d_delta_[pixel_index] : 0.0f;

    float r = kLightSpeed * static_cast<float>(ft >> 1) * a + d_delta;
    return r;
}

float Cs96BetaCalibParams::Map2RangeRadian(int pixel_index, float mirror_angle) const
{
    float omega_delta = (pixel_index < static_cast<int>(omega_delta_.size()))
        ? omega_delta_[pixel_index] : 0.0f;

    // 注意: 这里不再叠加 omega0_。omega0 已包含在 CalculateHorizontalAngle 的
    // "hc*2 + omega0" 中（与下位机 get_horizontal_corners + decode 一致），
    // 此处仅叠加逐线水平角偏移 omega_delta。
    return (mirror_angle + omega_delta) * static_cast<float>(kRadian);
}

} 
