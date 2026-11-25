#ifndef YS_SCENE_CS96_BETA_CALIB_PARAMS_H_
#define YS_SCENE_CS96_BETA_CALIB_PARAMS_H_

#include <cstdint>
#include <string>
#include <vector>

namespace ys_scene {

class Cs96BetaCalibParams
{
public:
    Cs96BetaCalibParams();

    bool IsInited() const { return inited_; }

    void Init(const std::string& ini_path, const std::string& encoder_csv_path);
    void CalculateTheta();
    float CalculateHorizontalAngle(int frame_id, uint32_t mirror_angle, uint16_t slot_number);
    float CalculateDistance(int pixel_index, uint32_t ft) const;
    float Map2RangeRadian(int pixel_index, float mirror_angle) const;

    // 通过转镜角度高 8 位(scale) 判断所在面: 面起始发光刻度 8/68/128/188
    static int GetFaceIdByScale(uint32_t mirror_angle);

    static constexpr int kLinesVertical = 96;
    static constexpr int kLinesHorizontal = 800;
    static constexpr int kFaces = 4;
    static constexpr double kRadian = (3.14159265358979323846 / 180.0);
    static constexpr float kLightSpeed = 299792458.0f * 1e-12f;

    int lines_v_{kLinesVertical};
    int lines_h_{kLinesHorizontal};
    float omega0_{-60.0f};
    int u_{47};

    int sat_peak_max_{1612};
    int sat_peak_min_{0};

    std::vector<float> encoder_angle_table_;
    std::vector<float> starting_angle_;
    std::vector<float> theta_;
    std::vector<float> omega_delta_;
    std::vector<float> d_delta_;
    std::vector<float> a_;
    std::vector<float> ld_intensity_;

private:
    bool inited_{false};

    static constexpr uint32_t kErrorFt = 0x10;
};

} 

#endif
