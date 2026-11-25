#ifndef YS_SCENE_CS96_BETA_PARAM_LOADER_H_
#define YS_SCENE_CS96_BETA_PARAM_LOADER_H_

#include <memory>
#include <string>

namespace ys_scene {

class Cs96BetaCalibParams;

class Cs96BetaParamLoader
{
public:
    static bool LoadFromIni(Cs96BetaCalibParams& params, const std::string& ini_path);
    static bool LoadEncoderAngle(Cs96BetaCalibParams& params, const std::string& csv_path);
};

} 

#endif
