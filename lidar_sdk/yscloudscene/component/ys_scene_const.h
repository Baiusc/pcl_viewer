#ifndef YS_SCENE_CONST_H_
#define YS_SCENE_CONST_H_

#include <cmath>
#include <limits>

namespace ys_scene {
	constexpr float ZERO_TOLERANCE_F = std::numeric_limits<float>::epsilon();
	constexpr double ZERO_TOLERANCE_D = static_cast<double>(ZERO_TOLERANCE_F);
	constexpr float PC_ONE = static_cast<float>(1.0);
	
} // namespace ys_scene

#endif // YS_SCENE_CONST_H_