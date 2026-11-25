#pragma once

#include <cstdint>

namespace ys_scene {

struct PickedPointInfo
{
    float x = 0.0f;
    float y = 0.0f;
    float z = 0.0f;
    uint8_t reflectivity = 0;

    uint16_t row = 0;
    uint16_t col = 0;

    float theta_deg = 0.0f;
    float omega_deg = 0.0f;
    float rho = 0.0f;

    struct EchoDetail
    {
        bool valid = false;
        uint32_t leading_tof = 0;
        uint32_t trailing_tof = 0;
        uint32_t pulse_width = 0;
        uint16_t peak = 0;
    };
    EchoDetail echoes[3];

    int point_index = -1;
};

} // namespace ys_scene
