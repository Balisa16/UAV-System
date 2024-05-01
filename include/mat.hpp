#pragma once

#include <types.hpp>
#include <math.h>

inline EMIRO::Euler to_euler(double w, double x, double y, double z)
{
    double sinr_cosp = 2 * (w * x + y * z);
    double cosr_cosp = 1 - 2 * (x * x + y * y);
    // angles.roll = std::atan2(sinr_cosp, cosr_cosp);

    // pitch (y-axis rotation)
    double sinp = std::sqrt(1 + 2 * (w * y - x * z));
    double cosp = std::sqrt(1 - 2 * (w * y - x * z));
    // angles.pitch = 2 * std::atan2(sinp, cosp) - M_PI / 2;

    // yaw (z-axis rotation)
    double siny_cosp = 2 * (w * z + x * y);
    double cosy_cosp = 1 - 2 * (y * y + z * z);
    // angles.yaw = std::atan2(siny_cosp, cosy_cosp);

    return {(float)std::atan2(sinr_cosp, cosr_cosp),
            (float)(2.0f * std::atan2(sinp, cosp) - M_PI / 2.0f),
            (float)std::atan2(siny_cosp, cosy_cosp)};
}