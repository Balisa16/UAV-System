#pragma once

#include <copter.hpp>
#include <gps.hpp>
#include <mat.hpp>
#include <simplepid.hpp>

namespace EMIRO
{
    class Control
    {
    private:
        std::shared_ptr<Copter> copter;
        std::shared_ptr<Logger> logger;
        std::shared_ptr<GPS> _gps;
        bool is_init = false;

        inline void check_init()
        {
            if (!is_init)
            {
                copter->Land();
                throw std::runtime_error("Bad control class implementation.");
            }
        }
        void go(bool yaw_control = true);

    public:
        float speed_limit = 1.0f;              // m/s
        int rpy_speed_limit = 1.55;            // rad/s
        float vx = 0.0f, vy = 0.0f, vz = 0.0f; // m/s
        float avx = 0, avy = 0, avz = 0;       // rad/s

        Control();

        Control(std::shared_ptr<Copter> copter, std::shared_ptr<Logger> logger, std::shared_ptr<GPS> gps);

        void go(const float &x, const float &y, const float &z,
                const int &yaw, const float &linear_precision = 0.2, const int &angular_precision = 5);

        ~Control();
    };
}