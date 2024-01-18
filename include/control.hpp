#ifndef CONTROL_HPP
#define CONTROL_HPP

#include <copter.hpp>
#include <gps.hpp>
#include <mat.hpp>

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

    Control::Control() {}

    Control::Control(std::shared_ptr<Copter> copter, std::shared_ptr<Logger> logger, std::shared_ptr<GPS> gps) : _gps(gps), copter(copter), logger(logger)
    {
        _gps->lock_pos();
        is_init = true;
    }

    void Control::go(bool yaw_control)
    {
        check_init();
        LinearSpeed speed = {vx, vy, vz};
        _gps->convert(speed);
        if (yaw_control)
            copter->set_vel(vx, vy, vz, avx, avy, avz);
        else
            copter->set_vel(vx, vy, vz, 0.0f, 0.0f, 0.0f);
    }

    inline void Control::go(const float &x, const float &y, const float &z,
                            const int &yaw, const float &linear_precision, const int &angular_precision)
    {
        check_init();
        logger->write_show(LogLevel::INFO, "Go : %.2f, %.2f, %.2f, %d", x, y, z, yaw);
        std::cout << std::fixed << std::setprecision(2);

        // Loop variable
        ros::Rate r(5);
        Position pos;
        Quaternion quat;
        Euler eul;
        float _yaw = 0.0f;

        while (ros::ok())
        {
            // Get current position and orientation
            copter->get_pose(&pos, &quat);
            eul = to_euler(quat.w, quat.x, quat.y, quat.z);
            eul.roll *= (180.0f / M_PI);
            eul.pitch *= (180.0f / M_PI);
            eul.yaw *= (180.0f / M_PI);
            eul.yaw = eul.yaw > 180 ? eul.yaw - 360 : eul.yaw;
            _yaw = eul.yaw;

            // Close if position in target zone
            if (std::fabs(pos.x - x) < linear_precision &&
                std::fabs(pos.y - y) < linear_precision &&
                std::fabs(pos.z - z) < linear_precision &&
                std::fabs(eul.yaw - yaw) < angular_precision)
                break;

            float diff_x = x - pos.x;
            float diff_y = y - pos.y;
            float diff_z = z - pos.z;
            float diff_yaw = yaw - eul.yaw;
            diff_yaw = diff_yaw > 180 ? -(360 - diff_yaw) : diff_yaw;
            vx = diff_x;
            vy = diff_y;
            vz = diff_z;
            avz = diff_yaw;
            if (std::fabs(vx) > speed_limit)
                vx = (vx > 0) ? speed_limit : -speed_limit;
            else if (std::fabs(vx) < linear_precision)
                vx = 0.0f;

            if (std::fabs(vy) > speed_limit)
                vy = (vy > 0) ? speed_limit : -speed_limit;
            else if (std::fabs(vy) < linear_precision)
                vy = 0.0f;

            if (std::fabs(vz) > speed_limit)
                vz = (vz > 0) ? speed_limit : -speed_limit;
            else if (std::fabs(diff_z) < linear_precision)
                vz = 0.0f;

            avz = diff_yaw * 3.14 / 180.0f;
            if (std::fabs(avz) > rpy_speed_limit)
                avz = (avz > 0) ? rpy_speed_limit : -rpy_speed_limit;
            else if (std::fabs(eul.yaw - yaw) < angular_precision)
                avz = 0.0f;

            // go(true);
            copter->set_vel(vx, vy, vz, 0.0f, 0.0f, avz);

            // Print position
            std::cout << C_MAGENTA << S_BOLD << " >>> " << C_RESET << "Target (" << diff_x << ", " << diff_y << ", " << diff_z << ", " << diff_yaw << "°)     \r";
            std::cout.flush();

            ros::spinOnce();
            r.sleep();
        }
        logger->write_show(LogLevel::INFO, "Reached (%.2f, %.2f, %.2f, %d°) => (%.2f, %.2f, %.2f, %d°)",
                           x, y, z, pos.x, pos.y, pos.z, (int)yaw, (int)_yaw);
    }

    Control::~Control()
    {
    }
}

#endif
