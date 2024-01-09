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
        std::shared_ptr<EMIRO::Copter> copter;
        std::shared_ptr<EMIRO::Logger> logger;
        std::shared_ptr<EMIRO::GPS> _gps;
        float speed_limit = 1.0f;
        bool is_init =  false;

        inline void check_init()
        {
            if (!is_init)
            {
                copter->Land();
                throw std::runtime_error("Bad control class implementation.");
            }
        }

    public:
        float vx = 0.0f, vy = 0.0f, vz = 0.0f;
        
        Control();

        Control(std::shared_ptr<EMIRO::Copter> copter, std::shared_ptr<EMIRO::Logger> logger, std::shared_ptr<EMIRO::GPS> gps);

        void go();

        void go(float x, float y, float z, float precision = 0.1f);

        ~Control();
    };

    Control::Control() {}

    Control::Control(std::shared_ptr<EMIRO::Copter> copter, std::shared_ptr<EMIRO::Logger> logger, std::shared_ptr<EMIRO::GPS> gps) : _gps(gps), copter(copter), logger(logger)
    {
        _gps->lock_pos();
        is_init = true;
    }

    void Control::go()
    {
        check_init();
        LinearSpeed speed = {vx, vy, vz};
        _gps->convert(speed);
        copter->set_vel(vx, vy, vz, 0.0f, 0.0f, 0.0f);
    }

    void Control::go(float x, float y, float z, float precision)
    {
        check_init();
        logger->write_show(LogLevel::INFO, "Go : %f, %f, %f", x, y, z);
        std::cout << std::fixed << std::setprecision(2);

        // Loop variable
        ros::Rate r(5);
        EMIRO::Position pos;
        EMIRO::Quaternion quat;

        while(ros::ok())
        {
            // Get current position and orientation
            copter->get_pose(&pos, &quat);
            Euler eul = to_euler(quat.w, quat.x, quat.y, quat.z);

            // Close if position in target zone
            if(std::fabs(pos.x - x) < precision && 
                std::fabs(pos.y - y) < precision)
                break;
            
            float diff_x = x - pos.x;
            float diff_y = y - pos.y;
            float diff_z = z - pos.z;
            vx = diff_x;
            vy = diff_y;
            vz = diff_z;
            if(std::fabs(vx) > speed_limit)
                vx = (vx > 0) ? speed_limit : -speed_limit;
            else if(std::fabs(vx) < precision)
                vx = 0.0f;

            if(std::fabs(vy) > speed_limit)
                vy = (vy > 0) ? speed_limit : -speed_limit;
            else if(std::fabs(vy) < precision)
                vy = 0.0f;

            if(std::fabs(vz) > speed_limit)
                vz = (vz > 0) ? speed_limit : -speed_limit;
            else if(std::fabs(diff_z) < precision)
                vz = 0.0f;

            go();
            
            // Print position
            std::cout << "To target x:" << diff_x << ", y:" << diff_y << ", z:" << diff_z << ", yaw:" << eul.yaw <<  "   \r";
            std::cout.flush();

            ros::spinOnce();
            r.sleep();
        }
        std::cout << std::fixed << std::setprecision(2);
        logger->write_show(LogLevel::INFO, "Reached x:%f->%f, y:%f->%f, z:%f->%f", x, pos.x, y, pos.y, z, pos.z);
    }

    Control::~Control()
    {
    }
}

#endif
