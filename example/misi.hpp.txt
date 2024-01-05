
#include <convert.hpp>
#include <enum.hpp>
#include <copter.hpp>
#include <servo.hpp>
#include <lidar.hpp>
#include <gps.hpp>
#include <rangefinder.hpp>

#include <iostream>
#include <cstring>
#include <ncurses.h>
#include <thread>
#include <chrono>
#include <cmath>
#include <memory>
#include <jsonread.hpp>

namespace EMIRO{
    class Misi2
    {
    private:
        std::shared_ptr<ros::NodeHandle> nh;
        std::shared_ptr<EMIRO::Copter> copter;
        std::shared_ptr<EMIRO::Logger> logger;
        // EMIRO::Servo ser;
        // EMIRO::Nav_Convert converter;
        // EMIRO::GlobalNav global_nav;
        // EMIRO::Lidar lidar_dev;
        // EMIRO::RangeFinder rangefinder;
        std::shared_ptr<EMIRO::JSONReader> jsonreader;

        EMIRO::WayPoint _start_pos;

        EMIRO::WayPoint lock_pos(int min_counter = 1, const char *title = "Press y to lock position", bool nav_change = false);


    public:

        Misi2(std::shared_ptr<ros::NodeHandle> nh_ptr, int argc, char **argv):
            nh(nh_ptr),
            jsonreader(std::make_shared<JSONReader>()),
            logger(std::make_shared<Logger>()),
            copter(std::make_shared<Copter>())
            {
                logger->init("Copter", FileType::CSV);
                logger->start(true);
                copter->init(nh, logger);
                jsonreader->init(COPTER_DIR + "/docs/plan.json");
                jsonreader->read();
            }
        
        void PreArm();

        void Run();

        void Land();

        ~Misi2();
    };

    inline EMIRO::WayPoint Misi2::lock_pos(int min_counter, const char *title, bool nav_change)
    {
        return EMIRO::WayPoint();
    }

    inline void Misi2::PreArm()
    {
        copter->FCUconnect(2.0f);
        copter->FCUstart(1.0f);
        copter->init_frame();
    }

    
    inline void Misi2::Run()
    {
        copter->takeoff(1.0f);
        ros::Duration(6).sleep();

        std::vector<JSONData> plan_point = jsonreader->get_data();
        if(!plan_point.size()) return;

        EMIRO::JSONData _data_temp;
        _data_temp = plan_point.front();
        plan_point.erase(plan_point.begin());

        ros::Rate out_rate(1);
        copter->set_speed(1.0f);
        while(ros::ok() && (plan_point.size() || !copter->is_reached(_data_temp.wp, 0.2f)))
        {
            if(copter->is_reached(_data_temp.wp, 0.2f))
            {
                _data_temp = plan_point.front();
                plan_point.erase(plan_point.begin());
                logger->write_show(LogLevel::INFO, "Yaw %f", copter->get_yaw());
                copter->Go(_data_temp.wp, true, "Go to " + _data_temp.header);
                copter->set_speed(_data_temp.speed);
            }
            // else
            //     copter->Go(_data_temp.wp);
            ros::spinOnce();
            out_rate.sleep();
        }
    }
    
    inline void Misi2::Land()
    {
        copter->Land();
    }

    Misi2::~Misi2()
    {
    }
}