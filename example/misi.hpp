
#include <convert.hpp>
#include <enum.hpp>
#include <copter.hpp>
#include <servo.hpp>
#include <lidar.hpp>
#include <GPSnav.hpp>
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
        ros::NodeHandle nh;
        std::shared_ptr<EMIRO::Copter> copter;
        std::shared_ptr<EMIRO::Logger> logger;
        EMIRO::Servo ser;
        EMIRO::Nav_Convert converter;
        EMIRO::GlobalNav global_nav;
        EMIRO::Lidar lidar_dev;
        EMIRO::RangeFinder rangefinder;
        EMIRO::JSONReader jsonreader;

        EMIRO::WayPoint _start_pos;

        EMIRO::WayPoint lock_pos(int min_counter = 1, const char *title = "Press y to lock position", bool nav_change = false);


    public:

        Misi2(int argc, char **argv);
        
        void PreArm();

        void Run();

        void Land();

        ~Misi2();
    };

    Misi2::Misi2(int argc, char **argv)
    {
        logger = std::make_shared<Logger>();
        copter = std::make_shared<Copter>();
        logger->init("Copter", FileType::CSV);
        logger->start(true);
        copter->init(&this->nh, logger);
        jsonreader.init(COPTER_DIR + "/docs/plan.json");
        // jsonreader.init(plan)
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

        std::vector<JSONData> plan_point = jsonreader.get_data();
        std::cout << "Size : " << plan_point.size() << std::endl;
        if(!plan_point.size()) return;

        EMIRO::JSONData _data_temp;
        _data_temp = plan_point.front();
        plan_point.erase(plan_point.begin());

        ros::Rate out_rate(1);
        while(ros::ok() && (plan_point.size() || !copter->is_reached(_data_temp.wp, 0.2f)))
        {
            if(copter->is_reached(_data_temp.wp, 0.2f))
            {
                _data_temp = plan_point.front();
                plan_point.erase(plan_point.begin());
                copter->Go(_data_temp.wp);
            }
            else
                copter->Go(_data_temp.wp, false);
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