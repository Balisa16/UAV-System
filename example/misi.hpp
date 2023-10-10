
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
        std::string plan = COPTER_DIR + "/docs/plan.json";
        std::cout << plan << std::endl;
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
        /*ros::Rate out_rate(1);
        while(ros::ok())
        {
            ros::spinOnce();
            out_rate.sleep();
        }

        // Waiting for the last position is reached;
        logger->write_show(LogLevel::INFO, "Waiting to LAND ");
        while(ros::ok() && !copter->check_alt(3.0f, 2.0f))
        {
            ros::spinOnce();
            out_rate.sleep();
        }

        logger->write_show(LogLevel::INFO, "Finished Outdoor Mission");*/
    }
    
    inline void Misi2::Land()
    {
        copter->Land();
    }

    Misi2::~Misi2()
    {
    }
}