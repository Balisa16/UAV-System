// rosrun emiro test1 1 0 1 0.5 5 0.3 8 1 1 1 1

// #include "misi.hpp"

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

// #include <iostream>

int main(int argc, char **argv)
{
    std::cout << "T1\n";
    ros::init(argc, argv, "mavcon_node");
    // EMIRO::Misi2 misi(argc, argv);
    // misi.PreArm();
    // misi.Run();
    // misi.Land();
    ros::NodeHandle nh;
    std::shared_ptr<EMIRO::Copter> copter;
    std::shared_ptr<EMIRO::Logger> logger;
    EMIRO::Servo ser;
    // EMIRO::Nav_Convert converter;
    // EMIRO::GlobalNav global_nav;
    // EMIRO::Lidar lidar_dev;
    // EMIRO::RangeFinder rangefinder;
    // EMIRO::JSONReader jsonreader;

    // EMIRO::WayPoint _start_pos;

    return 0;
}