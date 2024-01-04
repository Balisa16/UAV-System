#ifndef GPS_HPP
#define GPS_HPP

#include <iostream>
#include <cmath>
#include <iostream>
#include <cstring>
#include <ncurses.h>
#include <chrono>
#include <thread>
#include <Logger.hpp>
#include <copter.hpp>
#include "enum.hpp"

namespace EMIRO{
    
    class GPS
    {
    private:
        std::shared_ptr<Copter> copter;
        std::shared_ptr<Logger> log;
        WayPoint start_point;
        float init_deg;
        float radians;
        float mul_x, mul_y;

    public:
        GPS(std::shared_ptr<Copter> copter, std::shared_ptr<Logger> logger):
        copter(copter),
        log(logger){}

        void lock_pos();

        void init(float current_deg = 90.0f);

        LinearSpeed convert(LinearSpeed &linear_speed);

        float get_degree();

        ~GPS(){}
    };
}

#endif