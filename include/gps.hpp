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
        std::shared_ptr<EMIRO::Copter> copter;
        std::shared_ptr<EMIRO::Logger> log;
        WayPoint start_point;
        float radians;
        float mul_x, mul_y;
        bool is_locked = false;

    public:
        GPS();

        void init(std::shared_ptr<EMIRO::Copter> copter, std::shared_ptr<EMIRO::Logger> logger);

        /**
         * @brief Lock GPS start position and orientation
         * 
         */
        void lock_pos();

        void convert(LinearSpeed &linear_speed);

        ~GPS();
    };
}

#endif