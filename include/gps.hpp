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

namespace EMIRO
{

    class GPS
    {
    private:
        std::shared_ptr<EMIRO::Copter> copter;
        std::shared_ptr<EMIRO::Logger> log;
        WayPoint start_point;
        float radians;
        float mul_x, mul_y;
        bool is_locked = false, is_init = false;

    public:
        GPS() {}

        inline void init(std::shared_ptr<EMIRO::Copter> copter, std::shared_ptr<EMIRO::Logger> logger)
        {
            this->copter = copter;
            this->log = logger;
            log->write_show(LogLevel::INFO, "GPS initialized");
            is_init = true;
        }

        inline void lock_pos()
        {
        try_flag:
            log->wait("Lock GPS Position");

            ros::Duration(1).sleep();
            ros::Rate in_rate(2);

            EMIRO::WayPoint temp_wp;
            uint8_t count = 6;
            // Waiting for position to refresh
            while (ros::ok() && count)
            {
                copter->get_position(temp_wp);
                ros::spinOnce();
                in_rate.sleep();
                count--;
            }

            log->wait_success();

            // Ask if position is accepted
            log->write_show(LogLevel::ASK, "Position => x : %.2f, y : %.2f, z : %.2f, yaw : %d. Lock Position ? (y/n) ",
                            temp_wp.x, temp_wp.y, temp_wp.z, (int)temp_wp.yaw);
            char choice;
            std::cin >> choice;
            if (!(choice == 'y' || choice == 'Y'))
                goto try_flag;
            start_point = temp_wp;
            log->write_show(LogLevel::INFO, "Lock start on => x : %.2f, y : %.2f, z : %.2f, yaw : %d",
                            start_point.x, start_point.y, start_point.z, (int)start_point.yaw);
            is_locked = true;

            // Get radians value
            start_point.yaw += 90;
            if (start_point.yaw >= 360.0f)
                start_point.yaw = (int)start_point.yaw % 360;
            radians = start_point.yaw * M_PI / 180.0;
        }

        inline void convert(LinearSpeed &linear_speed)
        {
            if (!is_init || !is_locked)
            {
                log->write_show(LogLevel::ERROR, "GPS is not init or not locked");
                throw std::runtime_error("GPS is not init or not locked");
            }
            linear_speed = {
                -(linear_speed.linear_x * cos(radians) - linear_speed.linear_y * sin(radians)),
                linear_speed.linear_x * sin(radians) + linear_speed.linear_y * cos(radians),
                linear_speed.linear_z};
        }

        ~GPS() {}
    };
}
#endif