#include <gps.hpp>

namespace EMIRO{
    GPS::GPS()
    {

    }

    void GPS::init(std::shared_ptr<EMIRO::Copter> cptr, std::shared_ptr<EMIRO::Logger> logger)
    {
        this->copter = cptr;
        this->log = logger;
        log->write_show(LogLevel::INFO, "GPS initialized");
    }
    void GPS::lock_pos()
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

    void GPS::convert(LinearSpeed &linear_speed)
    {
        linear_speed = {
            -(linear_speed.linear_x * cos(radians) - linear_speed.linear_y * sin(radians)),
            linear_speed.linear_x * sin(radians) + linear_speed.linear_y * cos(radians),
            linear_speed.linear_z};
    }

    GPS::~GPS()
    {

    }
}