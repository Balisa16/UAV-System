#include <gps.hpp>

namespace EMIRO{

    void GPS::init(float current_deg)
    {
        current_deg += 90;
        if(current_deg >= 360.0f)
            current_deg = (int)current_deg%360;
        init_deg = current_deg;
        radians = current_deg * M_PI / 180.0;
    }

    void GPS::lock_pos()
    {
        try_flag:
        log->wait("Lock GPS Position");

        ros::Duration(1).sleep();
        ros::Rate in_rate(5);

        EMIRO::WayPoint temp_wp;
        uint8_t count = 10;
        while (ros::ok() && count)
        {
            copter->get_position(temp_wp);
            ros::spinOnce();
            in_rate.sleep();
            count--;
        }
        std::cout << "Result : " << temp_wp << std::endl;
        std::cout << "Accept ? (y/n) " << std::endl;
        char choice;
        std::cin >> choice;
        if (!(choice == 'y' || choice == 'Y'))
            goto try_flag;
        start_point = temp_wp;
    }

    LinearSpeed GPS::convert(LinearSpeed &linear_speed)
    {
        return {
            -(linear_speed.linear_x * cos(radians) - linear_speed.linear_y * sin(radians)),
            linear_speed.linear_x * sin(radians) + linear_speed.linear_y * cos(radians)};
    }

    float GPS::get_degree()
    {
        return init_deg;
    }
}