#include <gps.hpp>
#include <keyboard.hpp>

namespace EMIRO
{
    GPS::GPS() {}

    void
    GPS::init() { GPS::get_gps().gps_init(); }

    void
    GPS::lock_pos() { GPS::get_gps().gps_lock_pos(); }

    void
    GPS::convert(LinearSpeed &linear_speed)
    {
        GPS::get_gps().gps_convert(linear_speed);
    }

    void
    GPS::gps_init()
    {
        Copter::get_logger().write_show(LogLevel::INFO, "GPS initialized");
        is_init = true;
    }

    void
    GPS::gps_lock_pos()
    {
        Copter::get_logger().show(LogLevel::INFO, "Waiting for GPS position...");

        ros::Duration(1).sleep();
        ros::Rate in_rate(2);

        EMIRO::WayPoint temp_wp;
        // Waiting for position to refresh
        std::cout << std::fixed << std::setprecision(3);
        std::vector<WayPoint> pos_temp;
        int maks_data = 20;
        WayPoint wp_average;
        Keyboard keyboard;

        while (ros::ok())
        {
            Copter::get().get_position(temp_wp);
            pos_temp.push_back(temp_wp);

            // Remove old data
            if (pos_temp.size() > maks_data)
                pos_temp.erase(pos_temp.begin());

            // Calculate average and MAE
            wp_average.clear();
            for (WayPoint &w : pos_temp)
                wp_average += w;
            wp_average /= pos_temp.size();

            std::cout << CLEAR_LINE << "Position : [" << wp_average.x << ", " << wp_average.y
                      << ", " << wp_average.z << ", " << wp_average.yaw
                      << "], MAE : " << wp_average.x - temp_wp.x << ", "
                      << wp_average.y - temp_wp.y << ", "
                      << wp_average.z - temp_wp.z << ", "
                      << wp_average.yaw - temp_wp.yaw << "   Lock ? (y/n)\r"
                      << std::flush;

            char key = keyboard.get_key();
            if (key == 'y' || key == 'Y')
                break;
            else if (key == 'n' || key == 'N')
            {
                Copter::get_logger().write_show(LogLevel::INFO,
                                                "\033[KGPS position unlocked");
                return;
            }
            else if (key >= 32 && key <= 126)
                Copter::get_logger().show(
                    LogLevel::WARNING,
                    "\033[KUnrecognized keyboard input: \033[1m(%c)\033[0m", key);

            ros::spinOnce();
            in_rate.sleep();
        }

        // Ask if position is accepted
        Copter::get_logger().write_show(
            LogLevel::INFO,
            "Start position locked on => x : %.2f, y : %.2f, z : %.2f, yaw : %d.",
            wp_average.x, wp_average.y, wp_average.z, (int)wp_average.yaw);
        start_point = wp_average;
        is_locked = true;

        // Get radians value
        start_point.yaw += 90;
        if (start_point.yaw >= 360.0f)
            start_point.yaw = (int)start_point.yaw % 360;
        radians = start_point.yaw * M_PI / 180.0;
    }

    void
    GPS::gps_convert(LinearSpeed &linear_speed) const
    {
        if (!is_init || !is_locked)
        {
            Copter::get_logger().write_show(LogLevel::ERROR,
                                            "GPS is not init or not locked");
            throw std::runtime_error("GPS is not init or not locked");
        }
        linear_speed = {-(linear_speed.linear_x * cos(radians) -
                          linear_speed.linear_y * sin(radians)),
                        linear_speed.linear_x * sin(radians) +
                            linear_speed.linear_y * cos(radians),
                        linear_speed.linear_z};
    }

    GPS::~GPS() {}
} // namespace EMIRO