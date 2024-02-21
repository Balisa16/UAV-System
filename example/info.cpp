#include <emiro.hpp>

using namespace EMIRO;

int main(int argc, char **argv)
{
    Copter::init(argc, argv);

    WayPoint _temp_wp;
    ros::Rate _rate(4);
    while (ros::ok())
    {
        Copter::get_position(_temp_wp);
        std::cout << CLEAR_LINE << '\r' << _temp_wp << std::flush;

        ros::spinOnce();
        _rate.sleep();
    }

    return 0;
}