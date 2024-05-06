#include <copter.hpp>

using namespace EMIRO;

int main(int argc, char **argv)
{
    Copter::init(argc, argv);

    if (!Copter::PreArmedCheck())
    {
        Copter::Land();
        exit(EXIT_FAILURE);
    }

    ros::Rate _rate(2);
    while (ros::ok())
    {
        std::cout << "HDOP: " << Copter::get_hdop() << " , VDOP: " << Copter::get_vdop() << ", Satellites: " << Copter::get_satellites_num() << "GPS Status: " << Copter::get_gps_status() << "" << std::endl;
        ros::spinOnce();
        _rate.sleep();
    }

    return 0;
}