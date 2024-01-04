#include <control.hpp>

namespace EMIRO{
    Control::Control(std::shared_ptr<EMIRO::Copter> copter, std::shared_ptr<Logger> log):
        copter(copter),
        logger(log)
    {
        logger->write_show(LogLevel::INFO, "Used Manual Control");
        #define MANUAL_CONTROL
        gps.init(90.0f);
    }
    Control::~Control()
    {

    }

    void Control::set_home()
    {

    }

    void Control::setspeed_x()
    {

    }

    void Control::setspeed_y()
    {

    }

    void Control::setspeed_z()
    {

    }

    void Control::go()
    {
        LinearSpeed _speed_go = gps.convert(speed_limit);
        this->copter.set_vel(speed_x, _speed_go.linear_y, 0.0f, 0.0f, 0.0f, 0.0f);
    }
}