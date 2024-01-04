#include <control.hpp>

namespace EMIRO{
    void Control::init(std::shared_ptr<EMIRO::Copter> copter, std::shared_ptr<Logger> log)
    {
        this->copter = copter;
        this->logger = log;
        logger->write_show(LogLevel::INFO, "Used Manual Control");
#define MANUAL_CONTROL
        gps.init(this->copter, this->logger);
        gps.lock_pos();
    }

    void Control::reset_home()
    {
        gps.lock_pos();
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
        LinearSpeed speed = {speed_x, speed_y, speed_z};
        gps.convert(speed);
        copter->set_vel(speed_x, speed_y, speed_z, 0.0f, 0.0f, 0.0f);
    }
}