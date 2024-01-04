#include <control.hpp>

namespace EMIRO{

    void Control::reset_home()
    {
        gps.lock_pos();
    }

    void Control::go()
    {
        LinearSpeed speed = {vx, vy, vz};
        gps.convert(speed);
        copter->set_vel(vx, vy, vz, 0.0f, 0.0f, 0.0f);
    }
}