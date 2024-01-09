#ifndef CONTROL_HPP
#define CONTROL_HPP

#include <copter.hpp>
#include <gps.hpp>

namespace EMIRO
{
    class Control
    {
    private:
        std::shared_ptr<EMIRO::Copter> copter;
        std::shared_ptr<EMIRO::Logger> logger;
        std::shared_ptr<EMIRO::GPS> _gps;
        float speed_limit = 0.5f;
        bool is_init =  false;

    public:
        float vx = 0.0f, vy = 0.0f, vz = 0.0f;
        
        Control();

        Control(std::shared_ptr<EMIRO::Copter> copter, std::shared_ptr<EMIRO::Logger> logger, std::shared_ptr<EMIRO::GPS> gps);

        void go();

        ~Control();
    };

    Control::Control() {}

    Control::Control(std::shared_ptr<EMIRO::Copter> copter, std::shared_ptr<EMIRO::Logger> logger, std::shared_ptr<EMIRO::GPS> gps) : _gps(gps), copter(copter), logger(logger)
    {
        _gps->lock_pos();
        is_init = true;
    }

    void Control::go()
    {
        if (!is_init)
        {
            copter->Land();
            throw std::runtime_error("Bad control class implementation.");
        }
        LinearSpeed speed = {vx, vy, vz};
        _gps->convert(speed);
        copter->set_vel(vx, vy, vz, 0.0f, 0.0f, 0.0f);
    }

    Control::~Control()
    {
    }
}

#endif
