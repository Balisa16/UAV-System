#ifndef CONTROL
#define CONTROL

#include <copter.hpp>
#include <Logger.hpp>
#include <gps.hpp>
#include <notification.hpp>

namespace EMIRO
{
    class Control
    {
    private:
        std::shared_ptr<EMIRO::Copter> copter;
        std::shared_ptr<EMIRO::Logger> logger;
        EMIRO::GPS _gps;
        float speed_limit = 0.5f;

    public:
        Control(){}
        
        void init(std::shared_ptr<EMIRO::Copter> cptr,
            std::shared_ptr<EMIRO::Logger> log)
        {
            copter = cptr;
            logger = log;
            _gps = EMIRO::GPS();
            logger->write_show(LogLevel::INFO, "Used Manual Control");
            _gps.lock_pos();
        }

        float vx = 0.0f, vy = 0.0f, vz = 0.0f;

        void reset_home();

        void go();

        ~Control() {}

    };
}

#endif
