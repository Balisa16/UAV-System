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
        EMIRO::GPS* _gps;
        float speed_limit = 0.5f;

    public:
        Control(std::shared_ptr<EMIRO::Copter> cptr, 
            std::shared_ptr<EMIRO::Logger> log,
            EMIRO::GPS *gps):
        copter(cptr), logger(log), _gps(gps)
        {
            logger->write_show(LogLevel::INFO, "Used Manual Control");
            _gps->lock_pos();
        }

        float vx = 0.0f, vy = 0.0f, vz = 0.0f;

        /**
         * @brief Set the home of the copter
         */
        void init();

        void reset_home();

        void go();

        ~Control() {}

    };
}

#endif
