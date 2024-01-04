#ifndef CONTROL
#define CONTROL

#include <copter.hpp>
#include <Logger.hpp>
#include <gps.hpp>

namespace EMIRO
{
    class Control
    {
    public:
        Control(std::shared_ptr<EMIRO::Copter> copter, std::shared_ptr<EMIRO::Logger> log):
        copter(copter), logger(log){}

        float vx = 0.0f, vy = 0.0f, vz = 0.0f;

        /**
         * @brief Set the home of the copter
         */
        void init();

        void reset_home();

        void go();

        ~Control() {}

    private:
        std::shared_ptr<EMIRO::Copter> copter;
        std::shared_ptr<EMIRO::Logger> logger;
        EMIRO::GPS gps;
        float speed_limit = 0.5f;
    };
}

#endif
