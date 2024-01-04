#ifndef CONTROL
#define CONTROL

#include <copter.hpp>
#include <Logger.hpp>
#include <gps.hpp>

namespace EMIRO{
    class Control{
        public:
            Control(std::shared_ptr<EMIRO::Copter> copter, std::shared_ptr<Logger> log);
            /**
             * @brief Set the home of copter
             * 
             */
            void set_home();
            void setspeed_x();
            void setspeed_y();
            void setspeed_z();
            ~Control();
        private:
            std::shared_ptr<EMIRO::Copter> copter;
            std::shared_ptr<Logger> logger;
            EMIRO::GPS gps;
            float speed_limit = 0.5f;
            float speed_x = 0.0f, speed_y = 0.0f, speed_z = 0.0f;
            void go();
    };
}

#endif