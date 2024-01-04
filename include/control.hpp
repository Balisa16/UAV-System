#ifndef CONTROL
#define CONTROL

#include <copter.hpp>
#include <Logger.hpp>
#include <gps.hpp>

namespace EMIRO{
    class Control{
        public:
            Control(){}
            float speed_x = 0.0f, speed_y = 0.0f, speed_z = 0.0f;
            /**
             * @brief Set the home of copter
             * 
             */
            void init(std::shared_ptr<EMIRO::Copter> copter, std::shared_ptr<Logger> log);
            void reset_home();
            void setspeed_x();
            void setspeed_y();
            void setspeed_z();
            ~Control(){}
        private:
            std::shared_ptr<EMIRO::Copter> copter;
            std::shared_ptr<Logger> logger;
            EMIRO::GPS gps;
            float speed_limit = 0.5f;
            void go();
    };
}

#endif