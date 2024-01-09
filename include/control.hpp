#ifndef CONTROL
#define CONTROL

#include <copter.hpp>
#include <gps.hpp>

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
        float vx = 0.0f, vy = 0.0f, vz = 0.0f;
        
        Control();

        Control(std::shared_ptr<EMIRO::Logger> log);

        ~Control();
    };
}

#endif
