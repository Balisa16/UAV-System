#pragma once
#include <enum.hpp>

namespace EMIRO
{
    class Settings
    {
    public:
        void init()
        {
            mode = FlightMode::LAND;
        }
        static FlightMode mode;
    };
}