#ifndef GLOBAL_NAV_HPP
#define GLOBAL_NAV_HPP

#include <iostream>
#include <cmath>
#include "enum.hpp"

namespace EMIRO{
    
    class GlobalNav
    {
    private:
        float init_deg;
        float radians;
        float mul_x, mul_y;
    public:
        GlobalNav();

        void init(float current_deg = 90.0f);
        
        LinearSpeed convert(LinearSpeed linear_speed, float limit_speed, bool show = false);

        float get_degree();

        ~GlobalNav();
    };
}

#endif