#include <GPSnav.hpp>

namespace EMIRO{
    GlobalNav::GlobalNav(){}

    void GlobalNav::init(float current_deg)
    {
        current_deg += 90;
        if(current_deg >= 360.0f)
            current_deg = (int)current_deg%360;
        init_deg = current_deg;
        radians = current_deg * M_PI / 180.0;
        // std::cout << "Lock global degree\t: " << init_deg << "°" << std::endl; 
    }

    LinearSpeed GlobalNav::convert(LinearSpeed linear_speed, float limit_speed, bool show)
    {
        float x = linear_speed.linear_x * cos(radians) - linear_speed.linear_y * sin(radians);
        float y = linear_speed.linear_x * sin(radians) + linear_speed.linear_y * cos(radians);   

        // Make sure limit_speed is above of zero value
        limit_speed = std::abs(limit_speed);

        if(x < -limit_speed)
            x = -limit_speed;
        else if(x > limit_speed)
            x = limit_speed;
        
        if(y < -limit_speed)
            y = -limit_speed;
        else if(y > limit_speed)
            y = limit_speed;
        return {-x, y};
    }

    float GlobalNav::get_degree()
    {
        return init_deg;
    }

    GlobalNav::~GlobalNav(){}
}