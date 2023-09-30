#ifndef CONV_HEADER
#define CONV_HEADER

#include <iostream>
#include <cmath>
#include <enum.hpp>

namespace EMIRO{
    
    class Nav_Convert
    {
    private:
        double init_deg;
        double radians;
    public:
        /**
         * @brief Construct a new Nav_Convert object
         * 
         * @param current_deg Pre-arm degree (0 - 360)
         */
        Nav_Convert();

        void init(float current_deg = 90.0f);

        /**
         * @brief Convert Copter Local degree besed on T265 Camera into Global position
         * 
         * @param need_pos      Desire local position needed ({m, m})
         * @param show          Showing Conversion result ?
         * @return GlobalPos    Global position target ({m, m})
         */
        WayPoint convert(WayPoint need_pos);

        WayPoint2 convert(WayPoint2 need_pos);

        /**
         * @brief Get the Copter pre-arm degree value
         * 
         * @return float    Copter Init degree
         */
        float get_degree();

        ~Nav_Convert();
    };
}

#endif // CONV_HEADER