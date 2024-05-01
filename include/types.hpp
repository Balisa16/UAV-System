#pragma once

#include "enum.hpp"

namespace EMIRO
{
    typedef struct
    {
        bool is_detect;
        int x_pixel;
        int y_pixel;
    } VCoordinate;

    typedef struct
    {
        double x, y, z;
    } Position;

    typedef struct
    {
        double w, x, y, z;
    } Quaternion;

    typedef struct
    {
        Position position;
        Quaternion orientation;
    } Pose;

    typedef struct
    {
        float roll, pitch, yaw;
    } Euler;

    struct WayPoint
    {
        float x;
        float y;
        float z;
        float yaw;

        void operator+=(const WayPoint &wp)
        {
            x += wp.x;
            y += wp.y;
            z += wp.z;
            yaw += wp.yaw;
        }

        void operator/=(int i)
        {
            x /= i;
            y /= i;
            z /= i;
            yaw /= i;
        }

        void operator-=(const WayPoint &wp)
        {
            x -= wp.x;
            y -= wp.y;
            z -= wp.z;
            yaw -= wp.yaw;
        }

        WayPoint operator-(const WayPoint &wp)
        {
            WayPoint _wp_result;
            _wp_result.x = x - wp.x;
            _wp_result.y = y - wp.y;
            _wp_result.z = z - wp.z;
            _wp_result.yaw = yaw - wp.yaw;
            return _wp_result;
        }

        void operator=(const WayPoint &wp)
        {
            x = wp.x;
            y = wp.y;
            z = wp.z;
            yaw = wp.yaw;
        }

        void clear()
        {
            x = 0.f;
            y = 0.f;
            z = 0.f;
            yaw = 0.f;
        }

        friend std::ostream &operator<<(std::ostream &os, const WayPoint &wp)
        {
            std::cout << std::fixed << std::setprecision(3);
            os << "(" << wp.x << ", " << wp.y << ", " << wp.z;
            std::cout << std::fixed << std::setprecision(1);
            std::cout << ")\tyaw:" << wp.yaw << "°";
            std::cout << std::defaultfloat;
            return os;
        }
    };

    typedef struct
    {
        float lat;
        float lng;
        float alt;
        float yaw;
    } WayPointG;

    typedef struct
    {
        float x;
        float y;
        float z;
        float yaw;
        float speed;
    } WayPoint2;

    typedef struct
    {
        float linear_x;
        float linear_y;
        float linear_z;
    } LinearSpeed;

    typedef struct
    {
        int Hue;
        int Saturation;
        int Value;
    } HSV;

    typedef struct
    {
        HSV Low;
        HSV High;
    } Color_Range;

    typedef struct
    {
        int Aux_idx;
        int Left;
        int Standby;
        int Right;
    } DSServo;

    typedef struct
    {
        int Aux_idx;
        int Open;
        int Close;
    } NServo;

    typedef struct
    {
        float front_min;
        float right_min;
        float back_min;
        float left_min;
        float front_max;
        float right_max;
        float back_max;
        float left_max;
    } Lidar_Scan;

    typedef struct
    {
        float front;
        float back;
        float left;
        float right;
    } Axis;

    typedef struct
    {
        bool value;
        bool is_set;
    } ParamB;

    typedef struct
    {
        bool is_set;
        int value;
        int min;
        int maks;
    } ParamI;

    typedef struct
    {
        bool is_set;
        int value;
        int min;
        int maks;
    } ParamF;

    struct Option
    {
        int option_int;
        std::string option_desc;
        Option(int id, std::string desc) : option_int(id), option_desc(desc) {}
    };
    class ParamS
    {
    public:
        std::string param_id;
        std::string param_type;
        int value;
        std::vector<Option> options;

        ParamS(std::string param_id, std::string param_type, int value)
            : param_id(param_id), param_type(param_type), value(value) {}

        void add(Option op) { options.push_back(op); }
    };

    typedef struct
    {
        ParamB EK3_ENABLE;
        ParamB EK2_ENABLE;
        ParamI AHRS_EKF_TYPE;

        // Basic Param
        ParamI LAND_SPEED;      // 30 to 200
        ParamI LAND_SPEED_HIGH; // 0 to 2500

        // RC Option
        ParamB RC1_OPTION;
        ParamB RC2_OPTION;
        ParamB RC3_OPTION;
        ParamB RC4_OPTION;
        ParamB RC5_OPTION;
        ParamB RC6_OPTION;
        ParamB RC7_OPTION;
        ParamB RC8_OPTION;

        // CAN Bus
        ParamI CAN_P1_DRIVER;
        ParamI CAN_P2_DRIVER;
        ParamI CAN_D1_PROTOCOL;
        ParamI CAN_D2_PROTOCOL;

        ParamB EK3_SRC_OPTIONS;
        ParamF EK3_GLITCH_RAD;
        ParamI EK3_RNG_USE_HGT;
        // EKF Source
        ParamI EK3_SRC1_POSXY;
        ParamI EK3_SRC1_VELXY;
        ParamI EK3_SRC1_POSZ;
        ParamI EK3_SRC1_YAW;
        ParamI EK3_SRC2_POSXY;
        ParamI EK3_SRC2_VELXY;
        ParamI EK3_SRC2_POSZ;
        ParamI EK3_SRC2_YAW;
        ParamI EK3_SRC3_POSXY;
        ParamI EK3_SRC3_VELXY;
        ParamI EK3_SRC3_POSZ;
        ParamI EK3_SRC3_YAW;

        // GPS Parameters
        ParamI GPS_TYPE;
        ParamI GPS_TYPE2;

        // Viso Parameters
        ParamI VISO_TYPE;
        ParamF VISO_POS_X;
        ParamF VISO_POS_Y;
        ParamF VISO_POS_Z;
    } ArduParam;
}