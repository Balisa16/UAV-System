#ifndef ENUM_HEADER
#define ENUM_HEADER

#define DS1_LEFT 1000
#define DS1_STANDBY 1500
#define DS1_RIGHT 2000

#define DS2_LEFT 1000
#define DS2_STANDBY 1500
#define DS2_RIGHT 2000

#ifndef TERMINAL_COLOR_STYLE
#define C_RESET "\033[0m"
#define C_BLACK "\033[30m"
#define C_RED "\033[31m"
#define C_GREEN "\033[32m"
#define C_YELLOW "\033[33m"
#define C_BLUE "\033[34m"
#define C_MAGENTA "\033[35m"
#define C_CYAN "\033[36m"
#define C_WHITE "\033[37m"
#define S_BOLD "\033[1m"
#define S_ITALIC "\033[3m"
#define S_UNDERLINE "\033[4m"
#endif

#include <iostream>
#include <string>
#include <vector>

namespace EMIRO
{

    enum class Mode
    {
        Indoor,
        Outdoor
    };

    enum class CopterStatus
    {
        None,
        Armed,
        Takeoff,
        Flying,
        Land,
        Disarmed
    };

    enum class CopterMode
    {
        LAND,
        GUIDED,
        AUTO,
        RTL
    };

    enum Indoor_State
    {
        Takeoff,
        GetPayload,
        PreInterchange,
        Left_Right,
        DropPayload,
        ChangeNav
    };

    enum class EKF_Source
    {
        GPS_BARO,
        GPS_GY,
        T265_GY
    };

    enum class DSServo_Condition
    {
        Left,
        Standby,
        Right
    };

    enum class Servo_Condition
    {
        Open,
        Close
    };

    enum class Movement
    {
        Forward,
        TakeLoad,
        Left_Right,
        DropLoad
    };

    typedef struct
    {
        bool is_detect;
        int x_pixel;
        int y_pixel;
    } VCoordinate;

    typedef struct
    {
        float x, y, z;
    } Position;

    typedef struct
    {
        float w, x, y, z;
    } Quaternion;

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
            _wp_result.yaw = z - wp.yaw;
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

    static std::ostream &operator<<(std::ostream &stream, WayPoint &data)
    {
        stream << "\n\tx\t: " << data.x << " m\n\ty\t: " << data.y
               << " m\n\tz\t: " << data.z << " m\n\tyaw\t: " << data.yaw << " deg";
        return stream;
    }

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
} // namespace EMIRO

#endif // ENUM_HEADER
