#pragma once

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
#define CLEAR_LINE "\033[2K\r"
#endif

#include <iostream>
#include <string>
#include <vector>
#include <iomanip>

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

    enum class YawMode
    {
        ABSOLUTE,
        RELATIVE
    };
}
