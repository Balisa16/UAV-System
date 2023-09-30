#ifndef ENUM_HEADER
#define ENUM_HEADER

#define DS1_LEFT 1000
#define DS1_STANDBY 1500
#define DS1_RIGHT 2000

#define DS2_LEFT 1000
#define DS2_STANDBY 1500
#define DS2_RIGHT 2000

#include <iostream>

namespace EMIRO{

  enum class Mode{
    Indoor,
    Outdoor
  };

  enum class CopterMode{
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

  enum class LidarType
  {
    Simulator,
    A1,
    S1
  };

  enum class Movement
  {
    Forward,
    TakeLoad,
    Left_Right,
    DropLoad
  };

  typedef struct VCoordinate
  {
    bool is_detect;
    int x_pixel;
    int y_pixel; 
  };

  typedef struct{
    float w;
    float x;
    float y;
    float z;
  } Quaternion;

  typedef struct{
    float x;
    float y;
    float z;
    float yaw;
  }WayPoint;


  typedef struct{
    float x;
    float y;
    float z;
    float yaw;
    float speed;
  }WayPoint2;
  
  typedef struct{
    float linear_x;
    float linear_y;
  }LinearSpeed;

  static std::ostream& operator<<(std::ostream& stream, WayPoint& data)
  {
    stream << "\n\tx\t: " << data.x << " m\n\ty\t: " << data.y << " m\n\tz\t: " << data.z << " m\n\tyaw\t: " << data.yaw << " deg";
    return stream;
  }

  typedef struct{
    int Hue;
    int Saturation;
    int Value;
  }HSV;

  typedef struct{
    HSV Low;
    HSV High;
  }Color_Range;

  typedef struct{
    int Aux_idx;
    int Left;
    int Standby;
    int Right;
  }DSServo;

  typedef struct{
    int Aux_idx;
    int Open;
    int Close;
  }NServo;

  typedef struct{
    float front_min;
    float right_min;
    float back_min;
    float left_min;
    float front_max;
    float right_max;
    float back_max;
    float left_max;
  }Lidar_Scan;

  typedef struct{
    float front;
    float back;
    float left;
    float right;
  }Axis;

  typedef struct{
    bool value;
    bool is_set;
    ParamB(bool init_value = false, bool init_is_set = false) : value(init_value), is_set(init_is_set) {}
  }ParamB;
  
  typedef struct{
    bool is_set;
    int value;
    int min;
    int maks;
    ParamI(bool init_is_set = false, int init_value = 0, int init_min = 0, int init_maks = 0) : 
      is_set(init_is_set), value(init_value), min(init_min), maks(init_maks) {}
  }ParamI;

  typedef struct{
    bool is_set;
    int value;
    int min;
    int maks;
    ParamF(bool init_is_set = false, float init_value = 0.0f, float init_min = 0.0f, float init_maks = 0.0f) : 
      is_set(init_is_set), value(init_value), min(init_min), maks(init_maks) {}
  }ParamF;

  typedef struct{

  }ArduParam;
}


#endif // ENUM_HEADER