#ifndef RPLIDAR_HEADER
#define RPLIDAR_HEADER

#include <sensor_msgs/LaserScan.h>
#include <ros/ros.h>
#include "enum.hpp"
#include "copter.hpp"
#include <tuple>
#include <memory>

namespace EMIRO{
  class Lidar
  {
  private:
    ros::Subscriber lidar_sub;
    sensor_msgs::LaserScan scan2D;
    LidarType lidar_type;

    const std::string sim_lidar_path = "/spur/laser/scan";
    const std::string a1_lidar_path = "/scan";
    const std::string s1_lidar_path = "/scan";

    const float lidar_tolerance_1 = 0.9f;
    const float lidar_tolerance_2 = 1.5f;
    int idx_front[3], idx_right[3], idx_back[3], idx_left[3];

    bool lidar_init = false;
    void stop();
    void scan(const sensor_msgs::LaserScan::ConstPtr& input);
    std::shared_ptr<EMIRO::Copter> copter;

    float _temp1, _temp2, _temp3, _max_temp, _min_temp;

  public:
    Lidar(){}
    void init(std::shared_ptr<EMIRO::Copter> copter);
    void start(ros::NodeHandle *nh, LidarType lidar);
    std::tuple<bool, Lidar_Scan> check();
    float get_front(int idx);
    float max_front();
    float get_left(int idx);
    float get_right(int idx);
    float get_back(int idx);
    void axis(Axis& axis, int idx = 1);
    sensor_msgs::LaserScan get_raw();
    ~Lidar(){}
  };
}
#endif // RPLIDAR_HEADER