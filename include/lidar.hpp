#ifndef RPLIDAR_HEADER
#define RPLIDAR_HEADER

#include <sensor_msgs/LaserScan.h>
#include <ros/ros.h>
#include <mutex>
#include "enum.hpp"
#include "copter.hpp"
#include <tuple>
#include <memory>
#include <thread>

namespace EMIRO{
  std::mutex lidar_proc_mtx;

  enum class LidarType
  {
    Simulator,
    A1,
    S1
  };

  enum class LidarStatus
  {
    None,
    Init,
    Start,
    Run,
    Stop
  };

  typedef struct {
    float margin_left_2;
    float margin_left_1;
    float center;
    float margin_right_1;
    float margin_right_2;
  }SideRange;

  typedef struct{
    LidarType type;
    LidarStatus status;
    sensor_msgs::LaserScan in_data;
    bool out_is_valid;
    int out_size;
    SideRange out_left_rng;
    SideRange out_right_rng;
    SideRange out_front_rng;
    SideRange out_back_rng;
  }LidarRef;

  class Lidar
  {
  private:
    ros::Subscriber lidar_sub;

    const std::string sim_lidar_path = "/spur/laser/scan";
    const std::string a1_lidar_path = "/scan";
    const std::string s1_lidar_path = "/scan";

    const float lidar_tolerance_1 = 0.9f;
    const float lidar_tolerance_2 = 1.5f;

    void scan(const sensor_msgs::LaserScan::ConstPtr& input);
    std::shared_ptr<EMIRO::Copter> copter;

    float _temp1, _temp2, _temp3, _max_temp, _min_temp;

    LidarRef lidar_data;

  public:
    Lidar();
    void init(std::shared_ptr<EMIRO::Copter> copter, std::shared_ptr<EMIRO::Logger> logger);
    void start(ros::NodeHandle *nh, LidarType lidar);
    LidarStatus check();
    float get_front(int idx);
    float max_front();
    float get_left(int idx);
    float get_right(int idx);
    float get_back(int idx);
    void axis(Axis& axis, int idx = 1);
    sensor_msgs::LaserScan get_raw();
    void stop();
    ~Lidar(){}
  };
}
#endif // RPLIDAR_HEADER