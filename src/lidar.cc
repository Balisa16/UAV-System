#include <lidar.hpp>

namespace EMIRO{
  void Lidar::init(std::shared_ptr<EMIRO::Copter> copter)
  {
    this->copter = copter;
    ROS_INFO("Lidar initialized");
  }

  void Lidar::stop()
  {
    lidar_sub.shutdown();
  }

  void Lidar::scan(const sensor_msgs::LaserScan::ConstPtr& input)
  {
    scan2D = *input;
    if(!lidar_init && scan2D.ranges.size() > 0)
    {
      int step_12 = scan2D.ranges.size()/12;

      switch (lidar_type)
      {
      case LidarType::Simulator:
        // Right
        idx_right[0] = 0;
        idx_right[1] = step_12;
        idx_right[2] = step_12*11;

        // Front
        idx_front[0] = step_12*2;
        idx_front[1] = step_12*3;
        idx_front[2] = step_12*4;

        // Left
        idx_left[0] = step_12*5;
        idx_left[1] = step_12*6;
        idx_left[2] = step_12*7;

        // Back
        idx_back[0] = step_12*8;
        idx_back[1] = step_12*9;
        idx_back[2] = step_12*10;
        break;

      case LidarType::A1:
        // Back
        idx_back[0] = step_12*11;
        idx_back[1] = 0;
        idx_back[2] = step_12;

        // Right
        idx_right[0] = step_12*2;
        idx_right[1] = step_12*3;
        idx_right[2] = step_12*4;

        // Front
        idx_front[0] = step_12*5;
        idx_front[1] = step_12*6;
        idx_front[2] = step_12*7;

        // Left
        idx_left[0] = step_12*8;
        idx_left[1] = step_12*9;
        idx_left[2] = step_12*10;
        break;

      case LidarType::S1:
        // Back
        idx_back[0] = step_12*11;
        idx_back[1] = 0;
        idx_back[2] = step_12;

        // Right
        idx_right[0] = step_12*2;
        idx_right[1] = step_12*3;
        idx_right[2] = step_12*4;

        // Front
        idx_front[0] = step_12*5;
        idx_front[1] = step_12*6;
        idx_front[2] = step_12*7;

        // Left
        idx_left[0] = step_12*8;
        idx_left[1] = step_12*9;
        idx_left[2] = step_12*10;
        break;

      default:
        break;
      }
      lidar_init = true;
    }
    if(copter->get_current_mission() == Mode::Outdoor)
      stop();
  }

  void Lidar::start(ros::NodeHandle *nh, LidarType lidar)
  {
    lidar_type = lidar;

    std::string lidar_path;

    // Select match path
    switch (lidar_type)
    {
    case LidarType::Simulator:
      lidar_path = sim_lidar_path;
      ROS_WARN("Using simulator lidar configuration");
      break;

    case LidarType::A1:
      lidar_path = a1_lidar_path;
      ROS_INFO("Using A1 lidar configuration");
      break;

    case LidarType::S1:
      lidar_path = s1_lidar_path;
      ROS_INFO("Using S1 lidar configuration");
      break;
    
    default:
      ROS_ERROR("Unknown Lidar Type. Failed config lidar");
      break;
    }

    // Starting Lidar Node
    lidar_sub = (*nh).subscribe<sensor_msgs::LaserScan>(
      lidar_path,
      1,
      [this](const sensor_msgs::LaserScan::ConstPtr& input){
        scan(input);
      });
  }

  std::tuple<bool, Lidar_Scan> Lidar::check()
  {
    Lidar_Scan lidar_temp = {0.0f} ;
    if(!lidar_init)
    {
      // ROS_INFO("Lidar is not ready");
      return std::make_tuple(false, lidar_temp);
    }

    // Min-Max value
    lidar_temp.front_min = std::min({scan2D.ranges[idx_front[0]], scan2D.ranges[idx_front[1]], scan2D.ranges[idx_front[2]]});
    lidar_temp.right_min = std::min({scan2D.ranges[idx_right[0]], scan2D.ranges[idx_right[1]], scan2D.ranges[idx_right[2]]});
    lidar_temp.back_min = std::min({scan2D.ranges[idx_back[0]], scan2D.ranges[idx_back[1]], scan2D.ranges[idx_back[2]]});
    lidar_temp.left_min = std::min({scan2D.ranges[idx_left[0]], scan2D.ranges[idx_left[1]], scan2D.ranges[idx_left[2]]});

    lidar_temp.front_max = std::max({scan2D.ranges[idx_front[0]], scan2D.ranges[idx_front[1]], scan2D.ranges[idx_front[2]]});
    lidar_temp.right_max = std::max({scan2D.ranges[idx_right[0]], scan2D.ranges[idx_right[1]], scan2D.ranges[idx_right[2]]});
    lidar_temp.back_max = std::max({scan2D.ranges[idx_back[0]], scan2D.ranges[idx_back[1]], scan2D.ranges[idx_back[2]]});
    lidar_temp.left_max = std::max({scan2D.ranges[idx_left[0]], scan2D.ranges[idx_left[1]], scan2D.ranges[idx_left[2]]});

    return std::make_tuple(true, lidar_temp);
  }

  float Lidar::get_front(int idx)
  {
    return scan2D.ranges[idx_front[idx]];
  }

  float Lidar::max_front()
  {
    _temp1 = scan2D.ranges[idx_front[0]];
    _temp2 = scan2D.ranges[idx_front[1]];
    _temp3 = scan2D.ranges[idx_front[2]];
    _max_temp = _temp1 < 20.0f ? _temp1 : _temp2;
    _max_temp = _temp2 > _max_temp && _temp2 < 20.0f ? _temp2 : _max_temp; 
    _max_temp = _temp3 > _max_temp && _temp3 < 20.0f ? _temp3 : _max_temp;
    if(_max_temp > 39.0f)
      return 0;
    return _max_temp;
  }

  float Lidar::get_left(int idx)
  {
    return scan2D.ranges[idx_left[idx]];
  }

  float Lidar::get_right(int idx)
  {
    return scan2D.ranges[idx_right[idx]];
  }

  float Lidar::get_back(int idx)
  {
    return scan2D.ranges[idx_back[idx]];
  }

  sensor_msgs::LaserScan Lidar::get_raw(){
    return scan2D;
  }

  void Lidar::axis(Axis& axis, int idx)
  {
    axis.front = scan2D.ranges[idx_front[idx]];
    axis.back = scan2D.ranges[idx_back[idx]];
    axis.left = scan2D.ranges[idx_left[idx]];
    axis.right = scan2D.ranges[idx_right[idx]];
  }
}