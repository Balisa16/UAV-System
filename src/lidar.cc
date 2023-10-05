#include <lidar.hpp>

void lidar_proc(LidarRef& data_ref)
{
  std::lock_guard<std::mutex> lg(lidar_proc_mtx);
  data_ref.out_size = data_ref.in_data.ranges.size();
  
  float step_20 = data_ref.out_size/20;
  data_ref.out_is_valid = false;

  if(!lidar_init && data_ref.out_size > 0)
  {
    data_ref.out_is_valid = true;
    switch (lidar_type)
    {
    case LidarType::Simulator:
      // Right
      out_right_rng.margin_left_2 = data_ref.in_data.ranges[(int)(step_20*18)];
      out_right_rng.margin_left_1 = data_ref.in_data.ranges[(int)(step_20*19)];
      out_right_rng.center = data_ref.in_data.ranges[0];
      out_right_rng.margin_right_1 = data_ref.in_data.ranges[(int)(step_20*1)];
      out_right_rng.margin_right_2 = data_ref.in_data.ranges[(int)(step_20*2)];

      // Front
      out_front_rng.margin_left_2 = data_ref.in_data.ranges[(int)(step_20*3)];
      out_front_rng.margin_left_1 = data_ref.in_data.ranges[(int)(step_20*4)];
      out_front_rng.center = data_ref.in_data.ranges[(int)(step_20*5)];
      out_front_rng.margin_right_1 = data_ref.in_data.ranges[(int)(step_20*6)];
      out_front_rng.margin_right_2 = data_ref.in_data.ranges[(int)(step_20*7)];

      // Left
      out_left_rng.margin_left_2 = data_ref.in_data.ranges[(int)(step_20*8)];
      out_left_rng.margin_left_1 = data_ref.in_data.ranges[(int)(step_20*9)];
      out_left_rng.center = data_ref.in_data.ranges[(int)(step_20*10)];
      out_left_rng.margin_right_1 = data_ref.in_data.ranges[(int)(step_20*11)];
      out_left_rng.margin_right_2 = data_ref.in_data.ranges[(int)(step_20*12)];

      // Back
      out_back_rng.margin_left_2 = data_ref.in_data.ranges[(int)(step_20*13)];
      out_back_rng.margin_left_1 = data_ref.in_data.ranges[(int)(step_20*14)];
      out_back_rng.center = data_ref.in_data.ranges[(int)(step_20*15)];
      out_back_rng.margin_right_1 = data_ref.in_data.ranges[(int)(step_20*16)];
      out_back_rng.margin_right_2 = data_ref.in_data.ranges[(int)(step_20*17)];
      break;

    case LidarType::A1:
      // Back
      out_back_rng.margin_left_2 = data_ref.in_data.ranges[(int)(step_20*18)];
      out_back_rng.margin_left_1 = data_ref.in_data.ranges[(int)(step_20*19)];
      out_back_rng.center = data_ref.in_data.ranges[0];
      out_back_rng.margin_right_1 = data_ref.in_data.ranges[(int)(step_20*1)];
      out_back_rng.margin_right_2 = data_ref.in_data.ranges[(int)(step_20*2)];

      // Right
      out_right_rng.margin_left_2 = data_ref.in_data.ranges[(int)(step_20*3)];
      out_right_rng.margin_left_1 = data_ref.in_data.ranges[(int)(step_20*4)];
      out_right_rng.center = data_ref.in_data.ranges[(int)(step_20*5)];
      out_right_rng.margin_right_1 = data_ref.in_data.ranges[(int)(step_20*6)];
      out_right_rng.margin_right_2 = data_ref.in_data.ranges[(int)(step_20*7)];

      // Front
      out_front_rng.margin_left_2 = data_ref.in_data.ranges[(int)(step_20*8)];
      out_front_rng.margin_left_1 = data_ref.in_data.ranges[(int)(step_20*9)];
      out_front_rng.center = data_ref.in_data.ranges[(int)(step_20*10)];
      out_front_rng.margin_right_1 = data_ref.in_data.ranges[(int)(step_20*11)];
      out_front_rng.margin_right_2 = data_ref.in_data.ranges[(int)(step_20*12)];

      // Left
      out_left_rng.margin_left_2 = data_ref.in_data.ranges[(int)(step_20*13)];
      out_left_rng.margin_left_1 = data_ref.in_data.ranges[(int)(step_20*14)];
      out_left_rng.center = data_ref.in_data.ranges[(int)(step_20*15)];
      out_left_rng.margin_right_1 = data_ref.in_data.ranges[(int)(step_20*16)];
      out_left_rng.margin_right_2 = data_ref.in_data.ranges[(int)(step_20*17)];
      break;

    case LidarType::S1:
      // Back
      out_back_rng.margin_left_2 = data_ref.in_data.ranges[(int)(step_20*18)];
      out_back_rng.margin_left_1 = data_ref.in_data.ranges[(int)(step_20*19)];
      out_back_rng.center = data_ref.in_data.ranges[0];
      out_back_rng.margin_right_1 = data_ref.in_data.ranges[(int)(step_20*1)];
      out_back_rng.margin_right_2 = data_ref.in_data.ranges[(int)(step_20*2)];

      // Right
      out_right_rng.margin_left_2 = data_ref.in_data.ranges[(int)(step_20*3)];
      out_right_rng.margin_left_1 = data_ref.in_data.ranges[(int)(step_20*4)];
      out_right_rng.center = data_ref.in_data.ranges[(int)(step_20*5)];
      out_right_rng.margin_right_1 = data_ref.in_data.ranges[(int)(step_20*6)];
      out_right_rng.margin_right_2 = data_ref.in_data.ranges[(int)(step_20*7)];

      // Front
      out_front_rng.margin_left_2 = data_ref.in_data.ranges[(int)(step_20*8)];
      out_front_rng.margin_left_1 = data_ref.in_data.ranges[(int)(step_20*9)];
      out_front_rng.center = data_ref.in_data.ranges[(int)(step_20*10)];
      out_front_rng.margin_right_1 = data_ref.in_data.ranges[(int)(step_20*11)];
      out_front_rng.margin_right_2 = data_ref.in_data.ranges[(int)(step_20*12)];

      // Left
      out_left_rng.margin_left_2 = data_ref.in_data.ranges[(int)(step_20*13)];
      out_left_rng.margin_left_1 = data_ref.in_data.ranges[(int)(step_20*14)];
      out_left_rng.center = data_ref.in_data.ranges[(int)(step_20*15)];
      out_left_rng.margin_right_1 = data_ref.in_data.ranges[(int)(step_20*16)];
      out_left_rng.margin_right_2 = data_ref.in_data.ranges[(int)(step_20*17)];
      break;

    default:
      break;
    }
    lidar_init = true;
  }
  if(copter->get_current_mission() == Mode::Outdoor)
    stop();
}

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
    lidar_data.in_data = *input;
    range_size = lidar_data.in_data.ranges.size();
    std::thread th(lidar_proc, lidar_data);
    th.join();
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
    lidar_temp.front_min = std::min({lidar_data.in_data.ranges[idx_front[0]], lidar_data.in_data.ranges[idx_front[1]], lidar_data.in_data.ranges[idx_front[2]]});
    lidar_temp.right_min = std::min({lidar_data.in_data.ranges[idx_right[0]], lidar_data.in_data.ranges[idx_right[1]], lidar_data.in_data.ranges[idx_right[2]]});
    lidar_temp.back_min = std::min({lidar_data.in_data.ranges[idx_back[0]], lidar_data.in_data.ranges[idx_back[1]], lidar_data.in_data.ranges[idx_back[2]]});
    lidar_temp.left_min = std::min({lidar_data.in_data.ranges[idx_left[0]], lidar_data.in_data.ranges[idx_left[1]], lidar_data.in_data.ranges[idx_left[2]]});

    lidar_temp.front_max = std::max({lidar_data.in_data.ranges[idx_front[0]], lidar_data.in_data.ranges[idx_front[1]], lidar_data.in_data.ranges[idx_front[2]]});
    lidar_temp.right_max = std::max({lidar_data.in_data.ranges[idx_right[0]], lidar_data.in_data.ranges[idx_right[1]], lidar_data.in_data.ranges[idx_right[2]]});
    lidar_temp.back_max = std::max({lidar_data.in_data.ranges[idx_back[0]], lidar_data.in_data.ranges[idx_back[1]], lidar_data.in_data.ranges[idx_back[2]]});
    lidar_temp.left_max = std::max({lidar_data.in_data.ranges[idx_left[0]], lidar_data.in_data.ranges[idx_left[1]], lidar_data.in_data.ranges[idx_left[2]]});

    return std::make_tuple(true, lidar_temp);
  }

  float Lidar::get_front(int idx)
  {
    return lidar_data.in_data.ranges[idx_front[idx]];
  }

  float Lidar::max_front()
  {
    _temp1 = lidar_data.in_data.ranges[idx_front[0]];
    _temp2 = lidar_data.in_data.ranges[idx_front[1]];
    _temp3 = lidar_data.in_data.ranges[idx_front[2]];
    _max_temp = _temp1 < 20.0f ? _temp1 : _temp2;
    _max_temp = _temp2 > _max_temp && _temp2 < 20.0f ? _temp2 : _max_temp; 
    _max_temp = _temp3 > _max_temp && _temp3 < 20.0f ? _temp3 : _max_temp;
    if(_max_temp > 39.0f)
      return 0;
    return _max_temp;
  }

  float Lidar::get_left(int idx)
  {
    return lidar_data.in_data.ranges[idx_left[idx]];
  }

  float Lidar::get_right(int idx)
  {
    return lidar_data.in_data.ranges[idx_right[idx]];
  }

  float Lidar::get_back(int idx)
  {
    return lidar_data.in_data.ranges[idx_back[idx]];
  }

  sensor_msgs::LaserScan Lidar::get_raw(){
    return lidar_data.in_data;
  }

  void Lidar::axis(Axis& axis, int idx)
  {
    axis.front = lidar_data.in_data.ranges[idx_front[idx]];
    axis.back = lidar_data.in_data.ranges[idx_back[idx]];
    axis.left = lidar_data.in_data.ranges[idx_left[idx]];
    axis.right = lidar_data.in_data.ranges[idx_right[idx]];
  }
}