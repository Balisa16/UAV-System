#include <lidar.hpp>

namespace EMIRO{
  void lidar_proc(LidarRef *data_ref)
  {
    std::lock_guard<std::mutex> lg(lidar_proc_mtx);
    data_ref->out_size = data_ref->in_data.ranges.size();
    
    float step_20 = data_ref->out_size/20;
    data_ref->out_is_valid = false;

    if(data_ref->out_size > 0)
      data_ref->status =  LidarStatus::Run;

    if(data_ref->status == LidarStatus::Run)
    {
      data_ref->out_is_valid = true;
      switch (data_ref->type)
      {
      case LidarType::Simulator:
        // Right
        data_ref->out_right_rng.margin_left_2 = data_ref->in_data.ranges[(int)(step_20*18)];
        data_ref->out_right_rng.margin_left_1 = data_ref->in_data.ranges[(int)(step_20*19)];
        data_ref->out_right_rng.center = data_ref->in_data.ranges[0];
        data_ref->out_right_rng.margin_right_1 = data_ref->in_data.ranges[(int)(step_20*1)];
        data_ref->out_right_rng.margin_right_2 = data_ref->in_data.ranges[(int)(step_20*2)];

        // Front
        data_ref->out_front_rng.margin_left_2 = data_ref->in_data.ranges[(int)(step_20*3)];
        data_ref->out_front_rng.margin_left_1 = data_ref->in_data.ranges[(int)(step_20*4)];
        data_ref->out_front_rng.center = data_ref->in_data.ranges[(int)(step_20*5)];
        data_ref->out_front_rng.margin_right_1 = data_ref->in_data.ranges[(int)(step_20*6)];
        data_ref->out_front_rng.margin_right_2 = data_ref->in_data.ranges[(int)(step_20*7)];

        // Left
        data_ref->out_left_rng.margin_left_2 = data_ref->in_data.ranges[(int)(step_20*8)];
        data_ref->out_left_rng.margin_left_1 = data_ref->in_data.ranges[(int)(step_20*9)];
        data_ref->out_left_rng.center = data_ref->in_data.ranges[(int)(step_20*10)];
        data_ref->out_left_rng.margin_right_1 = data_ref->in_data.ranges[(int)(step_20*11)];
        data_ref->out_left_rng.margin_right_2 = data_ref->in_data.ranges[(int)(step_20*12)];

        // Back
        data_ref->out_back_rng.margin_left_2 = data_ref->in_data.ranges[(int)(step_20*13)];
        data_ref->out_back_rng.margin_left_1 = data_ref->in_data.ranges[(int)(step_20*14)];
        data_ref->out_back_rng.center = data_ref->in_data.ranges[(int)(step_20*15)];
        data_ref->out_back_rng.margin_right_1 = data_ref->in_data.ranges[(int)(step_20*16)];
        data_ref->out_back_rng.margin_right_2 = data_ref->in_data.ranges[(int)(step_20*17)];
        break;

      case LidarType::A1:
        // Back
        data_ref->out_back_rng.margin_left_2 = data_ref->in_data.ranges[(int)(step_20*18)];
        data_ref->out_back_rng.margin_left_1 = data_ref->in_data.ranges[(int)(step_20*19)];
        data_ref->out_back_rng.center = data_ref->in_data.ranges[0];
        data_ref->out_back_rng.margin_right_1 = data_ref->in_data.ranges[(int)(step_20*1)];
        data_ref->out_back_rng.margin_right_2 = data_ref->in_data.ranges[(int)(step_20*2)];

        // Right
        data_ref->out_right_rng.margin_left_2 = data_ref->in_data.ranges[(int)(step_20*3)];
        data_ref->out_right_rng.margin_left_1 = data_ref->in_data.ranges[(int)(step_20*4)];
        data_ref->out_right_rng.center = data_ref->in_data.ranges[(int)(step_20*5)];
        data_ref->out_right_rng.margin_right_1 = data_ref->in_data.ranges[(int)(step_20*6)];
        data_ref->out_right_rng.margin_right_2 = data_ref->in_data.ranges[(int)(step_20*7)];

        // Front
        data_ref->out_front_rng.margin_left_2 = data_ref->in_data.ranges[(int)(step_20*8)];
        data_ref->out_front_rng.margin_left_1 = data_ref->in_data.ranges[(int)(step_20*9)];
        data_ref->out_front_rng.center = data_ref->in_data.ranges[(int)(step_20*10)];
        data_ref->out_front_rng.margin_right_1 = data_ref->in_data.ranges[(int)(step_20*11)];
        data_ref->out_front_rng.margin_right_2 = data_ref->in_data.ranges[(int)(step_20*12)];

        // Left
        data_ref->out_left_rng.margin_left_2 = data_ref->in_data.ranges[(int)(step_20*13)];
        data_ref->out_left_rng.margin_left_1 = data_ref->in_data.ranges[(int)(step_20*14)];
        data_ref->out_left_rng.center = data_ref->in_data.ranges[(int)(step_20*15)];
        data_ref->out_left_rng.margin_right_1 = data_ref->in_data.ranges[(int)(step_20*16)];
        data_ref->out_left_rng.margin_right_2 = data_ref->in_data.ranges[(int)(step_20*17)];
        break;

      case LidarType::S1:
        // Back
        data_ref->out_back_rng.margin_left_2 = data_ref->in_data.ranges[(int)(step_20*18)];
        data_ref->out_back_rng.margin_left_1 = data_ref->in_data.ranges[(int)(step_20*19)];
        data_ref->out_back_rng.center = data_ref->in_data.ranges[0];
        data_ref->out_back_rng.margin_right_1 = data_ref->in_data.ranges[(int)(step_20*1)];
        data_ref->out_back_rng.margin_right_2 = data_ref->in_data.ranges[(int)(step_20*2)];

        // Right
        data_ref->out_right_rng.margin_left_2 = data_ref->in_data.ranges[(int)(step_20*3)];
        data_ref->out_right_rng.margin_left_1 = data_ref->in_data.ranges[(int)(step_20*4)];
        data_ref->out_right_rng.center = data_ref->in_data.ranges[(int)(step_20*5)];
        data_ref->out_right_rng.margin_right_1 = data_ref->in_data.ranges[(int)(step_20*6)];
        data_ref->out_right_rng.margin_right_2 = data_ref->in_data.ranges[(int)(step_20*7)];

        // Front
        data_ref->out_front_rng.margin_left_2 = data_ref->in_data.ranges[(int)(step_20*8)];
        data_ref->out_front_rng.margin_left_1 = data_ref->in_data.ranges[(int)(step_20*9)];
        data_ref->out_front_rng.center = data_ref->in_data.ranges[(int)(step_20*10)];
        data_ref->out_front_rng.margin_right_1 = data_ref->in_data.ranges[(int)(step_20*11)];
        data_ref->out_front_rng.margin_right_2 = data_ref->in_data.ranges[(int)(step_20*12)];

        // Left
        data_ref->out_left_rng.margin_left_2 = data_ref->in_data.ranges[(int)(step_20*13)];
        data_ref->out_left_rng.margin_left_1 = data_ref->in_data.ranges[(int)(step_20*14)];
        data_ref->out_left_rng.center = data_ref->in_data.ranges[(int)(step_20*15)];
        data_ref->out_left_rng.margin_right_1 = data_ref->in_data.ranges[(int)(step_20*16)];
        data_ref->out_left_rng.margin_right_2 = data_ref->in_data.ranges[(int)(step_20*17)];
        break;

      default:
        break;
      }
    }
  }

  Lidar::Lidar()
  {
    lidar_data.status = LidarStatus::None;
  }

  void Lidar::init(std::shared_ptr<EMIRO::Copter> copter, std::shared_ptr<EMIRO::Logger> logger)
  {
    lidar_data.status = LidarStatus::Init;
    this->copter = copter;
    this->logger = logger;
    logger->write_show(LogLevel::INFO, "Lidar initialized");
  }

  void Lidar::stop()
  {
    lidar_data.status = LidarStatus::Stop;
    lidar_sub.shutdown();
  }

  void Lidar::scan(const sensor_msgs::LaserScan::ConstPtr& input)
  {
    lidar_data.in_data = *input;
    std::thread th(lidar_proc, &lidar_data);
    // th.join();
    th.detach(); // Not waiting thread
  }

  void Lidar::start(ros::NodeHandle *nh, LidarType lidar)
  {
    lidar_data.status = LidarStatus::Start;
    lidar_data.type = lidar;

    std::string lidar_path;

    // Select match path
    switch (lidar_data.type)
    {
    case LidarType::Simulator:
      lidar_path = sim_lidar_path;
      logger->write_show(LogLevel::WARNING, "Using simulator lidar configuration");
      break;

    case LidarType::A1:
      lidar_path = a1_lidar_path;
      logger->write_show(LogLevel::WARNING, "Using A1 lidar configuration");
      break;

    case LidarType::S1:
      lidar_path = s1_lidar_path;
      logger->write_show(LogLevel::WARNING, "Using S1 lidar configuration");
      break;
    
    default:
      logger->write_show(LogLevel::ERROR, "Unknown Lidar Type. Failed config lidar");
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

  LidarStatus Lidar::check()
  {
    return lidar_data.status;
  }

  float Lidar::get_front(int idx)
  {
    return lidar_data.out_front_rng.center;
  }

  float Lidar::max_front()
  {
    return std::max({
        lidar_data.out_front_rng.margin_left_2,
        lidar_data.out_front_rng.margin_left_1, 
        lidar_data.out_front_rng.center, 
        lidar_data.out_front_rng.margin_right_1,
        lidar_data.out_front_rng.margin_right_2});
  }

  float Lidar::get_left(int idx)
  {
    return lidar_data.out_left_rng.center;
  }

  float Lidar::get_right(int idx)
  {
    return lidar_data.out_right_rng.center;
  }

  float Lidar::get_back(int idx)
  {
    return lidar_data.out_back_rng.center;
  }

  sensor_msgs::LaserScan Lidar::get_raw(){
    return lidar_data.in_data;
  }

  void Lidar::axis(Axis& axis, int idx)
  {
    axis.front = lidar_data.out_front_rng.center;
    axis.back = lidar_data.out_back_rng.center;
    axis.left = lidar_data.out_left_rng.center;
    axis.right = lidar_data.out_right_rng.center;
  }
}