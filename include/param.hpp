#pragma once

#include <Logger.hpp>
#include <iostream>
#include <mavros_msgs/ParamGet.h>
#include <mavros_msgs/ParamSet.h>
#include <mavros_msgs/ParamValue.h>
#include <memory>
#include <ros/ros.h>

#include <cmath>
#include <enum.hpp>
#include <fstream>
#include <jsoncpp/json/json.h>
#include <string>
#include <termios.h>
#include <unistd.h>
#include <vector>

namespace EMIRO {
class Param {
  public:
    Param();

    void init(std::string filename, ros::NodeHandle *nh,
              std::shared_ptr<Logger> logger);

    void load();

    ~Param();

  private:
    ros::NodeHandle node;
    std::shared_ptr<Logger> log;
    ros::ServiceClient param_set_client;
    ros::ServiceClient param_get_client;
    std::string filename;
    std::ifstream reader;

    bool enable_use = false;
};
} // namespace EMIRO