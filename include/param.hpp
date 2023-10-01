#ifndef PARAM_H
#define PARAM_H

#include <iostream>
#include <memory>
#include <ros/ros.h>
#include <mavros_msgs/ParamSet.h>
#include <mavros_msgs/ParamGet.h>
#include <mavros_msgs/ParamValue.h>
#include <logger.hpp>

#include <string>
#include <termios.h>
#include <unistd.h>
#include <fstream>
#include <jsoncpp/json/json.h>
#include <vector>
#include <cmath>
#include <enum.hpp>

namespace EMIRO{
	class Param
	{
	private:
		ros::NodeHandle node;
		std::shared_ptr<Logger> log;
		ros::ServiceClient param_set_client;
		ros::ServiceClient param_get_client;
		std::string filename;
		std::ifstream reader;

		bool enable_use = false;
	public:
		Param();
		void init(std::string filename, ros::NodeHandle *nh, std::shared_ptr<Logger> logger);
		void load();
		~Param();
		
	};
}

#endif // PARAM