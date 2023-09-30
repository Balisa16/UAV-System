#ifndef PARAM_H
#define PARAM_H

#include <iostream>
#include <memory>
#include <ros/ros.h>
#include <mavros_msgs/ParamSet.h>
#include <mavros_msgs/ParamGet.h>
#include <mavros_msgs/ParamValue.h>
#include <logger.hpp>

namespace EMIRO{
	class Param
	{
	private:
		ros::NodeHandle node;
		Logger log;
		ros::ServiceClient param_set_client;
		ros::ServiceClient param_get_client;
	public:
		Param();
		void init(ros::NodeHandle *nh, Logger *logger);
		void load();
		~Param();
		
	};
}

#endif // PARAM