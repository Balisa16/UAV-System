
#include <param.hpp>

namespace EMIRO{

	Param::Param(){}

	void Param::init(ros::NodeHandle& nh, Logger& logger)
	{
		node = nh;
		log = logger;
		param_set_client = nh.serviceClient<mavros_msgs::ParamSet>("/mavros/param/set");
		param_get_client = nh.serviceClient<mavros_msgs::ParamGet>("/mavros/param/get");
		log.write_show(LogLevel::INFO, "Init Parameter Service");
	}

	void load()
	{
		logger.write_show(LogLevel::INFO, "Init Parameter Service");
	}

	Param::~Param(){}
}