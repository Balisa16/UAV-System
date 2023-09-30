
#include <param.hpp>

namespace EMIRO{

	Param::Param(){}

	void Param::init(std::string filename, ros::NodeHandle *nh, std::shared_ptr<Logger> logger)
	{
		node = (*nh);
		log = logger;
		this->filename = filename;
		param_set_client = (*nh).serviceClient<mavros_msgs::ParamSet>("/mavros/param/set");
		param_get_client = (*nh).serviceClient<mavros_msgs::ParamGet>("/mavros/param/get");
		log->write_show(LogLevel::INFO, "Init Parameter Service");
	}

	void Param::load()
	{
		log->write_show(LogLevel::INFO, "Load Parameter Service");

		if(reader.is_open())
			reader.close();
		
		reader.open(filename);
	    if (reader.is_open())
			log->write_show(LogLevel::ERROR, "Failed Load JSON File");
	    else
	    {
	    	enable_use = true;

	    }
	}

	Param::~Param(){
		reader.close();
	}
}