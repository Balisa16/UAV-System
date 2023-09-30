
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
	    	Json::Value _json_data;
		    Json::CharReaderBuilder _reader_builder;
		    JSONCPP_STRING errs;
		    Json::parseFromStream(_reader_builder, reader, &_json_data, &errs);

		    std::vector<JSONData> data;
		    for (const auto& item : _json_data) {
		        CommandItem first_command = {
		            item["commands"][0]["name"].asString(),
		            item["commands"][0]["command"].asString()
		        };
		        JSONData data_json = JSONData(
		            item["host"].asString(),
		            item["ip"].asString(),
		            item["pass"].asString(),
		            first_command
		        );
		        for (int i = 1; i < item["commands"].size(); i++)
		        {
		            CommandItem cmd_item = {item["commands"][i]["name"].asString(), item["commands"][i]["command"].asString()};
		            data_json.add_item(cmd_item);
		        }
		        data.push_back(data_json);
		    }
	    	enable_use = true;
	    }
	}

	Param::~Param(){
		if(reader.is_open())
			reader.close();
	}
}