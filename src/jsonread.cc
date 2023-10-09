#include <jsonread.hpp>

namespace EMIRO{
	JSONReader::JSONReader(){}

	void JSONReader::init(std::string file_path)
	{
		this->file_path = file_path;
	}
	
	void JSONReader::read()
	{
		std::ifstream stream_reader(file_path);
	    if (!stream_reader.is_open()) {
	        std::cerr << "Failed to Open JSON File." << std::endl;
	        return;
	    }

	    Json::Value json_value;
	    Json::CharReaderBuilder reader;
	    JSONCPP_STRING errs;
	    Json::parseFromStream(reader, stream_reader, &json_value, &errs);

	    int counter = 1;
	    data.clear();
	    for (const auto& item : json_value) {
	        data.push_back({
	        	counter,
	        	item["header"].asString(),
	        	item["speed"].asFloat(),
	        	{
		        	item["x"].asFloat(),
		        	item["y"].asFloat(),
		        	item["z"].asFloat(),
		        	item["yaw"].asFloat()
	        	}
	        });
	       	counter++;
	    }
	}
	std::vector<JSONData> JSONReader::get_data()
	{
		return data;
	}
	JSONReader::~JSONReader(){}
}