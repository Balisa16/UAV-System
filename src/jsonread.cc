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

	    Json::Value jsonData;
	    Json::CharReaderBuilder reader;
	    JSONCPP_STRING errs;
	    Json::parseFromStream(reader, stream_reader, &jsonData, &errs);

	    int counter = 1;
	    std::cout << jsonData.size() << std::endl;
	    for (const auto& item : jsonData) {
	    	std::cout << "Read " << counter << std::endl;
	    	WayPoint _temp_wp = {
		        	item["x"].asFloat(),
		        	item["y"].asFloat(),
		        	item["z"].asFloat(),
		        	item["yaw"].asFloat()
	        	};
	        data.push_back({
	        	counter,
	        	item["header"].asString(),
	        	item["speed"].asFloat(),
	        	_temp_wp
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