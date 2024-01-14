#ifndef JSON_READER
#define JSON_READER

#include <iostream>
#include <vector>
#include <fstream>
#include <jsoncpp/json/json.h>
#include <enum.hpp>

namespace EMIRO{
	typedef struct
	{
		int index;
		std::string header;
		float speed;
		WayPoint wp;
	}Target;

	class JSONReader
	{
	private:
		std::string file_path = "";
		std::vector<Target> data;
	public:
		JSONReader();
		JSONReader(std::string path);
		std::vector<Target> get_data();
		void get_data(std::vector<Target> &target);
		~JSONReader();
		void operator=(std::string path);
	};

	JSONReader::JSONReader(){}

	JSONReader::JSONReader(std::string path)
	{
		operator=(path);
	}
	
	std::vector<Target> JSONReader::get_data()
	{
		return data;
	}

	void JSONReader::get_data(std::vector<Target> &target){
		target = data;
	}

	void JSONReader::operator=(std::string path){
		this->file_path = path;
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
		stream_reader.close();
	}

	JSONReader::~JSONReader(){}
}

#endif