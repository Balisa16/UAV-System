#ifndef JSON_READER
#define JSON_READER

#include <iostream>
#include <vector>
#include <fstream>
#include <json.hpp>
#include <enum.hpp>
#include <iomanip>
namespace EMIRO
{
	typedef struct
	{
		int index;
		std::string header;
		float speed;
		WayPoint wp;
	} Target;

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
		friend std::ostream &operator<<(std::ostream &os, Target target);
		void operator+=(const Target &target);
	};

	JSONReader::JSONReader() {}

	JSONReader::JSONReader(std::string path)
	{
		operator=(path);
	}

	inline std::vector<Target> JSONReader::get_data()
	{
		return data;
	}

	inline void JSONReader::get_data(std::vector<Target> &target)
	{
		target = data;
	}

	inline void JSONReader::operator=(std::string path)
	{
		this->file_path = path;
		std::ifstream stream_reader(file_path);
		if (!stream_reader.is_open())
		{
			std::cerr << "Failed to Open JSON File." << std::endl;
			return;
		}

		Json::Value json_value;
		Json::CharReaderBuilder reader;
		JSONCPP_STRING errs;
		Json::parseFromStream(reader, stream_reader, &json_value, &errs);

		int counter = 1;
		data.clear();
		for (const auto &item : json_value)
		{
			data.push_back({counter,
							item["header"].asString(),
							item["speed"].asFloat(),
							{item["x"].asFloat(),
							 item["y"].asFloat(),
							 item["z"].asFloat(),
							 item["yaw"].asFloat()}});
			counter++;
		}
		stream_reader.close();
	}

	inline void JSONReader::operator+=(const Target &target)
	{
		Json::Value root;
		Json::StreamWriterBuilder builder;
		const std::unique_ptr<Json::StreamWriter> writer(builder.newStreamWriter());
		std::ofstream stream_writer(file_path);
		if (!stream_writer.is_open())
		{
			std::cout << "Failed to Open JSON File." << std::endl;
			return;
		}

		root["header"] = target.header;
		root["speed"] = target.speed;
		root["x"] = target.wp.x;
		root["y"] = target.wp.y;
		root["z"] = target.wp.z;
		writer->write(root, &stream_writer);
		stream_writer.close();
	}

	inline std::ostream &operator<<(std::ostream &os, Target target)
	{
		os << std::fixed << std::setprecision(2) << "Index\t: " << target.index << "\nHeader\t: " << target.header << "\nSpeed\t: " << target.speed << "\nTarget\n\tx : " << target.wp.x << "\n\ty : " << target.wp.y << "\n\tz : " << target.wp.z << '\n'
		   << std::defaultfloat;
		return os;
	}

	JSONReader::~JSONReader() {}
}

#endif