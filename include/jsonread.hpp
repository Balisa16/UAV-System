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
	}JSONData;

	class JSONReader
	{
	private:
		std::string file_path = "";
		std::vector<JSONData> data;
	public:
		JSONReader();
		void init(std::string file_path);
		void read();
		std::vector<JSONData> get_data();
		~JSONReader();
		
	};
}

#endif