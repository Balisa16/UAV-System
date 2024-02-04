#include <jsonio.hpp>

namespace EMIRO {
JsonIO::JsonIO() {}

JsonIO::JsonIO(std::string path) { operator=(path); }

std::map<TargetKey, Target> &JsonIO::get_data_map() { return data; }

std::vector<Target> JsonIO::get_data_vector() {
    std::vector<Target> target;
    for (const auto &i : data)
        target.push_back(i.second);
    return target;
}

void JsonIO::get_data(std::map<TargetKey, Target> &target) { target = data; }

void JsonIO::operator=(std::string path) {
    this->file_path = path;
    std::ifstream stream_reader(file_path);
    if (!stream_reader.is_open()) {
        if (!boost::filesystem::exists(file_path)) {
            boost::filesystem::path parent_path =
                boost::filesystem::path(file_path).parent_path();
            if (!boost::filesystem::exists(parent_path)) {
                std::cout
                    << "\033[31m\033[1mError :\033[0m Directory invalid\n";
                std::cout << "\033[31m\033[1mInfo :\033[0m Current path is "
                          << boost::filesystem::current_path().string() << "\n";
                std::cout
                    << "\033[31m\033[1mInfo :\033[0m Your input file path is "
                    << parent_path.string() << "\n";
            } else {
                std::cout << "\033[31m\033[1mError :\033[0m File name invalid. "
                             "Here is the list of exist files\n";
                int cnt = 1;
                for (const auto &entry :
                     boost::filesystem::directory_iterator(parent_path))
                    if (boost::filesystem::is_regular_file(entry.path())) {
                        std::cout << cnt << ".\t: " << entry.path().filename()
                                  << '\n';
                        cnt++;
                    }
            }
            exit(EXIT_FAILURE);
        }

        std::cout << "\033[31m\033[1mError :\033[0m Failed Open File."
                  << std::endl;

        // std::cerr << "Failed to Open File." << std::endl;
        // FILE *pipe = popen("pwd", "r");
        // if (!pipe)
        // 	return;
        // char buffer[128];
        // std::string result;
        // while (fgets(buffer, sizeof(buffer), pipe) != nullptr)
        // 	result += buffer;
        // std::cout << "Current path : " << result << std::endl;
        // pclose(pipe);
        exit(EXIT_FAILURE);
    }

    Json::Value json_value;
    Json::CharReaderBuilder reader;
    JSONCPP_STRING errs;
    Json::parseFromStream(reader, stream_reader, &json_value, &errs);

    data_counter = 0;
    data.clear();
    for (const auto &item : json_value) {
        TargetKey t_key{data_counter, item["header"].asString()};
        if (t_key.exist(data)) {
            std::cerr << "Error: Key '" << item["header"].asString()
                      << "' already exists.\n";
            continue;
        }
        data.emplace(t_key,
                     Target(item["header"].asString(), item["speed"].asFloat(),
                            {item["x"].asFloat(), item["y"].asFloat(),
                             item["z"].asFloat(), item["yaw"].asFloat()}));
        data_counter++;
    }

    for (const auto &a : data) {
        std::cout << "Out : " << a.first.data_idx << "\t: " << a.first.data_name
                  << std::endl;
    }

    stream_reader.close();
}

void JsonIO::operator+=(const Target &target) {
    TargetKey t_key{data_counter, target.header};
    if (t_key.exist(data)) {
        std::cerr << "Error: Key '" << target.header << "' already exists."
                  << std::endl;
        return;
    }
    data.emplace(t_key, target);
    data_counter++;

    std::ifstream input_file(file_path);
    if (!input_file.is_open()) {
        std::cerr << "\033[31m\033[1mFailed Open File.\033[0m" << std::endl;
        exit(EXIT_FAILURE);
    }

    std::vector<std::string> lines;
    std::string line;
    while (std::getline(input_file, line) && !input_file.eof())
        lines.push_back(line);
    input_file.close();

    Json::Value root;
    Json::StreamWriterBuilder builder;
    const std::unique_ptr<Json::StreamWriter> writer(builder.newStreamWriter());
    std::ofstream stream_writer(file_path);
    if (!stream_writer.is_open()) {
        std::cout << "Failed Open File." << std::endl;
        return;
    }
    if (lines.size() <= 0)
        stream_writer << "[\n";
    if (lines.size()) {
        std::string latest_line = lines.back();
        lines.pop_back();
        for (std::string &line : lines)
            stream_writer << line << '\n';
        stream_writer << latest_line << ",\n";
    }

    root["header"] = target.header;
    root["speed"] = target.speed;
    root["x"] = target.wp.x;
    root["y"] = target.wp.y;
    root["z"] = target.wp.z;
    root["yaw"] = target.wp.yaw;
    writer->write(root, &stream_writer);
    stream_writer << "\n]";
    stream_writer.close();
}

void JsonIO::operator-=(const Target &target) {}

JsonIO::~JsonIO() {}
} // namespace EMIRO