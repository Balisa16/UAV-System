#include <jsonio.hpp>

namespace EMIRO
{
    JsonIO::JsonIO() {}

    JsonIO::JsonIO(std::string path) { operator=(path); }

    std::map<TargetKey, Target> &JsonIO::get_data_map() { return data; }

    std::vector<Target> JsonIO::get_data_vector() const
    {
        std::vector<Target> target;
        for (const auto &i : data)
            target.push_back(i.second);
        return target;
    }

    void JsonIO::get_data(std::map<TargetKey, Target> &target) const { target = data; }

    void JsonIO::optimize_distance()
    {
        std::vector<Target> data_vector;
        for (auto &i : data)
            data_vector.push_back(i.second);

        float before_distance = 0;
        for (int i = 1; i < data_vector.size(); i++)
            before_distance += sqrt(pow(data_vector[i].wp.x - data_vector[i - 1].wp.x, 2) + pow(data_vector[i].wp.y - data_vector[i - 1].wp.y, 2) + pow(data_vector[i].wp.z - data_vector[i - 1].wp.z, 2));

        WayPoint prev_target, curr_target;
        bool _first = true;
        int maks_data = data_vector.size() < data.size() ? data_vector.size() : data.size();
        float total_distance = 0;

        for (int i = 0; i < maks_data - 1; i++)
        {
            if (!data_vector[i].header.compare("Home") || !data_vector[i].header.compare("Takeoff"))
                continue;
            float min_distance = MAXFLOAT;
            int selected_idx = i;
            for (int j = i + 1; j < maks_data; j++)
            {
                if (!data_vector[j].header.compare("Home") || !data_vector[j].header.compare("Takeoff"))
                    continue;
                float distance = sqrt(pow(data_vector[j].wp.x - data_vector[i].wp.x, 2) + pow(data_vector[j].wp.y - data_vector[i].wp.y, 2) + pow(data_vector[j].wp.z - data_vector[i].wp.z, 2));
                if (distance < min_distance)
                {
                    min_distance = distance;
                    selected_idx = j;
                }
            }
            Target _temp = data_vector[i];
            data_vector[i] = data_vector[selected_idx];
            data_vector[selected_idx] = _temp;
            total_distance += min_distance;
        }

        if (before_distance <= total_distance)
        {
            std::cout << "Already Optimized : " << before_distance << std::endl;
            return;
        }
        std::cout << "Optimized : " << before_distance << "m => " << total_distance << "m\n";

        std::map<TargetKey, Target> _updated_data;
        for (int i = 0; i < data_vector.size(); i++)
            for (auto j : data)
                if (j.second.header == data_vector[i].header)
                {
                    TargetKey _key(i, j.second.header);
                    _updated_data.emplace(_key, data_vector[i]);
                    break;
                }

        data = _updated_data;
    }

    void JsonIO::operator=(std::string path)
    {
        this->file_path = path;
        std::ifstream stream_reader(file_path);
        if (!stream_reader.is_open())
        {
            if (!boost::filesystem::exists(file_path))
            {
                boost::filesystem::path parent_path =
                    boost::filesystem::path(file_path).parent_path();
                if (!boost::filesystem::exists(parent_path))
                {
                    std::cout
                        << "\033[31m\033[1mError :\033[0m Directory invalid\n";
                    std::cout << "\033[31m\033[1mInfo :\033[0m Current path is "
                              << boost::filesystem::current_path().string() << "\n";
                    std::cout
                        << "\033[31m\033[1mInfo :\033[0m Your input file path is "
                        << parent_path.string() << "\n";
                }
                else
                {
                    std::cout << "\033[31m\033[1mError :\033[0m File name invalid. "
                                 "Here is the list of exist files\n";
                    int cnt = 1;
                    for (const auto &entry :
                         boost::filesystem::directory_iterator(parent_path))
                        if (boost::filesystem::is_regular_file(entry.path()))
                        {
                            std::cout << cnt << ".\t: " << entry.path().filename()
                                      << '\n';
                            cnt++;
                        }
                }
                exit(EXIT_FAILURE);
            }

            std::cout << "\033[31m\033[1mError :\033[0m Failed Open File."
                      << std::endl;
            exit(EXIT_FAILURE);
        }

        Json::Value json_value;
        Json::CharReaderBuilder reader;
        JSONCPP_STRING errs;
        Json::parseFromStream(reader, stream_reader, &json_value, &errs);

        data_counter = 0;
        data.clear();
        for (const auto &item : json_value)
        {
            TargetKey t_key{data_counter, item["header"].asString()};
            if (t_key.exist(data))
            {
                std::cerr << "Error: Key '" << item["header"].asString()
                          << "' already exists.\n";
                continue;
            }
            Target _new_data = Target(item["header"].asString(),
                                      item["speed"].asFloat(),
                                      {item["x"].asFloat(),
                                       item["y"].asFloat(),
                                       item["z"].asFloat(),
                                       item["yaw"].asFloat()});
            data.emplace(t_key, _new_data);
            data_counter++;
        }

        stream_reader.close();
    }

    void JsonIO::operator+=(const Target &target)
    {
        TargetKey t_key{data_counter, target.header};
        if (t_key.exist(data))
        {
            std::cerr << "Error: Key '" << target.header << "' already exists."
                      << std::endl;
            return;
        }
        data.emplace(t_key, target);
        data_counter++;

        std::ifstream input_file(file_path);
        if (!input_file.is_open())
        {
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
        if (!stream_writer.is_open())
        {
            std::cout << "Failed Open File." << std::endl;
            return;
        }
        if (lines.size() <= 0)
            stream_writer << "[\n";
        if (lines.size())
        {
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