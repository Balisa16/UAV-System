
#include <param.hpp>

namespace EMIRO
{

    Param::Param() {}

    void Param::init(std::string filename, ros::NodeHandle *nh,
                     std::shared_ptr<Logger> logger)
    {
        node = (*nh);
        log = logger;
        this->filename = filename;
        param_set_client =
            (*nh).serviceClient<mavros_msgs::ParamSet>("/mavros/param/set");
        param_get_client =
            (*nh).serviceClient<mavros_msgs::ParamGet>("/mavros/param/get");
        log->write_show(LogLevel::INFO, "Init Parameter Service");
    }

    void Param::load()
    {
        log->write_show(LogLevel::INFO, "Load Parameter Service");

        if (reader.is_open())
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

            // Read from stream
            std::vector<ParamS> selection_param;
            for (const auto &item : _json_data)
            {
                std::string _param_id = item["param_id"].asString();
                std::string _param_type = item["type"].asString();
                if (_param_type.c_str() == "select")
                {
                    int _value = stoi(item["type"].asString());
                    ParamS _new_param = ParamS(_param_id, _param_type, _value);
                    for (const auto &_selection : item["selection"])
                    {
                        Option _option(stoi(_selection["value"].asString()),
                                       _selection["description"].asString());
                        _new_param.add(_option);
                    }
                    selection_param.push_back(_new_param);

                    /*for (int i = 0; i < item["selection"].size(); i++)
                    {
                        int _int_id = item["selectio"][i]["name"].asString();
                        std::string _descr = ;
                    }*/
                }
                else if (_param_type.c_str() == "float")
                {
                }
                else if (_param_type.c_str() == "bool")
                {
                }
                else
                {
                    log->write_show(LogLevel::WARNING, "Unknown type of %s",
                                    _param_id.c_str());
                }
            }
            enable_use = true;
        }
    }

    Param::~Param()
    {
        if (reader.is_open())
            reader.close();
    }
} // namespace EMIRO