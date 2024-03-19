#pragma once

#include <boost/filesystem.hpp>
#include <enum.hpp>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <json.hpp>
#include <map>
#include <vector>
#include <cmath>

namespace EMIRO
{
    struct Target
    {
        std::string header;
        float speed;
        WayPoint wp;

        Target(std::string header, float speed, WayPoint wp)
            : header(header), speed(speed), wp(wp) {}

        friend std::ostream &operator<<(std::ostream &os, Target target)
        {
            os << std::fixed << std::setprecision(2) << "\033[1;32m>>> "
               << target.header << " <<<\033[0m\nSpeed\t: " << target.speed
               << "\nTarget\n - x   : " << target.wp.x
               << "\n - y   : " << target.wp.y << "\n - z   : " << target.wp.z
               << "\n - yaw : " << target.wp.yaw << '\n'
               << std::defaultfloat;
            return os;
        }
    };

    struct TargetKey
    {
        uint64_t data_idx;
        std::string data_name;

        TargetKey(uint64_t idx, std::string name)
            : data_idx(idx), data_name(name) {}

        // Operator <
        bool operator<(const TargetKey &other) const
        {
            if (data_idx != other.data_idx)
            {
                return data_idx < other.data_idx;
            }
            return data_name < other.data_name;
        }

        bool operator==(const TargetKey &other) const
        {
            return data_name == other.data_name;
        }

        bool exist(std::map<TargetKey, Target> &data) const
        {
            for (auto &a : data)
                if (this->operator==(a.first))
                    return true;
            return false;
        }
    };

    class JsonIO
    {
    public:
        JsonIO();
        JsonIO(std::string path);

        /**
         * @brief Get the data in map format
         *
         * @return std::map<TargetKey, Target>& data
         */
        std::map<TargetKey, Target> &get_data_map();

        /**
         * @brief Get the data in vector format
         *
         * @return std::vector<Target> data
         */
        std::vector<Target> get_data_vector() const;

        /**
         * @brief Get the data in map format
         *
         * @param target reference of std::map
         */
        void get_data(std::map<TargetKey, Target> &target) const;

        /**
         * @brief Do simple optimization of track
         *
         */
        void optimize_distance();
        ~JsonIO();
        void operator=(std::string path);
        void operator+=(const Target &target);
        void operator-=(const Target &target);

    private:
        std::string file_path = "";
        uint64_t data_counter = 0;
        std::map<TargetKey, Target> data;
    };
} // namespace EMIRO