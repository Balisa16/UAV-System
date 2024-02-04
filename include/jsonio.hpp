#pragma once

#include <boost/filesystem.hpp>
#include <enum.hpp>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <json.hpp>
#include <map>
#include <vector>

namespace EMIRO {
struct Target {
    std::string header;
    float speed;
    WayPoint wp;

    Target(std::string header, float speed, WayPoint wp)
        : header(header), speed(speed), wp(wp) {}

    friend std::ostream &operator<<(std::ostream &os, Target target) {
        os << std::fixed << std::setprecision(2)
           << "\nHeader\t: " << target.header << "\nSpeed\t: " << target.speed
           << "\nTarget\n\tx : " << target.wp.x << "\n\ty : " << target.wp.y
           << "\n\tz : " << target.wp.z << '\n'
           << std::defaultfloat;
        return os;
    }
};

struct TargetKey {
    uint64_t data_idx;
    std::string data_name;

    TargetKey(uint64_t idx, std::string name)
        : data_idx(idx), data_name(name) {}

    // Operator <
    bool operator<(const TargetKey &other) const {
        if (data_idx != other.data_idx) {
            return data_idx < other.data_idx;
        }
        return data_name < other.data_name;
    }

    bool operator==(const TargetKey &other) const {
        return data_name == other.data_name;
    }

    bool exist(std::map<TargetKey, Target> &data) const {
        for (auto &a : data)
            if (this->operator==(a.first))
                return true;
        return false;
    }
};

class JsonIO {
  public:
    JsonIO();
    JsonIO(std::string path);
    std::map<TargetKey, Target> &get_data_map();
    std::vector<Target> get_data_vector();
    void get_data(std::map<TargetKey, Target> &target);
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