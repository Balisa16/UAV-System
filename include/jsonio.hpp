#pragma once

#include <boost/filesystem.hpp>
#include <enum.hpp>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <json.hpp>
#include <vector>

namespace EMIRO {
typedef struct {
    int index;
    std::string header;
    float speed;
    WayPoint wp;
} Target;

class JsonIO {
  public:
    JsonIO();
    JsonIO(std::string path);
    std::vector<Target> get_data();
    void get_data(std::vector<Target> &target);
    ~JsonIO();
    void operator=(std::string path);
    friend std::ostream &operator<<(std::ostream &os, Target target);
    void operator+=(const Target &target);
    void operator-=(const Target &target);

  private:
    std::string file_path = "";
    std::vector<Target> data;
    std::vector<std::string> header_id;
};
} // namespace EMIRO