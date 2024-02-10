#include <copter.hpp>
#include <jsonio.hpp>

using namespace EMIRO;

int main()
{
    JsonIO reader;
    reader = COPTER_DIR + "/docs/plan.json";
    // std::map<std::string, Target> target = reader.get_ordered_data();
    // for (auto i : target)
    //     std::cout << i.first << "\t: " << i.second << std::endl;

    std::vector<Target> target = reader.get_data_vector();
    for (auto i : target)
        std::cout << i << std::endl;
    Target new_data = {"Waypoint 39", 1.00, {0.0, 2.0, 1.0, 0.0}};
    reader += new_data;
    return 0;
}