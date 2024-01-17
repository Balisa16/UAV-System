#include <jsonio.hpp>

using namespace EMIRO;

int main()
{
    JsonIO reader;
    reader = "../copter/plan.json";
    std::vector<Target> target = reader.get_data();
    for (auto i : target)
        std::cout << i;
    Target new_data = {
        0,
        "Waypoint 2",
        1.00,
        {0.0, 2.0, 1.0, 0.0}};
    reader += new_data;
    return 0;
}