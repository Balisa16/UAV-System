#include <jsonread.hpp>

using namespace EMIRO;

int main()
{
    JSONReader reader;
    reader = "../copter/plan.json";
    std::vector<Target> target = reader.get_data();
    for(auto i : target)
        std::cout << i ;
    return 0;
}