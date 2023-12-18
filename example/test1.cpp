// rosrun emiro test1 1 0 1 0.5 5 0.3 8 1 1 1 1

#include "misi.hpp"

int main(int argc, char **argv)
{
    std::cout << "Sander" << std::endl;
    ros::init(argc, argv, "mavcon_node");
    std::shared_ptr<ros::NodeHandle> nh = std::make_shared<ros::NodeHandle>();
    ros::init(argc, argv, "mavcon_node");
    EMIRO::Misi2 misi(nh, argc, argv);
    misi.PreArm();
    misi.Run();
    misi.Land();
    return 0;

    return 0;
}

// #include <iostream>

// int main()
// {
//     std::cout << "Sander\n";
//     return 0;
// }