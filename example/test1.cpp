// rosrun emiro test1 1 0 1 0.5 5 0.3 8 1 1 1 1

#include "misi.hpp"

int main(int argc, char **argv)
{
    ros::init(argc, argv, "mavcon_node");
    EMIRO::Misi2 misi(argc, argv);
    misi.PreArm();
    misi.Run();
    misi.Land();
    return 0;
}