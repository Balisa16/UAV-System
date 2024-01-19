#include <control.hpp>
#include <jsonio.hpp>

using namespace EMIRO;

int main(int argc, char **argv)
{
    ros::init(argc, argv, "manual_node");
    std::shared_ptr<ros::NodeHandle> nh = std::make_shared<ros::NodeHandle>();

    // Logger
    std::shared_ptr<Logger> logger = std::make_shared<Logger>();
    logger->init("Manual", FileType::CSV);
    logger->start(true);

    std::shared_ptr<Copter> copter = std::make_shared<Copter>();
    copter->init(nh, logger);
    std::shared_ptr<GPS> gps = std::make_shared<GPS>();
    gps->init(copter, logger);
    std::shared_ptr<Control> control = std::make_shared<Control>(copter, logger, gps);

    // Takeoff Copter
    copter->takeoff(1);
    ros::Duration(10).sleep();

    // Read JSON point
    JsonIO reader;
    reader = "../copter/plan.json";
    std::vector<Target> target = reader.get_data();

    // Set Speed limit
    control->speed_limit = 0.5;
    for (Target &t : target)
    {
        if (!ros::ok())
        {
            copter->Land();
            ros::Duration(5).sleep();
            exit(EXIT_FAILURE);
        }
        std::cout << C_GREEN << S_BOLD << '[' << t.header << ']' << C_RESET << '\n';
        control->go(t.wp.x, t.wp.y, t.wp.z, t.wp.yaw, 0.05f, 5);
    }

    copter->Land();
    ros::Duration(5).sleep();
    return 0;
}