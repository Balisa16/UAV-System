#include <control.hpp>

int main(int argc, char **argv)
{
    ros::init(argc, argv, "manual_node");
    std::shared_ptr<ros::NodeHandle> nh = std::make_shared<ros::NodeHandle>();
    
    // Logger
    std::shared_ptr<EMIRO::Logger> logger = std::make_shared<EMIRO::Logger>();
    logger->init("Manual", EMIRO::FileType::CSV);
    logger->start(true);

    std::shared_ptr<EMIRO::Copter> copter = std::make_shared<EMIRO::Copter>();
    copter->init(nh, logger);
    std::shared_ptr<EMIRO::GPS> gps = std::make_shared<EMIRO::GPS>();
    gps->init(copter, logger);
    std::shared_ptr<EMIRO::Control> control = std::make_shared<EMIRO::Control>(copter, logger, gps);

    // Takeoff Copter
    copter->takeoff(1);
    ros::Duration(10).sleep();
    
    // Loop Variable
    uint8_t cnt = 20;

    control->go(0.0f, 4.0f, 0.5f, 0.05f);
    
    copter->Land();
    
    return 0;
}