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
    
    EMIRO::Control manual(copter, logger);
    manual.init();

    ros::Duration(2).sleep();

    ros::Rate r(1);
    copter->takeoff(1);
    uint8_t cnt = 20;
    while (ros::ok() && cnt)
    {
        // manual.vy = 0.5f;
        // manual.go();
        ros::spinOnce();
        r.sleep();
        cnt--;
    }

    copter->Land();
    
    return 0;
}