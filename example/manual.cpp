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


    ros::Rate r(1);
    copter->takeoff(1);
    ros::Duration(10).sleep();
    uint8_t cnt = 20;
    while (ros::ok() && cnt)
    {
        control->vy = 0.5f;
        control->go();
        ros::spinOnce();
        r.sleep();
        cnt--;
    }
    
    copter->Land();
    
    return 0;
}